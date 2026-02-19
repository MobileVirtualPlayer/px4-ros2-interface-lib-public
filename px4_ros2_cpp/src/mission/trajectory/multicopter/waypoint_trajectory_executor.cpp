/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/utils/geometry.hpp>

using namespace px4_ros2::literals;  // NOLINT

namespace px4_ros2::multicopter {
WaypointTrajectoryExecutor::WaypointTrajectoryExecutor(ModeBase& mode, float acceptance_radius)
    : _acceptance_radius(acceptance_radius), _node(mode.node())
{
  _multicopter_setpoint = std::make_shared<MulticopterGotoGlobalSetpointType>(mode);
  _rover_setpoint = std::make_shared<RoverPositionSetpointType>(mode);
  _vehicle_global_position = std::make_shared<OdometryGlobalPosition>(mode);
  _vehicle_attitude = std::make_shared<OdometryAttitude>(mode);
  _vehicle_status = std::make_shared<VehicleStatus>(mode);
  _map_projection = std::make_unique<MapProjection>(mode);
  
  // Subscribe to mission current waypoint from mission_manager
  _mission_current_sub = mode.node().create_subscription<std_msgs::msg::UInt16>(
      "/mission/current_waypoint",
      rclcpp::QoS(10),
      [this](std_msgs::msg::UInt16::ConstSharedPtr msg) {
        missionCurrentCallback(msg);
      });
  
  // Subscribe to mission speed from mission_manager
  _mission_speed_sub = mode.node().create_subscription<std_msgs::msg::Float32>(
      "/mission/speed",
      rclcpp::QoS(10).transient_local(),
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        missionSpeedCallback(msg);
      });
}

bool WaypointTrajectoryExecutor::navigationItemTypeSupported(NavigationItemType type)
{
  return type == NavigationItemType::Waypoint;
}

bool WaypointTrajectoryExecutor::frameSupported(MissionFrame frame)
{
  return frame == MissionFrame::Global;
}

void WaypointTrajectoryExecutor::runTrajectory(const TrajectoryConfig& config)
{
  _current_trajectory = config;
  _current_index = config.start_index;
  
  // Capture the rover's current position as the start point for the first waypoint
  if (isRover() && _vehicle_global_position->positionValid()) {
    const Eigen::Vector3d& current_global_pos = _vehicle_global_position->position();
    _previous_waypoint_ned = _map_projection->globalToLocal(
        Eigen::Vector2d(current_global_pos[0], current_global_pos[1]));
    RCLCPP_DEBUG(_node.get_logger(), 
                 "Starting trajectory - rover start position: N=%.2f, E=%.2f",
                 (*_previous_waypoint_ned)[0], (*_previous_waypoint_ned)[1]);
  } else {
    _previous_waypoint_ned.reset();  // Reset for multicopters or if position not valid
  }
}

void WaypointTrajectoryExecutor::updateSetpoint()
{
  if (!_current_index) {
    return;
  }
  const auto* navigation_item =
      std::get_if<NavigationItem>(&_current_trajectory.trajectory->items()[*_current_index]);
  if (!navigation_item) {
    // Not expected to happen
    continueNextItem();
    return;
  }

  if (!_vehicle_global_position->positionValid()) {
    static constexpr double kPositionGracePeriodS = 5.0;
    if (!_position_invalid_since.has_value()) {
      _position_invalid_since = _node.get_clock()->now();
    }
    const double invalid_s =
        (_node.get_clock()->now() - *_position_invalid_since).seconds();
    if (invalid_s < kPositionGracePeriodS) {
      RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                           "Waiting for valid global position (%.1fs / %.1fs)...",
                           invalid_s, kPositionGracePeriodS);
      return;
    }
    const bool msg_fresh = _vehicle_global_position->lastValid();
    bool lat_lon_valid = false;
    bool alt_valid = false;
    if (msg_fresh) {
      lat_lon_valid = _vehicle_global_position->last().lat_lon_valid;
      alt_valid = _vehicle_global_position->last().alt_valid;
    }
    RCLCPP_ERROR(_node.get_logger(),
                 "Global position not valid after %.1fs grace period "
                 "(msg_fresh=%d lat_lon_valid=%d alt_valid=%d), aborting",
                 invalid_s, msg_fresh, lat_lon_valid, alt_valid);
    _current_trajectory.on_failure();
    return;
  }
  _position_invalid_since.reset();

  const auto& waypoint = std::get<Waypoint>(navigation_item->data);
  const Eigen::Vector3d& target_position = waypoint.coordinate;
  std::optional<float> heading_target_rad{};

  if (horizontalDistanceToGlobalPosition(_vehicle_global_position->position(), target_position) >
      0.1f) {
    // Stop caring about heading when the arctangent becomes undefined
    heading_target_rad =
        headingToGlobalPosition(_vehicle_global_position->position(), target_position);
  }

  const auto& options = _current_trajectory.options;
  
  if (isRover()) {
    RCLCPP_DEBUG(_node.get_logger(), "[DEBUG] Rover mode detected - using rover position setpoint");
    
    // Rover: Use 2D position setpoint with local NED coordinates
    // Convert global target (lat, lon, alt) to local NED (north, east)
    Eigen::Vector2f target_local_2d = _map_projection->globalToLocal(
        Eigen::Vector2d(target_position[0], target_position[1]));
    
    // Use mission speed if available, otherwise fall back to horizontal velocity
    std::optional<float> cruising_speed = _mission_speed.has_value() ? _mission_speed : options.horizontal_velocity;
    
    // Determine arrival speed by looking ahead to the next waypoint
    std::optional<float> arrival_speed;
    if (cruising_speed.has_value() && *_current_index + 1 <= _current_trajectory.end_index) {
      // Get the next waypoint
      const auto* next_navigation_item =
          std::get_if<NavigationItem>(&_current_trajectory.trajectory->items()[*_current_index + 1]);
      
      if (next_navigation_item) {
        const auto& next_waypoint = std::get<Waypoint>(next_navigation_item->data);
        const Eigen::Vector3d& next_position = next_waypoint.coordinate;
        
        // Calculate bearing from current waypoint to next waypoint
        const float bearing_current_to_next = headingToGlobalPosition(target_position, next_position);
        
        // Calculate bearing from rover's position to current waypoint
        const float bearing_to_current = headingToGlobalPosition(
            _vehicle_global_position->position(), target_position);
        
        // Calculate the turn angle at the current waypoint
        const float turn_angle = fabsf(wrapPi(bearing_current_to_next - bearing_to_current));
        
        // If rover needs to turn more than 90 degrees at arrival, it should stop for pivot turn
        static constexpr float kPivotTurnThreshold = M_PI_2;  // 90 degrees
        if (turn_angle > kPivotTurnThreshold) {
          arrival_speed = 0.0f;
          RCLCPP_DEBUG(_node.get_logger(), 
                       "[DEBUG] Sharp turn ahead (%.1f°) - arrival_speed = 0 (pivot turn)", 
                       turn_angle * 180.0f / M_PI);
        } else {
          arrival_speed = *cruising_speed;
          RCLCPP_DEBUG(_node.get_logger(), 
                       "[DEBUG] Gentle turn ahead (%.1f°) - arrival_speed = %.2f m/s", 
                       turn_angle * 180.0f / M_PI, *arrival_speed);
        }
      } else {
        // Next item is not a navigation waypoint (might be an action), maintain speed
        arrival_speed = *cruising_speed;
      }
    } else {
      // This is the last waypoint, slow to a stop
      arrival_speed = 0.0f;
      RCLCPP_DEBUG(_node.get_logger(), "[DEBUG] Last waypoint - arrival_speed = 0");
    }
    
    // Use the previous waypoint position as start_ned if available
    _rover_setpoint->update(target_local_2d, _previous_waypoint_ned, cruising_speed, arrival_speed, std::nullopt);
  } else {
    // Multicopter: Use 3D goto setpoint with global coordinates
    _multicopter_setpoint->update(target_position, heading_target_rad, options.horizontal_velocity,
                                  options.vertical_velocity, options.max_heading_rate);
  }

  float acceptance_radius = _acceptance_radius;
  if (*_current_index == _current_trajectory.end_index && _current_trajectory.stop_at_last) {
    acceptance_radius /= 2.f;
  }

  if (positionReached(target_position, acceptance_radius)) {
    RCLCPP_INFO(_node.get_logger(), 
                "Waypoint %d reached (error < %.2f m) - transitioning to next waypoint",
                *_current_index, acceptance_radius);
    continueNextItem();
  }
}

void WaypointTrajectoryExecutor::continueNextItem()
{
  const int index_reached = *_current_index;
  
  // Capture the rover's current position in NED before moving to the next waypoint
  // This will be used as start_ned for the next waypoint
  if (isRover() && _vehicle_global_position->positionValid()) {
    const Eigen::Vector3d& current_global_pos = _vehicle_global_position->position();
    _previous_waypoint_ned = _map_projection->globalToLocal(
        Eigen::Vector2d(current_global_pos[0], current_global_pos[1]));
    RCLCPP_DEBUG(_node.get_logger(), 
                 "Captured rover start position: N=%.2f, E=%.2f",
                 (*_previous_waypoint_ned)[0], (*_previous_waypoint_ned)[1]);
  }
  
  _current_index = *_current_index + 1;
  if (*_current_index > _current_trajectory.end_index) {
    RCLCPP_INFO(_node.get_logger(), "All waypoints completed - ending trajectory");
    _current_index.reset();
  } else {
    RCLCPP_INFO(_node.get_logger(), "Moving to waypoint %d", *_current_index);
  }
  // Call the callback after updating the state (as it might set a new trajectory already)
  _current_trajectory.on_index_reached(index_reached);
}

bool WaypointTrajectoryExecutor::positionReached(const Eigen::Vector3d& target_position_m,
                                                 float acceptance_radius) const
{
  // Rovers use horizontal distance only, multicopters use 3D distance
  const float position_error = isRover() ?
      horizontalDistanceToGlobalPosition(_vehicle_global_position->position(), target_position_m) :
      distanceToGlobalPosition(_vehicle_global_position->position(), target_position_m);
  return position_error < acceptance_radius;
}

bool WaypointTrajectoryExecutor::headingReached(float target_heading_rad) const
{
  static constexpr float kHeadingErrorThreshold = 7.0_deg;
  const float heading_error_wrapped = wrapPi(target_heading_rad - _vehicle_attitude->yaw());
  return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
}

bool WaypointTrajectoryExecutor::isRover() const
{
  return _vehicle_status->last().vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROVER;
}

void WaypointTrajectoryExecutor::missionCurrentCallback(
    std_msgs::msg::UInt16::ConstSharedPtr msg)
{
  // Only update if we have an active trajectory
  if (!_current_index) {
    return;
  }

  // Check if mission manager's current index has changed
  const uint16_t new_current_index = msg->data;
  
  // Update our tracking index if it changed
  if (new_current_index != *_current_index && 
      new_current_index >= _current_trajectory.start_index &&
      new_current_index <= _current_trajectory.end_index) {
    
    RCLCPP_INFO(_node.get_logger(), 
                "Mission index updated from %d to %d - updating setpoint",
                *_current_index, new_current_index);
    
    // Capture the rover's current position before jumping to the new waypoint
    if (isRover() && _vehicle_global_position->positionValid()) {
      const Eigen::Vector3d& current_global_pos = _vehicle_global_position->position();
      _previous_waypoint_ned = _map_projection->globalToLocal(
          Eigen::Vector2d(current_global_pos[0], current_global_pos[1]));
    }
    
    _current_index = new_current_index;
    
    // Immediately update the setpoint for the new waypoint
    updateSetpoint();
  }
}

void WaypointTrajectoryExecutor::missionSpeedCallback(
    std_msgs::msg::Float32::ConstSharedPtr msg)
{
  _mission_speed = msg->data;
  RCLCPP_DEBUG(_node.get_logger(), "Mission speed updated to: %.2f m/s", *_mission_speed);
}

}  // namespace px4_ros2::multicopter
