/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/mission/trajectory/multicopter/waypoint_trajectory_executor.hpp>
#include <px4_ros2/utils/message_version.hpp>

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
    RCLCPP_ERROR(_node.get_logger(), "Global position not valid, aborting");
    _current_trajectory.on_failure();
    return;
  }

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
    std::optional<float> arrival_speed = std::nullopt;  // Stop at waypoint
    
    _rover_setpoint->update(target_local_2d, std::nullopt, cruising_speed, arrival_speed, heading_target_rad);
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
    continueNextItem();
  }
}

void WaypointTrajectoryExecutor::continueNextItem()
{
  const int index_reached = *_current_index;
  _current_index = *_current_index + 1;
  if (*_current_index > _current_trajectory.end_index) {
    _current_index.reset();
  }
  // Call the callback after updating the state (as it might set a new trajectory already)
  _current_trajectory.on_index_reached(index_reached);
}

bool WaypointTrajectoryExecutor::positionReached(const Eigen::Vector3d& target_position_m,
                                                 float acceptance_radius) const
{
  const float position_error =
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
                "Mission index updated from %d to %d - updating rover setpoint",
                *_current_index, new_current_index);
    
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
