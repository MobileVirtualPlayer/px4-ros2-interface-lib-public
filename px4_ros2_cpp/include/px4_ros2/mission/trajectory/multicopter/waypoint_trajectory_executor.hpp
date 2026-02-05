/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once
#include <memory>
#include <optional>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/position.hpp>
#include <px4_ros2/mission/mission.hpp>
#include <px4_ros2/mission/trajectory/trajectory_executor.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

/** \ingroup mission_multicopter
 *  @{
 */

namespace px4_ros2::multicopter {
/**
 * @brief Trajectory executor using goto setpoints (supports multicopter and rover)
 * @ingroup mission_multicopter
 */
class WaypointTrajectoryExecutor : public TrajectoryExecutorInterface {
 public:
  explicit WaypointTrajectoryExecutor(ModeBase& mode, float acceptance_radius = 2.0f);

  ~WaypointTrajectoryExecutor() override = default;

  bool navigationItemTypeSupported(NavigationItemType type) override;
  bool frameSupported(MissionFrame frame) override;
  void runTrajectory(const TrajectoryConfig& config) override;
  void updateSetpoint() override;

 private:
  void continueNextItem();
  bool positionReached(const Eigen::Vector3d& target_position_m, float acceptance_radius) const;
  bool headingReached(float target_heading_rad) const;
  bool isRover() const;
  void missionCurrentCallback(std_msgs::msg::UInt16::ConstSharedPtr msg);
  void missionSpeedCallback(std_msgs::msg::Float32::ConstSharedPtr msg);

  TrajectoryConfig _current_trajectory;
  const float _acceptance_radius;
  std::shared_ptr<OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<VehicleStatus> _vehicle_status;
  std::shared_ptr<MulticopterGotoGlobalSetpointType> _multicopter_setpoint;
  std::shared_ptr<RoverPositionSetpointType> _rover_setpoint;
  std::unique_ptr<MapProjection> _map_projection;
  std::optional<int> _current_index;
  rclcpp::Node& _node;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _mission_current_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _mission_speed_sub;
  std::optional<float> _mission_speed;
};

}  // namespace px4_ros2::multicopter
/** @}*/
