/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/obstacle/obstacle_distance.hpp>

namespace px4_ros2 {

ObstacleDistance::ObstacleDistance(rclcpp::Node& node) : _node(node)
{
  _pub = node.create_publisher<px4_msgs::msg::ObstacleDistance>(
      "fmu/in/obstacle_distance", rclcpp::QoS(1).best_effort());
}

void ObstacleDistance::update(const std::array<uint16_t, 72>& distances,
                               uint16_t min_dist_cm, uint16_t max_dist_cm)
{
  px4_msgs::msg::ObstacleDistance msg{};
  msg.timestamp    = _node.get_clock()->now().nanoseconds() / 1000ULL;
  msg.frame        = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
  msg.sensor_type  = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
  msg.increment    = 5.0f;
  msg.angle_offset = 0.0f;
  msg.min_distance = min_dist_cm;
  msg.max_distance = max_dist_cm;
  msg.distances    = distances;
  _pub->publish(msg);
}

}  // namespace px4_ros2
