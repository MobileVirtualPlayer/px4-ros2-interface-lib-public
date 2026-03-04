/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <array>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_ros2 {

/**
 * @brief Helper class for publishing obstacle distances to PX4.
 *
 * Publishes px4_msgs::msg::ObstacleDistance to /fmu/in/obstacle_distance
 * at whatever rate the caller invokes update(). Intended for companion-computer
 * geofence enforcement: the caller computes per-sector distances to geofence
 * boundaries and PX4 uses the stream for collision prevention / motion planning.
 *
 * Frame: MAV_FRAME_BODY_FRD (sector 0 = forward, clockwise).
 * Resolution: 5° per sector, 72 sectors = 360°.
 * Units: centimeters (UINT16_MAX = no obstacle).
 */
class ObstacleDistance {
public:
  explicit ObstacleDistance(rclcpp::Node& node);

  /**
   * @brief Publish one obstacle distance message.
   *
   * @param distances  72-element array of distances in cm. Index 0 corresponds
   *                   to the vehicle's forward direction; indices increase
   *                   clockwise. UINT16_MAX indicates no obstacle detected.
   * @param min_dist_cm  Minimum measurable distance in cm (default 100 = 1 m).
   * @param max_dist_cm  Maximum measurable distance in cm (default 10000 = 100 m).
   */
  void update(const std::array<uint16_t, 72>& distances,
              uint16_t min_dist_cm = 100,
              uint16_t max_dist_cm = 10000);

private:
  rclcpp::Node& _node;
  rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _pub;
};

}  // namespace px4_ros2
