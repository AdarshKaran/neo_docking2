// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEO_DOCKING2_DOCK_HPP_
#define NEO_DOCKING2_DOCK_HPP_

#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking_core/charging_dock.hpp"
#include "opennav_docking/pose_filter.hpp"

namespace neo_docking2
{

class NeoDock : public opennav_docking_core::ChargingDock
{
public:
  /**
   * @brief Constructor
   */
  NeoDock()
  : ChargingDock()
  {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() {}

  /**
   * 
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() {}

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() {}

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock with pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame);

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @copydoc opennav_docking_core::ChargingDock::isDocked
   */
  virtual bool isDocked();

  /**
   * @copydoc opennav_docking_core::ChargingDock::isCharging
   */
  virtual bool isCharging();

  /**
   * @copydoc opennav_docking_core::ChargingDock::disableCharging
   */
  virtual bool disableCharging();

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging();

  /**
   * @brief Method to charge the gazebo linear_battery if the robot has docked
   */
  void start_charging();

  /**
   * @brief Method to stop charging the gazebo linear_battery if the robot has undocked
   */
  void stop_charging();

protected:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);

  // Optionally subscribe to a detected dock pose topic
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;

  // Trigger to publish dock pose as external feedback
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_publishing_dock_pose_;

  // If subscribed to a detected pose topic, will contain latest message
  geometry_msgs::msg::PoseStamped detected_dock_pose_;
  // This is the actual dock pose once it has the specified translation/rotation applied
  // If not subscribed to a topic, this is simply the database dock pose
  geometry_msgs::msg::PoseStamped dock_pose_;

  // Subscribe to battery message, used to determine if charging
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  bool is_charging_;
  bool use_battery_status_;

  // Optionally subscribe to joint state message, used to determine if stalled
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<std::string> stall_joint_names_;
  double stall_velocity_threshold_, stall_effort_threshold_;
  bool is_stalled_;

  // An external reference (such as image_proc::TrackMarkerNode) can be used to detect dock
  bool use_external_detection_pose_;
  double external_detection_timeout_;
  tf2::Quaternion external_detection_rotation_;
  double external_detection_translation_x_;
  double external_detection_translation_y_;

  // Filtering of detected poses
  std::shared_ptr<opennav_docking::PoseFilter> filter_;

  // Threshold that battery current must exceed to be "charging" (in Amperes)
  int charging_threshold_;
  // If not using an external pose reference, this is the distance threshold
  double docking_threshold_;
  std::string base_frame_id_;
  // Offset for staging pose relative to dock pose
  double staging_x_offset_;
  double staging_yaw_offset_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

}  // namespace neo_docking2

#endif  // NEO_DOCKING2_DOCK_HPP_