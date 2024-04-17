// Licensed under MIT. See LICENSE file. Copyright Carologistics.

//   created on: 2024-06-02
//   MIT License
//   Copyright (c) 2023 Saurabh Borse
//   Permission is hereby granted, free of charge, to any person obtaining a
//   copy of this software and associated documentation files (the "Software"),
//   to deal in the Software without restriction, including without limitation
//   the rights to use, copy, modify, merge, publish, distribute, sublicense,
//   and/or sell copies of the Software, and to permit persons to whom the
//   Software is furnished to do so, subject to the following conditions: The
//   above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.

//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//   DEALINGS IN THE SOFTWARE.

#ifndef RECOVERY_BEHAVIORS__RECOVER_POSE_HPP_
#define RECOVERY_BEHAVIORS__RECOVER_POSE_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>
// #include <sstream>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "recovery_msgs/action/recover_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_behaviors {

using namespace nav2_behaviors; // NOLINT
using RecoverPoseAction = recovery_msgs::action::RecoverPose;

class RecoverPoseCls : public TimedBehavior<RecoverPoseAction> {
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using RecoverPoseActionGoal = RecoverPoseAction::Goal;
  using RecoverPoseActionResult = RecoverPoseAction::Result;

  RecoverPoseCls();
  ~RecoverPoseCls();

  ResultStatus
  onRun(const std::shared_ptr<const RecoverPoseAction::Goal> command) override;
  void onConfigure() override;
  ResultStatus onCycleUpdate() override;
  CostmapInfoType getResourceInfo() override { return CostmapInfoType::BOTH; }

protected:
  bool isCollisionFree(const double &distance,
                       geometry_msgs::msg::Pose2D &pose2d_local,
                       geometry_msgs::msg::Pose2D &pose2d_global);

  bool dynamicCollisionCheck(const double &distance,
                             const geometry_msgs::msg::Twist &cmd_vel,
                             geometry_msgs::msg::Pose2D &pose2d);

  RecoverPoseAction::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_local_pose;
  geometry_msgs::msg::PoseStamped initial_global_pose;
  geometry_msgs::msg::PoseStamped current_pose;
  double command_dist_;
  double command_speed_;
  double robot_radius;
  double angle_threshold;
  double angle_heading;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

} // namespace nav2_behaviors
// namespace nav2_behaviors

#endif // RECOVERY_BEHAVIORS__RECOVER_POSE_HPP_
