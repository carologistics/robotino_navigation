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

#ifndef RECOVERY_BEHAVIORS__BACK_UP_HPP_
#define RECOVERY_BEHAVIORS__BACK_UP_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>
// #include <sstream>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behaviors {

using namespace nav2_behaviors; // NOLINT
using BackUpAction = nav2_msgs::action::BackUp;

class BackUpCls : public TimedBehavior<BackUpAction> {
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using BackUpActionGoal = BackUpAction::Goal;
  using BackUpActionResult = BackUpAction::Result;

  BackUpCls();
  ~BackUpCls();

  ResultStatus
  onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  void onConfigure() override;
  ResultStatus onCycleUpdate() override;
  CostmapInfoType getResourceInfo() override { return CostmapInfoType::BOTH; }

protected:
  bool isCollisionFree(const double &distance,
                       geometry_msgs::msg::Pose2D &pose2d);

  bool dynamicCollisionCheck(const double &distance,
                             const geometry_msgs::msg::Twist &cmd_vel,
                             geometry_msgs::msg::Pose2D &pose2d);

  BackUpAction::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose;
  double command_x_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

} // namespace nav2_behaviors
// namespace nav2_behaviors

#endif // RECOVERY_BEHAVIORS__BACK_UP_HPP_
