// Licensed under MIT. See LICENSE file. Copyright Carologistics.

//   created on: 2024-06-02
//
//   MIT License
//
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "recovery_behavior/back_up.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors {

BackUpCls::BackUpCls()
    : TimedBehavior<BackUpAction>(),
      feedback_(std::make_shared<BackUpAction::Feedback>()), command_x_(0.0),
      command_speed_(0.0) {}

BackUpCls::~BackUpCls() = default;

void BackUpCls::onConfigure() {
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
}

ResultStatus
BackUpCls::onRun(const std::shared_ptr<const BackUpAction::Goal> command) {
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(logger_,
                "Backing up in Y and Z not supported, will only move in X.");
    return ResultStatus{Status::FAILED, BackUpActionResult::INVALID_INPUT};
  }

  // Silently ensure that both the speed and direction are negative.
  command_x_ = -std::fabs(command->target.x);
  command_speed_ = -std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(initial_pose, *tf_, local_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return ResultStatus{Status::FAILED, BackUpActionResult::TF_ERROR};
  }

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = initial_pose.pose.position.x;
  pose2d.y = initial_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(initial_pose.pose.orientation);

  RCLCPP_INFO(
      logger_,
      "[onRun]: BackUp configured, simulating ahead for collision check");

  if (!isCollisionFree(command_x_, pose2d)) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting BackUp behavior");
    return ResultStatus{Status::FAILED, BackUpActionResult::COLLISION_AHEAD};
  }
  return ResultStatus{Status::SUCCEEDED, BackUpActionResult::NONE};
}

ResultStatus BackUpCls::onCycleUpdate() {
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 &&
      command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the Backup "
                         "goal - Exiting Backup");
    return ResultStatus{Status::FAILED, BackUpActionResult::NONE};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, local_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return ResultStatus{Status::FAILED, BackUpActionResult::TF_ERROR};
  }

  double diff_x = initial_pose.pose.position.x - current_pose.pose.position.x;
  double diff_y = initial_pose.pose.position.y - current_pose.pose.position.y;
  double distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    this->stopRobot();
    return ResultStatus{Status::SUCCEEDED, BackUpActionResult::NONE};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = this->clock_->now();
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->twist.linear.y = 0.0;
  cmd_vel->twist.angular.z = 0.0;
  cmd_vel->twist.linear.x = command_speed_;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!dynamicCollisionCheck(distance, cmd_vel->twist, pose2d)) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
    return ResultStatus{Status::FAILED, BackUpActionResult::COLLISION_AHEAD};
  }

  vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, BackUpActionResult::NONE};
}

bool BackUpCls::isCollisionFree(const double &distance,
                                geometry_msgs::msg::Pose2D &pose2d) {
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;
  double posechange_x = distance;
  double posechange_y = 0.0;

  pose2d.x = init_pose.x + posechange_x * cos(init_pose.theta) -
             posechange_y * sin(init_pose.theta);
  pose2d.y = init_pose.y + posechange_x * sin(init_pose.theta) +
             posechange_y * cos(init_pose.theta);
  pose2d.theta = init_pose.theta;

  RCLCPP_INFO(logger_, "Simulating robot pose: %f, %f, %f", pose2d.x, pose2d.y,
              pose2d.theta);

  if (!this->local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
    RCLCPP_INFO(logger_,
                "[local_collision_checker_]: Declared possibility of collision "
                "ahead at: %f, %f, %f ",
                pose2d.x, pose2d.y, pose2d.theta);
    return false;
  }

  fetch_data = false;

  return true;
}

bool BackUpCls::dynamicCollisionCheck(const double &distance,
                                      const geometry_msgs::msg::Twist &cmd_vel,
                                      geometry_msgs::msg::Pose2D &pose2d) {

  // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change_x = 0.0;
  double sim_position_change_y = 0.0;
  const double target_dist = abs(command_x_) - distance;
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  RCLCPP_INFO(
      logger_,
      "simulating ahead of time for given data distance:.%f, pose:,%f, %f, %f,",
      distance, pose2d.x, pose2d.y, pose2d.theta);

  while (true) {
    sim_position_change_x =
        cmd_vel.linear.x * (cycle_count / this->cycle_frequency_);
    sim_position_change_y =
        cmd_vel.linear.y * (cycle_count / this->cycle_frequency_);
    pose2d.x = init_pose.x + sim_position_change_x * cos(init_pose.theta) -
               sim_position_change_y * sin(init_pose.theta);
    pose2d.y = init_pose.y + sim_position_change_x * sin(init_pose.theta) +
               sim_position_change_y * cos(init_pose.theta);
    cycle_count += 1;

    RCLCPP_INFO(logger_, "sim_position chnage: %f, %f", sim_position_change_x,
                sim_position_change_y);
    RCLCPP_INFO(logger_, "current cycle count: %d", cycle_count);
    RCLCPP_INFO(logger_,
                "simulated poses for given distance and pose:,%f, %f, %f,",
                pose2d.x, pose2d.y, pose2d.theta);

    if (target_dist <= hypot(sim_position_change_x, sim_position_change_y)) {
      break;
    }

    if (!this->local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      RCLCPP_INFO(logger_,
                  "[local_collision_checker_]: Unable to identify the free "
                  "space around robot at: %f, %f, %f ",
                  pose2d.x, pose2d.y, pose2d.theta);
      return false;
    }
    fetch_data = false;
  }
  return true;
}

}; // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUpCls, nav2_core::Behavior)
