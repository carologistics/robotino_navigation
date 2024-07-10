// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include <chrono>
#include <cmath>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "recovery_behavior/recover_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors {

RecoverPoseCls::RecoverPoseCls()
    : TimedBehavior<RecoverPoseAction>(),
      feedback_(std::make_shared<RecoverPoseAction::Feedback>()) {}

RecoverPoseCls::~RecoverPoseCls() = default;

void RecoverPoseCls::onConfigure() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "robot_radius",
                                               rclcpp::ParameterValue(0.450));
  node->get_parameter("robot_radius", robot_radius);
}

ResultStatus RecoverPoseCls::onRun(
    const std::shared_ptr<const RecoverPoseAction::Goal> command) {
  command_dist_ = command->target.x;
  command_speed_ = command->speed;
  command_time_allowance_ = command->time_allowance;
  angle_threshold = (std::atan2((robot_radius / 2), command_dist_));
  angle_heading = 0.0;

  end_time_ = this->clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(initial_local_pose, *tf_, local_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return ResultStatus{Status::FAILED, RecoverPoseActionResult::TF_ERROR};
  }

  RCLCPP_ERROR(logger_, "Initial_local robot pose: %f, %f, %f",
               initial_local_pose.pose.position.x,
               initial_local_pose.pose.position.y,
               tf2::getYaw(initial_local_pose.pose.orientation));

  geometry_msgs::msg::Pose2D pose2d_local;
  pose2d_local.x = initial_local_pose.pose.position.x;
  pose2d_local.y = initial_local_pose.pose.position.y;
  pose2d_local.theta = tf2::getYaw(initial_local_pose.pose.orientation);

  if (!nav2_util::getCurrentPose(initial_global_pose, *tf_, global_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return ResultStatus{Status::FAILED, RecoverPoseActionResult::TF_ERROR};
  }

  RCLCPP_ERROR(logger_, "Initial_global robot pose: %f, %f, %f",
               initial_global_pose.pose.position.x,
               initial_global_pose.pose.position.y,
               tf2::getYaw(initial_global_pose.pose.orientation));

  geometry_msgs::msg::Pose2D pose2d_global;
  pose2d_global.x = initial_global_pose.pose.position.x;
  pose2d_global.y = initial_global_pose.pose.position.y;
  pose2d_global.theta = tf2::getYaw(initial_global_pose.pose.orientation);

  RCLCPP_INFO(logger_, "[onRun]:RecoverPoseAction configured, simulating ahead "
                       "for collision check");

  if (!isCollisionFree(command_dist_, pose2d_local, pose2d_global)) {
    this->stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting RecoverPose behavior");
    return ResultStatus{Status::FAILED,
                        RecoverPoseActionResult::COLLISION_AHEAD};
  }
  return ResultStatus{Status::SUCCEEDED, RecoverPoseActionResult::NONE};
}

ResultStatus RecoverPoseCls::onCycleUpdate() {
  RCLCPP_INFO(
      logger_,
      "[onCycleUpdate]: Collision check complete, Executing recovery behavior");
  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();

  if (time_remaining.seconds() < 0.0 &&
      command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_,
                "[onCycleUpdate]:Exceeded time allowance before reaching the "
                "RecoverPoseAction goal - Exiting RecoverPoseAction");
    return ResultStatus{Status::FAILED, RecoverPoseActionResult::NONE};
  }

  if (!nav2_util::getCurrentPose(current_pose, *tf_, local_frame_,
                                 robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_,
                 "[onCycleUpdate]: Current robot pose is not available.");
    return ResultStatus{Status::FAILED, RecoverPoseActionResult::TF_ERROR};
  }

  double diff_x =
      initial_local_pose.pose.position.x - current_pose.pose.position.x;
  double diff_y =
      initial_local_pose.pose.position.y - current_pose.pose.position.y;
  double distance = hypot(diff_x, diff_y);
  feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_dist_)) {
    this->stopRobot();
    return ResultStatus{Status::SUCCEEDED, RecoverPoseActionResult::NONE};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = this->clock_->now();
  cmd_vel->header.frame_id = this->robot_base_frame_;
  cmd_vel->twist.linear.y = command_speed_ * sin(angle_heading);
  cmd_vel->twist.angular.z = 0.0;
  cmd_vel->twist.linear.x = command_speed_ * cos(angle_heading);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  // if (!dynamicCollisionCheck(distance, cmd_vel->twist, pose2d)) {
  //   this->stopRobot();
  //   RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
  //   return ResultStatus{Status::FAILED,
  //                       RecoverPoseActionResult::COLLISION_AHEAD};
  // }

  vel_pub_->publish(std::move(cmd_vel));

  return ResultStatus{Status::RUNNING, RecoverPoseActionResult::NONE};
}

bool RecoverPoseCls::isCollisionFree(
    const double &distance, geometry_msgs::msg::Pose2D &pose2d_local,
    geometry_msgs::msg::Pose2D &pose2d_global) {

  geometry_msgs::msg::Pose2D local_pose = pose2d_local;
  geometry_msgs::msg::Pose2D global_pose = pose2d_global;
  bool fetch_local_data = true;
  bool fetch_global_data = true;
  double projected_distance = distance;
  double posechange_x;
  double posechange_y;

  for (float i = 0; i <= 2 * M_PI; i += angle_threshold) {

    posechange_x = projected_distance * cos(i);
    posechange_y = projected_distance * sin(i);

    pose2d_local.x = local_pose.x + posechange_x * cos(local_pose.theta) -
                     posechange_y * sin(local_pose.theta);
    pose2d_local.y = local_pose.y + posechange_x * sin(local_pose.theta) +
                     posechange_y * cos(local_pose.theta);
    pose2d_local.theta = local_pose.theta;

    pose2d_global.x = global_pose.x + posechange_x * cos(global_pose.theta) -
                      posechange_y * sin(global_pose.theta);
    pose2d_global.y = global_pose.y + posechange_x * sin(global_pose.theta) +
                      posechange_y * cos(global_pose.theta);
    pose2d_global.theta = global_pose.theta;

    // RCLCPP_INFO(logger_, "Simulating robot pose for heading angle: %f,:", i);
    // RCLCPP_ERROR(logger_, "Simulating robot_local pose: %f, %f, %f",
    // pose2d_local.x, pose2d_local.y, pose2d_local.theta);
    // RCLCPP_ERROR(logger_, "Simulating robot_global pose: %f, %f, %f",
    // pose2d_global.x, pose2d_global.y, pose2d_global.theta);

    if (this->global_collision_checker_->isCollisionFree(pose2d_global,
                                                         fetch_global_data)) {
      RCLCPP_INFO(logger_,
                  "[global_collision_checker_]: Declared availibility of free "
                  "space at: %f, %f, %f ",
                  pose2d_global.x, pose2d_global.y, pose2d_global.theta);
      if (this->local_collision_checker_->isCollisionFree(pose2d_local,
                                                          fetch_local_data)) {
        // RCLCPP_INFO(logger_, "[local_collision_checker_]: Declared
        // availibility of free space at: %f, %f, %f ", pose2d_local.x,
        // pose2d_local.y, pose2d_local.theta);
        angle_heading = i;
        return true;
      }
      fetch_local_data = false;
    }
    fetch_global_data = false;
  }

  RCLCPP_INFO(
      logger_,
      "[collision_checkers]: Unable to identify the free space around robot");
  return false;
}

bool RecoverPoseCls::dynamicCollisionCheck(
    const double &distance, const geometry_msgs::msg::Twist &cmd_vel,
    geometry_msgs::msg::Pose2D &pose2d) {
  int cycle_count = 0;
  double sim_position_change_x = 0.0;
  double sim_position_change_y = 0.0;
  const double target_dist = abs(command_dist_) - distance;
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  // RCLCPP_INFO(logger_, "simulating ahead of time for given data distance:.%f,
  // pose:,%f, %f, %f,", distance, pose2d.x, pose2d.y, pose2d.theta);

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

    // RCLCPP_INFO(logger_, "sim_position chnage: %f, %f",
    // sim_position_change_x, sim_position_change_y); RCLCPP_INFO(logger_,
    // "current cycle count: %d", cycle_count); RCLCPP_INFO(logger_, "simulated
    // poses for given distance and pose:,%f, %f, %f,", pose2d.x, pose2d.y,
    // pose2d.theta);

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

} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::RecoverPoseCls, nav2_core::Behavior)
