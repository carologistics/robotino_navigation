// Licensed under MIT. See LICENSE file. Copyright Carologistics.

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
