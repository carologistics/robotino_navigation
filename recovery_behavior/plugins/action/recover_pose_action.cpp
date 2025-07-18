// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include <memory>
#include <string>

#include "recovery_behavior/plugins/action/recover_pose_action.hpp"

namespace nav2_behavior_tree {

RecoverPoseAction::RecoverPoseAction(const std::string &xml_tag_name,
                                     const std::string &action_name,
                                     const BT::NodeConfiguration &conf)
    : BtActionNode<recovery_msgs::action::RecoverPose>(xml_tag_name,
                                                       action_name, conf) {}

void RecoverPoseAction::initialize() {
  double dist;
  getInput("recovery_distance", dist);
  double speed;
  getInput("recovery_speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.recovery_distance.x = dist;
  goal_.recovery_distance.y = 0.0;
  goal_.recovery_distance.z = 0.0;
  goal_.recovery_speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  // initialized_ = true;
}

void RecoverPoseAction::on_tick() {
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  increment_recovery_count();
}

BT::NodeStatus RecoverPoseAction::on_success() {
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RecoverPoseAction::on_aborted() {
  setOutput("error_code_id", result_.result->error_code);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RecoverPoseAction::on_cancelled() {
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<nav2_behavior_tree::RecoverPoseAction>(
        name, "recoverpose", config);
  };

  factory.registerBuilder<nav2_behavior_tree::RecoverPoseAction>("RecoverPose",
                                                                 builder);
}
