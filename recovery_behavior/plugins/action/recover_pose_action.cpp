// Licensed under MIT. See LICENSE file. Copyright Carologistics.

// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "recovery_behavior/plugins/action/recover_pose_action.hpp"

namespace nav2_behavior_tree {

RecoverPoseAction::RecoverPoseAction(const std::string &xml_tag_name,
                                     const std::string &action_name,
                                     const BT::NodeConfiguration &conf)
    : BtActionNode<recovery_msgs::action::RecoverPose>(xml_tag_name,
                                                       action_name, conf),
      initialized_(false) {}

void RecoverPoseAction::initialize() {
  double dist;
  getInput("backup_dist", dist);
  double speed;
  getInput("backup_speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  double robot_footprint;
  getInput("robot_footprint", robot_footprint);

  // Populate the input message
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  goal_.robot_footprint = robot_footprint;
  initialized_ = true;
}

void RecoverPoseAction::on_tick() {
  if (!initialized_) {
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
