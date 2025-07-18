// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#ifndef RECOVERY_BEHAVIOR__PLUGINS__ACTION__RECOVER_POSE_ACTION_HPP_
#define RECOVERY_BEHAVIOR__PLUGINS__ACTION__RECOVER_POSE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "recovery_msgs/action/recover_pose.hpp"

namespace nav2_behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::REcoverPose
 */
class RecoverPoseAction
    : public BtActionNode<recovery_msgs::action::RecoverPose> {
  using Action = recovery_msgs::action::RecoverPose;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::RecoverPoseAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  RecoverPoseAction(const std::string &xml_tag_name,
                    const std::string &action_name,
                    const BT::NodeConfiguration &conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the
   * action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of
   * the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<double>("recovery_distance", 0.50, "Distance to backup"),
         BT::InputPort<double>("recovery_speed", 0.25,
                               "Speed at which to backup"),
         BT::InputPort<double>("time_allowance", 10.0,
                               "Allowed time for reversing"),
         BT::OutputPort<ActionResult::_error_code_type>(
             "error_code_id", "The RecoverPose behavior server error code")});
  }

private:
  // bool initialized_;
};

} // namespace nav2_behavior_tree

#endif // RECOVERY_BEHAVIOR__PLUGINS__ACTION__RECOVER_POSE_ACTION_HPP_
