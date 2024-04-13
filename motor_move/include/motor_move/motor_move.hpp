#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "motor_move_msgs/action/motor_move.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace motor_move {
using MotorMoveAction = motor_move_msgs::action::MotorMove;
using GoalHandleMotorMove = rclcpp_action::ServerGoalHandle<MotorMoveAction>;
class MotorMove : public rclcpp::Node {
public:
    explicit MotorMove(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MotorMove();
private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp_action::Server<MotorMoveAction>::SharedPtr action_server_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    rclcpp_action::GoalResponse handle_goal(
       const rclcpp_action::GoalUUID & uuid,
       std::shared_ptr<const MotorMoveAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotorMove> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
};
} // namespace motor_move
