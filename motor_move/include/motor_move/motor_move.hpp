#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <motor_move_msgs/action/motor_move.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace motor_move {
using MotorMoveAction = motor_move_msgs::action::MotorMove;
using GoalHandleMotorMove = rclcpp_action::ServerGoalHandle<MotorMoveAction>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Pose = geometry_msgs::msg::Pose;
class MotorMove : public rclcpp::Node {
public:
    explicit MotorMove(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MotorMove();
private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp_action::Server<MotorMoveAction>::SharedPtr action_server_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex target_pose_mutex_;
    PoseStamped target_pose_;

    PoseStamped to_frame(const geometry_msgs::msg::PoseStamped::SharedPtr point_ptr, std::string frame_id);

    rclcpp_action::GoalResponse handle_goal(
       const rclcpp_action::GoalUUID & uuid,
       std::shared_ptr<const MotorMoveAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotorMove> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMotorMove> goal_handle);
    inline float calculate_distance(const PoseStamped pose);
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);
};
} // namespace motor_move
