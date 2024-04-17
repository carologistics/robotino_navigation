// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/motor_move.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace motor_move {

void MotorMove::set_matrix_parameter(const std::string &name,
                                     const Eigen::MatrixXd &matrix) {
  std::vector<double> flat_matrix(matrix.data(), matrix.data() + matrix.size());
  this->declare_parameter(name, rclcpp::ParameterValue(flat_matrix));
}

Eigen::MatrixXd MotorMove::get_matrix_parameter(const std::string &name,
                                                int rows, int cols) {
  std::vector<double> flat_matrix;
  this->get_parameter(name, flat_matrix);
  Eigen::MatrixXd matrix = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      flat_matrix.data(), rows, cols);
  return matrix;
}

std::string MotorMove::matrix_to_string(const Eigen::MatrixXd &matrix) {
  std::ostringstream ss;
  ss << matrix;
  return ss.str();
}
MotorMove::MotorMove(const rclcpp::NodeOptions &options)
    : Node("motor_move", options), mimo_() {

  namespace_ = this->get_namespace();
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  if (namespace_ != "/") {
    // Remove leading '/' from namespace if it exists
    if (!namespace_.empty() && namespace_[0] == '/') {
      namespace_ = namespace_.substr(1);
    }
    odom_frame = namespace_ + "/" + odom_frame;
    base_frame = namespace_ + "/" + base_frame;
  }

  base_frame_ = base_frame;
  odom_frame_ = odom_frame;
  RCLCPP_INFO(this->get_logger(), "Namespace: %s", namespace_.c_str());
  RCLCPP_INFO(this->get_logger(), "base_link frame id: %s",
              base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom frame id: %s", odom_frame_.c_str());
  cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, _1, _2),
      std::bind(&MotorMove::handle_cancel, this, _1),
      std::bind(&MotorMove::handle_accepted, this, _1));
  // Initialize matrices
  Eigen::MatrixXd Kp(3, 3);
  Kp << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  Eigen::MatrixXd Ki(3, 3);
  Ki << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;

  Eigen::MatrixXd Kd(3, 3);
  Kd << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;

  std::vector<double> default_parameter = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  this->declare_parameter("Kp", default_parameter);
  this->declare_parameter("Ki", default_parameter);
  this->declare_parameter("Kd", default_parameter);
  Eigen::MatrixXd Kp_retrieved = get_matrix_parameter("Kp", 3, 3);
  Eigen::MatrixXd Ki_retrieved = get_matrix_parameter("Ki", 3, 3);
  Eigen::MatrixXd Kd_retrieved = get_matrix_parameter("Kd", 3, 3);

  RCLCPP_INFO(this->get_logger(), "Kp:\n%s",
              matrix_to_string(Kp_retrieved).c_str());
  RCLCPP_INFO(this->get_logger(), "Ki:\n%s",
              matrix_to_string(Ki_retrieved).c_str());
  RCLCPP_INFO(this->get_logger(), "Kd:\n%s",
              matrix_to_string(Kd_retrieved).c_str());

  mimo_.set_Kp(Kp);
  mimo_.set_Ki(Ki);
  mimo_.set_Kd(Kd);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}
MotorMove::~MotorMove() {}

PoseStamped MotorMove::to_frame(const PoseStamped::SharedPtr point_ptr,
                                std::string frame_id) {
  PoseStamped point_out;
  try {
    tf_buffer_->transform(*point_ptr, point_out, frame_id);
    return point_out;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    throw ex;
  }
}
rclcpp_action::GoalResponse
MotorMove::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const MotorMoveAction::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal requst with order x: %f y: %f",
              goal->motor_goal.pose.position.x,
              goal->motor_goal.pose.position.y);
  try {
    std::lock_guard lock{target_pose_mutex_};
    target_pose_ =
        to_frame(std::make_shared<PoseStamped>(goal->motor_goal), odom_frame_);
    target_pose_.header.stamp = rclcpp::Time(0);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotorMove::handle_cancel(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancle goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotorMove::handle_accepted(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&MotorMove::execute, this, _1), goal_handle}.detach();
}

inline float MotorMove::calculate_distance(const PoseStamped pose) {
  return std::sqrt(pose.pose.position.x * pose.pose.position.x +
                   pose.pose.position.z + pose.pose.position.z);
}

double MotorMove::quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  // Convert geometry_msgs::Quaternion to tf2::Quaternion
  tf2::Quaternion tf2_quat;
  tf2::convert(q, tf2_quat);

  // Create a 3x3 rotation matrix from the quaternion
  tf2::Matrix3x3 m(tf2_quat);

  // Extract roll, pitch, and yaw from the matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

void MotorMove::execute(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  MotorMoveAction::Feedback::SharedPtr feedback =
      std::make_shared<MotorMoveAction::Feedback>();
  float &distance = feedback->distance_to_target;
  (void)goal_handle;
  rclcpp::Rate loop_rate(15);
  rclcpp::Time current_time = this->now();
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Execute goal");
    PoseStamped error =
        to_frame(std::make_shared<PoseStamped>(target_pose_), base_frame_);
    distance = calculate_distance(error);
    goal_handle->publish_feedback(feedback);
    float yaw = quaternionToYaw(error.pose.orientation);
    if (yaw > 0.0872665f || distance > 0.05) {

      RCLCPP_INFO(this->get_logger(), "Distance to yes");
      rclcpp::Time previous_time = current_time;
      current_time = this->now();
      rclcpp::Duration delta_t = current_time - previous_time;
      RCLCPP_INFO(this->get_logger(), "Time delte %f", delta_t.seconds());
      Eigen::MatrixXd error_matrix(3, 1);
      error_matrix << error.pose.position.x, error.pose.position.y, yaw;
      Eigen::MatrixXd output = mimo_.compute(error_matrix, delta_t.seconds());
      RCLCPP_INFO(this->get_logger(), "dsf delte %f", output(0, 0));
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = output(0, 0);
      cmd_vel.linear.y = output(1, 0);
      cmd_vel.angular.z = output(2, 0);
      cmd_vel_->publish(cmd_vel);
    }
    RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
    RCLCPP_INFO(this->get_logger(), "Yaw to target: %f", yaw);
    RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f", error.pose.position.x,
                error.pose.position.y);
    loop_rate.sleep();
  }
}
} // namespace motor_move
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_move::MotorMove>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
