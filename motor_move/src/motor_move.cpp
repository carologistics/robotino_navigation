// Licensed under MIT. See LICENSE file. Copyright Carologistics.

#include "motor_move/motor_move.hpp" // Include the header file for the MotorMove class.
#include <eigen3/Eigen/src/Core/Matrix.h> // Include Eigen for matrix operations.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // JW
#include "tf2/utils.h"

namespace motor_move {

// Set a parameter as a flattened matrix
void MotorMove::set_matrix_parameter(const std::string &name,
                                     const Eigen::MatrixXd &matrix) {
  std::vector<double> flat_matrix(matrix.data(), matrix.data() + matrix.size()); // Flatten Eigen matrix to a vector: flat_matrix(begin,end)
  this->declare_parameter(name, rclcpp::ParameterValue(flat_matrix)); // rclcpp := "ros client library" assigns parameter "name" the parameter value of flat_matrix
}

//Retrieve a matrix parameter and reshape it into a proper Eigen matrix
Eigen::MatrixXd MotorMove::get_matrix_parameter(const std::string &name,
                                                int rows, int cols) {
  std::vector<double> flat_matrix; // Vector to store the parameter.
  this->get_parameter(name, flat_matrix); // Retrieve the parameter as a vector.
  Eigen::MatrixXd matrix = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      flat_matrix.data(), rows, cols); // Map the vector back to a matrix.
  return matrix;
}

// Convert an Eigen matrix to a string for logging or debugging
std::string MotorMove::matrix_to_string(const Eigen::MatrixXd &matrix) {
  std::ostringstream ss; // String stream for conversion.
  ss << matrix; // Insert matrix data into the stream.
  return ss.str(); // Return the string representation.
}

// Constructor for MotorMove class
MotorMove::MotorMove(const rclcpp::NodeOptions &options)
    : Node("motor_move", options), mimo_() { // Initialize the node with the name "motor_move".

  namespace_ = this->get_namespace(); // Retrieve the namespace of the node.
  std::string odom_frame = "odom"; // Default odometry frame.
  std::string base_frame = "base_link"; // Default base frame.

  if (namespace_ != "/") { // Check if namespace is not root.
    if (!namespace_.empty() && namespace_[0] == '/') { // Remove leading '/' if it exists.
      namespace_ = namespace_.substr(1);
    }
    odom_frame = namespace_ + "/" + odom_frame; // Prefix namespace to odometry frame.
    base_frame = namespace_ + "/" + base_frame; // Prefix namespace to base frame.

  }

  base_frame_ = base_frame; // Assign base frame to class variable.
  odom_frame_ = odom_frame; // Assign odometry frame to class variable.
  RCLCPP_INFO(this->get_logger(), "Namespace: %s", namespace_.c_str()); // Log the namespace.
  RCLCPP_INFO(this->get_logger(), "base_link frame id: %s",
              base_frame_.c_str()); // Log base frame.
  RCLCPP_INFO(this->get_logger(), "odom frame id: %s", odom_frame_.c_str()); // Log odometry frame.

  // Create a publisher for cmd_vel topic with a queue size of 10.
  cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  using namespace std::placeholders; // Simplify placeholder usage for callbacks.

  // Create an action server for motor_move_action. Here the goal inputs get read
  action_server_ = rclcpp_action::create_server<MotorMoveAction>(
      this, "motor_move_action",
      std::bind(&MotorMove::handle_goal, this, _1, _2), // Callback for goal reception.
      std::bind(&MotorMove::handle_cancel, this, _1), // Callback for cancel requests.
      std::bind(&MotorMove::handle_accepted, this, _1)); // Callback for accepted goals.

  // Initialize control matrices (Kp, Ki, Kd)
  Eigen::MatrixXd Kp(3, 3); // Proportional gain matrix.
  Kp << 1.8, 0, 0, 0, 1.8, 0, 0, 0, 1.8;

  Eigen::MatrixXd Ki(3, 3); // Integral gain matrix.
  Ki << 0.4, 0, 0, 0, 0.4, 0, 0, 0, 0.4;

  Eigen::MatrixXd Kd(3, 3); // Derivative gain matrix.
  Kd << 0.00, 0, 0, 0, 0.00, 0, 0, 0, 0.00;

  // Declare parameters for control gains as flattened vectors.
  std::vector<double> default_parameter = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  this->declare_parameter("Kp", default_parameter);
  this->declare_parameter("Ki", default_parameter);
  this->declare_parameter("Kd", default_parameter);

  // Retrieve control gains from parameters.
  Eigen::MatrixXd Kp_retrieved = get_matrix_parameter("Kp", 3, 3);
  Eigen::MatrixXd Ki_retrieved = get_matrix_parameter("Ki", 3, 3);
  Eigen::MatrixXd Kd_retrieved = get_matrix_parameter("Kd", 3, 3);

  // Log the retrieved control matrices.
  RCLCPP_INFO(this->get_logger(), "Kp:\n%s",
              matrix_to_string(Kp_retrieved).c_str());
  RCLCPP_INFO(this->get_logger(), "Ki:\n%s",
              matrix_to_string(Ki_retrieved).c_str());
  RCLCPP_INFO(this->get_logger(), "Kd:\n%s",
              matrix_to_string(Kd_retrieved).c_str());

  // Set control gains in the MIMO controller.
  mimo_.set_Kp(Kp);
  mimo_.set_Ki(Ki);
  mimo_.set_Kd(Kd);

  // Initialize TF2 buffer and listener for transformations.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

// Destructor for MotorMove class
MotorMove::~MotorMove() {}

// Transform a pose to a specified frame
PoseStamped MotorMove::to_frame(const PoseStamped::SharedPtr point_ptr,
                                std::string frame_id) {
  PoseStamped point_out; // Output pose.
  try {
    tf_buffer_->transform(*point_ptr, point_out, frame_id); // Perform the transformation.
    return point_out;
  } catch (const tf2::TransformException &ex) { // Handle transformation errors.
    RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
    throw ex; // Re-throw exception.
  }
}

// Handle incoming goals
rclcpp_action::GoalResponse
MotorMove::handle_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const MotorMoveAction::Goal> goal) {
  (void)uuid; // Silence unused variable warning.
  RCLCPP_INFO(this->get_logger(), "Received goal requst with order x: %f y: %f",
              goal->motor_goal.pose.position.x,
              goal->motor_goal.pose.position.y); // Log goal details.
  try {
    std::lock_guard lock{target_pose_mutex_}; // Lock the mutex for thread safety.
    target_pose_ =
        to_frame(std::make_shared<PoseStamped>(goal->motor_goal), odom_frame_); // Transform target pose to odom frame.
    target_pose_.header.stamp = rclcpp::Time(0); // Set timestamp to zero.
  } catch (tf2::TransformException &ex) { // Handle transformation errors.
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return rclcpp_action::GoalResponse::REJECT; // Reject the goal.
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept the goal.
}

// Handle goal cancellation requests
rclcpp_action::CancelResponse MotorMove::handle_cancel(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  (void)goal_handle; // Silence unused variable warning.
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT; // Accept cancellation.
}

// Handle accepted goals and start execution
void MotorMove::handle_accepted(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  using namespace std::placeholders; // Simplify placeholder usage.
  std::thread{std::bind(&MotorMove::execute, this, _1), goal_handle}.detach(); // Start goal execution in a new thread.
}

// Calculate distance to a target pose
inline float MotorMove::calculate_distance(const PoseStamped pose) {
  return std::sqrt(pose.pose.position.x * pose.pose.position.x +
                   pose.pose.position.y * pose.pose.position.y); // error? Should multiply z by itself.
}

// // Convert a quaternion to yaw angle
// double MotorMove::quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
//   tf2::Quaternion tf2_quat; // TF2 quaternion.
//   tf2::convert(q, tf2_quat); // Convert geometry_msgs quaternion to TF2 quaternion.
//   tf2::Matrix3x3 m(tf2_quat); // Create a rotation matrix from the quaternion.
//   double roll, pitch, yaw; // Variables to store roll, pitch, and yaw.
//   m.getRPY(roll, pitch, yaw); // Extract roll, pitch, and yaw.
//   return yaw; // Return yaw angle.
// }

// Execute the goal
void MotorMove::execute(
    const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
  MotorMoveAction::Feedback::SharedPtr feedback =
      std::make_shared<MotorMoveAction::Feedback>(); // Create feedback object.
  float &distance = feedback->distance_to_target; // Reference to distance feedback.
  (void)goal_handle; // Silence unused variable warning.
  rclcpp::Rate loop_rate(15); // Set loop rate to 15 Hz.
  rclcpp::Time current_time = this->now(); // Get current time.
  while (rclcpp::ok()) { // Main loop.
    if (goal_handle->is_canceling()) { // Check if goal is canceling.
      goal_handle->publish_feedback(feedback); // Publish feedback.
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return; // Exit the loop.
    }
    RCLCPP_INFO(this->get_logger(), "Execute goal");
    PoseStamped error =
        to_frame(std::make_shared<PoseStamped>(target_pose_), base_frame_); // Transform target pose to base frame.
    distance = calculate_distance(error); // Calculate distance to target.
    goal_handle->publish_feedback(feedback); // Publish feedback.
    float yaw = tf2::getYaw(error.pose.orientation); // Calculate yaw to target.
    if (yaw > 0.0872665f || distance > 0.05) { // Check thresholds for yaw and distance.

      RCLCPP_INFO(this->get_logger(), "Distance to yes");
      rclcpp::Time previous_time = current_time; // Store previous time.
      current_time = this->now(); // Update current time.
      rclcpp::Duration delta_t = current_time - previous_time; // Calculate time difference.
      RCLCPP_INFO(this->get_logger(), "Time delta %f", delta_t.seconds());
      Eigen::MatrixXd error_matrix(3, 1); // Create error matrix.
      error_matrix << error.pose.position.x, error.pose.position.y, yaw; // Fill error matrix.
      Eigen::MatrixXd output = mimo_.compute(error_matrix, delta_t.seconds()); // Compute control output.
      RCLCPP_INFO(this->get_logger(), "Output delta %f", output(0, 0));
      geometry_msgs::msg::Twist cmd_vel; // Create Twist message for velocity commands.
      cmd_vel.linear.x = output(0, 0); // Set linear x velocity.
      cmd_vel.linear.y = output(1, 0); // Set linear y velocity.
      cmd_vel.angular.z = output(2, 0); // Set angular velocity.
      cmd_vel_->publish(cmd_vel); // Publish velocity command.
    }
    RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance); // Log distance.
    RCLCPP_INFO(this->get_logger(), "Yaw to target: %f", yaw); // Log yaw.
    RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f", error.pose.position.x,
                error.pose.position.y); // Log position deltas.
    loop_rate.sleep(); // Sleep to maintain loop rate.
  }
}
} // namespace motor_move

// Main function
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // Initialize ROS2.
  auto node = std::make_shared<motor_move::MotorMove>(); // Create MotorMove node.
  rclcpp::spin(node); // Spin the node to process callbacks.
  rclcpp::shutdown(); // Shutdown ROS2.
}
