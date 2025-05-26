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
  Ki << 0.38, 0, 0, 0, 0.38, 0, 0, 0, 0.38;

  Eigen::MatrixXd Kd(3, 3); // Derivative gain matrix.
  Kd << 0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2;

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

inline float MotorMove::calculate_distance(const geometry_msgs::msg::PoseStamped &target_pose) {
  // 1) Roboter-Pose im odom-Frame abfragen (base_link → odom)
  geometry_msgs::msg::TransformStamped base_tf;
  try {
    base_tf = tf_buffer_->lookupTransform(
      odom_frame_,         // Ziel-Frame: odom
      base_frame_,         // Quelle-Frame: base_link
      tf2::TimePointZero   // neuester verfügbarer Transform
    );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "lookupTransform odom->base_link failed: %s", ex.what());
    return std::numeric_limits<float>::infinity();
  }

  // 2) In tf2::Transform umwandeln
  tf2::Transform tf_base;
  tf2::fromMsg(base_tf.transform, tf_base);

  // 3) Ziel-Pose im odom-Frame als tf2::Transform aufbauen
  tf2::Transform tf_target;
  tf_target.setOrigin(tf2::Vector3(
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z
  ));
  tf2::Quaternion q(
    target_pose.pose.orientation.x,
    target_pose.pose.orientation.y,
    target_pose.pose.orientation.z,
    target_pose.pose.orientation.w
  );
  tf_target.setRotation(q);

  // 4) Relative Transform (base_link⁻¹ * target) berechnen
  tf2::Transform tf_diff = tf_base.inverseTimes(tf_target);

  // 5) Aus der Differenz den euklidischen Abstand extrahieren
  auto d = tf_diff.getOrigin();
  return std::sqrt(d.x()*d.x() + d.y()*d.y() + d.z()*d.z());
}

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
    
    // Feedback-Distanz berechnen – beide Koordinaten im odom-Frame:
    distance = calculate_distance(target_pose_);
    goal_handle->publish_feedback(feedback);

    // Yaw weiterhin im base_frame berechnen, um Winkelkorrektur zu steuern:
    auto error = to_frame(std::make_shared<PoseStamped>(target_pose_), base_frame_);
    float yaw = tf2::getYaw(error.pose.orientation);

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
    }else {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_->publish(stop_cmd);
    
    goal_handle->succeed(std::make_shared<MotorMoveAction::Result>());
    RCLCPP_INFO(this->get_logger(), "Ziel erreicht.");
    return;
    RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance); // Log distance.
    RCLCPP_INFO(this->get_logger(), "Yaw to target: %f", yaw); // Log yaw.
    RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f", error.pose.position.x,
                error.pose.position.y); // Log position deltas.
    loop_rate.sleep(); // Sleep to maintain loop rate.
    }
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
