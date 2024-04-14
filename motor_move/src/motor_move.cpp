#include "motor_move/motor_move.hpp"

namespace motor_move {
    MotorMove::MotorMove(const rclcpp::NodeOptions & options) : Node("motor_move", options) {
       cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
       using namespace std::placeholders;
       action_server_ = rclcpp_action::create_server<MotorMoveAction>(
           this,
            "motor_move_action",
           std::bind(&MotorMove::handle_goal, this, _1, _2),
           std::bind(&MotorMove::handle_cancel, this, _1),
           std::bind(&MotorMove::handle_accepted, this, _1)
       );

       tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
       tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }
    MotorMove::~MotorMove() {
    }

    PoseStamped
    MotorMove::to_frame(const PoseStamped::SharedPtr point_ptr, std::string frame_id) {
        PoseStamped point_out;
        try {
            tf_buffer_->transform(*point_ptr, point_out, frame_id);
            return point_out;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
            throw ex;
        }
    }
    rclcpp_action::GoalResponse MotorMove::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MotorMoveAction::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal requst with order x: %f y: %f",
                    goal->motor_goal.pose.position.x, goal->motor_goal.pose.position.y);
        try {
            std::lock_guard lock{target_pose_mutex_};
            target_pose_ = to_frame(std::make_shared<PoseStamped>(goal->motor_goal), "odom");
        }
        catch (tf2::TransformException & ex) {
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

    void MotorMove::handle_accepted(const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
        using namespace std::placeholders;
        std::thread{std::bind(&MotorMove::execute, this, _1), goal_handle}.detach();
    }

    void MotorMove::execute(const std::shared_ptr<GoalHandleMotorMove> goal_handle) {
        (void)goal_handle;
        rclcpp::Rate loop_rate(15);
        while(rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Execute goal");
            PoseStamped error = to_frame(std::make_shared<PoseStamped>(target_pose_), "base_link");
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Delta x: %f y: %f",
                        error.pose.position.x, error.pose.position.y);
        }
    }
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_move::MotorMove>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
