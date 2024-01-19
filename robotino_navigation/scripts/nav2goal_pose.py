#!/usr/bin/env python3

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
import rclpy
from ur_interface.srv import JointangleReq

class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('robotino_nav2goal_pose')
        self.navtopose_client = ActionClient(self, NavigateToPose, '/robotinobase1/navigate_to_pose')
        self.TcpPose_srv = self.create_service(JointangleReq, '/pose_req', self.NavPoseSrv_cb)
        
    def NavPoseSrv_cb(self, request, response):
        if request.data == 'True':
            self.get_logger().info(f"[NavPoseSrv_cb]:Tcp_pose request received: {request.tcp_pose}")
            self.pose = Point()
            self.quat = Quaternion()
            self.pose.x = request.tcp_pose[0]
            self.pose.y = request.tcp_pose[1]
            self.pose.z = request.tcp_pose[2]
            self.quat.x = request.tcp_pose[3]
            self.quat.y = request.tcp_pose[4]
            self.quat.z = request.tcp_pose[5]
            self.quat.w = request.tcp_pose[6]
            self.SendAutonavGoal(self.pose, self.quat)
            response.response = True
            response.message = "TcpPoseSrv_cb:UserdDefined JointAngle Request Initiated"
        return response

    def SendAutonavGoal(self, position, orientation):
        while not self.navtopose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Server still not available; waiting...")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation = orientation
        self.get_logger().info("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))
        self.send_goal_future = self.navtopose_client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, send_goal_future):
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, get_result_future):
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached to the goal-pose")
        elif status == 6:
            self.get_logger().info("Retrying navigation to the goal-pose")

def main(args=None):
  rclpy.init(args=args)
  auto_navigate  = AutonomousNavigation()
  try:
    rclpy.spin(auto_navigate)
  except KeyboardInterrupt:
      pass
  auto_navigate.destroy_node()
  rclpy.shutdown()
  rclpy.spin(auto_navigate)

if __name__ == '__main__':
    main()
