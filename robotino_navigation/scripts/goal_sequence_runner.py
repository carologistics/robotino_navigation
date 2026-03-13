#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler


class GoalSequenceRunner(Node):
    def __init__(self):
        super().__init__("goal_sequence_runner")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            "/robotinobase1/navigate_to_pose",
            result_service_qos_profile=qos,
            goal_service_qos_profile=qos,
            cancel_service_qos_profile=qos,
            feedback_sub_qos_profile=qos,
            status_sub_qos_profile=qos,
        )

        default_seq_path = Path(get_package_share_directory("robotino_navigation")) / "config" / "goal_sequences.yaml"
        self.declare_parameter("sequence_file", str(default_seq_path))
        sequence_path = Path(self.get_parameter("sequence_file").get_parameter_value().string_value)

        self.declare_parameter("use_left_field", False)
        use_left_field = self.get_parameter("use_left_field").get_parameter_value().bool_value
        sequence_key = "sequence1" if use_left_field else "sequence2"
        self.get_logger().info(f"use_left_field={use_left_field} → running '{sequence_key}'")

        self._poses = self._load_pose_table(sequence_path)
        self._sequence = self._load_sequence(sequence_path, sequence_key)

    def _load_pose_table(self, sequence_path: Path):
        poses_file = self._load_yaml(sequence_path).get("poses_file")
        if not poses_file:
            raise RuntimeError("poses_file missing in goal sequence config")
        poses_path = sequence_path.parent / poses_file if not Path(poses_file).is_absolute() else Path(poses_file)
        pose_yaml = self._load_yaml(poses_path)
        try:
            return pose_yaml["/**"]["ros__parameters"]["static_transforms"]
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"Invalid pose table format in {poses_path}") from exc

    def _load_sequence(self, sequence_path: Path, sequence_key: str = "sequence1"):
        data = self._load_yaml(sequence_path)
        sequence = data.get(sequence_key, [])
        if not sequence:
            raise RuntimeError(f"No sequence steps defined for key '{sequence_key}'")
        return sequence

    def _load_yaml(self, path: Path):
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    def run(self):
        sequence_aborted = False
        for step in self._sequence:
            pose_name = step["pose"]
            timeout = float(step.get("timeout", 120.0))
            dwell = float(step.get("dwell", 0.0))
            pose_cfg = self._poses.get(pose_name)
            if pose_cfg is None:
                self.get_logger().error(f"Pose '{pose_name}' not found; aborting sequence")
                sequence_aborted = True
                break
            goal = self._build_goal(pose_cfg)

            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("NavigateToPose action server not available; aborting")
                sequence_aborted = True
                break

            self.get_logger().info(f"Sending goal {pose_name} (timeout {timeout}s, dwell {dwell}s)")
            send_future = self._action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f"Goal {pose_name} rejected")
                sequence_aborted = True
                break

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
            if not result_future.done():
                self.get_logger().warn(f"Goal {pose_name} timed out; cancelling")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                sequence_aborted = True
                break
            else:
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f"Goal {pose_name} succeeded")
                else:
                    self.get_logger().warn(f"Goal {pose_name} finished with status {status}")
                    sequence_aborted = True
                    break

            if dwell > 0:
                time.sleep(dwell)

        if sequence_aborted:
            self.get_logger().warn("Sequence aborted")
        else:
            self.get_logger().info("Sequence complete")

    def _build_goal(self, pose_cfg):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = pose_cfg.get("parent_frame_id", "map")
        goal.pose.pose.position.x = pose_cfg["translation"][0]
        goal.pose.pose.position.y = pose_cfg["translation"][1]
        goal.pose.pose.position.z = pose_cfg["translation"][2]
        yaw, pitch, roll = pose_cfg.get("rotation", [0.0, 0.0, 0.0])
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal


def main(args=None):
    rclpy.init(args=args)
    node = GoalSequenceRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
