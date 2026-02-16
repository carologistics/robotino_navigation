#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


class Robotino3ScanRemap(Node):

    def __init__(self):
        super().__init__("robotino_laserscan_republisher")

        # Initialize subscribers for laser scan data
        self.create_subscription(LaserScan, self.get_namespace() + "/SickLaser_Front", self.FrontScan_cb, 10)
        self.create_subscription(Clock, "/clock", self.Timer_cb, 10)
        self.create_subscription(LaserScan, self.get_namespace() + "/SickLaser_Rear", self.RearScan_cb, 10)

        # Initialize publishers for laser scan data
        self.Front_publisher = self.create_publisher(LaserScan, self.get_namespace() + "/SickLaser_Front_Remaped", 10)
        self.Rear_publisher = self.create_publisher(LaserScan, self.get_namespace() + "/SickLaser_Rear_Remaped", 10)

        # Initialize parameters
        self.declare_parameter("frame_prefix", "robotinobase1")
        self.clock_received = False

    # callback function to get simulation time from clock
    def Timer_cb(self, clock_msg):
        self.time_stamp = Time()
        self.time_stamp = clock_msg.clock
        self.clock_received = True

    # callback functions to get range from laser scan data and republish it
    def FrontScan_cb(self, msg):
        if self.clock_received:
            scan_f = LaserScan()
            scan_f.header.stamp = self.time_stamp
            scan_f.header.frame_id = (
                self.get_parameter("frame_prefix").get_parameter_value().string_value + "/" + msg.header.frame_id
            )
            scan_f.angle_min = msg.angle_max
            scan_f.angle_max = msg.angle_min
            scan_f.angle_increment = -(msg.angle_increment)
            scan_f.time_increment = msg.time_increment
            scan_f.scan_time = msg.scan_time
            scan_f.range_min = msg.range_min
            scan_f.range_max = msg.range_max
            scan_f.ranges = [float()] * 180
            var_len = len(msg.ranges)
            # self.get_logger().info(f'range length: {var_len}')
            for i in range(var_len):
                scan_f.ranges[179 - i] = msg.ranges[i]
            self.Front_publisher.publish(scan_f)

    # callback functions to get range from laser scan data and republish it
    def RearScan_cb(self, msg_r):
        if self.clock_received:
            scan_r = LaserScan()
            scan_r.header.stamp = self.time_stamp
            scan_r.header.frame_id = (
                self.get_parameter("frame_prefix").get_parameter_value().string_value + "/" + msg_r.header.frame_id
            )
            scan_r.angle_min = msg_r.angle_max
            scan_r.angle_max = msg_r.angle_min
            scan_r.angle_increment = -(msg_r.angle_increment)
            scan_r.time_increment = msg_r.time_increment
            scan_r.scan_time = msg_r.scan_time
            scan_r.range_min = msg_r.range_min
            scan_r.range_max = msg_r.range_max
            scan_r.ranges = [float()] * 180
            var_len = len(scan_r.ranges)
            for i in range(var_len):
                scan_r.ranges[179 - i] = msg_r.ranges[i]
            self.Rear_publisher.publish(scan_r)


def main():
    rclpy.init()
    scan_republisher = Robotino3ScanRemap()
    try:
        rclpy.spin(scan_republisher)
    except KeyboardInterrupt:
        pass
    scan_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
