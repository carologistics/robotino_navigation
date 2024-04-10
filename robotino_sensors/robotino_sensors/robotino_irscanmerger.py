#!/usr/bin/env python3
# Author: Saurabh Borse(saurabh.borse@alumni.fh-aachen.de)
#  MIT License
#  Copyright (c) 2023 Saurabh Borse
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
import math

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

num_ir_sensors = 9
robotino_base_radius = 0.225


class Robotino3IrScanMerger(Node):

    def __init__(self):
        super().__init__("robotino_irscanmerger", namespace="")

        # Initialize QoS profile
        self.Laserscan_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, depth=5
        )
        self.declare_parameter("frame_prefix", "robotinobase1")

        # Initialize subscribers for all ir sensors
        for i in range(num_ir_sensors):
            self.subscribers = []
            self.topic = self.get_namespace() + f"/ir{i+1}"
            self.callback_fcn = getattr(self, f"IrScan_cb_{i+1}")
            subscriber = self.create_subscription(Range, self.topic, self.callback_fcn, 10)
            self.subscribers.append(subscriber)
            self.get_logger().info(f"Subscriber {i} created for topic {self.topic}")
            # self.ir_scan_range_init = getattr(self, f"ir{i+1}_scan_range")
            # self.ir_scan_range_init = 0.0
        self.create_subscription(Clock, "/clock", self.Timer_cb, 10)

        # Initialize publisher for merged ir scan
        self.publisher = self.create_publisher(
            LaserScan, self.get_namespace() + "/ir_scan_merged", qos_profile=self.Laserscan_qos_profile
        )
        self.ir1_scan_range = 0.0
        self.ir2_scan_range = 0.0
        self.ir3_scan_range = 0.0
        self.ir4_scan_range = 0.0
        self.ir5_scan_range = 0.0
        self.ir6_scan_range = 0.0
        self.ir7_scan_range = 0.0
        self.ir8_scan_range = 0.0
        self.ir9_scan_range = 0.0
        self.timer = self.create_timer(0.0333, self.On_Timer)
        self.clock_received = False

    # callback function to get simulation time from clock
    def Timer_cb(self, clock_msg):
        self.time_stamp = Time()
        self.time_stamp = clock_msg.clock
        self.clock_received = True

    # callback functions to get range from ir sensors
    def IrScan_cb_1(self, msg):
        self.ir1_scan_range = msg.range

    def IrScan_cb_2(self, data):
        self.ir2_scan_range = data.range

    def IrScan_cb_3(self, data):
        self.ir3_scan_range = data.range

    def IrScan_cb_4(self, data):
        self.ir4_scan_range = data.range

    def IrScan_cb_5(self, data):
        self.ir5_scan_range = data.range

    def IrScan_cb_6(self, data):
        self.ir6_scan_range = data.range

    def IrScan_cb_7(self, data):
        self.ir7_scan_range = data.range

    def IrScan_cb_8(self, data):
        self.ir8_scan_range = data.range

    def IrScan_cb_9(self, data):
        self.ir9_scan_range = data.range

    # callback function to publish merged ir scan
    def On_Timer(self):
        if self.clock_received:
            msg = LaserScan()
            msg.header.frame_id = (
                self.get_parameter("frame_prefix").get_parameter_value().string_value + "/" + "irsensor_merge"
            )
            msg.header.stamp = self.time_stamp
            msg.angle_min = 0.0
            msg.angle_max = 2 * math.pi
            msg.angle_increment = 0.6982
            msg.range_min = 0.02
            msg.range_max = 0.5 + robotino_base_radius
            for i in range(num_ir_sensors):
                self.ir_scan_range = getattr(self, f"ir{i+1}_scan_range")
                msg.ranges.append(self.ir_scan_range + robotino_base_radius)
            self.publisher.publish(msg)


def main():
    rclpy.init()
    scanmerger_node = Robotino3IrScanMerger()
    try:
        rclpy.spin(scanmerger_node)
    except KeyboardInterrupt:
        pass
    scanmerger_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
