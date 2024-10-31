#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
# script: three robots plan extraction
import csv

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


class PathSubscriber(Node):
    def __init__(self):
        super().__init__("path_subscriber")

        # Initialize subscribers for the robot paths
        self.subscription1 = self.create_subscription(Path, "/robotinobase1/plan", self.path_callback1, 10)
        self.subscription2 = self.create_subscription(Path, "/robotinobase2/plan", self.path_callback2, 10)
        self.subscription3 = self.create_subscription(Path, "/robotinobase3/plan", self.path_callback3, 10)

        # Paths to store the coordinates
        self.all_paths1 = []
        self.all_paths2 = []
        self.all_paths3 = []

        # CSV file paths for saving paths
        # CSV file paths for saving paths using relative paths
        self.csv_file_1 = "ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase1_plans.csv"
        self.csv_file_2 = "ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase2_plans.csv"
        self.csv_file_3 = "ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase3_plans.csv"

        # Initialize CSV files
        self.initialize_csv(self.csv_file_1)
        self.initialize_csv(self.csv_file_2)
        self.initialize_csv(self.csv_file_3)

    def initialize_csv(self, file_path):
        """Initialize a CSV file with headers."""
        with open(file_path, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y"])  # Write the header row

    def path_callback1(self, msg):
        """Callback to process the path of robotinobase1."""
        path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
        self.all_paths1.extend(path)  # Accumulate all paths
        self.save_to_csv(self.csv_file_1, path)
        self.get_logger().info("Received path for robotinobase1")

    def path_callback2(self, msg):
        """Callback to process the path of robotinobase2."""
        path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
        self.all_paths2.extend(path)  # Accumulate all paths
        self.save_to_csv(self.csv_file_2, path)
        self.get_logger().info("Received path for robotinobase2")

    def path_callback3(self, msg):
        """Callback to process the path of robotinobase3."""
        path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
        self.all_paths3.extend(path)  # Accumulate all paths
        self.save_to_csv(self.csv_file_3, path)
        self.get_logger().info("Received path for robotinobase3")

    def save_to_csv(self, file_path, path):
        """Save path data to CSV file."""
        with open(file_path, "a", newline="") as file:
            writer = csv.writer(file)
            for x, y in path:
                writer.writerow([x, y])

    def plot_paths(self):
        """Plot all accumulated paths of all three robots."""
        if self.all_paths1 and self.all_paths2 and self.all_paths3:
            x1, y1 = zip(*self.all_paths1)
            x2, y2 = zip(*self.all_paths2)
            x3, y3 = zip(*self.all_paths3)

            plt.figure(figsize=(10, 6))

            # Plot paths with specified colors and styles
            plt.plot(x1, y1, "r-", marker="o", markersize=0.5, linewidth=0.15, label="Robotinobase1 (Red)")
            plt.plot(x2, y2, "g-", marker="o", markersize=0.5, linewidth=0.15, label="Robotinobase2 (Green)")
            plt.plot(x3, y3, "b-", marker="o", markersize=0.5, linewidth=0.15, label="Robotinobase3 (Blue)")

            # Label the start points just below the point for better visibility
            plt.text(x1[0], y1[0] - 0.1, "Start", color="red", fontsize=8, ha="center", va="top")
            plt.text(x2[0], y2[0] - 0.1, "Start", color="green", fontsize=8, ha="center", va="top")
            plt.text(x3[0], y3[0] - 0.1, "Start", color="blue", fontsize=8, ha="center", va="top")

            # Set axis limits
            plt.xlim(-6, 6)
            plt.ylim(0, 6)

            # Add grid, legend, and labels with increased font sizes
            plt.grid(True)
            plt.legend(loc="lower left", fontsize=12)  # Increased font size for the legend
            plt.xlabel("X Coordinate", fontsize=12)  # Increased font size for the label
            plt.ylabel("Y Coordinate", fontsize=12)  # Increased font size for the label
            plt.title("Paths of Robotino Bases", fontsize=14)  # Increased font size for the title

            # Show the plot
            plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PathSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_paths()  # Plot the paths before shutting down
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# # script: two robots plan extraction

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# import csv

# class PathSubscriber(Node):
#     def __init__(self):
#         super().__init__('path_subscriber')

#         # Initialize subscribers for the robot paths
#         self.subscription1 = self.create_subscription(Path, '/robotinobase1/plan', self.path_callback1, 10)
#         self.subscription2 = self.create_subscription(Path, '/robotinobase2/plan', self.path_callback2, 10)

#         # Paths to store the coordinates
#         self.all_paths1 = []
#         self.all_paths2 = []

#         # Counters to track the number of paths received
#         self.path_count1 = 0
#         self.path_count2 = 0

#         # CSV file paths for saving paths
#         self.csv_file_1 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase1_plans.csv'
#         self.csv_file_2 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase2_plans.csv'

#         # Initialize CSV files
#         self.initialize_csv(self.csv_file_1)
#         self.initialize_csv(self.csv_file_2)

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def path_callback1(self, msg):
#         """Callback to process the path of robotinobase1."""
#         self.path_count1 += 1  # Increment the path counter for robotinobase1

#         if self.path_count1 == 3:  # Record only the 2nd path
#             path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#             self.all_paths1.extend(path)  # Accumulate the 2nd path
#             self.save_to_csv(self.csv_file_1, path)
#             self.get_logger().info('Recorded the 2nd path for robotinobase1')

#     def path_callback2(self, msg):
#         """Callback to process the path of robotinobase2."""
#         self.path_count2 += 1  # Increment the path counter for robotinobase2

#         if self.path_count2 == 4:  # Record only the 4th path
#             path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#             self.all_paths2.extend(path)  # Accumulate the 4th path
#             self.save_to_csv(self.csv_file_2, path)
#             self.get_logger().info('Recorded the 4th path for robotinobase2')

#     def save_to_csv(self, file_path, path):
#         """Save path data to CSV file."""
#         with open(file_path, 'a', newline='') as file:
#             writer = csv.writer(file)
#             for x, y in path:
#                 writer.writerow([x, y])

#     def plot_paths(self):
#         """Plot the recorded paths of both robots."""
#         if self.all_paths1 and self.all_paths2:
#             x1, y1 = zip(*self.all_paths1)
#             x2, y2 = zip(*self.all_paths2)

#             plt.figure(figsize=(10, 6))

#             # Plot paths with specified colors and styles
#             plt.plot(x1, y1, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')
#             plt.plot(x2, y2, 'g-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase2 (Green)')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)
#             plt.xlabel('X Coordinate', fontsize=12)
#             plt.ylabel('Y Coordinate', fontsize=12)
#             plt.title('Paths of Robotino Bases', fontsize=14)

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_paths()  # Plot the paths before shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# #script: three robots plan extraction

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# import csv

# class PathSubscriber(Node):
#     def __init__(self):
#         super().__init__('path_subscriber')

#         # Initialize subscribers for the robot paths
#         self.subscription1 = self.create_subscription(Path, '/robotinobase1/plan', self.path_callback1, 10)
#         self.subscription2 = self.create_subscription(Path, '/robotinobase2/plan', self.path_callback2, 10)
#         self.subscription3 = self.create_subscription(Path, '/robotinobase3/plan', self.path_callback3, 10)

#         # Paths to store the coordinates
#         self.all_paths1 = []
#         self.all_paths2 = []
#         self.all_paths3 = []

#         # CSV file paths for saving paths
#         self.csv_file_1 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase1_plans.csv'
#         self.csv_file_2 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase2_plans.csv'
#         self.csv_file_3 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase3_plans.csv'

#         # Initialize CSV files
#         self.initialize_csv(self.csv_file_1)
#         self.initialize_csv(self.csv_file_2)
#         self.initialize_csv(self.csv_file_3)

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def path_callback1(self, msg):
#         """Callback to process the path of robotinobase1."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths1.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file_1, path)
#         self.get_logger().info('Received path for robotinobase1')

#     def path_callback2(self, msg):
#         """Callback to process the path of robotinobase2."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths2.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file_2, path)
#         self.get_logger().info('Received path for robotinobase2')

#     def path_callback3(self, msg):
#         """Callback to process the path of robotinobase3."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths3.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file_3, path)
#         self.get_logger().info('Received path for robotinobase3')

#     def save_to_csv(self, file_path, path):
#         """Save path data to CSV file."""
#         with open(file_path, 'a', newline='') as file:
#             writer = csv.writer(file)
#             for x, y in path:
#                 writer.writerow([x, y])

#     def plot_paths(self):
#         """Plot all accumulated paths of all three robots."""
#         if self.all_paths1 and self.all_paths2 and self.all_paths3:
#             x1, y1 = zip(*self.all_paths1)
#             x2, y2 = zip(*self.all_paths2)
#             x3, y3 = zip(*self.all_paths3)

#             plt.figure(figsize=(10, 6))

#             # Plot paths with specified colors and styles
#             plt.plot(x1, y1, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')
#             plt.plot(x2, y2, 'g-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase2 (Green)')
#             plt.plot(x3, y3, 'b-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase3 (Blue)')

#             # Label the start points just below the point for better visibility
#             plt.text(x1[0], y1[0] - 0.1, 'Start', color='red', fontsize=8, ha='center', va='top')
#             plt.text(x2[0], y2[0] - 0.1, 'Start', color='green', fontsize=8, ha='center', va='top')
#             plt.text(x3[0], y3[0] - 0.1, 'Start', color='blue', fontsize=8, ha='center', va='top')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)  # Increased font size for the legend
#             plt.xlabel('X Coordinate', fontsize=12)  # Increased font size for the label
#             plt.ylabel('Y Coordinate', fontsize=12)  # Increased font size for the label
#             plt.title('Paths of Robotino Bases', fontsize=14)  # Increased font size for the title

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_paths()  # Plot the paths before shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# # script: single robot plan extraction

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# import matplotlib.pyplot as plt
# import csv

# class PathSubscriber(Node):
#     def __init__(self):
#         super().__init__('path_subscriber')

#         # Initialize subscriber for the robot path
#         self.subscription = self.create_subscription(Path, '/robotinobase1/plan', self.path_callback, 10)

#         # Path to store the coordinates
#         self.all_paths = []

#         # CSV file path for saving path
#         self.csv_file = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase1_plans.csv'

#         # Initialize CSV file
#         self.initialize_csv(self.csv_file)

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def path_callback(self, msg):
#         """Callback to process the path of robotinobase1."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file, path)
#         self.get_logger().info('Received path for robotinobase1')

#     def save_to_csv(self, file_path, path):
#         """Save path data to CSV file."""
#         with open(file_path, 'a', newline='') as file:
#             writer = csv.writer(file)
#             for x, y in path:
#                 writer.writerow([x, y])

#     def plot_paths(self):
#         """Plot all accumulated paths of the robot."""
#         if self.all_paths:
#             x, y = zip(*self.all_paths)

#             plt.figure(figsize=(10, 6))

#             # Plot path with specified color and style
#             plt.plot(x, y, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')

#             # Label the start point just below the point for better visibility
#             plt.text(x[0], y[0] - 0.1, 'Start', color='red', fontsize=8, ha='center', va='top')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)  # Increased font size for the legend
#             plt.xlabel('X Coordinate', fontsize=12)  # Increased font size for the label
#             plt.ylabel('Y Coordinate', fontsize=12)  # Increased font size for the label
#             plt.title('Path of Robotino Base', fontsize=14)  # Increased font size for the title

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_paths()  # Plot the path before shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# # script: single robot plan extraction

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# import matplotlib.pyplot as plt
# import csv

# class PathSubscriber(Node):
#     def __init__(self):
#         super().__init__('path_subscriber')

#         # Initialize subscriber for the robot path
#         self.subscription = self.create_subscription(Path, '/robotinobase1/plan', self.path_callback, 10)

#         # Path to store the coordinates
#         self.all_paths = []

#         # CSV file path for saving path
#         self.csv_file = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase1_plans.csv'

#         # Initialize CSV file
#         self.initialize_csv(self.csv_file)

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def path_callback(self, msg):
#         """Callback to process the path of robotinobase1."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file, path)
#         self.get_logger().info('Received path for robotinobase1')

#     def save_to_csv(self, file_path, path):
#         """Save path data to CSV file."""
#         with open(file_path, 'a', newline='') as file:
#             writer = csv.writer(file)
#             for x, y in path:
#                 writer.writerow([x, y])

#     def plot_paths(self):
#         """Plot all accumulated paths of the robot."""
#         if self.all_paths:
#             x, y = zip(*self.all_paths)

#             plt.figure(figsize=(10, 6))

#             # Plot path with specified color and style
#             plt.plot(x, y, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')

#             # Label the start point just below the point for better visibility
#             plt.text(x[0], y[0] - 0.1, 'Start', color='red', fontsize=8, ha='center', va='top')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)  # Increased font size for the legend
#             plt.xlabel('X Coordinate', fontsize=12)  # Increased font size for the label
#             plt.ylabel('Y Coordinate', fontsize=12)  # Increased font size for the label
#             plt.title('Path of Robotino Base', fontsize=14)  # Increased font size for the title

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_paths()  # Plot the path before shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# #script: two robots plan extraction

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# import csv

# class PathSubscriber(Node):
#     def __init__(self):
#         super().__init__('path_subscriber')

#         # Initialize subscribers for the robot paths
#         self.subscription1 = self.create_subscription(Path, '/robotinobase1/plan', self.path_callback1, 10)
#         self.subscription2 = self.create_subscription(Path, '/robotinobase2/plan', self.path_callback2, 10)

#         # Paths to store the coordinates
#         self.all_paths1 = []
#         self.all_paths2 = []

#         # CSV file paths for saving paths
#         self.csv_file_1 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase1_plans.csv'
#         self.csv_file_2 = 'ws_mapf/evaluation/two_corridors/mapf/cluster2Run/robotinobase2_plans.csv'

#         # Initialize CSV files
#         self.initialize_csv(self.csv_file_1)
#         self.initialize_csv(self.csv_file_2)

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def path_callback1(self, msg):
#         """Callback to process the path of robotinobase1."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths1.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file_1, path)
#         self.get_logger().info('Received path for robotinobase1')

#     def path_callback2(self, msg):
#         """Callback to process the path of robotinobase2."""
#         path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#         self.all_paths2.extend(path)  # Accumulate all paths
#         self.save_to_csv(self.csv_file_2, path)
#         self.get_logger().info('Received path for robotinobase2')

#     def save_to_csv(self, file_path, path):
#         """Save path data to CSV file."""
#         with open(file_path, 'a', newline='') as file:
#             writer = csv.writer(file)
#             for x, y in path:
#                 writer.writerow([x, y])

#     def plot_paths(self):
#         """Plot all accumulated paths of both robots."""
#         if self.all_paths1 and self.all_paths2:
#             x1, y1 = zip(*self.all_paths1)
#             x2, y2 = zip(*self.all_paths2)

#             plt.figure(figsize=(10, 6))

#             # Plot paths with specified colors and styles
#             plt.plot(x1, y1, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')
#             plt.plot(x2, y2, 'g-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase2 (Green)')

#             # Label the start points just below the point for better visibility
#             plt.text(x1[0], y1[0] - 0.1, 'Start', color='red', fontsize=8, ha='center', va='top')
#             plt.text(x2[0], y2[0] - 0.1, 'Start', color='green', fontsize=8, ha='center', va='top')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)  # Increased font size for the legend
#             plt.xlabel('X Coordinate', fontsize=12)  # Increased font size for the label
#             plt.ylabel('Y Coordinate', fontsize=12)  # Increased font size for the label
#             plt.title('Paths of Robotino Bases', fontsize=14)  # Increased font size for the title

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_paths()  # Plot the paths before shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# # monorunScript: To extract the pose of the robots and plot it over the map

# import rclpy
# from rclpy.node import Node
# from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
# import csv
# import matplotlib.pyplot as plt

# class DualRobotPathSubscriber(Node):
#     def __init__(self):
#         super().__init__('dual_robot_path_subscriber')

#         # Create a TF buffer and listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Timer to periodically fetch and save transforms
#         self.timer = self.create_timer(0.01, self.timer_callback)

#         # CSV file paths for the robots' transformations
#         self.tf_file_1 = 'robotinobase1_plan.csv'
#         self.tf_file_2 = 'robotinobase2_plan.csv'

#         # Initialize CSV files
#         self.initialize_csv(self.tf_file_1)
#         self.initialize_csv(self.tf_file_2)

#         # Store poses for plotting
#         self.poses_1 = []
#         self.poses_2 = []

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def timer_callback(self):
#         """Timer callback to fetch and save transforms."""
#         self.try_fetch_and_save_transform('robotinobase1/base_link', self.tf_file_1, self.poses_1)
#         self.try_fetch_and_save_transform('robotinobase2/base_link', self.tf_file_2, self.poses_2)

#     def try_fetch_and_save_transform(self, child_frame, file_path, poses):
#         """Attempt to fetch and save the transform if available."""
#         if self.tf_buffer.can_transform('map', child_frame, rclpy.time.Time()):
#             try:
#                 transform = self.tf_buffer.lookup_transform('map', child_frame, rclpy.time.Time())
#                 x = round(transform.transform.translation.x, 3)
#                 y = round(transform.transform.translation.y, 3)
#                 poses.append((x, y))

#                 with open(file_path, 'a', newline='') as file:
#                     writer = csv.writer(file)
#                     writer.writerow([x, y])  # Write x and y coordinates to the CSV

#                 self.get_logger().info(f'Saved transform for {child_frame}: x={x}, y={y}')
#             except (LookupException, ConnectivityException, ExtrapolationException) as e:
#                 self.get_logger().error(f'Failed to fetch transform for {child_frame}: {e}')
#         else:
#             self.get_logger().info(f'Transform not available for {child_frame}, skipping this cycle.')

#     def plot_poses(self):
#         """Plot the recorded poses."""
#         if self.poses_1 or self.poses_2:
#             plt.figure(figsize=(10, 6))

#             # Plot Robotino Base 1 poses
#             if self.poses_1:
#                 x1, y1 = zip(*self.poses_1)
#                 plt.plot(x1, y1, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')

#             # Plot Robotino Base 2 poses
#             if self.poses_2:
#                 x2, y2 = zip(*self.poses_2)
#                 plt.plot(x2, y2, 'g-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase2 (Green)')

#             # Label the start points just below the point for better visibility
#             if self.poses_1:
#                 plt.text(x1[0], y1[0] - 0.1, 'Start', color='red', fontsize=8, ha='center', va='top')
#             if self.poses_2:
#                 plt.text(x2[0], y2[0] - 0.1, 'Start', color='green', fontsize=8, ha='center', va='top')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)
#             plt.xlabel('X Coordinate', fontsize=12)
#             plt.ylabel('Y Coordinate', fontsize=12)
#             plt.title('Paths of Robotino Bases', fontsize=14)

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = DualRobotPathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_poses()  # Plot the recorded poses when shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# # script: two robots plan extraction

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# import csv

# class PathSubscriber(Node):
#     def __init__(self):
#         super().__init__('path_subscriber')

#         # Initialize subscribers for the robot paths
#         self.subscription1 = self.create_subscription(Path, '/robotinobase1/plan', self.path_callback1, 10)
#         self.subscription2 = self.create_subscription(Path, '/robotinobase2/plan', self.path_callback2, 10)

#         # Paths to store the coordinates
#         self.all_paths1 = []
#         self.all_paths2 = []

#         # Counters to track the number of plans received
#         self.plan_count1 = 0
#         self.plan_count2 = 0

#         # CSV file paths for saving paths
#         self.csv_file_1 = 'ws_mapf/evaluation/two_corridors/sapf/cluster2Run/robotinobase1_plans_first_2.csv'
#         self.csv_file_2 = 'ws_mapf/evaluation/two_corridors/sapf/cluster2Run/robotinobase2_plans_first_2.csv'

#         # Initialize CSV files
#         self.initialize_csv(self.csv_file_1)
#         self.initialize_csv(self.csv_file_2)

#     def initialize_csv(self, file_path):
#         """Initialize a CSV file with headers."""
#         with open(file_path, 'w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(['x', 'y'])  # Write the header row

#     def path_callback1(self, msg):
#         """Callback to process the path of robotinobase1."""
#         if self.plan_count1 < 3:
#             path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#             self.all_paths1.extend(path)  # Accumulate all paths
#             self.save_to_csv(self.csv_file_1, path)
#             self.get_logger().info('Received path for robotinobase1')
#             self.plan_count1 += 1

#     def path_callback2(self, msg):
#         """Callback to process the path of robotinobase2."""
#         if self.plan_count2 < 4:
#             path = [(round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)) for pose in msg.poses]
#             self.all_paths2.extend(path)  # Accumulate all paths
#             self.save_to_csv(self.csv_file_2, path)
#             self.get_logger().info('Received path for robotinobase2')
#             self.plan_count2 += 1

#     def save_to_csv(self, file_path, path):
#         """Save path data to CSV file."""
#         with open(file_path, 'a', newline='') as file:
#             writer = csv.writer(file)
#             for x, y in path:
#                 writer.writerow([x, y])

#     def plot_paths(self):
#         """Plot all accumulated paths of both robots."""
#         if self.all_paths1 and self.all_paths2:
#             x1, y1 = zip(*self.all_paths1)
#             x2, y2 = zip(*self.all_paths2)

#             plt.figure(figsize=(10, 6))

#             # Plot paths with specified colors and styles
#             plt.plot(x1, y1, 'r-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase1 (Red)')
#             plt.plot(x2, y2, 'g-', marker='o', markersize=0.5, linewidth=0.15, label='Robotinobase2 (Green)')

#             # Label the start points just below the point for better visibility
#             if self.all_paths1:
#                 plt.text(x1[0], y1[0] - 0.1, 'Start', color='red', fontsize=8, ha='center', va='top')
#             if self.all_paths2:
#                 plt.text(x2[0], y2[0] - 0.1, 'Start', color='green', fontsize=8, ha='center', va='top')

#             # Set axis limits
#             plt.xlim(-6, 6)
#             plt.ylim(0, 6)

#             # Add grid, legend, and labels with increased font sizes
#             plt.grid(True)
#             plt.legend(loc='lower left', fontsize=12)  # Increased font size for the legend
#             plt.xlabel('X Coordinate', fontsize=12)  # Increased font size for the label
#             plt.ylabel('Y Coordinate', fontsize=12)  # Increased font size for the label
#             plt.title('Paths of Robotino Bases', fontsize=14)  # Increased font size for the title

#             # Show the plot
#             plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathSubscriber()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.plot_paths()  # Plot the paths before shutting down
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
