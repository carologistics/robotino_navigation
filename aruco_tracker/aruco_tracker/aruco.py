import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from transforms3d.euler import euler2quat
from geometry_msgs.msg import TransformStamped
# todo:transform the detected object's coordinates to the map frame.
# 2 node,pos and tf,
class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher = self.create_publisher(TransformStamped, 'aruco_pose_map', 10)
        self.br = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv.aruco.DetectorParameters()
        self.marker_size = 0.05  # Adjust based on actual size in meters
        
        self.camera_matrix = np.array([[541.83897791, 0, 312.34902706], 
                                       [0, 539.27016132, 234.15780227], 
                                       [0, 0, 1]], dtype=np.float32)  # Replace with your calibration
        self.dist_coeffs = np.array([0.11646948,-0.43243322,-0.00127437,0.00096187,0.46947971], dtype=np.float32)  # Replace with your calibration
        self.last_seen_ids = set()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.get_logger().info("Aruco Detector Node Started")
    
    def image_callback(self, msg):
        self.subscriptions = None
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        current_ids = set()
        now = self.get_clock().now()
        if ids is not None:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                self.marker_last_seen[marker_id] = now 
                self.transform_to_map(marker_id, tvecs[i], rvecs[i])
                cv.aruco.drawDetectedMarkers(frame, corners, ids)
                cv.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

        # Remove transforms for markers not seen this time
        vanished_ids = self.last_seen_ids - current_ids
        for marker_id in vanished_ids:
            pass

        self.last_seen_ids = current_ids
    def transform_to_map(self, marker_id, tvec,rvecs):
        try:
            marker_pose = PoseStamped()
            marker_pose.header.frame_id = "robotinobase2/camera_link"
            marker_pose.header.stamp = self.get_clock().now().to_msg()

            tvec = tvec.flatten()
            marker_pose.pose.position.x = tvec[0]
            marker_pose.pose.position.y = tvec[1]
            marker_pose.pose.position.z = tvec[2]

            # This is a placeholder. Convert rvecs properly to a quaternion if needed
            rvecs = rvecs.flatten()
            quat = euler2quat(rvecs[0],rvecs[1], rvecs[2])
            marker_pose.pose.orientation.w = quat[0]
            marker_pose.pose.orientation.x = quat[1]
            marker_pose.pose.orientation.y = quat[2]
            marker_pose.pose.orientation.z = quat[3]
            transformed_pose = self.tf_buffer.transform(marker_pose, "robotinobase2/base_link", timeout=rclpy.duration.Duration(seconds=1.0))

        # Now broadcast as TransformStamped (optional, if needed elsewhere)
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = "robotinobase2/base_link"
            transform_msg.child_frame_id = f"aruco_marker_{marker_id}"

            transform_msg.transform.translation.x = transformed_pose.pose.position.x
            transform_msg.transform.translation.y = transformed_pose.pose.position.y
            transform_msg.transform.translation.z = transformed_pose.pose.position.z
            transform_msg.transform.rotation = transformed_pose.pose.orientation

            self.tf_broadcaster.sendTransform(transform_msg)

            self.get_logger().info(f"Published ArUco Marker {marker_id} in base_link frame at ({transformed_pose.pose.position.x:.2f}, {transformed_pose.pose.position.y:.2f}, {transformed_pose.pose.position.z:.2f})")
        except tf2_ros.LookupException:
            self.get_logger().warn("No transformation found from camera_link to map.")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("Extrapolation error in TF lookup.")
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
