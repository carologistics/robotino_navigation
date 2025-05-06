import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
from transforms3d.euler import euler2quat
from geometry_msgs.msg import TransformStamped
# todo:transform the detected object's coordinates to the map frame.
# 2 node,pos and tf,
class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher = self.create_publisher(TransformStamped, 'tag_id_names', 10)
        self.br = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.camera_matrix = np.array([[602.368163 , 0, 316.403913], 
                                        [0, 600.842440, 247.873301], 
                                        [0, 0, 1]], dtype=np.float32)  # Replace with your calibration
        self.dist_coeffs = np.array([0.105207, -0.241722, -0.000910, -0.001297 ,0.000000], dtype=np.float32)  # Replace with your calibration

        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv.aruco.DetectorParameters()
        self.marker_size = 0.123  # Adjust based on actual size in meters
        self.tag_id_names = {
            101: "C-CS1-O", 103: "C-CS2-O", 201: "M-CS1-O", 203: "M-CS2-O",
            111: "C-RS1-O", 113: "C-RS2-O", 211: "M-RS1-O", 213: "M-RS2-O",
            121: "C-BS-O", 221: "M-BS-O", 131: "C-DS-O", 231: "M-DS-O",
            141: "C-SS-O", 241: "M-SS-O",
            102: "C-CS1-I", 104: "C-CS2-I", 202: "M-CS1-I", 204: "M-CS2-I",
            112: "C-RS1-I", 114: "C-RS2-I", 212: "M-RS1-I", 214: "M-RS2-I",
            122: "C-BS-I", 222: "M-BS-I", 132: "C-DS-I", 232: "M-DS-I",
            142: "C-SS-I", 242: "M-SS-I"
        } 
        
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.get_logger().info("Aruco Detector Node Started")
    
    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        current_ids = set()
        now = self.get_clock().now()
        if ids is not None:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                tag_id = ids[i][0]
                if tag_id not in self.tag_id_names:
                    continue  # Skip these tags
                self.transform_to_map(ids[i][0], tvecs[i], rvecs[i])
                cv.aruco.drawDetectedMarkers(frame, corners, ids)
                cv.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                pts = np.int32(corners[i][0])
                cv.fillConvexPoly(frame, pts, (0, 255, 0))  # Green filled box


    def transform_to_map(self, marker_id, tvec,rvecs):
        try:
            marker_name = self.tag_id_names.get(marker_id)
            marker_local = TransformStamped()
            
            marker_local.header.frame_id = "robotinobase2/camera_link"
            marker_local.header.stamp = self.get_clock().now().to_msg()
            marker_local.child_frame_id = marker_name
            pitch = rvecs[1] 
            row   = np.pi / 2.0  
            pitch2 = np.pi / 2.0
            rot_pitch = R.from_rotvec(pitch * np.array([0, 1, 0]))
            rot_row   = R.from_rotvec(row   * np.array([1, 0, 0]))
            rot_pitch2 = R.from_rotvec(pitch2 * np.array([0, 0, 1]))
            rot_total = rot_row * rot_pitch2*rot_pitch
            tvec = tvec.flatten()
            marker_local.transform.translation.x = tvec[0]
            marker_local.transform.translation.y = tvec[1]
            marker_local.transform.translation.z = tvec[2]
            self.get_logger().info(f"Published '{marker_name}' in base_link frame at ({rvecs})")
            
            quat = rot_total.as_quat()
            marker_local.transform.rotation.w = quat[0]
            marker_local.transform.rotation.x = quat[1]
            marker_local.transform.rotation.y = quat[2]
            marker_local.transform.rotation.z = quat[3]
            self.tf_broadcaster.sendTransform(marker_local)
            self.get_logger().info(f"Published '{marker_name}' in base_link frame at ({marker_local.transform.translation.x:.2f}, {marker_local.transform.translation.y:.2f}, {marker_local.transform.translation.z:.2f})")
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
#todo:add bounding box

# camera matrix real sense
# 602.368163 0.000000 316.403913
# 0.000000 600.842440 247.873301
# 0.000000 0.000000 1.000000

# distortion
# 0.105207 -0.241722 -0.000910 -0.001297 0.000000

# rectification
# 1.000000 0.000000 0.000000
# 0.000000 1.000000 0.000000
# 0.000000 0.000000 1.000000

# projection
# 608.750885 0.000000 315.699972 0.000000
# 0.000000 608.146404 247.587353 0.000000
# 0.000000 0.000000 1.000000 0.000000

# self.camera_matrix = np.array([[541.83897791, 0, 312.34902706], 
#                                       [0, 539.27016132, 234.15780227], 
#                                       [0, 0, 1]], dtype=np.float32)  # Replace with your calibration
# self.dist_coeffs = np.array([0.11646948,-0.43243322,-0.00127437,0.00096187,0.46947971], dtype=np.float32)  # Replace with your calibration
# #realsense
# self.camera_matrix = np.array([[602.368163 , 0, 316.403913], 
#                                        [0, 600.842440, 247.873301], 
#                                        [0, 0, 1]], dtype=np.float32)  # Replace with your calibration
#         self.dist_coeffs = np.array([0.105207, -0.241722, -0.000910, -0.001297 ,0.000000], dtype=np.float32)  # Replace with your calibration
