import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
from tf_transformations import quaternion_from_matrix


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('marker_length', 0.1778)  # meters

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Simulated camera intrinsics
        w, h = 640, 360
        fov = 1  # 60 degrees in radians
        fx = fy = w / (2 * np.tan(fov / 2))
        cx, cy = w / 2, h / 2
        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0,  0,  1]], dtype=np.float64)
        self.dist_coeffs = np.zeros(5)

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())

        self.get_logger().info(f"Subscribed to image topic: {self.image_topic}")

    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = self.estimate_pose(corners[i])
                self.publish_tf(marker_id, rvec, tvec, msg)

                # Debug output
                self.get_logger().info("*********")
                R, _ = cv2.Rodrigues(rvec)
                self.get_logger().info(f"Rotation matrix:\n{R}")
                T = np.eye(4)
                T[:3, :3] = R
                self.get_logger().info(f"Transformation matrix:\n{T}")
                self.get_logger().info(f"Translation vector:\n{tvec}")
                self.get_logger().info("*********")

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length / 2)

        cv2.imshow("Aruco Marker Detection", frame)
        cv2.waitKey(1)

    def estimate_pose(self, corners):
        half = self.marker_length / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        retval, rvec, tvec = cv2.solvePnP(
            objectPoints=obj_points,
            imagePoints=corners,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        return rvec, tvec

    def publish_tf(self, marker_id, rvec, tvec, msg):
        # R_marker_cv, _ = cv2.Rodrigues(rvec)          # marker in OpenCV camera frame
        # tvec_cv = tvec.reshape(3, 1)                  # shape (3, 1)

        # # Transform to ROS camera frame
        # R_cv_to_ros = np.array([[0, 0, 1],
        #                         [1, 0, 0],
        #                         [0, 1, 0]])

        # R_marker_ros = R_cv_to_ros @ R_marker_cv
        # tvec_ros = R_cv_to_ros @ tvec_cv
        
        # marker_T_ros = np.eye(4)
        # marker_T_ros[:3, :3] = R_marker_ros
        # q = quaternion_from_matrix(marker_T_ros)
        rvec = rvec.reshape(3, 1)
        tvec = tvec.reshape(3, 1)
        R_cv_to_ros = np.array([[0, 0, 1],
                                [1, 0, 0],
                                [0, 1, 0]])
        self.get_logger().info(f"Original tvec (OpenCV): {tvec.T}")
        rvec = R_cv_to_ros @ rvec
        tvec = R_cv_to_ros @ tvec
        
        self.get_logger().info(f"Transformed tvec (ROS): {tvec.T}")
        # self.get_logger().info(f"Original R (OpenCV):\n{R_marker_cv}")
        # self.get_logger().info(f"Transformed R (ROS):\n{R_marker_ros}")
        rot, jacobian = cv2.Rodrigues(rvec)
        rot_matrix = np.eye(4, dtype=np.float32)
        rot_matrix[0:3, 0:3] = rot

        

        # convert rotation matrix to quaternion
        quaternion = quaternion_from_matrix(rot_matrix)
        norm_quat = np.linalg.norm(quaternion)
        q = quaternion / norm_quat
        # Now use tvec_ros and q to publish


        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = "camera_link"
        tf_msg.child_frame_id = f"aruco_marker_{marker_id}"

        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[2])
        tf_msg.transform.translation.z = float(tvec[1])
       

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
