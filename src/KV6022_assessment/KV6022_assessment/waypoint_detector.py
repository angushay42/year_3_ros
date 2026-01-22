# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker

# student imported libraries (mine!)
import numpy as np
import json
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from visualization_msgs.msg import MarkerArray

class WaypointDetector(Node):
    def __init__(self):
        super().__init__('wp_detector')

        # self.wp_pub = self.create_publisher(
        #     MarkerArray,
        #     '/waypoints',
        #     10
        # )
        self.wp_sub = self.create_subscription(
            MarkerArray,
            '/waypoints',
            self.wp_callback,
            10
        )


    def wp_callback(self, data: MarkerArray):
        d = {}

        marker: Marker
        for marker in data.markers:
            if marker.type == Marker.SPHERE:
                d[marker.id] = {
                    "id": marker.id,
                    "pose": {
                        "x": float(marker.pose.position.x),
                        "y": float(marker.pose.position.y),
                        "z": float(marker.pose.position.z)
                    },
                    "orientation": {
                        "x": float(marker.pose.orientation.x),
                        "y": float(marker.pose.orientation.z),
                        "z": float(marker.pose.orientation.y),
                        "w": float(marker.pose.orientation.w),
                    }
                }
        with open('waypoints.txt', "w") as f:
            f.write(json.dumps(d, indent=2))


def main(args=None):
    rclpy.init(args=args)

    waypoint_det = WaypointDetector()

    rclpy.spin(waypoint_det)
    waypoint_det.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()