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
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker, MarkerArray

# student imported libraries (mine!)
import random

class ObjectDetector(Node):
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the dabai camera parameters
    color2depth_aspect = 1.0 # for a simulated camera

    def __init__(self):    
        super().__init__('detector')
        print('starting init')
        self.bridge = CvBridge()

        self.rp = None  # todo temp variable for testing
        self.depth_val = -1

        # subscribe to topic, when given data call callback
        # todo question, what is data? 
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            '/limo_camera/depth/camera_info',   # sensor_msgs/msg/CameraInfo
            self.camera_info_callback, 
            qos_profile=qos.qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image, 
            '/limo_camera/image',               # sensor_msgs/msg/Image
            self.image_color_callback, 
            10
        )
        self.image_sub = self.create_subscription(
            Image, 
            '/limo_camera/depth/image_raw',     # sensor_msgs/msg/Image
            self.image_depth_callback, 
            10
        )
        
        self.marker_array_pub = self.create_publisher(
            MarkerArray, 
            '/limo/object_markers', 
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        print('finished init')

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                rclpy.time.Time()
            )
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None
    
    def camera_info_callback(self, data):
        print('camera info callback')
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        print('image depth callback')

        self.image_depth_ros: Image = data
        if self.rp:
            self.depth_val = self.image_depth_ros.data[
                    (self.rp[1] * self.image_depth_ros.width) + self.rp[0]
                ]

    def image_color_callback(self, data: Image):
        print('image colour callback')

        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)


        # PROVIDE YOUR OBJECT DETECTION CODE HERE
        cx, cy = self.camera_model.cx(), self.camera_model.cy()
        cx, cy = self.camera_model.rectifyPoint((cx, cy))

        # get random valid point, need height and width
        if not self.rp:
            self.rp = (
                random.randint(0, self.image_depth_ros.width-1),
                285
                # random.randint((self.image_depth_ros.height-1) // 2, self.image_depth_ros.height-1),
            )
        _args = [
            # self.camera_model.projectPixelTo3dRay((cx, cy))
            f"x: {self.rp[0]}",
            f"y: {self.rp[1]}",
            f"depth: {image_depth[self.rp[1], self.rp[0]]}"
        ]
        print(
            ("=" * 50) 
            + ("{}, " * len(_args)).format(*_args) 
            + ("="*50)
        )
        
        """
        Super brief epic super simple swag soodough code
        get image
        find contours
        find centre of contour (moments)
        projectpixel to 3d
        map centre pixel from image to depth image


        """

        cv2.circle(image_depth, self.rp, 5, (0, 0, 255) , 2)
      
        cv2.imshow("colour image", image_color)
        cv2.imshow("detection mask", image_depth)
        cv2.waitKey(1)
        
        
        object_location_map = PoseStamped()
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = object_location_map.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = 0

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_array_pub.publish(marker_array)

def main(args=None):
    print('Starting OpenCV detector node.')
    
    print('starting rclpy init')
    rclpy.init(args=args)
    print('rclpy init success')

    print('creating object')
    image_projection = ObjectDetector()
    print('object created')
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
