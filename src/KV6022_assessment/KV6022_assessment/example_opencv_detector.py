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
import random
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from visualization_msgs.msg import MarkerArray

class PotHole:
    """Pothole helper class"""
    pose: Pose
    area: float

    def __init__(self, pose: Pose = None, area: float=0.0):
        self.pose = pose
        self.area = area

    def __hash__(self):
        return hash((
            self.pose.position.x,
            self.pose.position.y,
            self.pose.position.z,
            self.area)
        )

class ObjectDetector(Node):
    camera_model = None
    image_depth_ros = None
    camera_frame = None

    # global_frame = "base_link"  # unsure, is it world? map?
    global_frame = "map"    # it's map but navigation needs to be set up first

    found_objs: list[Pose] = []     # could be hashmap
    found_pots: dict[int, PotHole]
    obj_thresh = 0.2                # how accurate is this?
    depth_thresh = 0.4              # in metres
    area_thresh = 0.2               # in metres

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

        # ========================== Camera ===================================
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
        self.image_depth_sub = self.create_subscription(
            Image, 
            '/limo_camera/depth/image_raw',     # sensor_msgs/msg/Image
            self.image_depth_callback, 
            10
        )
        # ====================== end Camera ===================================
        

        # ============================ Pothole ================================
        self.pothole_pub = self.create_publisher(
            PoseArray,
            '/pothole',
            5       # think this is too fast
        )
        self.pothole_sub = self.create_subscription(
            PoseArray, 
            '/pothole',
            self.pothole_callback,
            10
        )
        # ============================ end Pothole ============================


        # ============================ Marker =================================
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/limo/object_markers', 
            10
        )
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/limo/object_markers',
            self.marker_callback,
            10
        )
        # ============================ end Marker =============================


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        print('finished init')
    
    def camera_info_callback(self, data: CameraInfo):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        
        self.camera_frame = data.header.frame_id   # it's most likely depth_link

    def image_depth_callback(self, data: Image):
        self.image_depth_ros = data

    def image_color_callback(self, data: Image):
        # wait for camera_model and depth image to arrive
        if not all([
            self.camera_frame, 
            self.camera_model, 
            self.image_depth_ros, 
        ]):
            return
        
        if not (
            self.tf_buffer.can_transform(
                self.global_frame, 
                self.camera_frame,
                rclpy.time.Time()
            )
        ):
            return
        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            self.get_logger().warn(f"Bridge error: {e}")
        
        hsv_img = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)

        # make threshold
        hsv_thresh = cv2.inRange(
            hsv_img,
            np.array((140, 100, 50)),
            np.array((179, 255, 255))
        )
        
        contours, hierachy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        poses = PoseArray()
        poses.header.frame_id = self.global_frame
        poses.header.stamp = rclpy.time.Time().to_msg()

        for cnt in contours:
            img_moments = cv2.moments(cnt)
            if img_moments["m00"] <= 0:
                self.get_logger().warn("invalid contour {}".format(cnt))
                continue
            centre = (
                int(img_moments["m10"] / img_moments["m00"]), 
                int(img_moments["m01"] / img_moments["m00"])
            )
            cv2.circle(image_color, centre, 2, (255, 255, 255), 1)

            # project ray from camera to pixel
            # turn into numpy array for element wise ops
            ray = np.array(self.camera_model.projectPixelTo3dRay(centre))
            if not ((0 < centre[0] <= self.image_depth_ros.width) 
                and (0 < centre[1] <= self.image_depth_ros.height)
            ):
                continue
            depth = image_depth[centre[1], centre[0]]   # flipped
            if depth > self.depth_thresh:               # TODO testing if this filters
                continue

            # scale unit vector (ray) with depth (in metres)
            ray *= depth

            # get pose
            local_pose = Pose(
                position=Point(
                    x=ray[0],
                    y=ray[1],
                    z=ray[2]
                ),
                orientation=Quaternion(x=0.0,y=0.0,z=0.0,
                    w=1.0   # or convert from euler (0,0,0)
                )
            )

            try:
                global_pose = do_transform_pose(
                    pose=local_pose, 
                    transform=self.tf_buffer.lookup_transform(
                        self.global_frame,
                        self.camera_frame,
                        rclpy.time.Time(),
                    )
                )

                poses.poses.append(global_pose)
            except Exception as e:
                self.get_logger().warn(f"could not retrieve transform: {e}")
                continue    # skip 

        self.pothole_pub.publish(poses)

        cv2.imshow("colour image", image_color)
        cv2.imshow("detection mask", image_depth)
        cv2.waitKey(1)

    def check_pose_exists(self, pose: Pose):
        """New object will return False. True if exists"""

        """
        possible alternative filters:
        - distance
        - area 
        """
        for o in self.found_objs:
            dist = np.sqrt(
                ((o.position.x - pose.position.x)**2)
                + ((o.position.y - pose.position.y)**2)
                + ((o.position.z - pose.position.z)**2)
            )
            if dist < self.obj_thresh:
                return True
        return False

    def pothole_callback(self, data: PoseArray):
        # filter poses based on some threshold
        marr = MarkerArray()
        
        for num, pose in enumerate(data.poses):
            if self.check_pose_exists(pose):
                continue
            self.get_logger().info('appending pose')
            self.found_objs.append(pose)

            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.id = len(self.found_objs)    # should be monotonic
            marr.markers.append(marker)
        
        if not marr.markers:
            return
        self.marker_pub.publish(marr)

    def marker_callback(self, data: MarkerArray):
        self.get_logger().warn('MARKERs: {}'.format(data))


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
