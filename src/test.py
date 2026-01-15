import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

rclpy.init()
node = Node('marker_pub')
pub = node.create_publisher(Marker, '/marker', 10)

m = Marker()
m.header.frame_id = 'map'
m.header.stamp = node.get_clock().now().to_msg()
m.type = 2
m.action = 0
m.pose.position.x = 0.0
m.pose.orientation.w = 1.0
m.scale.x = 0.2
m.scale.y = 0.2
m.scale.z = 0.2
m.color.r = 1.0
m.color.a = 1.0

pub.publish(m)