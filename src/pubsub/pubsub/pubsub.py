import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import math

"""
Adapt your controller for LIMO differential and mecanum mode by publishing to /cmd_vel accordingly.
Drive the robot to move in a circle
Draw a square. Go straight, stop, turn 90°. Repeat it four times.
"""

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # m/s
        cmd_vel.angular.z = math.radians(90)

        self.publisher_.publish(cmd_vel)
        self.get_logger().info('Publishing: "%s"' % cmd_vel.linear.x)

        self.i += 1

    def circle(self, radius, start_dir):
        # drive forward from origin to radius
        # turn either either left or right (start_dir)
        

        pass

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()