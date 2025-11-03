import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import math
import time

MAX_VEL = 1.0
"""
R = v / w
R is radius length? v is linear vel, w is angular vel
R is length from ICC to centre of robot (P)
"""

"""
Adapt your controller for LIMO differential and mecanum mode by publishing to /cmd_vel accordingly.
Drive the robot to move in a circle
Draw a square. Go straight, stop, turn 90°. Repeat it four times.
"""

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.circle(0.5, 180, 0.2)
        
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)


        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # m/s
        cmd_vel.angular.z = math.radians(90)

        self.publisher_.publish(cmd_vel)
        self.get_logger().info('Publishing: "%s"' % cmd_vel)

        self.i += 1

    def circle(self, radius: float, start_dir: float, vel: float):
        if not (0 <= start_dir < 360):
            raise TypeError("invalid start direction")
        
        # ensure velocity is < max_vel
        if not (0 < vel <= MAX_VEL):
            raise TypeError("invalid velocity")
        
        if radius == 0:
            raise TypeError("invalid radius")
        
        start_dir = math.radians(start_dir)

        # turn start_dir
        # drive forward from origin to radius
        st = Twist()
        st.linear.x = vel
        st.angular.z = start_dir

        # time = distance / speed
        time_st = radius / vel
        
        self.publisher_.publish(st)
        self.get_logger().info('Publishing: "%s"' % st)

        # robot will move at vel speed and we will stop it 
        # after t seconds, moving it d distance
        time.sleep(time_st)

        st.linear.x = 0.0
        st.angular.z = 0.0

        self.publisher_.publish(st)
        self.get_logger().info('Publishing: "%s"' % st)

        # calculate w as v/R
        cir = Twist()
        cir.linear.x = vel
        cir.angular.z = math.radians(vel/radius)

        # circumference = 2*pi*r, which is distance
        self.publisher_.publish(cir)
        self.get_logger().info('Publishing: "%s"' % cir)

        time.sleep(2 * math.pi * radius)
        

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