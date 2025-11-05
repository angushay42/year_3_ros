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
w = v/r

R = 0.5m
v = 0.2m/s
w = 0.2 / 0.5
"""

"""
Adapt your controller for LIMO differential and mecanum mode by publishing to /cmd_vel accordingly.
Drive the robot to move in a circle
Draw a square. Go straight, stop, turn 90°. Repeat it four times.
"""

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 100)
        self.circle(0.5, 180, 0.4)
        
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def move_1(self):
        curr = time.time()
        elapsed_time = curr - self.start_time

        self.get_logger().info(f"Elapsed Time: {elapsed_time:.2f}s / Move Duration: {self.move_duration:.2f}s")

        if elapsed_time >= self.move_duration:
            self.timer.cancel()
            self.get_logger().info("Movement complete.")
            return

        self.publisher_.publish(self.st)
        self.get_logger().info(f"Publishing: {self.st}")

    def circle(self, radius: float, start_dir: float, vel: float):
        if not (0 <= start_dir < 360):
            raise TypeError("invalid start direction")
        
        # ensure velocity is < max_vel
        if not (0 < vel <= MAX_VEL):
            raise TypeError("invalid velocity")
        
        if radius == 0:
            raise TypeError("invalid radius")

        # turn start_dir
        # drive forward from origin to radius
        self.st = Twist()
        self.st.linear.x = vel
        self.st.angular.z = vel / radius
        # self.st.angular.z = 10.0

        rate = 0.01 # publish rate

        # time = distance / speed
        self.move_duration = ((2*math.pi * radius)/ vel) * 5.33
        
        self.start_time = time.time()

        # start timer
        self.timer = self.create_timer(rate, self.move_1)
        

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