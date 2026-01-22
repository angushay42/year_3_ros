#! /usr/bin/env python3


from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener

# student imports
import json
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

def main():
    rclpy.init()

    navigator = BasicNavigator()
    
    marker_node = rclpy.node.Node('marker_pub')
    marker_pub = marker_node.create_publisher(MarkerArray, 'goal_wp', 10)

    tf_buffer = Buffer()
    # tf_listener = TransformListener(tf_buffer, navigator.get_clock())

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    try:
        transform = tf_buffer.lookup_transform('map', 'base_footprint', navigator.get_clock().now())
        
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = transform.transform.translation.x
        initial_pose.pose.position.y = transform.transform.translation.y
        initial_pose.pose.orientation = transform.transform.rotation
    except Exception as e:
        print("Could not get transform from 'map' to 'base_footprint'. Using default initial pose.")
   
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # set our demo's goal poses to follow
    goal_poses = []
    d = {}

    # TODO
    try:
        with open('demo_wp.txt', 'r') as f: 
            d: dict[int, dict] = json.load(f)
    except json.JSONDecodeError as e:
        print(f'error getting waypoints from json: {e}')
        return
    
    for id, mark_pose in d.items():
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rclpy.time.Time().to_msg()
        p.pose = Pose(
            position=Point(**mark_pose['pose']),
            orientation=Quaternion(**mark_pose['orientation'])
        )
        goal_poses.append(p)

    marr = MarkerArray()
    marr.markers = [
        Marker(
            header=Header(
                frame_id='map',
                stamp=rclpy.time.Time().to_msg()
            ),
            pose=x.pose
        )
        for x in goal_poses
    ]
    
    marker_pub.publish(marr)


    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()
    follow_waypoints_task = navigator.followWaypoints(goal_poses)

    try:
        i = 0
        while not navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Executing current waypoint: '
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(goal_poses))
                )
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelTask()
    except KeyboardInterrupt:
        print('keyboard interrupt')
        navigator.cancelTask()
        navigator.lifecycleShutdown()
            
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print('Goal failed!{error_code}:{error_msg}')
    else:
        print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()
    #exit(0)


if __name__ == '__main__':
    main()
