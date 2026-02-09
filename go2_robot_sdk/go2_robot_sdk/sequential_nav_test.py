#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
#from tf_transformations import quaternion_from_euler


class SequentialNavTest(Node):
    def __init__(self):
        super().__init__('sequential_nav_test')
        
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('Nav2 action server ready')

        self.run_sequence()

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def send_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def run_sequence(self):
        #Forward 2 meters
        self.send_goal(self.make_pose(1.0, 0.0, 0.0))

        #Rotate 180 degrees in place
        self.send_goal(self.make_pose(1.0, 0.0, math.pi))

        time.sleep(2.0)

        #return 2 meters
        self.send_goal(self.make_pose(0.0, 0.0, math.pi))

def main():
    rclpy.init()
    node = SequentialNavTest()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

