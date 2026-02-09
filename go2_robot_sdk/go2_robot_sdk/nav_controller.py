#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess

class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')
        self.srv = self.create_service(SetBool, 'nav_control', self.callback)
        self.test_running = False
        self.get_logger().info('Nav controller ready')
        self.get_logger().info('Start test: ros2 service call /nav_control std_srvs/srv/SetBool "{data: true}"')

    def callback(self, request, response):
        if request.data and not self.test_running:
            self.test_running = True
            self.get_logger().info('Starting navigation test...')

            result = subprocess.run(
                ['ros2', 'run', 'go2_robot_sdk', 'sequential_nav_test'],
                capture_output=True,
                text=True
            )

            self.test_running = False
            response.success = True
            response.message = f'Test completed. Output: {result.stdout[:100]}...'

        elif not request.data and self.test_running:
            #could add stop logic here
            response.success = True
            response.message = 'Stop requested'
        else:
            response.success = False
            response.message = 'Already in requested state'

        return response

def main():
    rclpy.init()
    rclpy.spin(NavController())
    rclpy.shutdown()
