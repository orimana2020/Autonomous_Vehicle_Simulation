#!/usr/bin/env python3

from interfaces.srv import GetPath
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import sys


class PathPlanning_Client(Node):
    def __init__(self):
        super().__init__('calc_path_client')
        self.cli = self.create_client(GetPath, '/calc_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPath.Request()

    def send_request(self, start, goal):
        self.req.start = start
        self.req.goal = goal
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

def main(args=None):
    rclpy.init(args=args)
    path_planning_client = PathPlanning_Client()
    start = Point()
    start.x = float(sys.argv[1]) # 0.0
    start.y = float(sys.argv[2])# 0.0
    goal = Point()
    goal.x = float(sys.argv[3])# 6.22
    goal.y = float(sys.argv[4])#-4.5
    print('sending_request')
    response = path_planning_client.send_request(start, goal)
    print(response.path)
    path_planning_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()