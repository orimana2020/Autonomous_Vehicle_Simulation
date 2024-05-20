#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import GetPath
from AStar_python_interface import A_Star, CSpace
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap


class PathPlanningService(Node):
    def __init__(self):
        super().__init__('calc_path')
        self.calc_path_service = self.create_service(GetPath, '/calc_path', self.get_path_callback)
        self.get_map()
        self.astar = A_Star(self.map_data, inflation=int(0.35/self.map_resolution))
        self.converter = CSpace(self.map_resolution, self.map_origin_x, self.map_origin_y, self.map_rows, self.map_cols)

    def get_path_callback(self, request, response):
        response.path = self.get_path(request.start , request.goal)
        return response

    def get_path(self, start, goal):
        start_index = self.converter.meter2pixel([start.x, start.y])
        goal_index = self.converter.meter2pixel([goal.x, goal.y])
        np.save('start_index', np.array(start_index))
        np.save('goal_index', np.array(goal_index))
        print('start_AStar')
        path_index = self.astar.find_path(start_index, goal_index)
        np.save('path_maze_index_sim', np.array(path_index))
        if path_index is not None:
            path_meter = self.converter.pathindex2pathmeter(path_index)
            np.save('path_maze_meter_sim', path_meter)
            path = []
            for coord in path_meter:
                point = Point()
                point.x = coord[0]
                point.y = coord[1]
                path.append(point)
            return path
        return None

    def get_map(self):
        self.map_client = self.create_client(GetMap, '/map_server/map')
        while not self.map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service not available, waiting again...')
        self.map_req = GetMap.Request()
        self.map_future = self.map_client.call_async(self.map_req)
        rclpy.spin_until_future_complete(self, self.map_future)
        map_response = self.map_future.result()
        print(f'map response :{map_response}')
        # self.map_origin = map_response.map.info.origin
        self.map_resolution = map_response.map.info.resolution
        self.map_width = map_response.map.info.width
        self.map_height = map_response.map.info.height
        self.map_origin_x = map_response.map.info.origin.position.x
        self.map_origin_y = map_response.map.info.origin.position.y
        map_data = np.array(map_response.map.data)
        self.map_data = map_data.reshape((self.map_height, self.map_width))
        self.map_rows, self.map_cols = self.map_data.shape


def main():
    rclpy.init()
    pathplanning_service = PathPlanningService()
    rclpy.spin(pathplanning_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()