#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import GetPath
from py_AStar import A_Star
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap
from py_Utils import CSpace

class PathPlanningService(Node):
    def __init__(self):
        super().__init__('calc_path')
        self.calc_path_service = self.create_service(GetPath, '/calc_path', self.get_path_callback)
        self.declare_parameter('map_name', 'default_map')
        self.map_name = str(self.get_parameter('map_name').get_parameter_value().string_value)
        
        self.get_map()
        self.astar = A_Star(self.map_data, inflation=int(0.35/self.map_resolution))
        self.converter = CSpace(self.map_resolution, self.map_origin_x, self.map_origin_y, self.map_data.shape)

    def get_path_callback(self, request, response):
        response.path = self.get_path(request.start , request.goal, request.path_name)
        return response

    def get_path(self, start, goal, path_name):
        start_index = self.converter.meter2pixel([start.x, start.y])
        goal_index = self.converter.meter2pixel([goal.x, goal.y])
        path_index = self.astar.find_path(start_index, goal_index)
        path_name_ =  path_name.data
        if path_index is not None:
            np.save(path_name_+'_index', np.array(path_index))
            path_meter = self.converter.pathindex2pathmeter(path_index)
            np.save(path_name_+'_meter', path_meter)
            path = []
            for coord in path_meter:
                point = Point()
                point.x = coord[0]
                point.y = coord[1]
                path.append(point)
            print('Path found')
            return path
        print('No path found')
        return None

    def get_map(self):
        map_dict = np.load(self.map_name+'.npy', allow_pickle=True)
        self.map_resolution =  map_dict.item().get('map_resolution')
        self.map_width = map_dict.item().get('map_width')
        self.map_height = map_dict.item().get('map_height')
        self.map_origin_x = map_dict.item().get('map_origin_x')
        self.map_origin_y = map_dict.item().get('map_origin_y')
        self.map_origin_z = map_dict.item().get('map_origin_z')
        self.map_data = map_dict.item().get('map_data')



def main(args=None):
    rclpy.init(args=args)
    pathplanning_service = PathPlanningService()
    rclpy.spin(pathplanning_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()