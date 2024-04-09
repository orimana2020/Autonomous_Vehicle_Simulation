from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name='rc_car' 
    path_planing_service = Node( 
        package=package_name,
        executable="PathPlanning_service.py",
        name="pathplanning_service",
        parameters=[{'use_sim_time': True}]
    )
    # Launch them all!
    return LaunchDescription([
        path_planing_service,
    ])