from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name='rc_car' 
    wall_following = Node( 
        package="rc_car",
        executable="PID_wall_following.py",
        name="wall_following",
        parameters=[{'use_sim_time': True}]
    )


    # Launch them all!
    return LaunchDescription([
        wall_following,
        
    ])
