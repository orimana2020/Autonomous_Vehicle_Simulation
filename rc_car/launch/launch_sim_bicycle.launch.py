import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name='rc_car' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items())
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items())
        
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in','/cmd_vel_out'),
                    ('/cmd_vel_out','/diff_cont/reference')]
        )
        

    # Set the path to the world file
    world_file_name = 'maze1.world'
    # world_file_name = 'wall_following.world'
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds',world_file_name )
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description','-entity', 'rc_car'], output='screen')
    

    bicycle_drive_spawner = Node( # spawn the controller
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    odom_computation = Node( 
        package="rc_car",
        executable="odom_publisher_bicycle.py",
        name="compute_odom",
        parameters=[{'use_sim_time': True}]
    )

    # default_rviz_config_path = os.path.join(get_package_share_directory(package_name), 'rviz_config/main.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    #     parameters=[{'use_sim_time': True}],
    # )



    # robot_localization_node = Node( #not working well
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[os.path.join(get_package_share_directory(package_name),'config','ekf.yaml'), {'use_sim_time': True}]
    # )

    # relay_topic_to_tf_node = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     arguments=['/diff_cont/tf_odometry', '/tf'],
    #     output='screen',
    # )



    # Launch them all!
    return LaunchDescription([
        # DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='world',default_value=world_path,
                                            description='Full path to the world model file to load'),
    
    
        rsp,
        joystick,
        twist_mux,
        twist_stamper,
        gazebo,
        spawn_entity,
        bicycle_drive_spawner,
        joint_broad_spawner,
        odom_computation,
        # rviz_node,
        # robot_localization_node,
        
    ])
