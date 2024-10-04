## Project description
 This project integrates all the components for Akermann autonomous racing car with ROS2 humble + Gazebo classic simulation.

It includes control, path planing, path tracking, mapping and localization.  

supported sensors: Encoder, Lidar, RGB-D camera, IMU.

## install ROS2 humble - see documentation 

## Install packages
```terminal
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-joint-state-publisher ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-twist-mux ros-humble-twist-stamper ros-humble-gazebo-ros*      
```
```terminal
sudo apt-get install python3-pip
pip install setuptools==58.2.0
sudo apt install joystick jstest-gtk evtest 
sudo apt install python3-serial 
sudo apt install libserial-dev
sudo adduser $USER dialout
sudo apt install net-tools
```

# How to use - Unmapped environment:

Terminal 1: General simulation loading
```terminal
source install/setup.bash
ros2 launch rc_car launch_sim.launch.py world:=./src/rc_car/worlds/maze1.world
```

Terminal 2: Rviz
```terminal
source install/setup.bash
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/main.rviz --ros-args -p use_sim_time:=true
```
-Or when use localization-

```terminal
source install/setup.bash
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/localization.rviz --ros-args -p use_sim_time:=true
```

Terminal 3: Mapping, add map to Rviz
```terminal
source install/setup.bash
ros2 launch rc_car online_async_launch.py use_sim_time:=true
```
Save the map with rviz: add new panel->slam tool box plugin, 

Serialize the map,reset the simulation,


Change the mapper params online async.yaml: mode: localization

map_file_name: /home/USER_NAME/dev_ws/my_map_serial


map_start_at_dock: true

Terminal 4: Localiztion
```terminal
source install/setup.bash
ros2 launch rc_car localization.launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true
```
step 1: in rviz, manually write "map" in fixed frame

step 2: change set initial position

step 3: add map

step 4: topic->durability policy->transient local

# How to use - Mapped environment:


Terminal 1: General simulation loading
```terminal
source install/setup.bash
ros2 launch rc_car launch_sim.launch.py 
```

Terminal 2: Rviz
```terminal
source install/setup.bash
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/localization.rviz --ros-args -p use_sim_time:=true
```

Terminal 3: Localization
```terminal
source install/setup.bash
ros2 launch rc_car localization.launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true
```

Choose path planning algorithm:
Terminal 4: Path Planning service - AStar
```terminal
source install/setup.bash
ros2 run rc_car ros_PathPlanning_service_astar.py --ros-args -p map_name:=sim_map -p use_sime_time:=true
```
OR
Terminal 4: Path Planning service - PRM
```terminal
source install/setup.bash
ros2 run rc_car ros_PathPlanning_service_prm.py --ros-args -p map_name:=sim_map -p use_sime_time:=true
```

Terminal 5: Path Planning client
```terminal
source install/setup.bash
ros2 run rc_car ros_PathPlanning_client.py 0 0 6.22 -4.5 path4 --ros-args -p use_sim_time:=true 
```
Terminal 6: Path Tracking
```terminal
source install/setup.bash
ros2 run rc_car ros_Path_Tracking.py --ros-args -p use_sime_time:=true -p show_path:=true -p show_marker:=true -p path_name:=path4
```



### Convert Map
```terminal
Get map as numpy array: 
ros2 run rc_car ros_Get_map_client.py --ros-args -p map_name:=sim_map
```


# localization usnig optitrack
```terminal
ros2 launch rc_car optitrack_map_server.launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true
```
```terminal
ros2 run rc_car ros_optitrack_map_to_odom_publisher.py --ros-args use_sim_time:=true
```






<!-- git clone -b humble https://github.com/ros-controls/gazebo_ros2_control -->