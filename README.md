# Autonomous_RC
installtion for simulation

install ros2 humble - see documentation 

sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-joint-state-publisher

sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control

sudo apt install joystick jstest-gtk evtest 

sudo apt install ros-humble-slam-toolbox

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*

sudo apt install ros-humble-twist-mux 

sudo apt install ros-humble-twist-stamper

git clone -b humble https://github.com/ros-controls/gazebo_ros2_control

sudo apt-get install python3-pip
pip install setuptools==58.2.0




# ------------ ROS BICYCLE SIMULATION ------------------

TERMINAL 1 :
source install/setup.bash
ros2 launch rc_car launch_sim_bicycle.launch.py 

TERMINAL 2 :
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/main.rviz --ros-args -p use_sim_time:=true

-or when use localization-
source install/setup.bash
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/localization.rviz --ros-args -p use_sim_time:=true


# mapping
terminal 3: slam toolbox (mapping)
add map to rviz
ros2 launch rc_car online_async_launch.py use_sim_time:=true
save the map with rviz, add new panel->slam tool box plugin, serialize the map
reset the simulation, 
change the mapper params online async.yaml:
mode: localization
map_file_name: /home/ori/dev_ws/my_map_serial
map_start_at_dock: true

need to copy the localizion_launch from slam_toolbox and rerun the simulation
this is not the same localization_launch from Nav2 package(in this repo)

# localiztion
TERMINAL 3 :
source install/setup.bash
ros2 launch rc_car localization_launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true
step 1: in rviz, manually write "map" in fixed frame
step 2: change set initial position
step 3: add map
step 4: topic->durability policy->transient local

TERMINAL 4:
source install/setup.bash
ros2 launch rc_car navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
step 1:add map
step 2: topic: /global_costmap, color scheme: cost_map


----------------------------- bicycle simulation with mapped environment

ros2 launch rc_car launch_sim_bicycle.launch.py 
ros2 launch rc_car localization_launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/rviz_map_maze1.rviz --ros-args -p use_sim_time:=true



