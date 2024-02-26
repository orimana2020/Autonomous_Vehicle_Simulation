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
ros2 launch rc_car launch_sim_bicycle.launch.py 

TERMINAL 2 :
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/main.rviz --ros-args -p use_sim_time:=true

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
ros2 launch rc_car localization_launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true

step 1: in rviz, manually write "map" in fixed frame
step 2: change set initial position
step 3: add map
step 4: topic->durability policy->transient local

TERMINAL 4:
ros2 launch rc_car navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true

step 1:add map
step 2: topic: /global_costmap, color scheme: cost_map


----------------------------- bicycle simulation with mapped environment

ros2 launch rc_car launch_sim_bicycle.launch.py 
ros2 launch rc_car localization_launch.py map:=./src/rc_car/maps/maze_1/maze1_map.yaml use_sim_time:=true
ros2 run rviz2 rviz2 -d src/rc_car/rviz_config/rviz_map_maze1.rviz --ros-args -p use_sim_time:=true




# --------------------- Real Robot --------------------

ros2 launch rc_car launch_robot.launch.py
ros2 launch rc_car rplidar.launch.py 

mapping:
ros2 launch rc_car online_async_launch.py use_sim_time:=false
save the map with rviz, add new panel->slam tool box plugin, serialize the map

localiztion:
ros2 launch rc_car localization_launch.py map:=my_lab3.yaml use_sim_time:=false

navigation:



---------- General Notes ---------------------
burn to arduino: https://github.com/joshnewans/ros_arduino_bridge.git

download to src file at the PI:
https://github.com/joshnewans/diffdrive_arduino/tree/humble
https://github.com/joshnewans/serial


find path to some serial device:
ls /dev/serial/by-path/

ros2 ros2_control list_hardware_interfaces


how to remap:
ros2 run teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstampted


low level control packages:
1. ros_arduino_bridge - burn to arduino
pyserial-miniterm -e /dev/ttyACM0 57600
with o - open loop, e- encoders, m-closed loop
run in terminal 
pyserial-miniterm -e /dev/ttyACM0 57600


arduino ros bridge:
https://github.com/joshnewans/ros_arduino_bridge.git

add permission to arduino: not needed because of dialout
sudo chmod a+rw /dev/ttyACM0


REAL ROBOT NOTES
download imager to burn OS to PI
download ubuntu mate for PI 64 from https://ubuntu-mate.org/download/


* CRITICAL*
after installtion update TIME and DATE before sudo apt update and upgrade

install ros:https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
        -> skip: set locale
        -> setup sources
        -> install ros2 packages , desktop version
