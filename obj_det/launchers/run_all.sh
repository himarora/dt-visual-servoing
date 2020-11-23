#!/bin/bash
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/exercise_ws/devel/setup.bash
pip3 install torch && pip3 install torchvision && pip3 uninstall dataclasses -y #FIXME this is not the way
python3 /code/solution.py &
roslaunch --wait obj_det obj_det_node.launch veh:=$VEHICLE_NAME &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
roslaunch --wait duckietown_demos lane_following.launch
