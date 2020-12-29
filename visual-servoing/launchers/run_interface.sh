#!/bin/bash

source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/exercise_ws/devel/setup.bash
pip3 install dubins
python3 /code/solution.py &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
