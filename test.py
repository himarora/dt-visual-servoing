import os
import rospy
# import roslaunch
from rosagent import ROSAgent
import subprocess
import time

# Now, initialize the ROS stuff here:
print("roscore")
roscore = subprocess.Popen(["roscore"], shell=True)
time.sleep(3)

print("roslaunch")
roslaunch = subprocess.Popen(["roslaunch lf_slim.launch"], shell=True)