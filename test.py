import os
import rospy
import roslaunch
from rosagent import ROSAgent
import subprocess
import time

print("roslaunch")

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
roslaunch_path = os.path.join(os.getcwd(), "lf_slim.launch")
launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_path])
launch.start()