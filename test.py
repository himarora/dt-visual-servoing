import os
import rospy
import roslaunch
from rosagent import ROSAgent
import subprocess
import time

class Test():
    def __init__(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_path = os.path.join(os.getcwd(), "lf_slim.launch")
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_path])
        self.launch.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.launch.shutdown()

if __name__ == '__main__':
    t = Test()
    
    while not rospy.is_shutdown():
        print("hi!")
        time.sleep(1)

    t.shutdown()