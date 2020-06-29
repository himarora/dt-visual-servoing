#!/usr/bin/env python
import os.path

from duckietown_utils import get_duckiefleet_root
from pi_camera.camera_info import load_camera_info_2
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CamInfoReader(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        # Load parameters
        # TODO cali_file_name is not needed and should be the robot name by default
        self.cali_file_name = self.setupParam("~cali_file_name", "default")
        self.image_type = self.setupParam("~image_type", "compressed")

        # Setup publisher
        self.pub_camera_info = rospy.Publisher(
            "~camera_info", CameraInfo, queue_size=1)

        # Get path to calibration yaml file
        self.cali_file = (get_duckiefleet_root() + "/calibrations/camera_intrinsic/"
                          + self.cali_file_name + ".yaml")
        self.camera_info_msg = None

        # Load calibration yaml file
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead."
                          % (self.node_name, self.cali_file))
            self.cali_file = (get_duckiefleet_root() +
                              "/calibrations/camera_intrinsic/default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" %
                      (self.node_name, self.cali_file))
        self.camera_info_msg = load_camera_info_2(self.cali_file)
        self.camera_info_msg.header.frame_id = rospy.get_namespace() + \
            "camera_optical_frame"
        rospy.loginfo("[%s] CameraInfo: %s" %
                      (self.node_name, self.camera_info_msg))
        # self.timer_pub = rospy.Timer(rospy.Duration.from_sec(1.0/self.pub_freq),self.cbTimer)

        img_type = CompressedImage if self.image_type == "compressed" else Image
        typemsg = "CompressedImage" if self.image_type == "compressed" else "Image"
        rospy.logwarn("[%s] ==============%s", self.node_name, typemsg)
        self.sub_img_compressed = rospy.Subscriber(
            "~compressed_image", img_type, self.cbCompressedImage, queue_size=1)

    def cbCompressedImage(self, msg):
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = msg.header.stamp
            self.pub_camera_info.publish(self.camera_info_msg)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('cam_info_reader', anonymous=False)
    node = CamInfoReader()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
