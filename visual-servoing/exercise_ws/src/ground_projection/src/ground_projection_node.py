#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import os
import yaml
import numpy as np

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from image_processing.ground_projection_geometry import Point, GroundProjectionGeometry
from image_processing.rectification import Rectify
from image_geometry import PinholeCameraModel

from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Point as PointMsg
from duckietown_msgs.msg import Segment, SegmentList
from custom_msgs.msg import Pixel, PixelList, PixelListList
np.set_printoptions(precision=3, suppress=True)

class GroundProjectionNode(DTROS):
    """
    This node projects the line segments detected in the image to the ground plane and in the robot's reference frame.
    In this way it enables lane localization in the 2D ground plane. This projection is performed using the homography
    matrix obtained from the extrinsic calibration procedure.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Subscribers:
        ~camera_info (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera. Needed for rectifying the segments.
        ~lineseglist_in (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in pixel space from unrectified images

    Publishers:
        ~lineseglist_out (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in the ground plane relative to the robot origin
        ~debug/ground_projection_image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug image that shows the robot relative to the projected segments. Useful to check if the extrinsic calibration is accurate.
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(GroundProjectionNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        self.bridge = CvBridge()
        self.ground_projector = None
        self.rectifier = None
        self.homography = self.load_extrinsics()
        self.first_processing_done = False
        self.camera_info_received = False

        # subscribers
        self.sub_camera_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_lineseglist_ = rospy.Subscriber("~lineseglist_in", SegmentList, self.lineseglist_cb, queue_size=1)
        self.sub_color_coordinates = rospy.Subscriber("/agent/line_detector_node/color_coordinates", PixelListList,
                                                      self.colorcoordinates_cb, queue_size=1)
        self.sub_color_coordinates_checkpoint = rospy.Subscriber(
            "/agent/line_detector_node/color_coordinates_checkpoint", PixelListList,
            self.colorcoordinates_checkpoint_cb, queue_size=1)

        self.sub_vs_matching_points = rospy.Subscriber(
            "/agent/line_detector_node/vs_match_points", PixelList,
            self.matching_points_cb, queue_size=1)

        # publishers
        self.pub_lineseglist = rospy.Publisher("~lineseglist_out",
                                               SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_groundpixels = rospy.Publisher("~color_coordinates_ground",
                                                PixelListList, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_groundpixels_checkpoint = rospy.Publisher("~color_coordinates_ground_checkpoint",
                                                           PixelListList, queue_size=1,
                                                           dt_topic_type=TopicType.PERCEPTION)
        self.pub_matching_points = rospy.Publisher("~vs_match_points_ground",
                                                   PixelList, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_debug_img = rospy.Publisher("~debug/ground_projection_image/compressed",
                                             CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG)

        self.bridge = CvBridge()

        self.debug_img_bg = None

        # Seems to be never used:
        # self.service_homog_ = rospy.Service("~estimate_homography", EstimateHomography, self.estimate_homography_cb)
        # self.service_gnd_coord_ = rospy.Service("~get_ground_coordinate", GetGroundCoord, self.get_ground_coordinate_cb)
        # self.service_img_coord_ = rospy.Service("~get_image_coordinate", GetImageCoord, self.get_image_coordinate_cb)

    def cb_camera_info(self, msg):
        """
        Initializes a :py:class:`image_processing.GroundProjectionGeometry` object and a
        :py:class:`image_processing.Rectify` object for image rectification

        Args:
            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.

        """
        if not self.camera_info_received:
            self.rectifier = Rectify(msg)
            self.ground_projector = GroundProjectionGeometry(im_width=msg.width,
                                                             im_height=msg.height,
                                                             homography=np.array(self.homography).reshape((3, 3)))
        self.camera_info_received = True

    def colorcoordinates_cb(self, pixel_list_list_msg):
        msg = self.get_ground_projected_msg(pixel_list_list_msg)
        self.pub_groundpixels.publish(msg)

    def colorcoordinates_checkpoint_cb(self, pixel_list_list_msg):
        msg = self.get_ground_projected_msg(pixel_list_list_msg)
        self.pub_groundpixels_checkpoint.publish(msg)

    def matching_points_cb(self, pixel_list_msg):
        msg = self.get_ground_projected_msg(pixel_list_msg)
        self.pub_matching_points.publish(msg)

    def get_ground_projected_msg_list(self, pixel_list_msg):
        ground_pixel_list = []
        for pixel in pixel_list_msg.pixels:
            ground_point = self.pixel_msg_to_ground_msg(pixel)
            ground_pixel = Pixel()
            ground_pixel.x, ground_pixel.y = ground_point.x, ground_point.y
            ground_pixel_list.append(ground_pixel)
        pixel_list_msg = PixelList()
        pixel_list_msg.pixels = ground_pixel_list
        return pixel_list_msg

    def get_ground_projected_msg(self, pixel_list_list_msg):
        pixel_list_list = pixel_list_list_msg.pixel_lists
        ground_pixel_list_list = []
        for i, pixel_list in enumerate(pixel_list_list):
            ground_pixel_list = []
            raw_image_pixels = []
            raw_ground_pixels = []
            for pixel in pixel_list.pixels:
                ground_point = self.pixel_msg_to_ground_msg(pixel)
                ground_pixel = Pixel()
                ground_pixel.x, ground_pixel.y = ground_point.x, ground_point.y
                ground_pixel_list.append(ground_pixel)
                raw_image_pixels.append((pixel.x, pixel.y))
                raw_ground_pixels.append((ground_pixel.x, ground_pixel.y))
            raw_image_pixels = np.array(raw_image_pixels)
            raw_ground_pixels = np.array(raw_ground_pixels)
            # print(
            #     f"{i} Image Space: {raw_image_pixels.mean(axis=0), raw_image_pixels.std(axis=0)} Ground Space: {raw_ground_pixels.mean(axis=0), raw_ground_pixels.std(axis=0)}")
            pixel_list_msg = PixelList()
            pixel_list_msg.pixels = ground_pixel_list
            ground_pixel_list_list.append(pixel_list_msg)
        msg = PixelListList()
        msg.pixel_lists = ground_pixel_list_list
        return msg

    def pixel_msg_to_ground_msg(self, point_msg):
        """
        Creates a :py:class:`ground_projection.Point` object from a normalized point message from an unrectified
        image. It converts it to pixel coordinates and rectifies it. Then projects it to the ground plane and
        converts it to a ROS Point message.

        Args:
            point_msg (:obj:`geometry_msgs.msg.Point`): Normalized point coordinates from an unrectified image.

        Returns:
            :obj:`geometry_msgs.msg.Point`: Point coordinates in the ground reference frame.

        """
        # normalized coordinates to pixel:
        norm_pt = Point.from_message(point_msg)
        pixel = self.ground_projector.vector2pixel(norm_pt)
        # rectify
        rect = self.rectifier.rectify_point(pixel)
        # convert to Point
        rect_pt = Point.from_message(rect)
        # project on ground
        ground_pt = self.ground_projector.pixel2ground(rect_pt)
        # point to message
        ground_pt_msg = PointMsg()
        ground_pt_msg.x = ground_pt.x
        ground_pt_msg.y = ground_pt.y
        ground_pt_msg.z = ground_pt.z

        return ground_pt_msg

    def lineseglist_cb(self, seglist_msg):
        """
        Projects a list of line segments on the ground reference frame point by point by
        calling :py:meth:`pixel_msg_to_ground_msg`. Then publishes the projected list of segments.

        Args:
            seglist_msg (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in pixel space from unrectified images

        """
        if self.camera_info_received:
            seglist_out = SegmentList()
            seglist_out.header = seglist_msg.header
            for received_segment in seglist_msg.segments:
                new_segment = Segment()
                new_segment.points[0] = self.pixel_msg_to_ground_msg(received_segment.pixels_normalized[0])
                new_segment.points[1] = self.pixel_msg_to_ground_msg(received_segment.pixels_normalized[1])
                new_segment.color = received_segment.color
                # TODO what about normal and points
                seglist_out.segments.append(new_segment)
            self.pub_lineseglist.publish(seglist_out)

            if not self.first_processing_done:
                self.log('First projected segments published.')
                self.first_processing_done = True

            if self.pub_debug_img.get_num_connections() > 0:
                debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.debug_image(seglist_out))
                debug_image_msg.header = seglist_out.header
                self.pub_debug_img.publish(debug_image_msg)
        else:
            self.log('Waiting for a CameraInfo message', 'warn')

    # def get_ground_coordinate_cb(self, req):
    #     return GetGroundCoordResponse(self.pixel_msg_to_ground_msg(req.uv))
    #
    # def get_image_coordinate_cb(self, req):
    #     return GetImageCoordResponse(self.gpg.ground2pixel(req.gp))
    #
    # def estimate_homography_cb(self, req):
    #     rospy.loginfo("Estimating homography")
    #     rospy.loginfo("Waiting for raw image")
    #     img_msg = rospy.wait_for_message("/" + self.robot_name + "/camera_node/image/raw", Image)
    #     rospy.loginfo("Got raw image")
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr(e)
    #     self.gp.estimate_homography(cv_image)
    #     rospy.loginfo("wrote homography")
    #     return EstimateHomographyResponse()

    def load_extrinsics(self):
        """
        Loads the homography matrix from the extrinsic calibration file.

        Returns:
            :obj:`numpy array`: the loaded homography matrix

        """
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_extrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                     % cali_file, 'warn')
            cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = 'Found no calibration file ... aborting'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        try:
            with open(cali_file, 'r') as stream:
                calib_data = yaml.load(stream)
        except yaml.YAMLError:
            msg = 'Error in parsing calibration file %s ... aborting' % cali_file
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        return calib_data['homography']

    def debug_image(self, seg_list):
        """
        Generates a debug image with all the projected segments plotted with respect to the robot's origin.

        Args:
            seg_list (:obj:`duckietown_msgs.msg.SegmentList`): Line segments in the ground plane relative to the robot origin

        Returns:
            :obj:`numpy array`: an OpenCV image

        """
        # dimensions of the image are 1m x 1m so, 1px = 2.5mm
        # the origin is at x=200 and y=300

        # if that's the first call, generate the background
        if self.debug_img_bg is None:

            # initialize gray image
            self.debug_img_bg = np.ones((400, 400, 3), np.uint8) * 128

            # draw vertical lines of the grid
            for vline in np.arange(40, 361, 40):
                cv2.line(self.debug_img_bg,
                         pt1=(vline, 20),
                         pt2=(vline, 300),
                         color=(255, 255, 0),
                         thickness=1)

            # draw the coordinates
            cv2.putText(self.debug_img_bg, "-20cm", (120 - 25, 300 + 15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, "  0cm", (200 - 25, 300 + 15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, "+20cm", (280 - 25, 300 + 15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

            # draw horizontal lines of the grid
            for hline in np.arange(20, 301, 40):
                cv2.line(self.debug_img_bg,
                         pt1=(40, hline),
                         pt2=(360, hline),
                         color=(255, 255, 0),
                         thickness=1)

            # draw the coordinates
            cv2.putText(self.debug_img_bg, "20cm", (2, 220 + 3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)
            cv2.putText(self.debug_img_bg, " 0cm", (2, 300 + 3), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 0), 1)

            # draw robot marker at the center
            cv2.line(self.debug_img_bg,
                     pt1=(200 + 0, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

            cv2.line(self.debug_img_bg,
                     pt1=(200 + 20, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

            cv2.line(self.debug_img_bg,
                     pt1=(200 - 20, 300 - 20),
                     pt2=(200 + 0, 300 + 0),
                     color=(255, 0, 0),
                     thickness=1)

        # map segment color variables to BGR colors
        color_map = {Segment.WHITE: (255, 255, 255),
                     Segment.RED: (0, 0, 255),
                     Segment.YELLOW: (0, 255, 255)}

        image = self.debug_img_bg.copy()

        # plot every segment if both ends are in the scope of the image (within 50cm from the origin)
        for segment in seg_list.segments:
            if not np.any(np.abs([segment.points[0].x, segment.points[0].y,
                                  segment.points[1].x, segment.points[1].y]) > 0.50):
                cv2.line(image,
                         pt1=(int(segment.points[0].y * -400) + 200, int(segment.points[0].x * -400) + 300),
                         pt2=(int(segment.points[1].y * -400) + 200, int(segment.points[1].x * -400) + 300),
                         color=color_map.get(segment.color, (0, 0, 0)),
                         thickness=1)

        return image


if __name__ == '__main__':
    ground_projection_node = GroundProjectionNode(node_name='ground_projection')
    rospy.spin()
