#!/usr/bin/env python3

import numpy as np
from math import atan
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList, AntiInstagramThresholds, BoolStamped
from std_msgs.msg import String
from line_detector import LineDetector, ColorRange, plotSegments, plotMaps
from image_processing.anti_instagram import AntiInstagram
from custom_msgs.msg import Pixel, PixelList, PixelListList, FloatList
from duckietown.dtros import DTROS, NodeType, TopicType


class LineDetectorNode(DTROS):
    """
    The ``LineDetectorNode`` is responsible for detecting the line white, yellow and red line segment in an image and
    is used for lane localization.

    Upon receiving an image, this node reduces its resolution, cuts off the top part so that only the
    road-containing part of the image is left, extracts the white, red, and yellow segments and publishes them.
    The main functionality of this node is implemented in the :py:class:`line_detector.LineDetector` class.

    The performance of this node can be very sensitive to its configuration parameters. Therefore, it also provides a
    number of debug topics which can be used for fine-tuning these parameters. These configuration parameters can be
    changed dynamically while the node is running via ``rosparam set`` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~line_detector_parameters (:obj:`dict`): A dictionary with the parameters for the detector. The full list can be found in :py:class:`line_detector.LineDetector`.
        ~colors (:obj:`dict`): A dictionary of colors and color ranges to be detected in the image. The keys (color names) should match the ones in the Segment message definition, otherwise an exception will be thrown! See the ``config`` directory in the node code for the default ranges.
        ~img_size (:obj:`list` of ``int``): The desired downsized resolution of the image. Lower resolution would result in faster detection but lower performance, default is ``[120,160]``
        ~top_cutoff (:obj:`int`): The number of rows to be removed from the top of the image _after_ resizing, default is 40

    Subscriber:
        ~camera_node/image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): The camera images
        ~anti_instagram_node/thresholds(:obj:`duckietown_msgs.msg.AntiInstagramThresholds`): The thresholds to do color correction

    Publishers:
        ~segment_list (:obj:`duckietown_msgs.msg.SegmentList`): A list of the detected segments. Each segment is an :obj:`duckietown_msgs.msg.Segment` message
        ~debug/segments/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the segments drawn on the input image
        ~debug/edges/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the Canny edges drawn on the input image
        ~debug/maps/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the regions falling in each color range drawn on the input image
        ~debug/ranges_HS (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the input image and the color ranges, Hue-Saturation projection
        ~debug/ranges_SV (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the input image and the color ranges, Saturation-Value projection
        ~debug/ranges_HV (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the input image and the color ranges, Hue-Value projection

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LineDetectorNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        # Define parameters
        self._line_detector_parameters = rospy.get_param('~line_detector_parameters', None)
        self._colors = rospy.get_param('~colors', None)
        self._img_size = rospy.get_param('~img_size', None)
        self._top_cutoff = rospy.get_param('~top_cutoff', None)

        self.bridge = CvBridge()

        # The thresholds to be used for AntiInstagram color correction
        self.ai_thresholds_received = False
        self.anti_instagram_thresholds = dict()
        self.ai = AntiInstagram()

        # This holds the colormaps for the debug/ranges images after they are computed once
        self.colormaps = dict()

        # Create a new LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self._line_detector_parameters)
        # Update the color ranges objects
        self.color_ranges = {
            color: ColorRange.fromDict(d)
            for color, d in self._colors.items()
        }

        # Publishers
        self.pub_lines = rospy.Publisher(
            "~segment_list", SegmentList, queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )
        self.pub_d_segments = rospy.Publisher(
            "~debug/segments/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_edges = rospy.Publisher(
            "~debug/edges/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_maps = rospy.Publisher(
            "~debug/maps/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_d_color_coordinates_img = rospy.Publisher(
            "~debug/color_coordinates/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_d_color_coordinates_img_checkpoint = rospy.Publisher(
            "~debug/color_coordinates_checkpoint/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_d_checkpoint_image = rospy.Publisher(
            "/agent/line_detector_node/debug/checkpoint/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        # these are not compressed because compression adds undesired blur
        self.pub_d_ranges_HS = rospy.Publisher(
            "~debug/ranges_HS", Image, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_ranges_SV = rospy.Publisher(
            "~debug/ranges_SV", Image, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_ranges_HV = rospy.Publisher(
            "~debug/ranges_HV", Image, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )
        self.pub_color_coordinates = rospy.Publisher(
            "~color_coordinates", PixelListList, queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_color_coordinates_checkpoint = rospy.Publisher(
            "~color_coordinates_checkpoint", PixelListList, queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_d_vs_lines = rospy.Publisher(
            "/agent/line_detector_node/debug/vs_lines/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_d_vs_lines_checkpoint = rospy.Publisher(
            "/agent/line_detector_node/debug/vs_lines_checkpoint/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_d_vs_lines_all = rospy.Publisher(
            "/agent/line_detector_node/debug/vs_lines_all/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_d_vs_lines_checkpoint_all = rospy.Publisher(
            "/agent/line_detector_node/debug/vs_lines_checkpoint_all/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        self.pub_homography = rospy.Publisher(
            "/agent/line_detector_node/homography", FloatList, queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_d_vs_pose_img = rospy.Publisher(
            "/agent/line_detector_node/vs_target_pose/compressed", CompressedImage, queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        # Subscribers
        self.sub_image = rospy.Subscriber(
            "~image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1
        )

        self.sub_thresholds = rospy.Subscriber(
            "~thresholds",
            AntiInstagramThresholds,
            self.thresholds_cb,
            queue_size=1
        )

        self.sub_color_coordinates_ground = rospy.Subscriber(
            "/agent/ground_projection_node/color_coordinates_ground",
            PixelListList,
            self.color_coordinates_ground_cb,
            queue_size=1
        )
        self.sub_color_coordinates_checkpoint_ground = rospy.Subscriber(
            "/agent/ground_projection_node/color_coordinates_ground_checkpoint",
            PixelListList,
            self.color_coordinates_checkpoint_ground_cb,
            queue_size=1
        )

        self.sub_d_vs_pose = rospy.Subscriber(
            "/agent/lane_filter_node/debug/vs_pose",
            FloatList,
            self.vs_lane_pose_cb,
            queue_size=1
        )

        self.sub_key_press = rospy.Subscriber("/agent/line_detector_node/key_pressed", String, self.cb_key_pressed,
                                              queue_size=1)
        self.checkpoint_image = None
        self.current_image = None
        self.current_lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        self.checkpoint_lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        self.current_lines_cv = [[None] * 4, [None] * 4, [None] * 4]  # WYR
        self.checkpoint_lines_cv = [[None] * 4, [None] * 4, [None] * 4]  # WYR
        self.current_lines_par = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        self.checkpoint_lines_par = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        self.current_lines_cv_all = [[], [], []]
        self.checkpoint_lines_cv_all = [[], [], []]
        self.current_ground_color_coordinates = []
        self.checkpoint_ground_color_coordinates = []
        self.img_shape = None
        self.H = None
        self.debug_img_bg = None

        self.arr_cutoff = np.array([
            self._top_cutoff, self._top_cutoff
        ])
        self.arr_ratio = np.array([
            1. / self._img_size[1],
            1. / self._img_size[0]
        ])

    def thresholds_cb(self, thresh_msg):
        self.anti_instagram_thresholds["lower"] = thresh_msg.low
        self.anti_instagram_thresholds["higher"] = thresh_msg.high
        self.ai_thresholds_received = True

    def prepare_image(self, image_msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        # Perform color correction
        if self.ai_thresholds_received:
            image = self.ai.apply_color_balance(
                self.anti_instagram_thresholds["lower"],
                self.anti_instagram_thresholds["higher"],
                image
            )

        # Resize the image to the desired dimensions
        height_original, width_original = image.shape[0:2]
        img_size = (self._img_size[1], self._img_size[0])
        if img_size[0] != width_original or img_size[1] != height_original:
            image = cv2.resize(image, img_size, interpolation=cv2.INTER_NEAREST)
        image = image[self._top_cutoff:, :, :]
        return image

    def cb_key_pressed(self, msg):
        """
        Saves checkpoint image and calls the method to extract color coordinates
        :param msg: Dummy message
        :return: None
        """
        print("Saved checkpoint")
        self.checkpoint_image = self.current_image
        self.publish_color_coordinates(self.checkpoint_image, checkpoint=True)

    @staticmethod
    def get_abc(x, y, vx, vy):
        a = 1 / ((vx / vy) * y - x)
        b = - vx / (vy * (vx / vy * y - x))
        c = 1
        return a[0], b[0], c

    @staticmethod
    def pixels_to_pixel_list_msg(pixels):
        pixel_msgs = []
        for pixel in pixels:
            pixel_msg = Pixel()
            pixel_msg.x, pixel_msg.y = pixel
            pixel_msgs.append(pixel_msg)
        pixel_list = PixelList()
        pixel_list.pixels = pixel_msgs
        return pixel_list

    @staticmethod
    def pixel_list_msg_to_pixels(pixel_list_msg, dist_thres=5.):
        pixel_list = pixel_list_msg.pixels
        pixels = []
        for pixel in pixel_list:
            point = [pixel.x, pixel.y]
            if np.sqrt(point[0] ** 2 + point[1] ** 2) < dist_thres:
                pixels.append([point[0], point[1]])
        return np.array(pixels, np.float32)

    def color_coordinates_msg_to_pixels(self, color_coordinates_msg):
        pixel_lists = color_coordinates_msg.pixel_lists
        pixel_list_list = []
        for pixel_l in pixel_lists:
            pixels = self.pixel_list_msg_to_pixels(pixel_l, dist_thres=0.5)
            pixel_list_list.append(pixels)
        return pixel_list_list

    @staticmethod
    def get_intersections(a_0, b_0, c_0, a_1, b_1, c_1):
        M = np.array([[a_0, b_0],
                      [a_1, b_1]]).astype(float).reshape(2, 2)
        b = np.array([[-c_0],
                      [-c_1]]).astype(float)
        pt_intercept = np.linalg.inv(M) @ b
        return pt_intercept

    @staticmethod
    def compute_homography(lines1, lines2):
        a_white_0, b_white_0, c_white_0 = lines1[0]
        a_yellow_0, b_yellow_0, c_yellow_0 = lines1[1]
        a_red_0, b_red_0, c_red_0 = lines1[2]
        a_white_1, b_white_1, c_white_1 = lines2[0]
        a_yellow_1, b_yellow_1, c_yellow_1 = lines2[1]
        a_red_1, b_red_1, c_red_1 = lines2[2]

        try:
            get_slope = lambda l: -l[0] / l[1]
            white_l_0 = np.array((a_white_0, b_white_0, c_white_0))
            white_l_1 = np.array((a_white_1, b_white_1, c_white_1))

            theta = atan(get_slope(white_l_1)) - atan(get_slope(white_l_0))
            R = np.array([[np.cos(theta), -np.sin(theta), 0],
                          [np.sin(theta), np.cos(theta), 0],
                          [0, 0, 1]])

            int_white_red_0 = LineDetectorNode.get_intersections(a_white_0, b_white_0, c_white_0,
                                                                 a_red_0, b_red_0, c_red_0)
            int_white_red_1 = LineDetectorNode.get_intersections(a_white_1, b_white_1, c_white_1,
                                                                 a_red_1, b_red_1, c_red_1)
            t = int_white_red_1 - int_white_red_0
            T = np.array([[1, 0, t[0]],
                          [0, 1, t[1]],
                          [0, 0, 1]], dtype='float')
            A = T @ R
        except TypeError:
            A = None
        return A

        ## Approach 1
        # lines = np.array(
        #     [[a_white_0, b_white_0, c_white_0],
        #      [a_red_0, b_red_0, c_red_0],
        #      [a_yellow_0, b_yellow_0, c_yellow_0]]
        # ).astype(float)
        # b = np.array(
        #     [a_white_1, a_red_1, a_yellow_1, b_white_1, b_red_1, b_yellow_1, c_white_1, c_red_1, c_yellow_1]).astype(
        #     float)
        # A = np.zeros((9, 9), dtype=float)
        # A[:3, :3] = A[3:6, 3:6] = A[6:, 6:] = lines
        # H_prime = np.linalg.inv(A) @ b
        # H_prime = H_prime.reshape(-1) / H_prime[8]
        # H_prime = np.array(
        #     [[H_prime[0], H_prime[1], H_prime[2]],
        #      [H_prime[3], H_prime[4], H_prime[5]],
        #      [H_prime[6], H_prime[7], H_prime[8]]
        #      ])
        # H = np.linalg.inv(H_prime).T
        # H = np.linalg.pinv(A) @ b
        # return H

        ## Approach 2
        # A = np.array([
        #     [a_white_1, 0, 0, b_white_1, 0, 0, c_white_1, 0, 0],
        #     [a_red_1, 0, 0, b_red_1, 0, 0, c_red_1, 0, 0],
        #     [a_yellow_1, 0, 0, b_yellow_1, 0, 0, c_yellow_1, 0, 0],
        #     [0, a_white_1, 0, 0, b_white_1, 0, 0, c_white_1, 0],
        #     [0, a_red_1, 0, 0, b_red_1, 0, 0, c_red_1, 0],
        #     [0, a_yellow_1, 0, 0, b_yellow_1, 0, 0, c_yellow_1, 0]
        # ], dtype='float')
        #
        # b = np.array([a_white_0,
        #               a_red_0,
        #               a_yellow_0,
        #               b_white_0,
        #               b_red_0,
        #               b_yellow_0
        #               ], dtype='float')
        # try:
        #     C = np.linalg.pinv(A) @ b
        #     affine = np.array(
        #         [[C[0], C[1], C[2]],
        #          [C[3], C[4], C[5]],
        #          [0, 0, 1]]
        #     )
        #     return affine
        # except np.linalg.LinAlgError:
        #     return None

    def vs_lane_pose_cb(self, msg):
        theta, p1, p2 = msg.H
        x = int((-p2 * -400) + 200)
        y = int((-p1 * -400) + 300)
        theta *= -1.
        self.init_debug_bg_img()
        image = self.debug_img_bg.copy()
        arrow_len = 30
        arrow_end = (int(x + arrow_len * np.sin(theta)), int((y + arrow_len * np.cos(theta))))
        cv2.arrowedLine(image, (x, y), arrow_end, color=(0, 0, 255), thickness=5, tipLength=0.5)
        debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        if self.pub_d_vs_pose_img.get_num_connections() > 0:
            self.pub_d_vs_pose_img.publish(debug_image_msg)

    @staticmethod
    def parallelize_lines(lines, tol=0.02, step_size=0.01, max_iter=500, dist_wy=0.3):
        lin_w = lines[0]
        lin_y = lines[1]
        lin_r = lines[2]
        # Find intersection between white and red line
        x_wr = (lin_r[2] / lin_r[1] - lin_w[2] / lin_w[1]) / (lin_w[0] / lin_w[1] - lin_r[0] / lin_r[1])
        y_wr = -lin_w[0] / lin_w[1] * x_wr - lin_w[2] / lin_w[1]
        # Find intersection between yellow and red line
        x_yr = (lin_r[2] / lin_r[1] - lin_y[2] / lin_y[1]) / (lin_y[0] / lin_y[1] - lin_r[0] / lin_r[1])
        y_yr = -lin_y[0] / lin_y[1] * x_yr - lin_y[2] / lin_y[1]
        # Find midpoint on red line
        x_r_mid = (x_yr + x_wr) / 2
        y_r_mid = (y_yr + y_wr) / 2
        # Get initial slopes
        m_w = -lin_w[0] / lin_w[1]
        m_y = -lin_y[0] / lin_y[1]
        m_r = -lin_r[0] / lin_r[1]
        # Bias the closer line to change less during parallel enforcement
        m_r_pen = -(1 / m_r)
        dist_m_wr = abs(m_w - m_r_pen)
        dist_m_yr = abs(m_y - m_r_pen)
        bias_w = 1.0
        bias_y = 1.0
        # Check white line status
        if dist_m_wr < 0.3 or dist_m_yr > dist_m_wr * 2:
            bias_w = 0.1
        if dist_m_yr < 0.3 or dist_m_wr > dist_m_yr * 2:
            bias_y = 0.1

        # Equate the slopes for white and yellow line
        lin_w_new = list(lin_w)
        lin_y_new = list(lin_y)
        lin_r_new = list(lin_r)
        ii = 0
        while abs(m_w - m_y) > tol and ii < max_iter:
            # print(m_w)
            # print(m_y)
            sign = 1
            if m_w < m_y:
                sign = -1
            m_w = m_w - sign * bias_w * step_size
            m_y = m_y + sign * bias_y * step_size
            lin_w_new[1] = lin_w_new[1] + bias_w * step_size
            lin_y_new[1] = lin_y_new[1] - bias_y * step_size
            if lin_w_new[1] > 50:
                lin_w_new[1] = -lin_w_new[1]
            if lin_y_new[1] > 50:
                lin_y_new[1] = -lin_y_new[1]
            ii += 1

        sign_m_w = m_w / abs(m_w)
        sign_m_y = m_y / abs(m_y)
        m_w = sign_m_w * max(abs(m_w), 0.01)  # Cap slope to not blow up at 0
        m_y = sign_m_y * max(abs(m_y), 0.01)  # Cap slope to not blow up at 0
        lin_w_new[1] = -lin_w_new[0] / m_w
        lin_y_new[1] = -lin_y_new[0] / m_y

        # Have both white and yellow line meet red line at same point as before
        lin_w_new[2] = -(y_wr - m_w * x_wr) * lin_w_new[1]
        lin_y_new[2] = -(y_yr - m_y * x_yr) * lin_y_new[1]
        # Make red line perpendicular to other 2 lines
        lin_r_new[1] = -lin_r_new[0] / (-1 / ((m_w + m_y) / 2))
        m_r = -lin_r_new[0] / lin_r_new[1]
        # Make red line go through original midpoint
        lin_r_new[2] = -(y_r_mid - m_r * x_r_mid) * lin_r_new[1]

        # Enforce distance between yellow and white line
        x_rand = 0.0
        ii = 0
        dist_curr = dist_wy * 2
        while abs(dist_wy - dist_curr) > tol / 100 and ii < max_iter:
            y_rand_w = m_w * x_rand - lin_w_new[2] / lin_w_new[1]  # Get random point on white line
            y_rand_y = m_y * x_rand - lin_y_new[2] / lin_y_new[1]  # Get random point on yellow line
            diff_vect = np.array([0.0, y_rand_y - y_rand_w])  # Get vector between two points
            dir_vect = np.array([lin_w_new[1], -lin_w_new[0]])  # Get direction vector for white line

            v_proj = np.dot(dir_vect, diff_vect) / np.linalg.norm(dir_vect) * diff_vect / np.linalg.norm(dir_vect)
            dist_curr = np.linalg.norm(diff_vect - v_proj)

            curr_step = step_size
            if dist_curr > dist_wy:
                curr_step = -step_size

            lin_w_new[2] = lin_w_new[2] + curr_step / 2
            lin_y_new[2] = lin_y_new[2] + curr_step / 2
            ii += 1

        # Save results! We are done from the numerical perspective!
        lines_new = [lin_w_new, lin_y_new, lin_r_new]
        return lines_new

    def set_lines_from_ground_coordinates(self, pixel_list_list, checkpoint=False):
        """
        Computes equation of lines from ground projected coordinates and computes the homography
        :param color_coordinates_msg:
        :param checkpoint:
        :return:
        """
        lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        lines_cv = [[None] * 4, [None] * 4, [None] * 4]  # WYR

        # pixel_lists = color_coordinates_msg.pixel_lists
        # pixel_list_list = []
        # for pixel_l in pixel_lists:
        #     pixels = self.pixel_list_msg_to_pixels(pixel_l, dist_thres=0.5)
        #     pixel_list_list.append(pixels)

        for i, pixels in enumerate(pixel_list_list):
            if len(pixels) >= 2:
                line_cv = [vx, vy, x, y] = cv2.fitLine(pixels, cv2.DIST_HUBER, 0, 0.01, 0.01)
                lines_cv[i] = line_cv
                color_line = self.get_abc(x, y, vx, vy)
                lines[i] = color_line

        # If all three lines are detected, force them to be parallel/perpendicular
        par_lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        all_lines_valid = True
        for l in lines:
            if None in l:
                all_lines_valid = False
                break
        if all_lines_valid:
            par_lines = self.parallelize_lines(lines)

        # Set line equations to corresponding values depending on whether they are for checkpoint or not
        if checkpoint:
            self.checkpoint_lines = lines
            self.checkpoint_lines_par = par_lines
            self.checkpoint_lines_cv = lines_cv
            self.checkpoint_ground_color_coordinates = pixel_list_list
        else:
            self.current_lines = lines
            self.current_lines_par = par_lines
            self.current_lines_cv = lines_cv
            self.current_ground_color_coordinates = pixel_list_list

        publisher = self.pub_d_vs_lines_checkpoint if checkpoint else self.pub_d_vs_lines
        if publisher.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(
                self.debug_ground_lines(checkpoint=checkpoint, all=False))
            # debug_image_msg.header = seglist_out.header
            publisher.publish(debug_image_msg)
        print(f"CUR: {self.current_lines}, CKP: {self.checkpoint_lines}")
        print(f"CUR_PAR: {self.current_lines_par}, CKP_PAR: {self.checkpoint_lines_par}")
        # If the current image or the checkpoint image changes, compute the homography
        # Needs to have all three lines
        self.H_par = self.compute_homography(self.current_lines_par, self.checkpoint_lines_par)
        # self.H_par = self.compute_homography(self.checkpoint_lines_par, self.current_lines_par)
        # self.H = self.compute_homography(self.current_lines, self.checkpoint_lines)
        # print(f"H: {self.H}")
        print(f"H_PAR: {self.H_par}")
        if self.H_par is not None:
            homography_msg = FloatList()
            homography_msg.H = list(self.H_par.flatten())
            self.pub_homography.publish(homography_msg)

    @staticmethod
    def group_points_together(filtered_pts, pt_group_dist_x=0.1, pt_group_dist_y=0.1, group_size_ignore=2):
        filtered_pts = np.array(filtered_pts).tolist()
        list_of_pt_groups = []
        for curr_pt in filtered_pts:
            # Find closest point that is not already part of a group we are in
            new_pt_found = False
            excluded_pts = [curr_pt]
            while len(excluded_pts) != len(filtered_pts) and not new_pt_found:
                closest_pt, dist, dist_x, dist_y = LineDetectorNode.find_closest_pt(curr_pt, excluded_pts, filtered_pts)
                excluded_pts.append(closest_pt)

                # Check if this closest point is already in the same group as us
                already_grouped = False
                for group in list_of_pt_groups:
                    if curr_pt in group and closest_pt in group:
                        already_grouped = True

                if not already_grouped:
                    new_pt_found = True

            # Check if while loop exited bc every point is in the same group, at this point we're done
            if len(excluded_pts) == len(filtered_pts):
                break

            # Get our current group (if we have one)
            new_group = [curr_pt]
            for group in list_of_pt_groups:
                if curr_pt in group:
                    new_group = group
                    list_of_pt_groups.remove(group)

            # Check if closest point forms a group with current point
            if dist_x < pt_group_dist_x and dist_y < pt_group_dist_y:
                # Check if closest point not in our group belongs to a different group and merge it with our group if yes
                # Otherwise just append it to our group
                new_group.append(closest_pt)
                for group in list_of_pt_groups:
                    if closest_pt in group:
                        new_group.remove(closest_pt)
                        new_group.extend(group)
                        list_of_pt_groups.remove(group)

            # Add the newest group to the list of groups
            list_of_pt_groups.append(new_group)

        # Get rid of groups that are too small as they are untrustworthy
        for group in list_of_pt_groups:
            if len(group) < group_size_ignore:
                list_of_pt_groups.remove(group)

        return list_of_pt_groups

    @staticmethod
    def find_closest_pt(target_pt, excluded_pts, pt_list):
        closest_pt = []
        min_dist = 10000.0  # Some silly large number
        for pt in pt_list:
            if pt not in excluded_pts:
                dist = ((target_pt[0] - pt[0]) ** 2 + (target_pt[1] - pt[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    min_dist_x = abs(pt[0] - target_pt[0])
                    min_dist_y = abs(pt[1] - target_pt[1])
                    closest_pt = pt
        return closest_pt, min_dist, min_dist_x, min_dist_y

    def set_cluster_lines_from_ground_coordinates(self, pixel_list_list, checkpoint=False):
        lines_cv = []
        for i, pixel_list in enumerate(pixel_list_list):  # WYR
            if len(pixel_list) >= 2:
                grouped_pixels = self.group_points_together(pixel_list)
                lines_cv.append([])
                for group in grouped_pixels:
                    if len(group) >= 2:
                        line_cv = cv2.fitLine(np.array(group), cv2.DIST_HUBER, 0, 0.01, 0.01)
                        lines_cv[-1].append(line_cv)
        if checkpoint:
            self.checkpoint_lines_cv_all = lines_cv
        else:
            self.current_lines_cv_all = lines_cv

        publisher = self.pub_d_vs_lines_checkpoint_all if checkpoint else self.pub_d_vs_lines_all
        if publisher.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(
                self.debug_ground_lines(checkpoint=checkpoint, all=True))
            publisher.publish(debug_image_msg)

    def handle_ground_coordinates(self, color_coordinates_msg, checkpoint=False):
        pixel_list_list = self.color_coordinates_msg_to_pixels(color_coordinates_msg)
        self.set_lines_from_ground_coordinates(pixel_list_list, checkpoint=checkpoint)
        self.set_cluster_lines_from_ground_coordinates(pixel_list_list, checkpoint=checkpoint)

    def color_coordinates_ground_cb(self, color_coordinates_msg):
        self.handle_ground_coordinates(color_coordinates_msg, checkpoint=False)
        # self.set_lines_from_ground_coordinates(color_coordinates_msg, checkpoint=False)

    def color_coordinates_checkpoint_ground_cb(self, color_coordinates_msg):
        self.handle_ground_coordinates(color_coordinates_msg, checkpoint=True)
        # self.set_lines_from_ground_coordinates(color_coordinates_msg, checkpoint=True)

    def publish_color_coordinates(self, image, checkpoint=False):
        """
        Publishes color coordinate values given current image or checkpoint image. Subscribed by Ground Projection Node.
        :param image: Current image or Checkpoint image
        :param checkpoint: Whether image is the current image or a checkpoint image
        :return: None
        """
        color_range = {
            "WHITE": {"MIN": np.array([0, 0, 100], np.uint8), "MAX": np.array([180, 70, 255], np.uint8)},
            "YELLOW": {"MIN": np.array([25, 140, 50], np.uint8), "MAX": np.array([35, 255, 255], np.uint8)},
            "RED": ({"MIN": np.array([0, 140, 100], np.uint8), "MAX": np.array([15, 255, 255], np.uint8)}, {
                "MIN": np.array([165, 140, 100], np.uint8), "MAX": np.array([180, 255, 255], np.uint8)
            })
        }
        im_edge = self.detector.canny_edges
        im_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pixel_list_msgs = []
        all_coordinates = []
        for color in ["WHITE", "YELLOW", "RED"]:
            im_color = 0.
            if color == "RED":
                for i in range(2):
                    im_color += cv2.inRange(im_hsv, color_range[color][i]["MIN"], color_range[color][i]["MAX"])
            else:
                im_color += cv2.inRange(im_hsv, color_range[color]["MIN"], color_range[color]["MAX"])
            ii = np.where(np.logical_and(im_color, im_edge))
            color_coordinates = np.array([ii[1], ii[0]]).T
            normalized_color_coordinates = (color_coordinates + self.arr_cutoff) * self.arr_ratio
            # color_coordinates = self.get_color_coordinates(im_color, im_edge)  # n x 2
            coordinates_list_msg = self.pixels_to_pixel_list_msg(normalized_color_coordinates)
            pixel_list_msgs.append(coordinates_list_msg)
            all_coordinates.append(color_coordinates)
        msg = PixelListList()
        msg.pixel_lists = pixel_list_msgs
        publisher = self.pub_color_coordinates_checkpoint if checkpoint else self.pub_color_coordinates
        publisher.publish(msg)
        colors = ((255, 255, 255), (0, 255, 255), (0, 0, 255))
        pub_d_img = self.pub_d_color_coordinates_img_checkpoint if checkpoint else self.pub_d_color_coordinates_img
        if pub_d_img.get_num_connections() > 0:
            debug_image = image.copy() * 0.2
            h, w = image.shape[:2]
            dist_thres = 85
            cv2.circle(debug_image, (80, 80), radius=5, color=(255, 105, 180), thickness=5)
            for i, color_coordinates in enumerate(all_coordinates):
                # if i == 0:
                filtered_coordinates = []
                for point in color_coordinates:
                    if np.sqrt((point[0] - 80) ** 2 + (point[1] - 80) ** 2) < dist_thres:
                        filtered_coordinates.append(point)
                        cv2.circle(debug_image, (point[0], point[1]), radius=0, color=colors[i], thickness=-1)
                if len(filtered_coordinates) > 10:
                    filtered_coordinates = np.array(filtered_coordinates)
                    [vx, vy, x, y] = cv2.fitLine(filtered_coordinates, cv2.DIST_HUBER, 0, 0.01, 0.01)
                    bottomy = h
                    bottomx = int((bottomy - y) * vx / vy + x)
                    topy = int(min(filtered_coordinates[:, 1]))
                    topx = int((topy - y) * vx / vy + x)
                    cv2.line(debug_image, (bottomx, bottomy), (topx, topy), colors[i], thickness=2)

            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_image)
            # debug_image_msg.header = image_msg.header
            pub_d_img.publish(debug_image_msg)

    def image_cb(self, image_msg):
        """
        Processes the incoming image messages.

        Performs the following steps for each incoming image:

        #. Performs color correction
        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows in order to remove the part of the image that doesn't include the road
        #. Extracts the line segments in the image using :py:class:`line_detector.LineDetector`
        #. Converts the coordinates of detected segments to normalized ones
        #. Creates and publishes the resultant :obj:`duckietown_msgs.msg.SegmentList` message
        #. Creates and publishes debug images if there is a subscriber to the respective topics

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message

        """
        # Decode from compressed image with OpenCV
        image = self.prepare_image(image_msg)
        if not self.img_shape:
            self.img_shape = image.shape

        # Extract the line segments for every color
        self.detector.setImage(image)
        detections = {
            color: self.detector.detectLines(ranges)
            for color, ranges in self.color_ranges.items()
        }

        # Construct a SegmentList
        segment_list = SegmentList()
        segment_list.header.stamp = image_msg.header.stamp

        # Remove the offset in coordinates coming from the removing of the top part and
        arr_cutoff = np.array([
            0, self._top_cutoff, 0, self._top_cutoff
        ])
        arr_ratio = np.array([
            1. / self._img_size[1], 1. / self._img_size[0],
            1. / self._img_size[1], 1. / self._img_size[0]
        ])

        # Fill in the segment_list with all the detected segments
        for color, det in detections.items():
            # Get the ID for the color from the Segment msg definition
            # Throw and exception otherwise
            if len(det.lines) > 0 and len(det.normals) > 0:
                try:
                    color_id = getattr(Segment, color)
                    lines_normalized = (det.lines + arr_cutoff) * arr_ratio
                    segment_list.segments.extend(
                        self._to_segment_msg(lines_normalized, det.normals, color_id)
                    )
                except AttributeError:
                    self.logerr("Color name %s is not defined in the Segment message" % color)

        # Publish the message
        self.pub_lines.publish(segment_list)

        # If there are any subscribers to the debug topics, generate a debug image and publish it
        if self.pub_d_segments.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in detections.items()}
            debug_img = plotSegments(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_segments.publish(debug_image_msg)

        if self.pub_d_edges.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.detector.canny_edges)
            debug_image_msg.header = image_msg.header
            self.pub_d_edges.publish(debug_image_msg)

        if self.pub_d_maps.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in detections.items()}
            debug_img = plotMaps(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_maps.publish(debug_image_msg)

        if self.pub_d_checkpoint_image.get_num_connections() > 0:
            if self.checkpoint_image is not None:
                debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.checkpoint_image)
                debug_image_msg.header = image_msg.header
                self.pub_d_checkpoint_image.publish(debug_image_msg)

        for channels in ['HS', 'SV', 'HV']:
            publisher = getattr(self, 'pub_d_ranges_%s' % channels)
            if publisher.get_num_connections() > 0:
                debug_img = self._plot_ranges_histogram(channels)
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                debug_image_msg.header = image_msg.header
                publisher.publish(debug_image_msg)

        self.current_image = image
        self.publish_color_coordinates(image, checkpoint=False)

    def init_debug_bg_img(self):
        if self.debug_img_bg is not None:
            return

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

    def debug_ground_lines(self, checkpoint=False, all=False):
        colors = ((255, 255, 255), (0, 255, 255), (0, 0, 255))
        colors_par = ((195, 195, 195), (33, 148, 194), (171, 38, 166))
        self.init_debug_bg_img()
        image = self.debug_img_bg.copy()
        if not all:
            lines_cv = self.checkpoint_lines_cv if checkpoint else self.current_lines_cv
            lines_cv = [[x] for x in lines_cv]  # Wrap each single line in a list
        else:
            lines_cv = self.checkpoint_lines_cv_all if checkpoint else self.current_lines_cv_all
        lines_par = self.checkpoint_lines_par if checkpoint else self.current_lines_par
        ground_color_coordinates = self.checkpoint_ground_color_coordinates if checkpoint else self.current_ground_color_coordinates
        for i, (lines_cv, c_coord) in enumerate(zip(lines_cv, ground_color_coordinates)):
            for point in c_coord:
                x = int((point[1] * -400) + 200)
                y = int((point[0] * -400) + 300)
                cv2.circle(image, (x, y), radius=2, color=colors[i], thickness=2)
            # For each line of each color
            for line_cv in lines_cv:
                if None not in line_cv:
                    vx, vy, x, y = line_cv
                    # bottomy = h
                    # bottomx = int((bottomy - y) * vx / vy + x)
                    bottomx = 0
                    bottomy = (bottomx - x) * vy / vx + y
                    topx = 0.7
                    topy = (topx - x) * vy / vx + y

                    bottomx = int(bottomx * -400 + 300)
                    topx = int(topx * -400 + 300)
                    bottomy = int(bottomy * -400 + 200)
                    topy = int(topy * -400 + 200)
                    # print(vx, bottomx, bottomy, topx, topy)
                    cv2.line(image, (bottomy, bottomx), (topy, topx), colors[i], thickness=1)
                # Only plot parallel lines if plotting three lines
                if not all:
                    line_par = lines_par[i]
                    if None not in line_par:
                        bottomx, topx = 0, 0.7
                        bottomy = -(line_par[0] * bottomx + line_par[2]) / line_par[1]
                        topy = -(line_par[0] * topx + line_par[2]) / line_par[1]

                        bottomx = int(bottomx * -400 + 300)
                        topx = int(topx * -400 + 300)
                        bottomy = int(bottomy * -400 + 200)
                        topy = int(topy * -400 + 200)
                        cv2.line(image, (bottomy, bottomx), (topy, topx), colors_par[i], thickness=1)
        return image

    @staticmethod
    def _to_segment_msg(lines, normals, color):
        """
        Converts line detections to a list of Segment messages.

        Converts the resultant line segments and normals from the line detection to a list of Segment messages.

        Args:
            lines (:obj:`numpy array`): An ``Nx4`` array where each row represents a line.
            normals (:obj:`numpy array`): An ``Nx2`` array where each row represents the normal of a line.
            color (:obj:`str`): Color name string, should be one of the pre-defined in the Segment message definition.

        Returns:
            :obj:`list` of :obj:`duckietown_msgs.msg.Segment`: List of Segment messages

        """
        segment_msg_list = []
        for x1, y1, x2, y2, norm_x, norm_y in np.hstack((lines, normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
            segment_msg_list.append(segment)
        return segment_msg_list

    def _plot_ranges_histogram(self, channels):
        """ Utility method for plotting color histograms and color ranges.

        Args:
            channels (:obj:`str`): The desired two channels, should be one of ``['HS','SV','HV']``

        Returns:
            :obj:`numpy array`: The resultant plot image

        """
        channel_to_axis = {'H': 0, 'S': 1, 'V': 2}
        axis_to_range = {0: 180, 1: 256, 2: 256}

        # Get which is the third channel that will not be shown in this plot
        missing_channel = 'HSV'.replace(channels[0], '').replace(channels[1], '')

        hsv_im = self.detector.hsv
        # Get the pixels as a list (flatten the horizontal and vertical dimensions)
        hsv_im = hsv_im.reshape((-1, 3))

        channel_idx = [channel_to_axis[channels[0]], channel_to_axis[channels[1]]]

        # Get only the relevant channels
        x_bins = np.arange(0, axis_to_range[channel_idx[1]] + 1, 2)
        y_bins = np.arange(0, axis_to_range[channel_idx[0]] + 1, 2)
        h, _, _ = np.histogram2d(x=hsv_im[:, channel_idx[0]], y=hsv_im[:, channel_idx[1]],
                                 bins=[y_bins, x_bins])
        # Log-normalized histogram
        np.log(h, out=h, where=(h != 0))
        h = (255 * h / np.max(h)).astype(np.uint8)

        # Make a color map, for the missing channel, just take the middle of the range
        if channels not in self.colormaps:
            colormap_1, colormap_0 = np.meshgrid(x_bins[:-1], y_bins[:-1])
            colormap_2 = np.ones_like(colormap_0) * (axis_to_range[channel_to_axis[missing_channel]] / 2)

            channel_to_map = {channels[0]: colormap_0,
                              channels[1]: colormap_1,
                              missing_channel: colormap_2}

            self.colormaps[channels] = np.stack([channel_to_map['H'], channel_to_map['S'], channel_to_map['V']],
                                                axis=-1).astype(np.uint8)
            self.colormaps[channels] = cv2.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)

        # resulting histogram image as a blend of the two images
        im = cv2.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
        im = cv2.addWeighted(im, 0.5, self.colormaps[channels], 1 - 0.5, 0.0)

        # now plot the color ranges on top
        for _, color_range in self.color_ranges.items():
            # convert HSV color to BGR
            c = color_range.representative
            c = np.uint8([[[c[0], c[1], c[2]]]])
            color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int)
            for i in range(len(color_range.low)):
                cv2.rectangle(
                    im,
                    pt1=((color_range.high[i, channel_idx[1]] / 2).astype(np.uint8),
                         (color_range.high[i, channel_idx[0]] / 2).astype(np.uint8)),
                    pt2=((color_range.low[i, channel_idx[1]] / 2).astype(np.uint8),
                         (color_range.low[i, channel_idx[0]] / 2).astype(np.uint8)),
                    color=color,
                    lineType=cv2.LINE_4
                )
        # ---
        return im


if __name__ == '__main__':
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name='line_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
