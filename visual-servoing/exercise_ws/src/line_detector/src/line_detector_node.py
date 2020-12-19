#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList, AntiInstagramThresholds, BoolStamped
from std_msgs.msg import String
from line_detector import LineDetector, ColorRange, plotSegments, plotMaps
from image_processing.anti_instagram import AntiInstagram
from custom_msgs.msg import Pixel, PixelList, PixelListList
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

        self.pub_d_color_coordinates_ground_img = rospy.Publisher(
            "~debug/color_coordinates_ground/compressed", CompressedImage, queue_size=1,
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

        self.sub_key_press = rospy.Subscriber("/agent/line_detector_node/key_pressed", String, self.cb_key_pressed,
                                              queue_size=1)
        self.checkpoint_image = None
        self.current_image = None
        self.current_lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        self.checkpoint_lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        self.current_lines_cv = [[None] * 4, [None] * 4, [None] * 4]  # WYR
        self.checkpoint_lines_cv = [[None] * 4, [None] * 4, [None] * 4]  # WYR
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

    @staticmethod
    def compute_homography(lines1, lines2):
        # print(lines1, lines2)
        a_white_0, b_white_0, c_white_0 = lines1[0]
        # a_red_0, b_red_0, c_red_0 = lines1[1]
        a_red_0, b_red_0, c_red_0 = 1., 1., 1.
        a_yellow_0, b_yellow_0, c_yellow_0 = lines1[1]
        a_white_1, b_white_1, c_white_1 = lines2[0]
        # a_red_1, b_red_1, c_red_1 = lines2[1]
        a_red_1, b_red_1, c_red_1 = 1., 1., 1.
        a_yellow_1, b_yellow_1, c_yellow_1 = lines2[1]
        lines = np.array(
            [[a_white_0, b_white_0, c_white_0],
             [a_red_0, b_red_0, c_red_0],
             [a_yellow_0, b_yellow_0, c_yellow_0]]
        ).astype(float)
        b = np.array(
            [a_white_1, a_red_1, a_yellow_1, b_white_1, b_red_1, b_yellow_1, c_white_1, c_red_1, c_yellow_1]).astype(
            float)
        # print(lines, b)
        A = np.zeros((9, 9), dtype=float)
        A[:3, :3] = A[3:6, 3:6] = A[6:, 6:] = lines
        H_prime = np.linalg.inv(A) @ b
        H_prime = H_prime.reshape(-1) / H_prime[8]
        H_prime = np.array(
            [[H_prime[0], H_prime[1], H_prime[2]],
             [H_prime[3], H_prime[4], H_prime[5]],
             [H_prime[6], H_prime[7], H_prime[8]]
             ])
        H = np.linalg.inv(H_prime).T
        return H

    def set_lines_from_ground_coordinates(self, color_coordinates_msg, checkpoint=False):
        """
        Computes equation of lines from ground projected coordinates and computes the homography
        :param color_coordinates_msg:
        :param checkpoint:
        :return:
        """
        pixel_lists = color_coordinates_msg.pixel_lists
        pixel_list_list = []
        lines = [[None] * 3, [None] * 3, [None] * 3]  # WYR
        lines_cv = [[None] * 4, [None] * 4, [None] * 4]  # WYR
        for i, pixel_l in enumerate(pixel_lists):
            pixels = self.pixel_list_msg_to_pixels(pixel_l, dist_thres=1.5)
            pixel_list_list.append(pixels)
            if len(pixels) >= 2:
                line_cv = [vx, vy, x, y] = cv2.fitLine(pixels, cv2.DIST_HUBER, 0, 0.01, 0.01)
                lines_cv[i] = line_cv
                color_line = self.get_abc(x, y, vx, vy)
                lines[i] = color_line
        # Set line equations to corresponding values depending on whether they are for checkpoint or not
        if checkpoint:
            self.checkpoint_lines = lines
            self.checkpoint_lines_cv = lines_cv
            self.checkpoint_ground_color_coordinates = pixel_list_list
        else:
            self.current_lines = lines
            self.current_lines_cv = lines_cv
            self.current_ground_color_coordinates = pixel_list_list

        publisher = self.pub_d_vs_lines_checkpoint if checkpoint else self.pub_d_vs_lines
        if publisher.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.debug_ground_lines(checkpoint=checkpoint))
            # debug_image_msg.header = seglist_out.header
            publisher.publish(debug_image_msg)
        # self.log(f"CUR: {self.current_lines}, CKP: {self.checkpoint_lines}")
        # self.log(f"CUR: {self.current_lines}, CKP: {self.checkpoint_lines}")
        # If the current image or the checkpoint image changes, compute the homography
        # Needs to have all three lines
        self.H = self.compute_homography(self.current_lines, self.checkpoint_lines)
        # print(self.H)

    def color_coordinates_ground_cb(self, color_coordinates_msg):
        self.set_lines_from_ground_coordinates(color_coordinates_msg, checkpoint=False)

    def color_coordinates_checkpoint_ground_cb(self, color_coordinates_msg):
        self.set_lines_from_ground_coordinates(color_coordinates_msg, checkpoint=True)

    def publish_color_coordinates(self, image, checkpoint=False):
        """
        Publishes color coordinate values given current image or checkpoint image. Subscribed by Ground Projection Node.
        :param image: Current image or Checkpoint image
        :param checkpoint: Whether image is the current image or a checkpoint image
        :return: None
        """
        color_range = {
            "WHITE": {"MIN": np.array([0, 0, 100], np.uint8), "MAX": np.array([180, 100, 255], np.uint8)},
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
        if not checkpoint and self.pub_d_color_coordinates_img.get_num_connections() > 0:
            debug_image = image.copy() * 0.2
            h, w = image.shape[:2]
            dist_thres = 75
            cv2.circle(debug_image, (80, 80), radius=5, color=(255, 105, 180), thickness=5)
            for i, color_coordinates in enumerate(all_coordinates):
                filtered_coordinates = []
                for point in color_coordinates:
                    if np.sqrt((point[0] - 80) ** 2 + (point[1] - 80) ** 2) < dist_thres:
                        filtered_coordinates.append(point)
                        cv2.circle(debug_image, (point[0], point[1]), radius=0, color=colors[i], thickness=-1)
                if len(filtered_coordinates) > 10:
                    filtered_coordinates = np.array(filtered_coordinates)
                    # if i == 1:
                    #     print(f"Stats of yellow coordinates in image space: {filtered_coordinates.mean(axis=0), filtered_coordinates.std(axis=0)}")
                    [vx, vy, x, y] = cv2.fitLine(filtered_coordinates, cv2.DIST_HUBER, 0, 0.01, 0.01)
                    bottomy = h
                    bottomx = int((bottomy - y) * vx / vy + x)
                    topy = int(min(filtered_coordinates[:, 1]))
                    topx = int((topy - y) * vx / vy + x)
                    cv2.line(debug_image, (bottomx, bottomy), (topx, topy), colors[i], thickness=2)

            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_image)
            # debug_image_msg.header = image_msg.header
            self.pub_d_color_coordinates_img.publish(debug_image_msg)

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

    def debug_ground_lines(self, checkpoint=False):
        colors = ((255, 255, 255), (0, 255, 255), (0, 0, 255))
        self.init_debug_bg_img()
        image = self.debug_img_bg.copy()
        h, w = self.img_shape[:2]
        lines_cv = self.checkpoint_lines_cv if checkpoint else self.current_lines_cv
        ground_color_coordinates = self.checkpoint_ground_color_coordinates if checkpoint else self.current_ground_color_coordinates
        for i, (line_cv, c_coord) in enumerate(zip(lines_cv, ground_color_coordinates)):
            # c_coord *= 100
            # if i == 1:
            #     print(c_coord.mean(axis=0), c_coord.std(axis=0))
            for point in c_coord:
                x = int((point[1] * -400) + 200)
                y = int((point[0] * -400) + 300)
                cv2.circle(image, (x, y), radius=2, color=colors[i], thickness=2)
            # if i == 1:
            #     print(f"Point: {(x, y)}")
            if None not in line_cv:
                vx, vy, x, y = line_cv
                # bottomy = h
                # bottomx = int((bottomy - y) * vx / vy + x)
                bottomx = 0
                bottomy = (bottomx - x) * vy / vx + y
                # topy = int(min(ground_color_coordinates[i][:, 1]))
                # topx = int((topy - y) * vx / vy + x)
                topx = h
                topy = (topx - x) * vy / vx + y
                # print(vx, vy, bottomx, bottomy)

                bottomx = int(bottomx * -400 + 200)
                topx = int(topx * -400 + 200)
                bottomy = int(bottomy * -400 + 300)
                topy = int(topy * -400 + 300)
                print(vx, bottomx, bottomy, topx, topy)
                cv2.line(image, (bottomx, bottomy), (topx, topy), colors[i], thickness=1)
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
