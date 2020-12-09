#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList, AntiInstagramThresholds
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
        self.anti_instagram_thresholds=dict()
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
            "/agent/ground_projection_node",
            PixelListList,
            self.color_coordinates_ground_cb,
            queue_size=1
        )

        self.lines = {c: [None] * 3 for c in ["WHITE", "YELLOW", "RED"]}


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

    @staticmethod
    def get_color_coordinates(color_mask, im_edge, dist_thres=75):
        i_coords, j_coords = np.meshgrid(range(color_mask.shape[0]),
                                         range(color_mask.shape[1]),
                                         indexing='ij')
        dist = np.sqrt(i_coords ** 2 + j_coords ** 2)
        dist_mask = np.logical_and(dist > dist_thres,
                                   i_coords > 10)
        ii = np.where(np.logical_and(np.logical_and(color_mask, im_edge),
                                     dist_mask))
        return np.array([ii[1], ii[0]]).T

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
            pixel_msg.u, pixel_msg.v = pixel
            pixel_msgs.append(pixel_msg)
        pixel_list = PixelList()
        pixel_list.pixels = pixel_msgs
        return pixel_list

    @staticmethod
    def pixel_list_msg_to_pixels(pixel_list_msg):
        pixel_list = pixel_list_msg.pixels
        pixels = []
        for pixel in pixel_list:
            pixels.append([pixel.u, pixel.v])
        return np.array(pixels, np.uint8)

    def color_coordinates_ground_cb(self, color_coordinates_msg):
        pixel_lists = color_coordinates_msg.pixel_list
        pixel_list_list = []
        for pixel_l in pixel_lists:
            pixels = self.pixel_list_msg_to_pixels(pixel_l)
            pixel_list_list.append(pixels)
            [vx, vy, x, y] = cv2.fitLine(pixels, cv2.DIST_HUBER, 0, 0.01, 0.01)
            color_line = self.get_abc(x, y, vx, vy)
            print(color_line)

    def publish_color_coordinates(self, image):
        color_range = {
            "WHITE": {"MIN": np.array([0, 0, 70], np.uint8), "MAX": np.array([180, 60, 255], np.uint8)},
            "YELLOW": {"MIN": np.array([23, 30, 60], np.uint8), "MAX": np.array([34, 255, 255], np.uint8)},
            "RED": ({"MIN": np.array([0, 30, 60], np.uint8), "MAX": np.array([12, 255, 255], np.uint8)}, {
                "MIN": np.array([170, 30, 60], np.uint8), "MAX": np.array([180, 255, 255], np.uint8)
            })
        }
        im_edge = self.detector.canny_edges
        im_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lines = {c: [None] * 3 for c in ["WHITE", "YELLOW", "RED"]}
        pixel_list_msgs = []
        for color in ["WHITE", "YELLOW", "RED"]:
            im_color = 0.
            if color == "RED":
                for i in range(2):
                    im_color += cv2.inRange(im_hsv, color_range[color][i]["MIN"], color_range[color][i]["MAX"])
            else:
                im_color += cv2.inRange(im_hsv, color_range[color]["MIN"], color_range[color]["MAX"])
            color_coordinates = self.get_color_coordinates(im_color, im_edge)   # n x 2
            coordinates_list_msg = self.pixels_to_pixel_list_msg(color_coordinates)
            pixel_list_msgs.append(coordinates_list_msg)
            try:
                [vx, vy, x, y] = cv2.fitLine(color_coordinates, cv2.DIST_HUBER, 0, 0.01, 0.01)
                color_line = self.get_abc(x, y, vx, vy)
                lines[color] = color_line
            # If no line is found
            except:
                pass
        msg = PixelListList()
        msg.pixel_lists = pixel_list_msgs
        self.pub_color_coordinates.publish(msg)
        self.lines = lines
        self.log(lines)

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

        for channels in ['HS', 'SV', 'HV']:
            publisher = getattr(self, 'pub_d_ranges_%s' % channels)
            if publisher.get_num_connections() > 0:
                debug_img = self._plot_ranges_histogram(channels)
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                debug_image_msg.header = image_msg.header
                publisher.publish(debug_image_msg)

        self.publish_color_coordinates(image)

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
            colormap_2 = np.ones_like(colormap_0) * (axis_to_range[channel_to_axis[missing_channel]]/2)

            channel_to_map = {channels[0]: colormap_0,
                              channels[1]: colormap_1,
                              missing_channel: colormap_2}

            self.colormaps[channels] = np.stack([channel_to_map['H'], channel_to_map['S'], channel_to_map['V']], axis=-1).astype(np.uint8)
            self.colormaps[channels] = cv2.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)

        # resulting histogram image as a blend of the two images
        im = cv2.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
        im = cv2.addWeighted(im, 0.5 , self.colormaps[channels], 1 - 0.5, 0.0)

        # now plot the color ranges on top
        for _, color_range in self.color_ranges.items():
            # convert HSV color to BGR
            c = color_range.representative
            c = np.uint8([[[c[0], c[1], c[2]]]])
            color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int)
            for i in range(len(color_range.low)):
                cv2.rectangle(
                    im,
                    pt1=((color_range.high[i, channel_idx[1]]/2).astype(np.uint8), (color_range.high[i, channel_idx[0]]/2).astype(np.uint8)),
                    pt2=((color_range.low[i, channel_idx[1]]/2).astype(np.uint8), (color_range.low[i, channel_idx[0]]/2).astype(np.uint8)),
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
