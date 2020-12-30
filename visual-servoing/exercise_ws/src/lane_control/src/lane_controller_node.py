#!/usr/bin/env python3
import numpy as np
from math import isnan
import rospy
import os

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading, SegmentList
from lane_controller.controller import LaneController
from lane_controller.planner import LanePlanner
from custom_msgs.msg import FloatList
from sensor_msgs.msg import Joy


class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocities, by processing the estimate error in
    lateral deviationa and heading.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
        ~intersection_navigation_pose (:obj:`LanePose`): The lane pose estimate from intersection navigation
        ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Confirmation that the control action was executed
        ~stop_line_reading (:obj:`StopLineReading`): Distance from stopline, to reduce speed
        ~obstacle_distance_reading (:obj:`stop_line_reading`): Distancefrom obstacle virtual stopline, to reduce speed
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        print("Controller init")
        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()
        self.params['~min_r'] = rospy.get_param('~min_r', None)
        self.params['~max_r'] = rospy.get_param('~max_r', None)
        self.params['~dt_path'] = rospy.get_param('~dt_path', None)
        self.params['~max_dist_fact'] = rospy.get_param('~max_dist_fact', None)
        self.params['~min_dist'] = rospy.get_param('~min_dist', None)
        self.params['~min_angle'] = rospy.get_param('~min_angle', None)
        self.params['~num_dt'] = rospy.get_param('~num_dt', None)
        self.params['~k_v'] = rospy.get_param('~k_v', None)
        self.params['~k_omega'] = rospy.get_param('~k_omega', None)
        self.params['~homog_tol'] = rospy.get_param('~homog_tol', None)
        self.params['~plan_min_exec_time'] = rospy.get_param('~plan_min_exec_time', None)
        

        # Need to create controller object before updating parameters, otherwise it will fail
        self.controller = LaneController(self.params)
        self.planner = LanePlanner(self.params)
        # self.updateParameters() # TODO: This needs be replaced by the new DTROS callback when it is implemented

        # Initialize variables
        self.path_dist = None
        self.u = None
        self.fsm_state = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.last_s = 0.0
        self.last_s_dir = 0.0
        self.last_s_turn = 0.0
        self.stop_line_distance = None
        self.stop_line_detected = False
        self.at_stop_line = False
        self.obstacle_stop_line_distance = None
        self.obstacle_stop_line_detected = False
        self.at_obstacle_stop_line = False

        self.current_pose_source = 'lane_filter'

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
                                           Twist2DStamped,
                                           queue_size=1,
                                           dt_topic_type=TopicType.CONTROL)

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber("/agent/lane_filter_node/lane_pose",
                                                 LanePose,
                                                 self.cbAllPoses,
                                                 "lane_filter",
                                                 queue_size=1)
        self.sub_intersection_navigation_pose = rospy.Subscriber("/agent/lane_controller_node/intersection_navigation_pose",
                                                                 LanePose,
                                                                 self.cbAllPoses,
                                                                 "intersection_navigation",
                                                                 queue_size=1)
        self.sub_wheels_cmd_executed = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd",
                                                        WheelsCmdStamped,
                                                        self.cbWheelsCmdExecuted,
                                                        queue_size=1)
        self.sub_stop_line = rospy.Subscriber("~stop_line_reading",
                                              StopLineReading,
                                              self.cbStopLineReading,
                                              queue_size=1)
        self.sub_obstacle_stop_line = rospy.Subscriber("~obstacle_distance_reading",
                                                        StopLineReading,
                                                        self.cbObstacleStopLineReading,
                                                        queue_size=1)
        self.sub_homography = rospy.Subscriber("/agent/line_detector_node/affine", 
                                                FloatList,
                                                self.updatePath,
                                                queue_size=1)
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.log("Initialized!")

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        # S key pressed
        print(joy_msg.buttons)
        if joy_msg.buttons[6] == 1:
            print("S pressed")
            self.save_toggl = False
        elif joy_msg.buttons[3] == 1:
            print("E pressed")
            self.save_toggl = True

    def cbObstacleStopLineReading(self,msg):
        """
        Callback storing the current obstacle distance, if detected.

        Args:
            msg(:obj:`StopLineReading`): Message containing information about the virtual obstacle stopline.
        """
        self.obstacle_stop_line_distance = np.sqrt(msg.stop_line_point.x ** 2 + msg.stop_line_point.y ** 2)
        self.obstacle_stop_line_detected = msg.stop_line_detected
        self.at_stop_line = msg.at_stop_line

    def cbStopLineReading(self, msg):
        """Callback storing current distance to the next stopline, if one is detected.

        Args:
            msg (:obj:`StopLineReading`): Message containing information about the next stop line.
        """
        self.stop_line_distance = np.sqrt(msg.stop_line_point.x ** 2 + msg.stop_line_point.y ** 2)
        self.stop_line_detected = msg.stop_line_detected
        self.at_obstacle_stop_line = msg.at_stop_line

    def cbMode(self, fsm_state_msg):
        self.fsm_state = fsm_state_msg.state  # String of current FSM state

        if self.fsm_state == 'INTERSECTION_CONTROL':
            self.current_pose_source = 'intersection_navigation'
        else:
            self.current_pose_source = 'lane_filter'

        if self.params['~verbose'] == 2:
            self.log("Pose source: %s" % self.current_pose_source)

    def cbAllPoses(self, input_pose_msg, pose_source):
        """Callback receiving pose messages from multiple topics.

        If the source of the message corresponds with the current wanted pose source, it computes a control command.

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
            pose_source (:obj:`String`): Source of the message, specified in the subscriber.
        """
        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg

            self.pose_msg = input_pose_msg

            self.getControlAction(self.pose_msg)

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        """Callback that reports if the requested control action was executed.

        Args:
            msg_wheels_cmd (:obj:`WheelsCmdStamped`): Executed wheel commands
        """
        self.wheels_cmd_executed = msg_wheels_cmd

    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)

    def updatePath(self, homog_msg):
        if homog_msg.H is not None:
            homog_mat = np.array(homog_msg.H).reshape(3,3)
            # Check if we are already very close to identity
            if (np.trace(homog_mat) - 3) > self.params['~homog_tol'] or \
                np.linalg.norm(homog_mat[0:2,2]) > self.params['~homog_tol'] or \
                any(np.isnan(homog_mat[0:2,2])) and \
                rospy.Time.now().to_sec() - self.last_s > self.params['~plan_min_exec_time']:

                print("path updating")

                path, u, dist = self.planner.get_new_path(homog_mat)
                self.path_dist = dist
                self.u = u
                self.last_s = rospy.Time.now().to_sec()
            else:
                self.path_dist = 0
                self.u = None

    def getControlAction(self, pose_msg):
        """Callback that receives a pose message and updates the related control command.

        Using a controller object, computes the control action using the current pose estimate.

        Args:
            pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        current_s = rospy.Time.now().to_sec()
        dt = None
        if self.last_s is not None:
            dt = (current_s - self.last_s)
        if self.at_stop_line or self.at_obstacle_stop_line:
            v = 0
            omega = 0
        elif self.u is not None:
            v, omega = self.controller.compute_control_action(self.u, self.path_dist, dt)
        else:
            v = 0.0
            omega = 0.0

        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        # Add commands to car message
        print("v: " + str(self.params['~k_v']*v))
        print("omega: " + str(self.params['~k_omega']*omega))
        car_control_msg.v = self.params['~k_v']*v
        car_control_msg.omega = self.params['~k_omega']*omega

        self.publishCmd(car_control_msg)
        

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
