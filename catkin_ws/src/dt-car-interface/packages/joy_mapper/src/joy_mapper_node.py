#!/usr/bin/env python
import rospy
import math

from duckietown import DTROS

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy


# Button List index of joy.buttons array:
#   0: A
#   1: B
#   2: X
#   3: Y
#   4: Left Back
#   5: Right Back
#   6: Back
#   7: Start
#   8: Logitek
#   9: Left joystick
#   10: Right joystick


class JoyMapperNode(DTROS):
    """Interprets the Joystick commands.

    The `JoyMapperNode` receives :obj:`Joy` messages from a phisical joystick or a virtual one,
    interprets the buttons presses and acts accordingly.

    **Joystick bindings:**

    +----------------------+------------------+------------------------------------------------+
    | Physical joystick    | Virtual joystick | Action                                         |
    +======================+==================+================================================+
    | Directional controls | Arrow keys       | Move the Duckiebot (if not in lane-following)  |
    +----------------------+------------------+------------------------------------------------+
    | Start button         | `A` key          | Start lane-following                           |
    +----------------------+------------------+------------------------------------------------+
    | Back button          | `S` key          | Stop lane-following                            |
    +----------------------+------------------+------------------------------------------------+
    | Y button             | `E` key          | Toggle Emergency Stop                          |
    +----------------------+------------------+------------------------------------------------+

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~speed_gain (:obj:`float`): Gain for the directional joystick keys (forward/reverse)
        ~steer_gain (:obj:`int`): Gain for the directional joystick keys (steering angle)
        ~bicycle_kinematics (:obj:`bool`): `True` for bicycle kinematics; `False` for holonomic
           kinematics. Default is `False`
        ~simulated_vehicle_length (:obj:`float`): Used in bicycle kinematics model

    Subscriber:
        joy (:obj:`Joy`): The command read from joystick
        emergency_stop (:obj:`BoolStamped`): The emergency stop status

    Publishers:
        ~car_cmd (:obj:`duckietown_msgs/Twist2DStamped`): Wheels command for Duckiebot, based
           on the directional buttons pressed
        ~joystick_override (:obj:`duckietown_msgs/BoolStamped`): Boolean that is used to control
           whether lane-following or joystick control is on
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(JoyMapperNode, self).__init__(node_name=node_name)

        # emergency stop disabled by default
        self.e_stop = False

        # Add the node parameters to the parameters dictionary
        self.parameters['~speed_gain'] = None
        self.parameters['~steer_gain'] = None
        self.parameters['~bicycle_kinematics'] = None
        self.parameters['~simulated_vehicle_length'] = None
        self.updateParameters()

        # Publications
        self.pub_car_cmd = self.publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = self.publisher("~joystick_override", BoolStamped, queue_size=1)
        self.pub_e_stop = self.publisher("~emergency_stop", BoolStamped, queue_size=1)

        # Subscription to the joystick command
        self.sub_joy = self.subscriber("~joy", Joy, self.cbJoy, queue_size=1)
        self.sub_e_stop = self.subscriber(
            "~emergency_stop", BoolStamped, self.cbEStop, queue_size=1
        )

    def cbEStop(self, estop_msg):
        """
        Callback that process the received :obj:`BoolStamped` messages.

        Args:
            estop_msg (:obj:`BoolStamped`): the emergency_stop message to process.
        """
        self.e_stop = estop_msg.data

    def cbJoy(self, joy_msg):
        """
        Callback that process the received :obj:`Joy` messages.

        Args:
            joy_msg (:obj:`Joy`): the joystick message to process.
        """
        # Navigation buttons
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.get_rostime()
        car_cmd_msg.v = joy_msg.axes[1] * self.parameters['~speed_gain']  # Left stick V-axis. Up is positive
        if self.parameters['~bicycle_kinematics']:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = joy_msg.axes[3] * self.parameters['~steer_gain']
            car_cmd_msg.omega = \
                car_cmd_msg.v / self.parameters['~simulated_vehicle_length'] * \
                math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = joy_msg.axes[3] * self.parameters['~steer_gain']
        self.pub_car_cmd.publish(car_cmd_msg)

        # Back button: Stop LF
        if joy_msg.buttons[6] == 1:
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = True
            self.log('override_msg = True')
            self.pub_joy_override.publish(override_msg)
            
        # Start button: Start LF
        elif joy_msg.buttons[7] == 1:
            override_msg = BoolStamped()
            override_msg.header.stamp = joy_msg.header.stamp
            override_msg.data = False
            self.log('override_msg = False')
            self.pub_joy_override.publish(override_msg)

        # Y button: Emergency Stop
        elif joy_msg.buttons[3] == 1:
            self.e_stop = not self.e_stop
            estop_msg = BoolStamped()
            estop_msg.header.stamp = joy_msg.header.stamp
            estop_msg.data = self.e_stop
            self.pub_e_stop.publish(estop_msg)

        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                self.log('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))


if __name__ == "__main__":
    # Initialize the node with rospy
    joy_mapper = JoyMapperNode(node_name="joy_mapper")
    # Keep it spinning to keep the node alive
    rospy.spin()
