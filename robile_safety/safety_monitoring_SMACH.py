#!/usr/bin/env python3

import rclpy
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        raise NotImplementedError()

    
class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        # TODO: define outcomes, class variables, and desired publisher/subscribers
        ### YOUR CODE HERE ###
        raise NotImplementedError()

    def execute(self, userdata):
        # TODO: implement state execution logic and return outcome
        ### YOUR CODE HERE ###
        raise NotImplementedError()


# TODO: define any additional if necessary
### YOUR CODE HERE ###

def main(args=None):
    """Main function to initialise and execute the state machine
    """

    # TODO: make it a ROS2 node, set any threshold values, and define state transitions
    ### YOUR CODE HERE ###

if __name__ == "__main__":
    main()