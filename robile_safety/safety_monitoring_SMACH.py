#!/usr/bin/env python3

import rclpy
import smach

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import yaml
import time

# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but in ROS1]

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery', 'collision_detected', 'idle'])
        self.node = node
        self.battery_level_sub = self.node.create_subscription(Float32, 'battery_voltage', self.battery_callback,10)
        self.laser_scan_sub = self.node.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.battery_lvl = None
        self.collision_detected = False

    def battery_callback(self, msg):   
        # check the battery level
        self.battery_lvl = msg.data

    def scan_callback(self, msg):
        # check if the robot collided
        if min(msg.ranges) < 0.5:
            self.collision_detected = True
        else:
            self.collision_detected = False

    def execute(self, userdata):
        """Checks the battery level and possible collisions
        """
        battery_threshold = 20.0

        while rclpy.ok():
            rclpy.spin_once(self.node)

            # check if the robot collided
            if self.collision_detected:
                return 'collision_detected'
            # check battery level and return outcome
            elif self.battery_lvl is not None and self.battery_lvl < battery_threshold: 
                return 'low_battery'
            # safe to continue
            else:
                return 'idle'

class RotateBase(smach.State):
    """State to rotate the Robile base
    """
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['rotated', 'failed'])
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self, userdata):
        """Rotates the base for 5 seconds
        """
        twist_start = Twist()
        twist_start.angular.z = 0.5
        self.cmd_vel_pub.publish(twist_start)

        # rotate for 5 seconds
        time.sleep(5)

        # stop rotating
        # twist.angular.z = 0.0
        twist_stop = Twist()
        self.cmd_vel_pub.publish(twist_stop)
        return 'rotated'

class Stop(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['stopped'])
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0.0
        # stop rotating
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        return 'stopped'
    
class Idle(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['idle'])
        self.node = node

    def execute(self, userdata):
        time.sleep(1)
        return 'idle'

def main(args=None):
    """Main function to initialise and execute the state machine
    """
    rclpy.init(args=args)
    node = rclpy.create_node('sm_node')

    # create SMACH state machine
    sm = smach.StateMachine(outcomes=['executed', 'terminated'])

    with sm:
        # add monitor and collision states
        smach.StateMachine.add('MONITOR', MonitorBatteryAndCollision(node),
                               transitions={'low_battery': 'ROTATE_BASE', 
                                            'collision_detected': 'STOP',
                                            'idle': 'IDLE'})
        smach.StateMachine.add('ROTATE_BASE', RotateBase(node),
                               transitions={'rotated':'MONITOR',
                                            'failed': 'STOP'})
        smach.StateMachine.add('STOP', Stop(node),
                                 transitions={'stopped': 'MONITOR'})
        smach.StateMachine.add('IDLE', Idle(node),
                                 transitions={'idle': 'MONITOR'})
    
    # execute SMACH plan
    outcome = sm.execute()

    # shutdown node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()