#!/usr/bin/env python3

import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        # inherit all the class variables from the parent class and make it a behavior
        super(Rotate, self).__init__(name)

        # topic to publish rotation commands
        self.topic_name = topic_name
        # angular velocity to rotate the robot
        self.ang_vel = ang_vel
        # publisher to publish rotation commands
        self.cmd_vel_pub = None

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.node = kwargs['node']
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)

        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        twist = Twist()
        twist.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(twist)
        
        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """Trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """

        # stop rotating when behavior finishes
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.logger.debug(f"[{self.name}] Terminating with status: {new_status}.")
        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """

    def __init__(self, name: str, topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        # topic to publish stop commands
        self.topic_name = topic_name
        # publisher for stop commands
        self.cmd_vel_pub = None

    def setup(self, **kwargs):
        """Sets up the publisher to publish stop commands
        """
        self.node = kwargs['node']
        if self.node is None:
            raise RuntimeError("Node not found in kwargs")
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.topic_name, 10)
        
        return True

    def update(self):
        """Publishes a zero velocity command to stop the robot
        """
        twist = Twist()  
        self.cmd_vel_pub.publish(twist)
        return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        """Handles the termination of thestop motion behavior
        """
        self.logger.debug(f"[{self.name}] Terminating with status: {new_status}.")

class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status
    """
    
    def __init__(self, battery_voltage_topic_name: str="/battery_voltage",
                 name: str='Battery2BB',
                 threshold: float=20.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)
        self.threshold = threshold # low battery level threshold

    def update(self):
        """Calls the parent to write the raw data to the blackboard and then check against the
        threshold to determine if a low warning flag should also be updated.
        """
        super().update()

        # check battery level against threshold
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning = True
        else:
            self.blackboard.battery_low_warning = False

        return pt.common.Status.SUCCESS

class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """
    
    def __init__(self, topic_name: str="/scan",
                 name: str='LaserScan2BB',
                 safe_range: float=0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                history=QoSHistoryPolicy.KEEP_LAST,
                                                depth=10))
        
        self.safe_range = safe_range    
        self.blackboard.register_key(key='collision_detected', access=pt.common.Access.WRITE)

    def update(self):
        """Checks laser scan measurements to detect possible collisions
        """
        super().update()

        # check if the robot collided
        if min(self.blackboard.laser_scan) < self.safe_range:
            self.blackboard.collision_detected = True
        else:
            self.blackboard.collision_detected = False

        return pt.common.Status.SUCCESS

class IsBatteryLow(pt.behaviour.Behaviour):
    def __init__(self, name="IsBatteryLow", low_battery_threshold=20.0):
        super().__init__(name)
        self.low_battery_threshold = low_battery_threshold
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="battery", access=pt.common.Access.READ)

    def update(self):
        """Checks if the battery level is below the threshold
        """
        battery_level = self.blackboard.get("battery") 
        if battery_level is None:
            return pt.common.Status.RUNNING
        if battery_level < self.low_battery_threshold:
            return pt.common.Status.SUCCESS  # Battery level is low
        else:
            return pt.common.Status.FAILURE  # Battery level is acceptable

class IsColliding(pt.behaviour.Behaviour):
    def __init__(self, name="IsColliding", collision_threshold=0.5):
        super().__init__(name)
        self.collision_threshold = collision_threshold
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="laser_scan", access=pt.common.Access.READ)

    def update(self):
        """Check if the robot is colliding
        """
        laser_data = self.blackboard.get("laser_scan")
        if laser_data is None:
            return pt.common.Status.RUNNING  # No laser data yet
        if laser_data and any(distance < self.collision_threshold for distance in laser_data):
            return pt.common.Status.SUCCESS  # Collision risk detected
        else:
            return pt.common.Status.FAILURE  # No collision risk
