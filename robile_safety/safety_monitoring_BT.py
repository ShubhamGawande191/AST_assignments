#!/usr/bin/env python3

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys
from robile_safety.behaviors import *


def create_root() -> pt.behaviour.Behaviour:
    """Structures a behavior tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")

    # instantiate behaviors
    battery_status = BatteryStatus2bb(name="BatteryStatus2bb")
    laser_scan = LaserScan2bb(name="LaserScan2bb")

    # add battery_status and laser_scan to topics2BB
    topics2BB.add_children([battery_status, laser_scan])

    # create sequences for collision
    collision_sequence = pt.composites.Sequence("CollisionSequence", memory=False)
    collision_check = IsColliding(name="IsColliding")
    stop = StopMotion(name="StopPlatform")
    collision_sequence.add_children([collision_check, stop])

    # add battery check to battery sequence
    battery_sequence = pt.composites.Sequence("BatterySequence", memory=False)
    battery_check = IsBatteryLow(name="IsBatteryLow")
    rotate = Rotate(name="Rotate")
    battery_sequence.add_children([battery_check, rotate])
    
    # idle behavior
    idle = pt.behaviours.Running(name="Idle")

    # add sequences to priorities selector
    priorities.add_children([collision_sequence, battery_sequence, idle])
    
    # add topics2BB and priorities to root
    root.add_children([topics2BB, priorities])

    return root

def main():
    """Initialises and executes the behavior tree
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
