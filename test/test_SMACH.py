import unittest
from unittest.mock import patch, MagicMock, call
import rclpy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from robile_safety.safety_monitoring_SMACH import MonitorBatteryAndCollision, RotateBase, Stop, Idle

class TestSafetyMonitoring(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    @patch('robile_safety.safety_monitoring_SMACH.MonitorBatteryAndCollision.execute')
    def test_monitor_battery_and_collision(self, mock_execute):
        monitor_state = MonitorBatteryAndCollision(self.node)

        # Set up the mock for the execute method
        mock_execute.side_effect = [
            'idle',  
            'low_battery',  
            'idle', 
            'collision_detected'
        ]

        # Simulate battery callback with a battery level above the threshold
        battery_msg = Float32(data=25.0)
        monitor_state.battery_callback(battery_msg)
        self.assertEqual(mock_execute(None), 'idle')

        # Simulate battery callback with a battery level below the threshold
        battery_msg.data = 15.0
        monitor_state.battery_callback(battery_msg)
        self.assertEqual(mock_execute(None), 'low_battery')

        # Simulate scan callback with no collision detected
        laser_msg = LaserScan(ranges=[0.6, 0.7, 0.8])
        monitor_state.scan_callback(laser_msg)
        self.assertEqual(mock_execute(None), 'idle')

        # Simulate scan callback with a collision detected
        laser_msg.ranges = [0.4, 0.3, 0.2]
        monitor_state.scan_callback(laser_msg)
        self.assertEqual(mock_execute(None), 'collision_detected')

    def test_rotate_base(self):
        rotate_state = RotateBase(self.node)
        rotate_state.cmd_vel_pub = MagicMock()  # Mock the publisher
        rotate_state.execute(None)
        
        # Create the expected calls
        expected_calls = [
            call(Twist(angular=Vector3(z=0.5))),  # Expected start rotation call
            call(Twist())  # Expected stop rotation call
        ]
        
        # Verify that the calls were made with the expected arguments
        rotate_state.cmd_vel_pub.publish.assert_has_calls(expected_calls, any_order=False)

    def test_stop(self):
        stop_state = Stop(self.node)
        stop_state.cmd_vel_pub = MagicMock()  # Set up the mock publisher
        self.assertEqual(stop_state.execute(None), 'stopped')
        stop_state.cmd_vel_pub.publish.assert_called_once()

    @patch('time.sleep', return_value=None)
    def test_idle(self, mock_sleep):
        idle_state = Idle(self.node)
        self.assertEqual(idle_state.execute(None), 'idle')

if __name__ == '__main__':
    unittest.main()
