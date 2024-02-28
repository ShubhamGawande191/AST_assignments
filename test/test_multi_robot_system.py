import sys
import unittest
import rclpy

class TestNumberOfRobots(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected_number_of_robots = int(sys.argv[1])  # Get expected number of robots from arguments
        rclpy.init(args=sys.argv)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_robot_count(self):
        node = rclpy.create_node('test_robot_count_node')
        discovered_robots = []

        def topic_callback(topic_list):
            nonlocal discovered_robots
            for topic_name, _ in topic_list:
                if 'joint_states' in topic_name and topic_name.startswith('/robot_'):
                    robot_name = topic_name.split('/')[1]
                    if robot_name not in discovered_robots:
                        discovered_robots.append(robot_name)

        node.get_topic_names_and_types(node_name=None, node_namespace=None, no_demangle=False, callback=topic_callback)
        rclpy.spin_once(node, timeout_sec=1.0)
        self.assertEqual(len(discovered_robots), self.expected_number_of_robots)
        node.destroy_node()

if __name__ == '__main__':
    unittest.main() 
