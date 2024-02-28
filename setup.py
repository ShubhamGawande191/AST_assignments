from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robile_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share/', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools', 'py_trees', 'rclpy'],
    zip_safe=True,
    maintainer='beelzebub',
    maintainer_email='shubhamgawande191@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    test_suite='test',
    entry_points={
        'console_scripts': [
        'safety_monitoring_bt = robile_safety.safety_monitoring_BT:main',
        'safety_monitoring_smach = robile_safety.safety_monitoring_SMACH:main',
        ],
    },
)
