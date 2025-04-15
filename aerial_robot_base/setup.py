from setuptools import setup
import os
from glob import glob

package_name = 'aerial_robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Junichiro',
    author_email='root@todo.jp',
    maintainer='root',
    maintainer_email='root@todo.jp',
    description='ROS2 port of aerial_robot_base package',
    # install_requires=['setuptools', 'ros2_numpy'],
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "robot_interface_node = aerial_robot_base.robot_interface_node:main"
        ],
    },
)
