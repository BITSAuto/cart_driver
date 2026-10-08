"""cart_driver in tesla_sim: perception + safety monitor + driver.

Start the sim first (run_tesla_sim.sh). Launch arguments:
  model       segmentation backend for perception
              (default: the sim-trained RGB-D student)
  perception  also start the traversability stack (default: true)
  cart_params cart_driver parameter file (default: config/tesla_sim.yaml).
              Deliberately not called 'params': launch arguments are global,
              and the included traversability launch reads 'params' for its
              own config -- sharing the name silently fed perception this
              file and dropped its bonnet mask.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MODEL = 'student:' + os.path.expanduser('~/traversability_data/students/b0_rgbd_sim/best.pt')


def generate_launch_description():
    share = get_package_share_directory('cart_driver')
    params = LaunchConfiguration('cart_params')
    sim_time = {'use_sim_time': True}
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('traversability'), 'launch', 'traversability.launch.py')),
        launch_arguments={'semantics': 'true', 'model': LaunchConfiguration('model')}.items(),
        condition=IfCondition(LaunchConfiguration('perception')))
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=DEFAULT_MODEL),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('cart_params', default_value=os.path.join(share, 'config', 'tesla_sim.yaml')),
        perception,
        Node(package='cart_driver', executable='safety', name='cart_safety',
             parameters=[params, sim_time], output='screen'),
        Node(package='cart_driver', executable='driver', name='cart_driver',
             parameters=[params, sim_time], output='screen'),
    ])
