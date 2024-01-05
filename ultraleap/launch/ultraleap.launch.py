import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

  pkg_name = 'ultraleap'
  log_level = 'info'

  ultraleap_joint_info_publisher_node = Node(
    package=pkg_name, executable='ultraleap_joint_info_publisher', arguments=['--ros-args', '--log-level', log_level])

  return LaunchDescription([
    ultraleap_joint_info_publisher_node
  ])
