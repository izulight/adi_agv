# Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (LaunchConfiguration, PythonExpression)
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  ## Get directory of config files
  tmcl_ros2_pkg_dir = get_package_share_directory('adi_tmcl')
  ## Include YAML file
  autogenerated_yaml = os.path.join(tmcl_ros2_pkg_dir, 'config', 'autogenerated', 'TMCM-1230.yaml')
  ext_yaml = os.path.join(tmcl_ros2_pkg_dir, 'config', 'TMCM-1230_Ext.yaml')

  # Declare the Launch Arguments with default value
  arg_log_level = DeclareLaunchArgument('log_level', default_value='INFO')
  log_level_ = LaunchConfiguration('log_level')

  arg_ns_prefix = DeclareLaunchArgument('ns_prefix', default_value='tmcm1')
  ns_prefix_ = LaunchConfiguration('ns_prefix')

  arg_tmcl_base_name = DeclareLaunchArgument('tmcl_base_name', default_value='tmcm1230')
  tmcl_base_name_ = LaunchConfiguration('tmcl_base_name')

  arg_board_parent_frame = DeclareLaunchArgument('board_parent_frame', default_value='map')
  arg_board_base_name = DeclareLaunchArgument('board_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_base_link'])
  arg_board_pos_x = DeclareLaunchArgument('board_pos_x', default_value='0')
  arg_board_pos_y = DeclareLaunchArgument('board_pos_y', default_value='0')
  arg_board_pos_z = DeclareLaunchArgument('board_pos_z', default_value='0')
  arg_board_pos_qx = DeclareLaunchArgument('board_pos_qx', default_value='0')
  arg_board_pos_qy = DeclareLaunchArgument('board_pos_qy', default_value='0')
  arg_board_pos_qz = DeclareLaunchArgument('board_pos_qz', default_value='0')
  arg_board_pos_qw = DeclareLaunchArgument('board_pos_qw', default_value='1')

  board_parent_frame_ = LaunchConfiguration('board_parent_frame')
  board_base_name_ = LaunchConfiguration('board_base_name')
  board_pos_x_ = LaunchConfiguration('board_pos_x')
  board_pos_y_ = LaunchConfiguration('board_pos_y')
  board_pos_z_ = LaunchConfiguration('board_pos_z')
  board_pos_qx_ = LaunchConfiguration('board_pos_qx')
  board_pos_qy_ = LaunchConfiguration('board_pos_qy')
  board_pos_qz_ = LaunchConfiguration('board_pos_qz')
  board_pos_qw_ = LaunchConfiguration('board_pos_qw')
  # motor0 Arguments
  arg_mtr0_base_name = DeclareLaunchArgument('mtr0_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr0_pos_x = DeclareLaunchArgument('mtr0_pos_x', default_value='0')
  arg_mtr0_pos_y = DeclareLaunchArgument('mtr0_pos_y', default_value='1')
  arg_mtr0_pos_z = DeclareLaunchArgument('mtr0_pos_z', default_value='0')
  arg_mtr0_pos_qx = DeclareLaunchArgument('mtr0_pos_qx', default_value='0')
  arg_mtr0_pos_qy = DeclareLaunchArgument('mtr0_pos_qy', default_value='0')
  arg_mtr0_pos_qz = DeclareLaunchArgument('mtr0_pos_qz', default_value='0')
  arg_mtr0_pos_qw = DeclareLaunchArgument('mtr0_pos_qw', default_value='1')

  mtr0_base_name_ = LaunchConfiguration('mtr0_base_name')
  mtr0_pos_x_ = LaunchConfiguration('mtr0_pos_x')
  mtr0_pos_y_ = LaunchConfiguration('mtr0_pos_y')
  mtr0_pos_z_ = LaunchConfiguration('mtr0_pos_z')
  mtr0_pos_qx_ = LaunchConfiguration('mtr0_pos_qx')
  mtr0_pos_qy_ = LaunchConfiguration('mtr0_pos_qy')
  mtr0_pos_qz_ = LaunchConfiguration('mtr0_pos_qz')
  mtr0_pos_qw_ = LaunchConfiguration('mtr0_pos_qw')


  # ADI TMCL ROS2 Node
  tmcm_1230_node = Node(
    package="adi_tmcl",
    executable="tmcl_ros2_node",
    name="tmcl_ros2_node",
    namespace=ns_prefix_,
    emulate_tty=True,
    parameters=[
      autogenerated_yaml,
      ext_yaml],
    arguments=["--ros-args", "--log-level", PythonExpression(expression=["'",ns_prefix_,".tmcl_ros2_node:=",log_level_,"'"])]
  )

  # TF Node
  tf_board = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", board_pos_x_, "--y", board_pos_y_, "--z", board_pos_z_,
      "--qx", board_pos_qx_, "--qy", board_pos_qy_, "--qz", board_pos_qz_, "--qw", board_pos_qw_,
      "--frame-id", board_parent_frame_, "--child-frame-id", board_base_name_]
  )

  tf_mtr0 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr0_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr0_pos_x_, "--y", mtr0_pos_y_, "--z", mtr0_pos_z_,
      "--qx", mtr0_pos_qx_, "--qy", mtr0_pos_qy_, "--qz", mtr0_pos_qz_, "--qw", mtr0_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr0_base_name_]
  )

  return LaunchDescription([
      arg_log_level,
      arg_ns_prefix,
      arg_tmcl_base_name,
      arg_board_parent_frame,
      arg_board_base_name,arg_board_pos_x, arg_board_pos_y, arg_board_pos_z, arg_board_pos_qx, arg_board_pos_qy, arg_board_pos_qz, arg_board_pos_qw,
      arg_mtr0_base_name, arg_mtr0_pos_x, arg_mtr0_pos_y, arg_mtr0_pos_z, arg_mtr0_pos_qx, arg_mtr0_pos_qy, arg_mtr0_pos_qz, arg_mtr0_pos_qw,
      tmcm_1230_node,
      tf_board,
      tf_mtr0,
  ])
