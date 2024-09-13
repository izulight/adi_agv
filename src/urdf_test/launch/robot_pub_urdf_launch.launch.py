import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
import launch
import launch_ros.actions
import xacro
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

# share_dir_path = os.path.join(get_package_share_directory('urdf'))

urdf_path = os.path.join("/home/analog/robot_ws/src/urdf_test/urdf/agv3.urdf")


# Pose where we want to spawn the robot
spawn_x_val = '0.0'
spawn_y_val = '0.0'
spawn_z_val = '0.0'
spawn_yaw_val = '0.0'

def generate_launch_description():
    robot_name_in_model = 'sam_bot'

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  #node_executable='robot_state_publisher',
                                  output='both',
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])

    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": [urdf_path]}],
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": [urdf_path]}],
    )

    return launch.LaunchDescription([
        rsp, 
        joint_state_pub_gui_node,
        joint_state_pub_node
        ])
