import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # 起動ノードの作成
    state_robot_node = Node(
        package = 'state_robot',
        executable = 'state_publisher',
        output='screen'
    )

    # #ベースリンクからレーザ座標系への変換
    # base_link2laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments= ["0", "0", "0", "0", "0", "0", "base_link", "laser"],
    # )

    #Lidar linkとlaser frameの変換
    lidar_link2laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    # map2odom=Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments= ["0", "0", "0", "0", "0", "0", "map", "odom"],
    # )

    # world2map = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments= ["0", "0", "0", "0", "0", "0", "world", "map"],
    # )

    any2miradar=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "base_link", "miradar"],
    )

    any2lidar=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "base_link", "laser"],
    )

    # 起動の作成
    urdf_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urdf_test'), 'launch/'),
            'robot_pub_urdf_launch.launch.py'])
    )

    urg_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urg_node2'), 'launch/'),
            'urg_node2.launch.py'])
    )

    imu_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('adi_imu_tr_driver_ros2'), 'launch/'),
            'adis_rcv_csv.launch.py'])
    )

    tof_launch = launch.actions.IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
           get_package_share_directory('tof_ros2cpp'), 'launch/'),
           'camera_EVAL-ADTF3175D-NXZ.launch.xml'])
    )

    v4l2_camera = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            parameters= [{"video_device": "/dev/video_logitech"}],
    )

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments= ['-d' + 'config/adi_robot_rviz.rviz'],
    )


    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    #Rviz2
    launch_discription.add_entity(state_robot_node)
    print("\033[92m[launch]state_robot launch add\033[0m")


    #launch_discription.add_entity(lidar_link2laser)


    launch_discription.add_action(urdf_launch)
    print("\033[92m[launch]urdf launch add\033[0m")

    launch_discription.add_entity(any2miradar)
    launch_discription.add_entity(any2lidar)
    print("\033[92m[launch]miradar2odom launch add\033[0m")

    launch_discription.add_entity(lidar_link2laser)#必要かどうかわからない（TFのlaunchには順序がある）

    launch_discription.add_entity(v4l2_camera)
    print("\033[92m[launch]UVC_camera launch add\033[0m")

    launch_discription.add_action(imu_launch)
    launch_discription.add_action(urg_launch)
    launch_discription.add_action(tof_launch)
    print("\033[92m[launch]tof_camera launch add\033[0m")

    launch_discription.add_action(rviz2_node)

    return launch_discription
