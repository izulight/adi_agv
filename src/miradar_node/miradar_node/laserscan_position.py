#!/usr/bin/env python3

# 参考・改変元 (ros2 set parameter): https://answers.ros.org/question/343322/ros2rclpy-how-to-set-parameter-hosted-by-another-node-via-service/
# 参考・改変元 (laser scan): https://qiita.com/Yuya-Shimizu/items/413b57b0305597be35be

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

PI = 3.141592

class LaserScanPosition(Node):
  def __init__(self):
    super().__init__("laserscan_position")

    self.create_subscription(LaserScan, "/miradar/scan", self.laserScanCallback, 10)  # Map Mode で使用するため，LaserScan Message を Subscribe する．
    self.viz_range_nearest_pub = self.create_publisher(Float32, "/viz/range_nearest", 10)  # 一番近い障害物までの距離 [m]

    self.range_front = 999
    self.range_nearest_obstacle = 999
    self.range_index = 999
    self.angle_nearest_obstacle = 999


    ### miradar_nodeのパラメータを変更するプログラム
          # miradar_node の launch では，01:sensor_mode が Map Mode になっておらず，手動で入力する必要があった．そのため，本 node の起動と同時に Map Mode に変更するためのプログラムを追加する．
    self.param_client = self.create_client(SetParameters, "/miradar_node/set_parameters")  # 他 node が持つパラメータを変更するためには，service 通信からパラメータを変更する必要がある．
    while not self.param_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().warn("Waiting for '/miradar_node/set_parameters' service server...")

    self.param_client_reqest = SetParameters.Request()
    self.param_client_reqest.parameters = [Parameter(name="01:sensor_mode", type_=Parameter.Type.INTEGER, value=2).to_parameter_msg()]  # センサモードのパラメータを，"2"に変更．
    self.param_client_future = self.param_client.call_async(self.param_client_reqest)
    
    rclpy.spin_until_future_complete(self, self.param_client_future)
    self.param_client_result = self.param_client_future.result()
    print("param_client_result: " + str(self.param_client_result))


  def laserScanCallback(self, msg):  # LaserScanのPublishをSubscribeして，いくつかの値をprintするCallback
                                    # LaserScan messageの仕様は，こちらを参照：http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html

    if min(msg.ranges) >= msg.range_min:  # 認識距離の下限以下の測定データを排除
      self.range_nearest_obstacle = min(msg.ranges)  # 一番近い障害物までの距離 [m]
      self.range_front = msg.ranges[len(msg.ranges)//2]  # 正面の障害物までの距離 [m]
      self.range_index = msg.ranges.index(min(msg.ranges))  # 一番近い障害物までの距離の，全距離データ配列におけるindex．しかし，値が不安定である．
      self.angle_nearest_obstacle = msg.angle_min + ( msg.ranges.index(min(msg.ranges)) * ( ( abs(msg.angle_min) + abs(msg.angle_max) ) / len(msg.ranges) ) )  # 一番近い障害物の方向 [rad]. しかし，indexの不安定性により，値が不安定である．

      vis_range_nearest = Float32()
      vis_range_nearest.data = self.range_nearest_obstacle
      self.viz_range_nearest_pub.publish(vis_range_nearest)

    print(#'Header: ' + str(msg.header) + '\n' + \
          # '  - angle_min [deg]: ' + str(msg.angle_min) + ', angle_max [deg]: ' + str(msg.angle_max) + '\n' + \
          # '  - range_min [m]: ' + str(msg.range_min) + ', range_max [m]: ' + str(msg.range_max) + '\n' + \
          '  - range_front [m]: ' + str(self.range_front) + '\n' + \
          '  - range_the-nearest-obstacle [m]: ' + str(self.range_nearest_obstacle) + '\n' + \
          ',    index: ' + str(self.range_index) + '\n' + \
          '  - angle_the-nearest-obstacle [deg]: ' + str(self.angle_nearest_obstacle)
    )

def main():
  rclpy.init()
  node = LaserScanPosition()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
