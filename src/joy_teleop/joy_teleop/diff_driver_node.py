import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import copy
import numpy as np
import math

class DiffDriver(Node):
    def __init__(self):
        super().__init__("diff_driver_node")

        self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        self.publisher_l = self.create_publisher(Twist, '/cmd_vel_0', 10)
        self.publisher_r = self.create_publisher(Twist, '/cmd_vel_1', 10)

        self.publisher_viz_l = self.create_publisher(Float32, '/viz/vel_l', 10)
        self.publisher_viz_r = self.create_publisher(Float32, '/viz/vel_r', 10)

        self.declare_parameter("tread", 0.0)
        self.declare_parameter("wheel_radius", 0.0)
        self.declare_parameter("reverse_l_flag", False)
        self.declare_parameter("reverse_r_flag", False)

        self.get_logger().info("ロボットの速度指令値をモータ速度にしてtmclに送ります")


    def callback(self, msg):
        tread = self.get_parameter("tread").value
        wheel_radius = self.get_parameter("wheel_radius").value

        is_rev_l = self.get_parameter("reverse_l_flag").value
        is_rev_r = self.get_parameter("reverse_r_flag").value

        # モータの速度指令値の算出
        vel_l = (-tread*msg.angular.z + 2.0*msg.linear.x) / (2.0*wheel_radius)
        vel_r = (tread*msg.angular.z + 2.0*msg.linear.x) / (2.0*wheel_radius)

        vel_l_msg = Float32()
        vel_l_msg.data = vel_l
        self.publisher_viz_l.publish(vel_l_msg)

        vel_r_msg = Float32()
        vel_r_msg.data = vel_l
        self.publisher_viz_r.publish(vel_r_msg)

        # Publish
        tx = Twist()
        tx.linear.x = self.get_rpm((-1 if is_rev_l else 1)*vel_l)
        self.publisher_l.publish(tx)
        tx.linear.x = self.get_rpm((-1 if is_rev_r else 1)*vel_r)
        self.publisher_r.publish(tx)

        return

    def get_rpm(self, rad_s):
        return 60 * rad_s /2*math.pi



def main():
    rclpy.init()
    node = DiffDriver()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
