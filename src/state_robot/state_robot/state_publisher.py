import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi

from adi_tmcl.msg import TmcInfo

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.voltage_publisher_0 = self.create_publisher(Float32, 'viz/voltage_0', 10)
        self.voltage_publisher_1 = self.create_publisher(Float32, 'viz/voltage_1', 10)
        self.speed_r_publisher = self.create_publisher(Float32, 'viz/speed_r', 10)
        self.speed_l_publisher = self.create_publisher(Float32, 'viz/speed_l', 10)

        # IMU data subscription
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)

        self.tmc_info_subscription_0 = self.create_subscription(
            TmcInfo,
            '/tmc_info_0',
            self.tmc_info_0_callback,
            10)

        self.tmc_info_subscription_1 = self.create_subscription(
            TmcInfo,
            '/tmc_info_1',
            self.tmc_info_1_callback,
            10)

        self.broadcaster = TransformBroadcaster(self)

        self.declare_parameter('wheel_separation', 0.517)
        self.declare_parameter('wheel_radius', 0.1)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.right_wheel_pos = 0.0
        self.left_wheel_pos = 0.0

        self.right_wheel_vel = 0.0
        self.left_wheel_vel = 0.0

        self.joint_state = JointState()
        self.joint_state.name = ['drivewhl_r_joint', 'drivewhl_l_joint']
        self.joint_state.position = [self.right_wheel_pos, self.left_wheel_pos]

        # ロボットの位置
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        #IMUの姿勢
        self.imu_orientation = Imu()

        # エンコーダの設定
        #self.encoder_resolution = 1 / (0.3515625 * (pi / 180))
        self.encoder_resolution = (0.3515625* (pi / 180))

        self.left_wheel_last_count = 0  # 前回のカウント数格納
        self.right_wheel_last_count = 0

        #基板電圧
        self.voltage_0 = 0.0
        self.voltage_1 = 0.0

        #速度
        self.speed_mps_r = 0.0
        self.speed_mps_l = 0.0

        # Timer to update odometry
        self.timer = self.create_timer(0.1, self.update_odometry)

    def tmc_info_0_callback(self, msg):
        # 左ホイールのカウントの差分を計算
        pulse_count_diff = msg.position - self.left_wheel_last_count
        self.left_wheel_last_count = msg.position

        # カウント差分を角度に変換
        angle_rad = pulse_count_diff * (0.3515625 * (pi / 180))

        # ホイールの直線速度の計算
         self.left_wheel_vel = -1 * angle_rad * self.wheel_radius
        # self.left_wheel_vel = -1 * angle_rad /0.1

        # ホイールの累積位置の更新
        self.left_wheel_pos += -1 * angle_rad

        # 基板電圧の更新
        viz_voltage = Float32()
        viz_voltage.data = msg.board_voltage
        self.voltage_publisher_0.publish(viz_voltage)


    def tmc_info_1_callback(self, msg):
        # 右ホイールのカウントの差分を計算
        pulse_count_diff = msg.position - self.right_wheel_last_count
        self.right_wheel_last_count = msg.position

        # カウント差分を角度に変換
        angle_rad = pulse_count_diff * (0.3515625 * (pi / 180))

        # ホイールの直線速度の計算
        self.right_wheel_vel = angle_rad * self.wheel_radius
        # self.right_wheel_vel = angle_rad * self.wheel_radius


        # ホイールの累積位置の更新
        self.right_wheel_pos += angle_rad

        # 基板電圧の更新
        viz_voltage = Float32()
        viz_voltage.data = msg.board_voltage
        self.voltage_publisher_1.publish(viz_voltage)


    def imu_callback(self, msg):
        # self.imu_orientation = msg.orientation
        # print(self.imu_orientation)

    def update_odometry(self):
        dt = 0.1  # assuming a control loop running at 10Hz

        # Calculate linear and angular velocities
        linear_vel = (self.right_wheel_vel + self.left_wheel_vel) / 2.0
        angular_vel = (self.right_wheel_vel - self.left_wheel_vel) / (self.wheel_separation)

        # Update robot's position
        dx = linear_vel * cos(self.th) * dt
        dy = linear_vel * sin(self.th) * dt
        dth = angular_vel * dt

        self.x += dx
        self.y += dy
        self.th += dth

        # Publish the joint states
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.publisher_.publish(self.joint_state)

        # Broadcast the transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = sin(self.th / 2.0)
        transform.transform.rotation.w = cos(self.th / 2.0)

        self.broadcaster.sendTransform(transform)

        # IMU Transform (fixed to base_link)
        
        imu_transform = TransformStamped()
        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = 'odom'#odom
        imu_transform.child_frame_id = 'imu_link'

        imu_transform.transform.translation.x = 0.0
        imu_transform.transform.translation.y = 0.0
        imu_transform.transform.translation.z = 0.3  # IMUがbase_linkから0.3m上に配置されている場合
        #imu_transform.transform.rotation = self.imu_orientation


        imu_transform.transform.rotation.x = 0.0
        imu_transform.transform.rotation.y = 0.0
        imu_transform.transform.rotation.z = 0.0
        imu_transform.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(imu_transform)

        # Print calculated linear and angular velocities
        # self.get_logger().info(f'Linear Velocity: {linear_vel:.2f} m/s, Angular Velocity: {angular_vel:.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
