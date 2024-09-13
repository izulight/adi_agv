import rclpy
from rclpy.node import Node
from adi_tmcl.srv import TmcCustomCmd

class Clear(Node):
    def __init__(self):
        super().__init__("brake_activate_node")

        self.cli = self.create_client(TmcCustomCmd, '/tmcm1/tmcl_custom_cmd')

        tx = TmcCustomCmd.Request()
        tx.instruction = "SAP"
        tx.instruction_type = 80
        tx.motor_num = 0
        tx.value = 1

        f0 = self.cli.call_async(tx)
        self.get_logger().info("motor0 brake PWM activate")
        print(f0.result())

        
        tx.motor_num = 1
        f1 = self.cli.call_async(tx)
        self.get_logger().info("motor1 brake PWM activate")
        print(f1.result())


def main():
    rclpy.init()
    Clear()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
