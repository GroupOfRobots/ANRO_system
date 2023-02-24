import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot


class TestConnection(Node):

    def __init__(self):
        super().__init__('Connection_test')
        print("Executing homing procedure")
        bot.set_homing_command(0) 

def main(args=None):
    rclpy.init(args=args)

    conn_tester = TestConnection()

    conn_tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()