import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from dobot_kinematics.dobot_forward_kin import calc_FwdKin
from math import degrees


import time

# To run the test, type the following command in another console
# ros2 launch dobot_description display.launch.py gui:=true DOF:=3 tool:=none

class FwdKinTester(Node):

    def __init__(self):
        super().__init__('dobot_forward_kinematics')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription_joints = self.create_subscription(
            JointState,
            'joint_states',
            self.joints_positions_callback,
            10)

        self.JT1 = 0
        self.JT2 = 0
        self.JT3 = 0

    def joints_positions_callback(self, msg):
        self.JT1 = degrees(msg.position[0])
        self.JT2 = degrees(msg.position[1])
        self.JT3 = degrees(msg.position[2]) + self.JT2

    def timer_callback(self):

        try:
            trans = self.tf_buffer.lookup_transform(
                'magician_base_link',
                'magician_link_4',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                "Waiting for TF data")
            time.sleep(1)
            return

        self.x, self.y, self.z = calc_FwdKin( self.JT1,  self.JT2,  self.JT3)

        print()
        print("RViz: ", trans.transform.translation.x * 1000, ",", trans.transform.translation.y * 1000, ",", trans.transform.translation.z * 1000)
        print("Fkin: ", self.x * 1000, ",", self.y * 1000, ",", self.z * 1000)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = FwdKinTester()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()