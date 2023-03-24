import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped 
import tf_transformations
import math
from contextlib import suppress
from dobot_msgs.msg import DobotAlarmCodes
import os
from std_msgs.msg import Float64

from dobot_driver.dobot_handle import bot

class DobotPublisher(Node):

    def __init__(self):
        super().__init__('dobot_state_publisher')
        self.publisher_joints = self.create_publisher(JointState, 'dobot_joint_states', 10)
        self.publisher_TCP = self.create_publisher(PoseStamped, 'dobot_TCP', 10)
        self.publisher_alarms = self.create_publisher(DobotAlarmCodes, 'dobot_alarms', 10)
        timer_period = 0.05  # 50ms = 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        timer_period_alarms = 0.1 # 100ms = 10 Hz
        self.timer_alarms = self.create_timer(timer_period_alarms, self.timer_callback_alarms)
        self.RAIL_IN_USE = False

        if os.environ.get('MAGICIAN_RAIL_IN_USE') == "true":
            self.RAIL_IN_USE = True
            self.publisher_rail = self.create_publisher(Float64, 'dobot_rail_pose', 10)

    def ignore_msg(self):
        bot.serial.reset_input_buffer()

    @staticmethod
    def check_received_msg_type(msg):
        for digit in msg:
            if isinstance(digit, float):
                return False
        return True

    def timer_callback_alarms(self):
        with suppress(ValueError, TypeError):
            alarms_array = bot.get_alarms_state()
            # self.get_logger().info("Raw alarms:  {0}".format(alarms_array))
            if (self.RAIL_IN_USE == True) and (isinstance(alarms_array, float)):
                self.timer_alarms.reset()
            if alarms_array == None:
                self.ignore_msg()
            if alarms_array != None:
                if DobotPublisher.check_received_msg_type(alarms_array):
                    alarms_msg = DobotAlarmCodes()
                    alarms_msg.header.stamp = self.get_clock().now().to_msg()
                    for i, x in enumerate(alarms_array):
                        for j in range(8):
                            if x & (1<<j) > 0:
                                alarm_value = 8*i + j
                                alarms_msg.alarms_list.append(int(f'{alarm_value:x}'))
                    self.publisher_alarms.publish(alarms_msg)
                else:
                    self.ignore_msg()


    def timer_callback(self):

        with suppress(ValueError, TypeError):
            [x, y, z, r, theta1, theta2, theta3, theta4] = bot.get_pose()

            joint_state = JointState()
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4']
            joint_state.position = [math.radians(theta1), math.radians(theta2), math.radians(theta3), math.radians(theta4)]
            self.publisher_joints.publish(joint_state)

            ####################################################
            dobot_TCP = PoseStamped()
            dobot_TCP.header.stamp = self.get_clock().now().to_msg()
            dobot_TCP.pose.position.x = x/1000
            dobot_TCP.pose.position.y = y/1000
            dobot_TCP.pose.position.z = z/1000

            q = tf_transformations.quaternion_from_euler(0, 0, r * math.pi/180)
            dobot_TCP.pose.orientation.x = q[0]
            dobot_TCP.pose.orientation.y = q[1]
            dobot_TCP.pose.orientation.z = q[2]
            dobot_TCP.pose.orientation.w = q[3]
            self.publisher_TCP.publish(dobot_TCP)
            ####################################################

            if self.RAIL_IN_USE == True:
                dobot_rail_pose = Float64()
                dobot_rail_pose.data = float(bot.get_sliding_rail_pose())
                self.publisher_rail.publish(dobot_rail_pose)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DobotPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()