from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('dobot_PTP_client')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        self.cli = self.create_client(srv_type = GripperControl, srv_name = 'dobot_gripper_service', callback_group = ReentrantCallbackGroup())
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GripperControl.Request()

        self.goals = []

    def send_request(self, gripper_state, keep_compressor_running):
        self.req.gripper_state = gripper_state
        self.req.keep_compressor_running = keep_compressor_running
        return self.cli.call(self.req)


    def cancel_done(self, future): 
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            if self.goals: 
                self.goals.pop(0)
                self.send_goal(self.goals[0])
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')


        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.achieved_pose))
            print("self goals: ", self.goals)
            time.sleep(self.goals[0][3])
            response = self.send_request(self.goals[0][2], False)
            self.get_logger().info(
            'Result of calling service: %s' %(response))
            self.goals.pop(0)
            if self.goals:
                time.sleep(self.goals[0][3])
            if self.goals: 
                self.send_goal(self.goals[0])
            else:
                rclpy.shutdown()
        elif status == GoalStatus.STATUS_CANCELED:
            print("self goals: ", self.goals)
            if self.goals: 
                self.goals.pop(0)
                self.send_goal(self.goals[0])
        
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def send_goal(self, _goal):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _goal[0]
        goal_msg.motion_type = _goal[1]
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = PickAndPlace()


    action_client.goals = [([125.0, -180.0, 60.0, 0.0], 1, "open", 2), ([230.0, 0.0, 50.0, 0.0], 1, "close", 2), ([150.0, 190.0, 50.0, 0.0], 1, "open", 2)]

    action_client.send_goal(action_client.goals[0])

    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)




if __name__ == '__main__':
    main()
