import time
import math
from dobot_msgs.action import PointToPoint
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped 
import tf_transformations
from dobot_driver.dobot_handle import bot
from .PTP_params_class import declare_PTP_params
from rcl_interfaces.msg import SetParametersResult
from threading import Thread

## Validation imports
from dobot_motion.dobot_inv_kin import calc_inv_kin
from dobot_motion.dobot_forward_kin import calc_FwdKin

class DobotPTPServer(Node):

    def __init__(self):
        super().__init__('dobot_PTP_server')

        self._action_server = ActionServer(
            self,
            PointToPoint,
            'PTP_action',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.subscription_joints = self.create_subscription(
            JointState,
            'dobot_joint_states',
            self.joints_positions_callback,
            10)

        self.subscription_TCP = self.create_subscription(
            PoseStamped,
            'dobot_TCP',
            self.tcp_position_callback,
            10)


        self.motion_type = None 
        self.target = [] 
        self.pose_arr = []
        self.dobot_pose = [] 
        self.mode_ACK = False

        self.motion_types_list = [1,2,4,5]

        declare_PTP_params(self)

        self.set_joint_params_thread = Thread(target = self.set_joint_params_func)
        self._is_running_joint_params = True
        self.allow_dynamic_reconfigure_joints = False

        self.set_cartesian_params_thread = Thread(target = self.set_cartesian_params_func)
        self._is_running_cartesian_params = True
        self.allow_dynamic_reconfigure_cartesian = False

        self.joints_sorted = False
        self.cartesian_sorted = False

        self.ready_to_set_second_part = False


        self.add_on_set_parameters_callback(self.parameters_callback)

        # VALIDATION

        self.axis_1_range = {"min": -125, "max": 125}
        self.axis_2_range = {"min": -5, "max": 90}
        self.axis_3_range = {"min": -15, "max": 70}
        self.axis_4_range = {"min": -150, "max": 150}

        # TCP pose before motion execution - in order to determine equation of linear trajectory
        self.dobot_pose_val = [] 

        self.trajectory_points = []

        self.subscription_TCP = self.create_subscription(PoseStamped, 'dobot_TCP', self.tcp_position_callback_val, 10)


    ###################################################################################################
    # Trajectory validation

    def tcp_position_callback_val(self, msg):
            quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            _, _, y = tf_transformations.euler_from_quaternion(quat)
            self.dobot_pose_val = [float(msg.pose.position.x)*1000, float(msg.pose.position.y)*1000, float(msg.pose.position.z)*1000, float(math.degrees(y))]

    def are_angles_in_range(self, angles):
        if (self.axis_1_range["min"] < angles[0] < self.axis_1_range["max"]) and \
           (self.axis_2_range["min"] < angles[1] < self.axis_2_range["max"]) and \
           (self.axis_3_range["min"] < angles[2] < self.axis_3_range["max"]) and \
           (self.axis_4_range["min"] < angles[3] < self.axis_4_range["max"]):
           return True
        return False
    
    def linear_trajecory_to_discrete_waypoints(self, start, target, step_len = 0.5):
        waypoints = []

        x, y, z = [start[0], target[0]], [start[1], target[1]], [start[2], target[2]]

        steps_num = 1
        while True:
            dist_now = math.dist([x[0], y[0], z[0]], [x[0] + (x[1]-x[0])*(1/steps_num), y[0] + (y[1]-y[0])*(1/steps_num), z[0] + (z[1]-z[0])*(1/steps_num)])
            dist_next = math.dist([x[0], y[0], z[0]], [x[0] + (x[1]-x[0])*(1/(steps_num+1)), y[0] + (y[1]-y[0])*(1/(steps_num+1)), z[0] + (z[1]-z[0])*(1/(steps_num+1))])

            if dist_now > step_len and dist_next < step_len:
                steps_num = steps_num + 3
                break
            else:
                steps_num = steps_num + 1

        for t in range(steps_num-1):
            t =t / (steps_num-2)
            waypoints.append([x[0] + (x[1]-x[0])*t, y[0] + (y[1]-y[0])*t, z[0] + (z[1]-z[0])*t])

        return waypoints
    
    def is_target_valid(self, target, target_type):

        # Target expressed in joint coordinates
        if target_type == 4:
            in_limit = self.are_angles_in_range(target)
            if in_limit == False:
                return (False, 'Joint limits violated')
            else:
                return (True, 'Trajectory is safe and feasible.')
            
        elif target_type == 5:
            xyz = target.tolist()
            cartesian_target = calc_FwdKin(xyz[0], xyz[1], xyz[2])
            cartesian_target_point = [float(cartesian_target[0]), float(cartesian_target[1]), float(cartesian_target[2])]
            waypoints = self.linear_trajecory_to_discrete_waypoints(self.dobot_pose_val, cartesian_target_point)
            for point in waypoints:
                end_tool_rotation = target.tolist()[3]
                point.append(end_tool_rotation)
                angles = calc_inv_kin(*point)
                if angles == False:
                    return (False, 'Inv Kin solving error!')
                in_limit = self.are_angles_in_range(angles)
                if in_limit == False:
                    return (False, 'Joint limits violated')
            return (True, 'Trajectory is safe and feasible.')


        # Target expressed in cartesian coordinates
        elif target_type == 1:
            angles = calc_inv_kin(*target)
            if angles == False:
                return (False, 'Inv Kin solving error!')
            in_limit = self.are_angles_in_range(angles)
            if in_limit == False:
                return (False, 'Joint limits violated')
            else:
                return (True, 'Trajectory is safe and feasible.')


        elif target_type == 2:
            waypoints = self.linear_trajecory_to_discrete_waypoints(self.dobot_pose_val, target)
            for point in waypoints:
                end_tool_rotation = target.tolist()[3]
                point.append(end_tool_rotation)
                angles = calc_inv_kin(*point)
                if angles == False:
                    return (False, 'Inv Kin solving error!')
                in_limit = self.are_angles_in_range(angles)
                if in_limit == False:
                    return (False, 'Joint limits violated')
            return (True, 'Trajectory is safe and feasible.')

        else:
            return (False, 'Wrong trajectory type!')


    ###################################################################################################

    def set_initial_params_values(self):
            self.set_joint_params_thread.start()
            self.set_cartesian_params_thread.start()

    def send_joint_parameters(self):
            params = self.sort_params_dict(self.joint_params_dict, "joint")
            bot.set_point_to_point_joint_params(params[0:4], params[4:8])

    def send_cartesian_parameters(self):
            params = self.sort_params_dict(self.cartesian_params_dict, "cartesian")
            bot.set_point_to_point_coordinate_params(params[0], params[1], params[2], params[3])


    def set_joint_params_func(self):
        while(self._is_running_joint_params):
            if len(self.joint_params_dict) == 8:
                self.send_joint_parameters()
                self._is_running_joint_params = False
                self.allow_dynamic_reconfigure_joints = True
                self.ready_to_set_second_part = True
                

    def set_cartesian_params_func(self):
        while(self._is_running_cartesian_params):
            if len(self.cartesian_params_dict) == 4 and self.ready_to_set_second_part:
                self.send_cartesian_parameters()
                self._is_running_cartesian_params = False
                self.allow_dynamic_reconfigure_cartesian = True



    # Sort the parameters in the order necessary to send the command.
    # The user may enter parameters in the YAML file in a different order than the default.
    def sort_params_dict(self, params_dict, params_type):
        if params_type == "joint":
            if not self.joints_sorted:
                new_dict = {}
                for idx in range(len(self.joint_params_dict)):
                    new_dict[self.joint_params_names[idx]] = params_dict[self.joint_params_names[idx]]
                self.joints_sorted = True 
                return list(new_dict.values())
            else:
                return list(self.joint_params_dict.values())

        elif params_type == "cartesian":
            if not self.cartesian_sorted:
                new_dict = {}
                for idx in range(len(self.cartesian_params_dict)):
                    new_dict[self.cartesian_params_names[idx]] = params_dict[self.cartesian_params_names[idx]]
                self.cartesian_sorted = True
                return list(new_dict.values())
            else:
                return list(self.cartesian_params_dict.values())



    def parameters_callback(self, params):
        for param in params:
            if param.name in self.joint_params_names:
                self.joint_params_dict[param.name] = param.value
                if self.allow_dynamic_reconfigure_joints:
                    self.send_joint_parameters()
               
            elif param.name in self.cartesian_params_names:
                self.cartesian_params_dict[param.name] = param.value
                if self.allow_dynamic_reconfigure_cartesian:
                    self.send_cartesian_parameters()

            else:
                return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)


    def joints_positions_callback(self, msg):
        if self.motion_type in [3, 4, 5, 6]:
            self.dobot_pose = [math.degrees(msg.position[0]), math.degrees(msg.position[1]), math.degrees(msg.position[2]), math.degrees(msg.position[3])]
            self.mode_ACK = True



    def tcp_position_callback(self, msg):
        if self.motion_type in [0, 1, 2, 7, 8, 9]:
            quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            _, _, y = tf_transformations.euler_from_quaternion(quat)
            self.dobot_pose = [float(msg.pose.position.x)*1000, float(msg.pose.position.y)*1000, float(msg.pose.position.z)*1000, float(math.degrees(y))]
            self.mode_ACK = True

    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    @staticmethod
    def is_ratio_valid(ratio):
        if 0.0 < ratio <= 1.0 and int(ratio * 100) != 0:
            return True
        return False


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.target = goal_request.target_pose
        self.motion_type = goal_request.motion_type

        validation_response = self.is_target_valid(self.target, self.motion_type)
        if validation_response[0] == False:
            self.get_logger().warn("Goal rejected: {0}".format(validation_response))
            return GoalResponse.REJECT

        self.get_logger().info("Result of calling validation service: is valid? {0}, description: {1}".format(validation_response[0], validation_response[1]))



        if DobotPTPServer.is_ratio_valid(goal_request.velocity_ratio) and DobotPTPServer.is_ratio_valid(goal_request.acceleration_ratio):
            vel_ratio = int(goal_request.velocity_ratio * 100)
            acc_ratio = int(goal_request.acceleration_ratio * 100)
            self.get_logger().info("Vel ratio: {0}".format(vel_ratio))
            self.get_logger().info("Acc ratio: {0}".format(acc_ratio))
            bot.set_point_to_point_common_params(vel_ratio, acc_ratio)
        else:
            self.get_logger().info('Wrong ratio in action goal field')
            return GoalResponse.REJECT

        while not self.mode_ACK:
            pass
        self.get_logger().info('Goal: {0}'.format(self.target))
        self.get_logger().info('Mode: {0}'.format(self.motion_type))
        self.get_logger().info('Received goal request')
        if self.motion_type in self.motion_types_list:
            return GoalResponse.ACCEPT 
        else:
            self.get_logger().info('The motion mode you specified does not exist!')
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    @staticmethod
    def is_goal_reached(target_pose, current_pose, threshold):
        for i in range(4):
            if abs(target_pose[i]-current_pose[i]) > threshold:
                return False
        return True

    @staticmethod
    def is_pose_stable(pose_arr):
        if len(pose_arr) >= 2:
            if pose_arr[-1] == pose_arr[-2]:
                return True
        return False


    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        bot.set_point_to_point_command(self.motion_type, self.target[0], self.target[1], self.target[2], self.target[3])

        feedback_msg = PointToPoint.Feedback()
        feedback_msg.current_pose = [0.0, 0.0, 0.0, 0.0]

        result = PointToPoint.Result()

        # Start executing the action
        while not (DobotPTPServer.is_goal_reached(self.target, self.dobot_pose, 0.2) and DobotPTPServer.is_pose_stable(self.pose_arr)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                bot.stop_queue(force=True) 
                bot.clear_queue()
                bot.start_queue()
                self.get_logger().info('Goal canceled')
                result.achieved_pose  = self.dobot_pose
                return result


            # Update sequence
            feedback_msg.current_pose = self.dobot_pose
            self.pose_arr.append(self.dobot_pose)

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.current_pose))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(0.1)


        goal_handle.succeed()


        result.achieved_pose  = self.dobot_pose

        self.get_logger().info('Returning result: {0}'.format(result.achieved_pose))

        return result


def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = DobotPTPServer()
    minimal_action_server.set_initial_params_values()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
