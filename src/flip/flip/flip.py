
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("flip_pub")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 6)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", False)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []  # List of JointTrajectoryPoint
        for name in goal_names:
            self.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))
            goal = self.get_parameter(name).value

            # TODO(anyone): remove this "if" part in ROS Iron
            if isinstance(goal, list):
                self.get_logger().warn(
                    f'Goal "{name}" is defined as a list. This is deprecated. '
                    "Use the following structure:\n<goal_name>:\n  "
                    "positions: [joint1, joint2, joint3, ...]\n  "
                    "velocities: [v_joint1, v_joint2, ...]\n  "
                    "accelerations: [a_joint1, a_joint2, ...]\n  "
                    "effort: [eff_joint1, eff_joint2, ...]"
                )

                if goal is None or len(goal) == 0:
                    raise Exception(f'Values for goal "{name}" not set!')

                float_goal = [float(value) for value in goal]

                point = JointTrajectoryPoint()
                point.positions = float_goal
                point.time_from_start = Duration(sec=4)

                self.goals.append(point)

            else:
                point = JointTrajectoryPoint()

                def get_sub_param(sub_param):
                    param_name = name + "." + sub_param
                    self.declare_parameter(param_name, [float()])
                    param_value = self.get_parameter(param_name).value

                    float_values = []

                    if len(param_value) != len(self.joints):
                        return [False, float_values]

                    float_values = [float(value) for value in param_value]

                    return [True, float_values]

                one_ok = False

                [ok, values] = get_sub_param("positions")
                if ok:
                    point.positions = values
                    one_ok = True

                [ok, values] = get_sub_param("velocities")
                if ok:
                    point.velocities = values
                    one_ok = True

                [ok, values] = get_sub_param("accelerations")
                if ok:
                    point.accelerations = values
                    one_ok = True

                [ok, values] = get_sub_param("effort")
                if ok:
                    point.effort = values
                    one_ok = True

                if one_ok:
                    point.time_from_start = Duration(sec=4)
                    self.goals.append(point)
                    self.get_logger().info(f'Goal "{name}" has definition {point}')

                else:
                    self.get_logger().warn(
                        f'Goal "{name}" definition is wrong. This goal will not be used. '
                        "Use the following structure: \n<goal_name>:\n  "
                        "positions: [joint1, joint2, joint3, ...]\n  "
                        "velocities: [v_joint1, v_joint2, ...]\n  "
                        "accelerations: [a_joint1, a_joint2, ...]\n  "
                        "effort: [eff_joint1, eff_joint2, ...]"
                    )

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            f'Publishing {len(goal_names)} goals on topic "{publish_topic}" every '
            "{wait_sec_between_publish} s"
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0
        self.target_counter = 0
        self.arduino_command_publisher = self.create_publisher(String, "arduinoCommand", 10)

    def timer_callback(self):
         # Publish a sample string message on the "arduinoCommand" topic
        # arduino_command_msg = String()
        # arduino_command_msg.data = "Release\n"      

        if self.starting_point_ok and self.i<len(self.goals):
	   
            self.get_logger().info(f"Sending goal {self.goals[self.i]}.")          

            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points.append(self.goals[self.i])
            self.publisher_.publish(traj)

            self.i += 1            
            # while self.i >=len(self.goals):
            # 	return
            ##self.i %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Commands all done!")

    def joint_state_callback(self, msg):
        ##self.get_logger().info("I got something")
        arduino_command_msg = String()
        arduino_command_msg.data = "Release\n"
        if msg.position[2] <=(3.0+0.05) and msg.position[2] >=(3.0-0.05) and self.i== 2:
            ##self.get_logger().info("I got the wrist") 
            #if self.i== 1:
            self.arduino_command_publisher.publish(arduino_command_msg)       
        
        return

        # if not self.joint_state_msg_received:

        #     # check start state
        #     limit_exceeded = [False] * len(msg.name)
        #     for idx, enum in enumerate(msg.name):
        #         if (msg.position[idx] < self.starting_point[enum][0]) or (
        #             msg.position[idx] > self.starting_point[enum][1]
        #         ):
        #             self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
        #             limit_exceeded[idx] = True

        #     if any(limit_exceeded):
        #         self.starting_point_ok = False
        #     else:
        #         self.starting_point_ok = True

        #     self.joint_state_msg_received = True
        # else:
        #     return


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
