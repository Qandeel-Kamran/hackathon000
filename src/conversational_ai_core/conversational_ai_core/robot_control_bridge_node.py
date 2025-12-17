#!/usr/bin/env python3

"""
Robot Control Bridge Node for Humanoid Robotics Education

This node translates AI commands to robot actions and manages communication
between the AI system and the robot control layer.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from conversational_ai_core.msg import AICommand, RobotState
from conversational_ai_core.srv import GetRobotInfo, ExecuteBehavior
import time


class RobotControlBridgeNode(Node):
    def __init__(self):
        super().__init__('robot_control_bridge_node')

        # Declare parameters
        self.declare_parameter('joint_velocity_limits', [1.0])
        self.declare_parameter('joint_acceleration_limits', [0.5])
        self.declare_parameter('safety_margin', 0.1)
        self.declare_parameter('command_timeout', 5.0)
        self.declare_parameter('retry_attempts', 3)

        # Get parameters
        self.joint_velocity_limits = self.get_parameter('joint_velocity_limits').value
        self.joint_acceleration_limits = self.get_parameter('joint_acceleration_limits').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.retry_attempts = self.get_parameter('retry_attempts').value

        # Publishers
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/robot/state', 10)

        # Subscribers
        self.ai_commands_sub = self.create_subscription(
            AICommand,
            'ai/commands',
            self.ai_command_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Services
        self.get_robot_info_srv = self.create_service(GetRobotInfo, 'get_robot_info', self.get_robot_info_callback)
        self.execute_behavior_srv = self.create_service(ExecuteBehavior, 'execute_behavior', self.execute_behavior_callback)

        # Internal state
        self.current_joint_states = JointState()
        self.robot_connected = False
        self.simulation_mode = True

        # Timer for state publishing
        self.state_timer = self.create_timer(0.1, self.publish_robot_state)

        self.get_logger().info('Robot Control Bridge Node initialized')

    def ai_command_callback(self, msg):
        """Handle AI commands and translate to robot actions"""
        self.get_logger().info(f'Received AI command: {msg.command_text} (intent: {msg.intent})')

        if msg.intent == 'motion_command':
            self.handle_motion_command(msg)
        elif msg.intent == 'safety_command':
            self.handle_safety_command(msg)
        elif msg.intent == 'music_command':
            self.handle_music_command(msg)
        else:
            self.get_logger().info(f'Unknown intent: {msg.intent}, command: {msg.command_text}')

    def joint_state_callback(self, msg):
        """Update joint states from robot"""
        self.current_joint_states = msg

    def handle_motion_command(self, ai_cmd):
        """Handle motion commands from AI"""
        command_text = ai_cmd.command_text.lower()

        # Parse motion command
        if 'forward' in command_text or 'move' in command_text and 'forward' in command_text:
            self.move_forward()
        elif 'backward' in command_text or 'back' in command_text:
            self.move_backward()
        elif 'left' in command_text and ('turn' in command_text or 'rotate' in command_text):
            self.turn_left()
        elif 'right' in command_text and ('turn' in command_text or 'rotate' in command_text):
            self.turn_right()
        elif 'dance' in command_text or 'music' in command_text:
            self.execute_dance_behavior()
        else:
            # Try to parse more complex motion commands
            self.parse_and_execute_motion(command_text)

    def handle_safety_command(self, ai_cmd):
        """Handle safety commands from AI"""
        if 'EMERGENCY_STOP' in ai_cmd.command_text:
            self.emergency_stop()
        else:
            self.get_logger().info('Safety command received but not an emergency stop')

    def handle_music_command(self, ai_cmd):
        """Handle music commands from AI"""
        # Parse music-related commands and execute rhythmic movements
        self.execute_rhythmic_movement(ai_cmd.command_text)

    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.5  # m/s
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -0.5  # m/s
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Moving backward')

    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Turning left')

    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Turning right')

    def emergency_stop(self):
        """Emergency stop - halt all robot movement"""
        # Stop linear and angular movement
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        # Stop any joint trajectories
        # Create an empty trajectory to stop current movement
        traj = JointTrajectory()
        traj.joint_names = self.current_joint_states.name if self.current_joint_states.name else ['joint1']
        traj.points = []
        self.joint_trajectory_pub.publish(traj)

        self.get_logger().info('Emergency stop activated - all movement halted')

    def execute_dance_behavior(self):
        """Execute a simple dance behavior"""
        self.get_logger().info('Executing dance behavior')
        # This would typically execute a predefined dance pattern
        # For now, just move in a simple pattern

    def execute_rhythmic_movement(self, command):
        """Execute rhythmic movement based on music command"""
        self.get_logger().info(f'Executing rhythmic movement for: {command}')
        # This would parse the music command and create rhythmic movements

    def parse_and_execute_motion(self, command_text):
        """Parse and execute more complex motion commands"""
        self.get_logger().info(f'Parsing complex motion command: {command_text}')
        # This would implement more sophisticated command parsing

    def get_robot_info_callback(self, request, response):
        """Service callback to get robot configuration and capabilities"""
        self.get_logger().info('Get robot info service called')

        # Fill response with robot information
        response.robot_id = "simulated_humanoid_robot"
        response.model_name = "Generic Humanoid Robot"
        response.joint_names = self.current_joint_states.name if self.current_joint_states.name else ["joint1", "joint2"]
        response.sensor_types = ["IMU", "Camera", "Joint Position", "Force/Torque"]
        response.available_behaviors = ["move_forward", "move_backward", "turn_left", "turn_right", "dance", "wave", "balance"]
        response.capabilities = ["navigation", "manipulation", "perception", "communication"]
        response.joint_limits = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]  # Example limits
        response.velocity_limits = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]    # Example velocity limits
        response.is_simulated = True
        response.description = "Generic simulated humanoid robot for educational purposes"

        return response

    def execute_behavior_callback(self, request, response):
        """Service callback to execute predefined robot behaviors"""
        self.get_logger().info(f'Execute behavior service called: {request.behavior_name}')

        # Execute the requested behavior
        success = self.execute_predefined_behavior(request.behavior_name, request.parameters)

        response.success = success
        response.error_message = "" if success else f"Could not execute behavior: {request.behavior_name}"
        response.execution_id = f"exec_{int(time.time())}"
        response.start_time = self.get_clock().now().to_msg()

        # For now, set end time to start time + 2 seconds as an example
        end_time = self.get_clock().now().to_msg()
        end_time.sec += 2
        response.end_time = end_time

        return response

    def execute_predefined_behavior(self, behavior_name, parameters):
        """Execute a predefined behavior"""
        self.get_logger().info(f'Executing behavior: {behavior_name} with params: {parameters}')

        if behavior_name == 'wave':
            self.execute_wave_behavior()
        elif behavior_name == 'balance':
            self.execute_balance_behavior()
        elif behavior_name == 'dance':
            self.execute_dance_behavior()
        elif behavior_name == 'move_forward':
            self.move_forward()
        elif behavior_name == 'move_backward':
            self.move_backward()
        elif behavior_name == 'turn_left':
            self.turn_left()
        elif behavior_name == 'turn_right':
            self.turn_right()
        else:
            self.get_logger().warn(f'Unknown behavior: {behavior_name}')
            return False

        return True

    def execute_wave_behavior(self):
        """Execute waving behavior"""
        self.get_logger().info('Executing wave behavior')
        # This would implement a waving motion

    def execute_balance_behavior(self):
        """Execute balancing behavior"""
        self.get_logger().info('Executing balance behavior')
        # This would implement balancing motions

    def publish_robot_state(self):
        """Publish robot state at regular intervals"""
        state_msg = RobotState()
        state_msg.robot_id = "simulated_humanoid_robot"
        state_msg.timestamp = self.get_clock().now().to_msg()

        # Set a default pose
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = 0.0
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        state_msg.pose.orientation.w = 1.0

        # Set joint states
        state_msg.joint_states = self.current_joint_states

        # Set default velocity
        state_msg.velocity.linear.x = 0.0
        state_msg.velocity.linear.y = 0.0
        state_msg.velocity.linear.z = 0.0
        state_msg.velocity.angular.x = 0.0
        state_msg.velocity.angular.y = 0.0
        state_msg.velocity.angular.z = 0.0

        # Set active behaviors (empty for now)
        state_msg.active_behaviors = []

        # Set battery level (simulated)
        state_msg.battery_level = 100.0

        # Set connection status
        state_msg.is_connected = True
        state_msg.is_simulated = True
        state_msg.status = "operational"

        self.robot_state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_control_bridge_node = RobotControlBridgeNode()

    try:
        rclpy.spin(robot_control_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()