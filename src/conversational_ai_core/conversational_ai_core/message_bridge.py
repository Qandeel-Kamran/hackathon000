#!/usr/bin/env python3

"""
Message Bridge for Core Communications

This module handles the core message types for AI-robot communication
and provides utilities for message handling.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from conversational_ai_core.msg import AICommand, RobotState


class MessageBridgeNode(Node):
    """
    A bridge node that handles core message types and provides utilities
    for AI-robot communication.
    """
    def __init__(self):
        super().__init__('message_bridge_node')

        # Publishers for core message types
        self.ai_commands_pub = self.create_publisher(AICommand, '/ai/commands', 10)
        self.ai_responses_pub = self.create_publisher(String, '/ai/responses', 10)
        self.robot_state_pub = self.create_publisher(RobotState, '/robot/state', 10)
        self.robot_movements_pub = self.create_publisher(JointState, '/robot/movements', 10)
        self.simulation_events_pub = self.create_publisher(String, '/simulation/events', 10)

        # Subscribers for core message types
        self.user_input_sub = self.create_subscription(
            String,
            '/user/input',
            self.user_input_callback,
            10
        )

        self.ai_commands_sub = self.create_subscription(
            AICommand,
            '/ai/commands',
            self.ai_command_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            RobotState,
            '/robot/state',
            self.robot_state_callback,
            10
        )

        # Internal state
        self.last_robot_state = None
        self.pending_commands = []

        self.get_logger().info('Message Bridge Node initialized')

    def user_input_callback(self, msg):
        """Handle user input and potentially create AI command"""
        self.get_logger().info(f'Received user input: {msg.data}')

        # Process the input and potentially generate AI command
        ai_cmd = self.process_user_input(msg.data)
        if ai_cmd:
            self.ai_commands_pub.publish(ai_cmd)

    def ai_command_callback(self, msg):
        """Handle AI commands"""
        self.get_logger().info(f'Received AI command: {msg.command_text}')

        # Process the AI command
        self.process_ai_command(msg)

    def robot_state_callback(self, msg):
        """Handle robot state updates"""
        self.last_robot_state = msg
        self.get_logger().debug(f'Updated robot state for: {msg.robot_id}')

    def process_user_input(self, user_input):
        """Process user input and create AI command if needed"""
        # Create AICommand message
        ai_cmd = AICommand()
        ai_cmd.command_text = user_input
        ai_cmd.user_id = "default_user"
        ai_cmd.timestamp = self.get_clock().now().to_msg()
        ai_cmd.intent = "unknown"

        # Determine intent based on keywords
        user_lower = user_input.lower()
        if any(word in user_lower for word in ['move', 'go', 'walk', 'turn', 'dance']):
            ai_cmd.intent = "motion_command"
        elif any(word in user_lower for word in ['what', 'how', 'explain', 'teach']):
            ai_cmd.intent = "educational_query"
        elif any(word in user_lower for word in ['stop', 'emergency']):
            ai_cmd.intent = "safety_command"

        return ai_cmd

    def process_ai_command(self, ai_cmd):
        """Process an AI command and take appropriate action"""
        self.get_logger().info(f'Processing AI command: {ai_cmd.command_text} (intent: {ai_cmd.intent})')

        # Based on intent, publish appropriate messages or take actions
        if ai_cmd.intent == "motion_command":
            # Convert motion command to robot movement
            self.execute_motion_command(ai_cmd)
        elif ai_cmd.intent == "safety_command":
            # Handle safety commands
            self.handle_safety_command(ai_cmd)
        elif ai_cmd.intent == "educational_query":
            # Handle educational queries
            self.handle_educational_query(ai_cmd)

    def execute_motion_command(self, ai_cmd):
        """Execute a motion command by publishing movement instructions"""
        self.get_logger().info(f'Executing motion command: {ai_cmd.command_text}')

        # Create a simple joint state message for the movement
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"

        # For demonstration, create a simple movement pattern
        joint_state.name = ["joint1", "joint2", "joint3"]
        joint_state.position = [0.1, 0.2, 0.3]
        joint_state.velocity = [0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0]

        # Publish the movement command
        self.robot_movements_pub.publish(joint_state)

    def handle_safety_command(self, ai_cmd):
        """Handle safety-related commands"""
        self.get_logger().warn(f'Safety command received: {ai_cmd.command_text}')

        # Publish simulation event for safety
        safety_event = String()
        safety_event.data = f"SAFETY_COMMAND:{ai_cmd.command_text}"
        self.simulation_events_pub.publish(safety_event)

    def handle_educational_query(self, ai_cmd):
        """Handle educational queries"""
        self.get_logger().info(f'Educational query: {ai_cmd.command_text}')

        # Publish simulation event for educational content
        edu_event = String()
        edu_event.data = f"EDUCATIONAL_QUERY:{ai_cmd.command_text}"
        self.simulation_events_pub.publish(edu_event)

    def send_ai_response(self, response_text):
        """Send an AI response to the user"""
        response_msg = String()
        response_msg.data = response_text
        self.ai_responses_pub.publish(response_msg)

    def get_current_robot_state(self):
        """Get the current robot state"""
        return self.last_robot_state


def main(args=None):
    rclpy.init(args=args)

    message_bridge = MessageBridgeNode()

    try:
        rclpy.spin(message_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        message_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()