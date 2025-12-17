#!/usr/bin/env python3

"""
Conversational AI Node for Humanoid Robotics Education

This node handles natural language processing and provides educational responses
about robotics concepts in simple language.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from conversational_ai_core.msg import AICommand, RobotState
from conversational_ai_core.srv import GetLessonContent
import time
import re


class ConversationalAINode(Node):
    def __init__(self):
        super().__init__('conversational_ai_node')

        # Declare parameters
        self.declare_parameter('nlp_model_path', '/models/nlp_model')
        self.declare_parameter('response_timeout', 2.0)
        self.declare_parameter('max_response_length', 500)
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('textbook_content_path', '/content/textbook')
        self.declare_parameter('simulation_only', True)
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('max_concurrent_requests', 10)
        self.declare_parameter('response_cache_size', 100)

        # Get parameters
        self.nlp_model_path = self.get_parameter('nlp_model_path').value
        self.response_timeout = self.get_parameter('response_timeout').value
        self.max_response_length = self.get_parameter('max_response_length').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.textbook_content_path = self.get_parameter('textbook_content_path').value
        self.simulation_only = self.get_parameter('simulation_only').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value

        # Publishers
        self.ai_commands_pub = self.create_publisher(AICommand, 'ai/commands', 10)
        self.ai_responses_pub = self.create_publisher(String, 'ai/responses', 10)
        self.robot_commands_pub = self.create_publisher(String, 'robot/commands', 10)

        # Subscribers
        self.user_input_sub = self.create_subscription(
            String,
            'user/input',
            self.user_input_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            RobotState,
            'robot/state',
            self.robot_state_callback,
            10
        )

        # Services
        self.lesson_content_client = self.create_client(GetLessonContent, 'get_lesson_content')

        # Internal state
        self.current_robot_state = None
        self.response_cache = {}

        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_pending_requests)

        self.get_logger().info('Conversational AI Node initialized')

    def user_input_callback(self, msg):
        """Handle user input and generate appropriate response"""
        user_input = msg.data
        self.get_logger().info(f'Received user input: {user_input}')

        # Process the input and generate response
        response = self.process_user_input(user_input)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.ai_responses_pub.publish(response_msg)

    def robot_state_callback(self, msg):
        """Update internal robot state"""
        self.current_robot_state = msg

    def process_user_input(self, user_input):
        """Process user input and generate appropriate response"""
        # Basic NLP processing
        user_input_lower = user_input.lower()

        # Intent recognition
        if any(word in user_input_lower for word in ['move', 'go', 'walk', 'step', 'forward', 'backward', 'turn', 'rotate']):
            intent = 'motion_command'
            return self.handle_motion_command(user_input)
        elif any(word in user_input_lower for word in ['what', 'how', 'explain', 'describe', 'tell me about', 'teach me']):
            intent = 'educational_query'
            return self.handle_educational_query(user_input)
        elif any(word in user_input_lower for word in ['stop', 'halt', 'emergency', 'help']):
            intent = 'safety_command'
            return self.handle_safety_command(user_input)
        elif any(word in user_input_lower for word in ['dance', 'music', 'rhythm', 'beat']):
            intent = 'music_command'
            return self.handle_music_command(user_input)
        else:
            # Default to educational query if intent is unclear
            return self.handle_educational_query(user_input)

    def handle_motion_command(self, user_input):
        """Handle motion-related commands"""
        # Parse the motion command
        response = f"I understand you want the robot to move. In simulation, I can help with: forward, backward, turn left, turn right, dance moves. Your command: '{user_input}'"

        # Create AICommand message
        ai_cmd = AICommand()
        ai_cmd.command_text = user_input
        ai_cmd.intent = 'motion_command'
        ai_cmd.timestamp = self.get_clock().now().to_msg()

        self.ai_commands_pub.publish(ai_cmd)

        return response

    def handle_educational_query(self, user_input):
        """Handle educational queries about robotics concepts"""
        # Identify the topic in the query
        topic = self.identify_topic(user_input)

        if topic:
            # Request lesson content for this topic
            response = f"Let me explain about {topic}. This concept is covered in the Physical AI & Humanoid Robotics textbook."

            # For now, provide a basic explanation
            if topic in ['kinematics', 'forward kinematics', 'inverse kinematics']:
                response += " Kinematics is the study of motion without considering the forces that cause it. Forward kinematics calculates the end-effector position from joint angles, while inverse kinematics calculates joint angles needed to achieve a desired end-effector position."
            elif topic in ['dynamics', 'motion', 'movement']:
                response += " Robot dynamics deals with the forces and torques required to create motion. It includes concepts like inertia, Coriolis forces, and gravity compensation."
            elif topic in ['control', 'controller', 'feedback']:
                response += " Robot control involves algorithms that determine how a robot should move to achieve a desired behavior. Common approaches include PID control, impedance control, and model predictive control."
            elif topic in ['perception', 'sensor', 'vision', 'camera']:
                response += " Robot perception is the ability to interpret sensory information from the environment. This includes computer vision, object recognition, and sensor fusion techniques."
            else:
                response += f" This topic ({topic}) is important in robotics. Would you like me to explain the basics?"
        else:
            response = "I can help explain robotics concepts from the Physical AI & Humanoid Robotics textbook. Could you ask about kinematics, dynamics, control, perception, or another robotics topic?"

        return response

    def handle_safety_command(self, user_input):
        """Handle safety-related commands"""
        if 'emergency' in user_input.lower() or 'stop' in user_input.lower():
            response = "Emergency stop activated in simulation. All robot movements have been halted safely."

            # Create safety command
            ai_cmd = AICommand()
            ai_cmd.command_text = 'EMERGENCY_STOP'
            ai_cmd.intent = 'safety_command'
            ai_cmd.timestamp = self.get_clock().now().to_msg()

            self.ai_commands_pub.publish(ai_cmd)
        else:
            response = "Safety is important in robotics. All operations in this system are simulation-only, ensuring no real hardware is at risk."

        return response

    def handle_music_command(self, user_input):
        """Handle music-related commands"""
        response = "Music-based movement system activated. The robot can synchronize movements with musical beats and rhythms. This helps students learn about timing and coordination in robotics."

        # Create music command
        ai_cmd = AICommand()
        ai_cmd.command_text = user_input
        ai_cmd.intent = 'music_command'
        ai_cmd.timestamp = self.get_clock().now().to_msg()

        self.ai_commands_pub.publish(ai_cmd)

        return response

    def identify_topic(self, user_input):
        """Identify the robotics topic from user input"""
        user_lower = user_input.lower()

        # Topic keywords mapping
        topics = {
            'kinematics': ['kinematics', 'forward kinematics', 'inverse kinematics', 'joint angles', 'end effector', 'position', 'pose'],
            'dynamics': ['dynamics', 'motion', 'movement', 'force', 'torque', 'inertia', 'acceleration'],
            'control': ['control', 'controller', 'feedback', 'pid', 'stability', 'tracking'],
            'perception': ['perception', 'sensor', 'vision', 'camera', 'lidar', 'object recognition', 'detection'],
            'navigation': ['navigation', 'path', 'planning', 'mapping', 'localization', 'slam', 'waypoint'],
            'manipulation': ['manipulation', 'grasp', 'pick', 'place', 'gripper', 'end effector', 'dexterity'],
            'learning': ['learning', 'machine learning', 'reinforcement', 'training', 'adaptation', 'ai'],
            'humanoid': ['humanoid', 'bipedal', 'walking', 'balance', 'locomotion', 'posture', 'gait']
        }

        for topic, keywords in topics.items():
            for keyword in keywords:
                if keyword in user_lower:
                    return topic

        return None

    def process_pending_requests(self):
        """Process any pending requests"""
        # This method runs periodically to handle background tasks
        pass


def main(args=None):
    rclpy.init(args=args)

    conversational_ai_node = ConversationalAINode()

    try:
        rclpy.spin(conversational_ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        conversational_ai_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()