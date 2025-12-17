#!/usr/bin/env python3

"""
Lesson Content Server for Humanoid Robotics Education

This node provides educational content based on textbook chapters and concepts.
"""

import rclpy
from rclpy.node import Node
from conversational_ai_core.srv import GetLessonContent
from conversational_ai_core.msg import LessonContent


class LessonContentServer(Node):
    def __init__(self):
        super().__init__('lesson_content_server')

        # Service server for getting lesson content
        self.get_lesson_content_srv = self.create_service(
            GetLessonContent,
            'get_lesson_content',
            self.get_lesson_content_callback
        )

        # Load educational content (in a real system, this would load from files/database)
        self.load_educational_content()

        self.get_logger().info('Lesson Content Server initialized')

    def load_educational_content(self):
        """Load educational content from textbook"""
        # This would typically load from files or database
        # For now, we'll create some sample content
        self.content_database = {
            'kinematics': {
                'title': 'Robot Kinematics',
                'content': 'Robot kinematics is the study of the motion of robotic mechanisms without considering the forces that cause the motion. It involves the relationships between the positions, velocities, and accelerations of the various links of the robot.',
                'related_concepts': ['forward_kinematics', 'inverse_kinematics', 'joint_space', 'task_space'],
                'example_code': [
                    '# Forward kinematics example:\n# Calculate end-effector position from joint angles',
                    '# Inverse kinematics example:\n# Calculate joint angles for desired end-effector position'
                ],
                'visual_aids': ['kinematics_diagram.png', 'joint_coordinate_system.png'],
                'difficulty_level': 'intermediate',
                'prerequisites': ['algebra', 'trigonometry'],
                'learning_objectives': [
                    'Understand forward kinematics',
                    'Understand inverse kinematics',
                    'Apply transformation matrices'
                ]
            },
            'dynamics': {
                'title': 'Robot Dynamics',
                'content': 'Robot dynamics deals with the forces and torques required to create motion. It includes the study of how forces affect the motion of robotic systems, including inertia, Coriolis forces, and gravity compensation.',
                'related_concepts': ['inertia', 'coriolis', 'gravity_compensation', 'force_control'],
                'example_code': [
                    '# Dynamic model of a simple manipulator',
                    '# Force control implementation'
                ],
                'visual_aids': ['dynamic_model.png', 'force_diagram.png'],
                'difficulty_level': 'advanced',
                'prerequisites': ['kinematics', 'physics', 'calculus'],
                'learning_objectives': [
                    'Understand dynamic modeling',
                    'Apply Newton-Euler formulation',
                    'Implement force control'
                ]
            },
            'control': {
                'title': 'Robot Control',
                'content': 'Robot control involves algorithms that determine how a robot should move to achieve a desired behavior. Common approaches include PID control, impedance control, and model predictive control.',
                'related_concepts': ['pid_control', 'feedback_control', 'impedance_control', 'trajectory_tracking'],
                'example_code': [
                    '# PID controller implementation',
                    '# Trajectory tracking algorithm'
                ],
                'visual_aids': ['control_block_diagram.png', 'pid_controller.png'],
                'difficulty_level': 'intermediate',
                'prerequisites': ['kinematics', 'differential_equations'],
                'learning_objectives': [
                    'Implement PID control',
                    'Design feedback controllers',
                    'Achieve trajectory tracking'
                ]
            }
        }

    def get_lesson_content_callback(self, request, response):
        """Service callback to get lesson content based on context"""
        self.get_logger().info(f'Get lesson content request: topic={request.topic}, difficulty={request.difficulty_level}')

        # Check if topic exists in our database
        if request.topic.lower() in self.content_database:
            content_data = self.content_database[request.topic.lower()]

            # Create LessonContent message
            lesson_content = LessonContent()
            lesson_content.lesson_id = f"lesson_{request.topic}"
            lesson_content.chapter = "Robotics Fundamentals"
            lesson_content.section = request.topic
            lesson_content.title = content_data['title']
            lesson_content.content = content_data['content']
            lesson_content.related_concepts = content_data['related_concepts']
            lesson_content.example_code = content_data['example_code']
            lesson_content.visual_aids = content_data['visual_aids']
            lesson_content.timestamp = self.get_clock().now().to_msg()
            lesson_content.difficulty_level = content_data['difficulty_level']
            lesson_content.prerequisites = content_data['prerequisites']
            lesson_content.learning_objectives = content_data['learning_objectives']

            response.success = True
            response.error_message = ""
            response.content = [lesson_content]

            # Find related topics
            related_topics = []
            for topic_key in self.content_database:
                if topic_key != request.topic.lower():
                    related_topics.append(topic_key)
            response.related_topics = related_topics

            # Suggest next topics based on difficulty
            next_topics = self.get_next_topics(request.topic.lower(), content_data['difficulty_level'])
            response.suggested_next_topics = next_topics

        else:
            # Topic not found, return empty content
            response.success = False
            response.error_message = f"Content for topic '{request.topic}' not found"
            response.content = []
            response.related_topics = list(self.content_database.keys())
            response.suggested_next_topics = []

        return response

    def get_next_topics(self, current_topic, current_difficulty):
        """Get suggested next topics based on current topic and difficulty"""
        # Simple logic: suggest topics that are related or of similar difficulty
        next_topics = []

        if current_topic == 'kinematics':
            next_topics = ['dynamics', 'control']
        elif current_topic == 'dynamics':
            next_topics = ['control']
        elif current_topic == 'control':
            next_topics = ['kinematics', 'dynamics']  # Could suggest review of prerequisites

        return next_topics


def main(args=None):
    rclpy.init(args=args)

    lesson_content_server = LessonContentServer()

    try:
        rclpy.spin(lesson_content_server)
    except KeyboardInterrupt:
        pass
    finally:
        lesson_content_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()