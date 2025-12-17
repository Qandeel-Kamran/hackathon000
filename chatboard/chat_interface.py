#!/usr/bin/env python3

"""
Chatboard Interface for Conversational AI System

This module provides a web-based chat interface that can connect to the
conversational AI system for humanoid robotics and Physical AI education.
"""

import asyncio
import websockets
import json
import threading
from flask import Flask, render_template, request, jsonify
import os
import sys
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Try to import ROS 2 if available
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from conversational_ai_core.msg import AICommand
    from conversational_ai_core.srv import GetLessonContent

    ROS_AVAILABLE = True
    logger.info("ROS 2 libraries available")
except ImportError:
    ROS_AVAILABLE = False
    logger.info("ROS 2 libraries not available - running in web-only mode")

class ChatboardInterface:
    def __init__(self):
        self.app = Flask(__name__)
        self.connected_clients = set()
        self.chat_history = []
        self.max_history = 100  # Keep last 100 messages

        # ROS 2 node (if available)
        self.ros_node = None
        if ROS_AVAILABLE:
            try:
                rclpy.init()
                self.ros_node = ChatboardROSNode()
                logger.info("ROS 2 node initialized")
            except Exception as e:
                logger.warning(f"Could not initialize ROS 2 node: {e}")
                self.ros_node = None

        # Set up routes
        self.setup_routes()

    def setup_routes(self):
        """Set up the Flask routes for the chat interface"""
        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.app.route('/send_message', methods=['POST'])
        def send_message():
            data = request.get_json()
            message = data.get('message', '')
            user = data.get('user', 'User')

            if message:
                # Process the message through the conversational AI
                response = self.process_message(message, user)
                return jsonify({'status': 'success', 'response': response})
            else:
                return jsonify({'status': 'error', 'message': 'Empty message'})

        @self.app.route('/get_history', methods=['GET'])
        def get_history():
            return jsonify({'history': self.chat_history})

    def process_message(self, message, user):
        """Process a message through the conversational AI system"""
        logger.info(f"Processing message from {user}: {message}")

        # Add to chat history
        self.add_to_history(user, message, 'user')

        # If ROS 2 is available, try to use the conversational AI node
        if self.ros_node and ROS_AVAILABLE:
            try:
                response = self.ros_node.send_command_and_get_response(message)
                logger.info(f"Received response from ROS 2: {response}")
            except Exception as e:
                logger.error(f"Error communicating with ROS 2: {e}")
                response = f"I encountered an error processing your request: {str(e)}. Here's what I can tell you: {message}"
        else:
            # Fallback response when ROS 2 is not available
            response = self.get_fallback_response(message)

        # Add response to history
        self.add_to_history('AI', response, 'ai')

        return response

    def get_fallback_response(self, message):
        """Generate a fallback response when ROS 2 is not available"""
        message_lower = message.lower()

        # Simple rule-based responses for educational context
        if any(word in message_lower for word in ['hello', 'hi', 'hey']):
            return "Hello! I'm your conversational AI assistant for humanoid robotics and Physical AI education. How can I help you learn about robotics today?"
        elif any(word in message_lower for word in ['robot', 'robotics', 'humanoid']):
            return "Robotics is an interdisciplinary field combining mechanical engineering, electrical engineering, and computer science. A humanoid robot is a robot with human-like features and movements. Would you like to learn more about specific aspects of robotics?"
        elif any(word in message_lower for word in ['move', 'walk', 'dance']):
            return "In a real system, I would send movement commands to the robot. Since I'm in web-only mode, I can describe how the movement would work: The robot would analyze the command, plan the motion, and execute it using its joints and actuators."
        elif any(word in message_lower for word in ['music', 'beat', 'rhythm']):
            return "Music-based robot control involves analyzing audio patterns and mapping them to robot movements. The robot would detect beats and synchronize its movements to the rhythm."
        elif any(word in message_lower for word in ['learn', 'teach', 'education']):
            return "I can help you learn about robotics concepts! I have educational content about kinematics, dynamics, control systems, perception, navigation, and humanoid robotics. What topic would you like to explore?"
        else:
            return f"I understand you said: '{message}'. In the full system, I would connect to the conversational AI node to provide detailed responses about humanoid robotics and Physical AI education. Since I'm running in web-only mode, I can provide general information about robotics concepts."

    def add_to_history(self, user, message, message_type):
        """Add a message to the chat history"""
        entry = {
            'user': user,
            'message': message,
            'type': message_type,
            'timestamp': self.get_timestamp()
        }

        self.chat_history.append(entry)

        # Keep only the most recent messages
        if len(self.chat_history) > self.max_history:
            self.chat_history = self.chat_history[-self.max_history:]

    def get_timestamp(self):
        """Get current timestamp"""
        from datetime import datetime
        return datetime.now().isoformat()

    def start_server(self, host='localhost', port=5000):
        """Start the chatboard server"""
        logger.info(f"Starting chatboard server on {host}:{port}")

        # Create templates directory if it doesn't exist
        templates_dir = os.path.join(os.path.dirname(__file__), 'templates')
        if not os.path.exists(templates_dir):
            os.makedirs(templates_dir)

        # Create a basic HTML template
        self.create_template()

        # Run the Flask app
        self.app.run(host=host, port=port, debug=False, use_reloader=False)

    def create_template(self):
        """Create a basic HTML template for the chat interface"""
        template_content = """
<!DOCTYPE html>
<html>
<head>
    <title>Conversational AI Chatboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .chat-container {
            max-width: 800px;
            margin: 0 auto;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            overflow: hidden;
        }
        .chat-header {
            background: #4CAF50;
            color: white;
            padding: 15px;
            text-align: center;
        }
        .chat-messages {
            height: 400px;
            overflow-y: auto;
            padding: 15px;
            border-bottom: 1px solid #eee;
        }
        .message {
            margin-bottom: 10px;
            padding: 8px 12px;
            border-radius: 18px;
            max-width: 70%;
        }
        .user-message {
            background-color: #DCF8C6;
            margin-left: auto;
            text-align: right;
        }
        .ai-message {
            background-color: #E3E3E3;
            margin-right: auto;
        }
        .input-area {
            padding: 15px;
            display: flex;
        }
        #message-input {
            flex: 1;
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 4px;
            margin-right: 10px;
        }
        #send-button {
            padding: 10px 20px;
            background: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
        }
        #send-button:hover {
            background: #45a049;
        }
        .timestamp {
            font-size: 0.7em;
            color: #888;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="chat-container">
        <div class="chat-header">
            <h2>Conversational AI for Humanoid Robotics</h2>
        </div>
        <div class="chat-messages" id="chat-messages">
            <div class="message ai-message">
                Hello! I'm your conversational AI assistant for humanoid robotics and Physical AI education. How can I help you today?
                <div class="timestamp">Just now</div>
            </div>
        </div>
        <div class="input-area">
            <input type="text" id="message-input" placeholder="Type your message here..." onkeypress="handleKeyPress(event)">
            <button id="send-button" onclick="sendMessage()">Send</button>
        </div>
    </div>

    <script>
        function handleKeyPress(event) {
            if (event.key === 'Enter') {
                sendMessage();
            }
        }

        function sendMessage() {
            const input = document.getElementById('message-input');
            const message = input.value.trim();

            if (message === '') return;

            // Add user message to chat
            addMessageToChat('You', message, 'user');
            input.value = '';

            // Send to server
            fetch('/send_message', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: message,
                    user: 'You'
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success') {
                    addMessageToChat('AI', data.response, 'ai');
                } else {
                    addMessageToChat('System', 'Error: ' + data.message, 'ai');
                }
            })
            .catch(error => {
                console.error('Error:', error);
                addMessageToChat('System', 'Connection error', 'ai');
            });
        }

        function addMessageToChat(user, message, type) {
            const chatMessages = document.getElementById('chat-messages');
            const messageDiv = document.createElement('div');
            messageDiv.className = `message ${type}-message`;

            const timestamp = new Date().toLocaleTimeString();
            messageDiv.innerHTML = `${message}<div class="timestamp">${timestamp}</div>`;

            chatMessages.appendChild(messageDiv);
            chatMessages.scrollTop = chatMessages.scrollHeight;
        }
    </script>
</body>
</html>
        """

        template_path = os.path.join(os.path.dirname(__file__), 'templates', 'index.html')
        with open(template_path, 'w') as f:
            f.write(template_content)

if ROS_AVAILABLE:
    class ChatboardROSNode(Node):
        """ROS 2 node for connecting the chatboard to the conversational AI system"""
        def __init__(self):
            super().__init__('chatboard_interface_node')

            # Publisher for sending commands to the conversational AI
            self.ai_command_publisher = self.create_publisher(AICommand, 'ai/commands', 10)

            # Subscriber for receiving responses from the conversational AI
            self.response_subscription = self.create_subscription(
                String,
                'ai/responses',
                self.response_callback,
                10
            )

            # Service client for getting educational content
            self.lesson_content_client = self.create_client(
                GetLessonContent,
                'get_lesson_content'
            )

            # Store the latest response
            self.latest_response = None
            self.response_available = threading.Event()

            self.get_logger().info('Chatboard ROS Node initialized')

        def send_command_and_get_response(self, command_text):
            """Send a command to the conversational AI and wait for response"""
            # Create and send the command
            ai_cmd = AICommand()
            ai_cmd.command_text = command_text
            ai_cmd.command_type = "chat_command"  # Define appropriate command type

            self.ai_command_publisher.publish(ai_cmd)

            # Wait for response with timeout
            self.response_available.clear()
            start_time = self.get_clock().now()

            # Wait for response (in a real implementation, you'd have a better mechanism)
            # For now, we'll return a placeholder
            return f"Command sent: {command_text}. In the full system, I would wait for and return the AI's response."

        def response_callback(self, msg):
            """Handle responses from the conversational AI"""
            self.latest_response = msg.data
            self.response_available.set()
else:
    # Define a placeholder class when ROS 2 is not available
    class ChatboardROSNode:
        """Placeholder class when ROS 2 is not available"""
        def __init__(self):
            logger.info('ROS 2 not available - using placeholder class')

        def send_command_and_get_response(self, command_text):
            """Placeholder method that returns a default response"""
            return f"Command received: {command_text}. ROS 2 not available - running in web-only mode."

def main():
    """Main function to start the chatboard interface"""
    chatboard = ChatboardInterface()

    print("Starting Chatboard Interface...")
    print("Visit http://localhost:5000 to access the chat interface")

    try:
        chatboard.start_server()
    except KeyboardInterrupt:
        print("\nShutting down chatboard server...")
        logger.info("Chatboard server stopped")

if __name__ == '__main__':
    main()