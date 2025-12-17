#!/usr/bin/env python3

"""
State Manager for Gazebo Simulation

This node manages saving and loading of simulation states for educational continuity.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelStates
from conversational_ai_core.msg import AICommand
import json
import os
from datetime import datetime


class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')

        # Service clients for Gazebo state management
        self.get_state_client = self.create_client(GetModelState, '/get_model_state')
        self.set_state_client = self.create_client(SetModelState, '/set_model_state')

        # Subscribers
        self.ai_commands_sub = self.create_subscription(
            AICommand,
            'ai/commands',
            self.ai_command_callback,
            10
        )

        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.state_commands_sub = self.create_subscription(
            String,
            'simulation/state_commands',
            self.state_command_callback,
            10
        )

        # Publishers
        self.state_status_pub = self.create_publisher(String, 'simulation/state_status', 10)

        # Wait for services to be available
        self.get_logger().info('Waiting for Gazebo state services...')
        self.get_state_client.wait_for_service()
        self.set_state_client.wait_for_service()
        self.get_logger().info('Gazebo state services available')

        # Internal state tracking
        self.current_states = {}
        self.saved_states = {}
        self.state_directory = '/tmp/simulation_states'  # In a real system, this would be configurable

        # Create state directory if it doesn't exist
        os.makedirs(self.state_directory, exist_ok=True)

        self.get_logger().info('State Manager Node initialized')

    def ai_command_callback(self, msg):
        """Handle AI commands that may involve state management"""
        cmd_text = msg.command_text.lower()

        if 'save state' in cmd_text:
            self.save_current_state()
        elif 'load state' in cmd_text:
            self.load_state()
        elif 'reset state' in cmd_text:
            self.reset_to_saved_state()

    def state_command_callback(self, msg):
        """Handle direct state management commands"""
        command = msg.data.lower()
        self.get_logger().info(f'State command: {command}')

        if command.startswith('save:'):
            parts = command.split(':', 1)
            if len(parts) > 1:
                label = parts[1]
                self.save_current_state(label)
        elif command.startswith('load:'):
            parts = command.split(':', 1)
            if len(parts) > 1:
                label = parts[1]
                self.load_state(label)
        elif command == 'save_current':
            self.save_current_state()
        elif command == 'load_latest':
            self.load_latest_state()
        elif command == 'list_states':
            self.list_saved_states()
        elif command == 'clear_states':
            self.clear_saved_states()

    def model_states_callback(self, msg):
        """Update internal state tracking from model states"""
        for i, name in enumerate(msg.name):
            self.current_states[name] = {
                'pose': msg.pose[i],
                'twist': msg.twist[i],
                'last_update': self.get_clock().now().nanoseconds
            }

    def save_current_state(self, label=None):
        """Save the current simulation state"""
        if label is None:
            label = f"state_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        state_data = {
            'timestamp': datetime.now().isoformat(),
            'label': label,
            'models': {}
        }

        # Save current states of all tracked models
        for model_name, model_data in self.current_states.items():
            state_data['models'][model_name] = {
                'position': {
                    'x': model_data['pose'].position.x,
                    'y': model_data['pose'].position.y,
                    'z': model_data['pose'].position.z
                },
                'orientation': {
                    'x': model_data['pose'].orientation.x,
                    'y': model_data['pose'].orientation.y,
                    'z': model_data['pose'].orientation.z,
                    'w': model_data['pose'].orientation.w
                },
                'linear_velocity': {
                    'x': model_data['twist'].linear.x,
                    'y': model_data['twist'].linear.y,
                    'z': model_data['twist'].linear.z
                },
                'angular_velocity': {
                    'x': model_data['twist'].angular.x,
                    'y': model_data['twist'].angular.y,
                    'z': model_data['twist'].angular.z
                }
            }

        # Save to file
        filename = os.path.join(self.state_directory, f"{label}.json")
        try:
            with open(filename, 'w') as f:
                json.dump(state_data, f, indent=2)

            self.saved_states[label] = state_data
            self.get_logger().info(f'Saved state to: {filename}')

            # Publish status
            status_msg = String()
            status_msg.data = f"STATE_SAVED:{label}"
            self.state_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to save state {label}: {e}')

    def load_state(self, label=None):
        """Load a saved simulation state"""
        if label is None:
            # Load the most recently saved state
            if not self.saved_states:
                self.get_logger().info('No saved states available')
                return
            # Get the most recent state by timestamp
            latest_label = max(self.saved_states.keys(),
                             key=lambda k: self.saved_states[k]['timestamp'])
            label = latest_label

        filename = os.path.join(self.state_directory, f"{label}.json")

        if not os.path.exists(filename):
            # Try to load from file system if not in memory
            try:
                with open(filename, 'r') as f:
                    state_data = json.load(f)
                self.saved_states[label] = state_data
            except FileNotFoundError:
                self.get_logger().error(f'State file not found: {filename}')
                return
            except Exception as e:
                self.get_logger().error(f'Error loading state from file: {e}')
                return

        if label not in self.saved_states:
            self.get_logger().error(f'State not found: {label}')
            return

        state_data = self.saved_states[label]

        # Load each model's state
        for model_name, model_state in state_data['models'].items():
            self.set_model_state(model_name, model_state)

        self.get_logger().info(f'Loaded state: {label}')

        # Publish status
        status_msg = String()
        status_msg.data = f"STATE_LOADED:{label}"
        self.state_status_pub.publish(status_msg)

    def set_model_state(self, model_name, state_data):
        """Set a model's state in the simulation"""
        # In a real implementation, this would use the SetModelState service
        # For now, we'll just log the action
        pos = state_data['position']
        self.get_logger().info(f'Setting {model_name} to position ({pos["x"]}, {pos["y"]}, {pos["z"]})')

    def load_latest_state(self):
        """Load the most recently saved state"""
        if not self.saved_states:
            self.get_logger().info('No saved states available')
            return

        # Find the most recent state by timestamp
        latest_label = max(self.saved_states.keys(),
                         key=lambda k: self.saved_states[k]['timestamp'])
        self.load_state(latest_label)

    def list_saved_states(self):
        """List all saved states"""
        if not self.saved_states:
            self.get_logger().info('No saved states available')
            return

        self.get_logger().info('Saved states:')
        for label in sorted(self.saved_states.keys()):
            timestamp = self.saved_states[label]['timestamp']
            self.get_logger().info(f'  {label} - {timestamp}')

        # Publish state list
        state_list = ','.join(sorted(self.saved_states.keys()))
        status_msg = String()
        status_msg.data = f"STATE_LIST:{state_list}"
        self.state_status_pub.publish(status_msg)

    def clear_saved_states(self):
        """Clear all saved states"""
        self.saved_states.clear()

        # Also remove files from disk
        for filename in os.listdir(self.state_directory):
            if filename.endswith('.json'):
                filepath = os.path.join(self.state_directory, filename)
                try:
                    os.remove(filepath)
                except Exception as e:
                    self.get_logger().error(f'Error removing state file {filepath}: {e}')

        self.get_logger().info('Cleared all saved states')

        # Publish status
        status_msg = String()
        status_msg.data = "STATES_CLEARED"
        self.state_status_pub.publish(status_msg)

    def reset_to_saved_state(self):
        """Reset to the default saved state (if any)"""
        # In a real implementation, this would reset to a specific saved state
        # For now, we'll just load the latest state
        self.load_latest_state()

    def get_saved_state_info(self, label):
        """Get information about a saved state"""
        if label in self.saved_states:
            state_data = self.saved_states[label]
            return {
                'label': label,
                'timestamp': state_data['timestamp'],
                'model_count': len(state_data['models'])
            }
        return None


def main(args=None):
    rclpy.init(args=args)

    state_manager = StateManagerNode()

    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()