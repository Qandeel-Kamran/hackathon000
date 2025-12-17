#!/usr/bin/env python3

"""
Simulation Bridge Node for Humanoid Robotics Education

This node manages communication between the AI system and the simulation environment.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates, ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnEntity, DeleteEntity
from conversational_ai_core.msg import RobotState
from conversational_ai_core.srv import GetRobotInfo
import time


class SimulationBridgeNode(Node):
    def __init__(self):
        super().__init__('simulation_bridge_node')

        # Declare parameters
        self.declare_parameter('simulation_frequency', 100.0)
        self.declare_parameter('physics_update_rate', 1000.0)
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('enable_visualization', True)

        # Get parameters
        self.simulation_frequency = self.get_parameter('simulation_frequency').value
        self.physics_update_rate = self.get_parameter('physics_update_rate').value
        self.real_time_factor = self.get_parameter('real_time_factor').value
        self.enable_visualization = self.get_parameter('enable_visualization').value

        # Publishers
        self.sim_state_pub = self.create_publisher(RobotState, 'simulation/state', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.model_states_pub = self.create_publisher(ModelStates, '/gazebo/model_states', 10)

        # Subscribers
        self.sim_commands_sub = self.create_subscription(
            String,
            'simulation/commands',
            self.sim_command_callback,
            10
        )

        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # Services for simulation interaction
        self.get_model_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        self.set_model_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
        self.delete_entity_client = self.create_client(DeleteEntity, '/gazebo/delete_entity')

        # Internal state
        self.model_states = ModelStates()
        self.simulation_active = True
        self.robot_model_name = "humanoid_robot"

        # Timer for simulation state publishing
        self.state_timer = self.create_timer(1.0/self.simulation_frequency, self.publish_simulation_state)

        # Timer for physics updates
        self.physics_timer = self.create_timer(1.0/self.physics_update_rate, self.physics_update)

        self.get_logger().info('Simulation Bridge Node initialized')

    def sim_command_callback(self, msg):
        """Handle simulation commands"""
        command = msg.data.lower()
        self.get_logger().info(f'Received simulation command: {command}')

        if command == 'pause':
            self.pause_simulation()
        elif command == 'resume':
            self.resume_simulation()
        elif command == 'reset':
            self.reset_simulation()
        elif command.startswith('spawn'):
            self.spawn_object(command)
        elif command.startswith('delete'):
            self.delete_object(command)
        else:
            self.get_logger().warn(f'Unknown simulation command: {command}')

    def model_states_callback(self, msg):
        """Update model states from Gazebo"""
        self.model_states = msg

    def publish_simulation_state(self):
        """Publish current simulation state"""
        if not self.simulation_active:
            return

        # Create robot state message from simulation data
        state_msg = RobotState()
        state_msg.robot_id = self.robot_model_name
        state_msg.timestamp = self.get_clock().now().to_msg()

        # Find the robot in model states
        robot_idx = -1
        for i, name in enumerate(self.model_states.name):
            if name == self.robot_model_name:
                robot_idx = i
                break

        if robot_idx >= 0:
            # Set pose from model state
            state_msg.pose = self.model_states.pose[robot_idx]

            # For joint states, we'll create a basic message
            # In a real implementation, this would come from the robot's joint state publisher
            joint_state = JointState()
            joint_state.name = ['joint1', 'joint2', 'joint3']  # Example joint names
            joint_state.position = [0.0, 0.0, 0.0]  # Example positions
            joint_state.velocity = [0.0, 0.0, 0.0]  # Example velocities
            joint_state.effort = [0.0, 0.0, 0.0]   # Example efforts
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = 'base_link'

            state_msg.joint_states = joint_state

            # Set velocity from model state
            state_msg.velocity.linear = self.model_states.twist[robot_idx].linear
            state_msg.velocity.angular = self.model_states.twist[robot_idx].angular

            # Set other state information
            state_msg.active_behaviors = []  # Would be populated by active behaviors
            state_msg.battery_level = 100.0  # Simulated battery level
            state_msg.is_connected = True
            state_msg.is_simulated = True
            state_msg.status = "operational"
        else:
            # If robot not found in model states, create a default state
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = 0.0
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.x = 0.0
            state_msg.pose.orientation.y = 0.0
            state_msg.pose.orientation.z = 0.0
            state_msg.pose.orientation.w = 1.0

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = 'base_link'
            state_msg.joint_states = joint_state

            state_msg.velocity.linear.x = 0.0
            state_msg.velocity.linear.y = 0.0
            state_msg.velocity.linear.z = 0.0
            state_msg.velocity.angular.x = 0.0
            state_msg.velocity.angular.y = 0.0
            state_msg.velocity.angular.z = 0.0
            state_msg.active_behaviors = []
            state_msg.battery_level = 100.0
            state_msg.is_connected = True
            state_msg.is_simulated = True
            state_msg.status = "robot_not_found"

        self.sim_state_pub.publish(state_msg)

    def physics_update(self):
        """Handle physics updates at high frequency"""
        # This method runs at the physics update rate
        # In a real implementation, this would handle physics-specific tasks
        pass

    def pause_simulation(self):
        """Pause the simulation"""
        self.simulation_active = False
        self.get_logger().info('Simulation paused')

        # Publish notification
        pause_msg = String()
        pause_msg.data = 'SIMULATION_PAUSED'
        # Note: In a real Gazebo implementation, you would call the pause service

    def resume_simulation(self):
        """Resume the simulation"""
        self.simulation_active = True
        self.get_logger().info('Simulation resumed')

        # Publish notification
        resume_msg = String()
        resume_msg.data = 'SIMULATION_RESUMED'
        # Note: In a real Gazebo implementation, you would call the unpause service

    def reset_simulation(self):
        """Reset the simulation to initial state"""
        self.get_logger().info('Resetting simulation')

        # Reset robot to initial position
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.0
        initial_pose.orientation.x = 0.0
        initial_pose.orientation.y = 0.0
        initial_pose.orientation.z = 0.0
        initial_pose.orientation.w = 1.0

        # Note: In a real implementation, you would call the reset service
        # and set the model state back to initial position

    def spawn_object(self, command):
        """Spawn an object in the simulation"""
        self.get_logger().info(f'Spawning object: {command}')

        # Parse the spawn command to extract object type and position
        # Format: "spawn <object_type> at <x> <y> <z>"
        try:
            parts = command.split()
            if len(parts) >= 5 and parts[2] == 'at':
                obj_type = parts[1]
                x = float(parts[3])
                y = float(parts[4])
                z = float(parts[5]) if len(parts) > 5 else 0.0

                # Note: In a real implementation, you would call the spawn service
                self.get_logger().info(f'Spawning {obj_type} at ({x}, {y}, {z})')
            else:
                self.get_logger().warn(f'Invalid spawn command format: {command}')
        except (ValueError, IndexError):
            self.get_logger().warn(f'Could not parse spawn command: {command}')

    def delete_object(self, command):
        """Delete an object from the simulation"""
        self.get_logger().info(f'Deleting object: {command}')

        # Parse the delete command
        # Format: "delete <object_name>"
        try:
            parts = command.split()
            if len(parts) >= 2:
                obj_name = parts[1]

                # Note: In a real implementation, you would call the delete service
                self.get_logger().info(f'Deleting object: {obj_name}')
            else:
                self.get_logger().warn(f'Invalid delete command format: {command}')
        except IndexError:
            self.get_logger().warn(f'Could not parse delete command: {command}')

    def get_robot_simulation_state(self):
        """Get the current simulation state of the robot"""
        # In a real implementation, this would call the get_model_state service
        # For now, return a default state
        robot_state = RobotState()
        robot_state.robot_id = self.robot_model_name
        robot_state.timestamp = self.get_clock().now().to_msg()
        robot_state.is_simulated = True
        robot_state.status = "operational"

        return robot_state


def main(args=None):
    rclpy.init(args=args)

    simulation_bridge_node = SimulationBridgeNode()

    try:
        rclpy.spin(simulation_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        simulation_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()