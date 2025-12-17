#!/usr/bin/env python3

"""
Multi-Robot Manager for Gazebo Simulation

This node manages multiple robots in the simulation environment for complex interactions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelStates
from conversational_ai_core.msg import AICommand, RobotState
import random


class MultiRobotManagerNode(Node):
    def __init__(self):
        super().__init__('multi_robot_manager_node')

        # Service clients for Gazebo interaction
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

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

        # Publishers
        self.robot_count_pub = self.create_publisher(Int32, 'simulation/robot_count', 10)
        self.multi_robot_state_pub = self.create_publisher(String, 'simulation/multi_robot_state', 10)

        # Wait for services to be available
        self.get_logger().info('Waiting for Gazebo services...')
        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.get_logger().info('Gazebo services available')

        # Track robots
        self.robots = {}
        self.robot_counter = 0
        self.max_robots = 10  # Maximum number of robots allowed

        # Timer for periodic state updates
        self.state_timer = self.create_timer(1.0, self.publish_multi_robot_state)

        self.get_logger().info('Multi-Robot Manager Node initialized')

    def ai_command_callback(self, msg):
        """Handle AI commands that may involve multiple robots"""
        cmd_text = msg.command_text.lower()

        if 'add robot' in cmd_text or 'spawn robot' in cmd_text:
            self.add_robot()
        elif 'remove robot' in cmd_text or 'delete robot' in cmd_text:
            self.remove_robot()
        elif 'multi robot' in cmd_text or 'multiple robots' in cmd_text:
            self.handle_multi_robot_command(msg)

    def model_states_callback(self, msg):
        """Update robot states from Gazebo model states"""
        # Update our internal robot tracking based on Gazebo model states
        current_robot_names = []

        for i, name in enumerate(msg.name):
            if name.startswith('humanoid_robot_') or name.startswith('simple_humanoid_'):
                current_robot_names.append(name)

                # Update robot state if it's in our tracking
                if name in self.robots:
                    self.robots[name]['pose'] = msg.pose[i]
                    self.robots[name]['twist'] = msg.twist[i]
                else:
                    # Add new robot to tracking
                    self.robots[name] = {
                        'pose': msg.pose[i],
                        'twist': msg.twist[i],
                        'spawn_time': self.get_clock().now().nanoseconds
                    }

        # Remove robots that are no longer in the simulation
        for robot_name in list(self.robots.keys()):
            if robot_name not in current_robot_names:
                del self.robots[robot_name]

        # Publish updated robot count
        count_msg = Int32()
        count_msg.data = len(self.robots)
        self.robot_count_pub.publish(count_msg)

    def add_robot(self):
        """Add a new robot to the simulation"""
        if len(self.robots) >= self.max_robots:
            self.get_logger().warn(f'Maximum robot count ({self.max_robots}) reached')
            return

        # Generate a unique robot name
        self.robot_counter += 1
        robot_name = f"humanoid_robot_{self.robot_counter}"

        # Random position to avoid overlapping
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        z = 1.0  # Height to spawn above ground

        # Create robot model string (simplified version)
        robot_model = f"""
        <robot name="{robot_name}">
          <link name="base_link">
            <visual>
              <geometry>
                <box size="0.3 0.2 0.1"/>
              </geometry>
              <material name="white">
                <color rgba="0.8 0.8 0.8 1.0"/>
              </material>
            </visual>
            <collision>
              <geometry>
                <box size="0.3 0.2 0.1"/>
              </geometry>
            </collision>
            <inertial>
              <mass value="5.0"/>
              <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
            </inertial>
          </link>
          <gazebo>
            <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
              <robotNamespace>/{robot_name}</robotNamespace>
            </plugin>
          </gazebo>
        </robot>
        """

        req = SpawnEntity.Request()
        req.name = robot_name
        req.xml = robot_model
        req.robot_namespace = robot_name
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.w = 1.0

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.spawn_robot_callback(f, robot_name, x, y, z))

        self.get_logger().info(f'Spawning robot: {robot_name} at ({x}, {y}, {z})')

    def remove_robot(self):
        """Remove a robot from the simulation"""
        if not self.robots:
            self.get_logger().info('No robots to remove')
            return

        # Remove the last added robot
        robot_names = list(self.robots.keys())
        robot_to_remove = robot_names[-1]

        req = DeleteEntity.Request()
        req.name = robot_to_remove

        future = self.delete_client.call_async(req)
        future.add_done_callback(lambda f: self.delete_robot_callback(f, robot_to_remove))

        self.get_logger().info(f'Deleting robot: {robot_to_remove}')

    def handle_multi_robot_command(self, ai_cmd):
        """Handle specific multi-robot commands from AI"""
        cmd_text = ai_cmd.command_text.lower()

        if 'formation' in cmd_text:
            self.set_formation()
        elif 'dance' in cmd_text or 'synchronize' in cmd_text:
            self.synchronize_robots()
        elif 'follow' in cmd_text:
            self.set_follow_behavior()
        elif 'scatter' in cmd_text:
            self.scatter_robots()

    def set_formation(self):
        """Set robots in a formation"""
        self.get_logger().info('Setting robots in formation')

        robot_names = list(self.robots.keys())
        if len(robot_names) < 2:
            self.get_logger().info('Need at least 2 robots for formation')
            return

        # Example: line formation
        for i, robot_name in enumerate(robot_names):
            target_x = i * 1.0  # Space robots 1m apart in x direction
            target_y = 0.0
            self.move_robot_to_position(robot_name, target_x, target_y, 1.0)

    def synchronize_robots(self):
        """Synchronize robot movements"""
        self.get_logger().info('Synchronizing robot movements')
        # In a real implementation, this would send synchronized commands to all robots

    def set_follow_behavior(self):
        """Set robots to follow each other"""
        self.get_logger().info('Setting follow behavior')
        robot_names = list(self.robots.keys())
        if len(robot_names) < 2:
            self.get_logger().info('Need at least 2 robots for follow behavior')
            return

        # First robot is leader, others follow
        for i in range(1, len(robot_names)):
            leader_name = robot_names[i-1]
            follower_name = robot_names[i]
            self.set_robot_follow(follower_name, leader_name)

    def scatter_robots(self):
        """Scatter robots to random positions"""
        self.get_logger().info('Scattering robots')
        for robot_name in self.robots.keys():
            x = random.uniform(-8, 8)
            y = random.uniform(-8, 8)
            self.move_robot_to_position(robot_name, x, y, 1.0)

    def move_robot_to_position(self, robot_name, x, y, z):
        """Move a specific robot to a position (this would use SetModelState service in real implementation)"""
        self.get_logger().info(f'Moving {robot_name} to ({x}, {y}, {z})')

    def set_robot_follow(self, follower_name, leader_name):
        """Set one robot to follow another (conceptual - would require more complex implementation)"""
        self.get_logger().info(f'Setting {follower_name} to follow {leader_name}')

    def spawn_robot_callback(self, future, name, x, y, z):
        """Callback for spawn robot service call"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned robot: {name}')
                # Add to our tracking
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0
                twist = Twist()

                self.robots[name] = {
                    'pose': pose,
                    'twist': twist,
                    'spawn_time': self.get_clock().now().nanoseconds
                }
            else:
                self.get_logger().error(f'Failed to spawn robot: {name}, error: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn robot service call failed: {e}')

    def delete_robot_callback(self, future, name):
        """Callback for delete robot service call"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully deleted robot: {name}')
                # Remove from our tracking
                if name in self.robots:
                    del self.robots[name]
            else:
                self.get_logger().error(f'Failed to delete robot: {name}, error: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Delete robot service call failed: {e}')

    def publish_multi_robot_state(self):
        """Publish the overall multi-robot state"""
        state_msg = String()

        if not self.robots:
            state_msg.data = "NO_ROBOTS:0"
        else:
            robot_info = []
            for name, data in self.robots.items():
                pos = data['pose'].position
                robot_info.append(f"{name}({pos.x:.1f},{pos.y:.1f})")

            state_msg.data = f"ROBOTS:{len(self.robots)}:" + ",".join(robot_info)

        self.multi_robot_state_pub.publish(state_msg)

    def get_robot_count(self):
        """Get the current number of robots"""
        return len(self.robots)


def main(args=None):
    rclpy.init(args=args)

    multi_robot_manager = MultiRobotManagerNode()

    try:
        rclpy.spin(multi_robot_manager)
    except KeyboardInterrupt:
        pass
    finally:
        multi_robot_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()