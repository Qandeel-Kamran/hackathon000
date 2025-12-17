#!/usr/bin/env python3

"""
Environment Manager for Gazebo Simulation

This node manages dynamic environment modifications for educational scenarios.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState, SetEntityState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point
from conversational_ai_core.msg import AICommand


class EnvironmentManagerNode(Node):
    def __init__(self):
        super().__init__('environment_manager_node')

        # Service clients for Gazebo interaction
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.get_state_client = self.create_client(GetEntityState, '/get_entity_state')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')

        # Subscribers
        self.ai_commands_sub = self.create_subscription(
            AICommand,
            'ai/commands',
            self.ai_command_callback,
            10
        )

        self.env_modification_sub = self.create_subscription(
            String,
            'simulation/environment_mod',
            self.env_modification_callback,
            10
        )

        # Publishers
        self.env_state_pub = self.create_publisher(String, 'simulation/environment_state', 10)

        # Wait for services to be available
        self.get_logger().info('Waiting for Gazebo services...')
        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.get_state_client.wait_for_service()
        self.set_state_client.wait_for_service()
        self.get_logger().info('Gazebo services available')

        # Track environment objects
        self.environment_objects = {}
        self.educational_scenarios = {
            'obstacle_course': ['obstacle_1', 'obstacle_2', 'goal_marker'],
            'navigation': ['waypoint_1', 'waypoint_2', 'target'],
            'manipulation': ['object_1', 'object_2', 'bin']
        }

        self.get_logger().info('Environment Manager Node initialized')

    def ai_command_callback(self, msg):
        """Handle AI commands that may require environment modifications"""
        if 'environment' in msg.command_text.lower() or 'scenario' in msg.command_text.lower():
            self.process_environment_command(msg)

    def env_modification_callback(self, msg):
        """Handle direct environment modification commands"""
        command = msg.data
        self.get_logger().info(f'Environment modification command: {command}')

        if command.startswith('add_object:'):
            self.add_object_to_environment(command)
        elif command.startswith('remove_object:'):
            self.remove_object_from_environment(command)
        elif command.startswith('modify_scenario:'):
            self.modify_scenario(command)
        elif command.startswith('reset_scenario:'):
            self.reset_scenario(command)
        else:
            self.get_logger().warn(f'Unknown environment command: {command}')

    def process_environment_command(self, ai_cmd):
        """Process environment-related AI commands"""
        cmd_text = ai_cmd.command_text.lower()

        if 'add obstacle' in cmd_text:
            self.add_obstacle()
        elif 'remove obstacle' in cmd_text:
            self.remove_obstacle()
        elif 'change scenario' in cmd_text:
            self.change_scenario()
        elif 'reset environment' in cmd_text:
            self.reset_environment()

    def add_object_to_environment(self, command):
        """Add an object to the environment"""
        try:
            parts = command.split(':')
            if len(parts) >= 3:
                obj_type = parts[1]
                obj_name = parts[2]

                # Create a simple object based on type
                if obj_type == 'box':
                    self.spawn_box_object(obj_name, 0, 0, 0.5)
                elif obj_type == 'cylinder':
                    self.spawn_cylinder_object(obj_name, 1, 0, 0.5)
                elif obj_type == 'sphere':
                    self.spawn_sphere_object(obj_name, 0, 1, 0.5)

                self.environment_objects[obj_name] = obj_type
                self.get_logger().info(f'Added {obj_type} object: {obj_name}')

                # Publish environment state update
                state_msg = String()
                state_msg.data = f'OBJECT_ADDED:{obj_name}'
                self.env_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f'Error adding object: {e}')

    def remove_object_from_environment(self, command):
        """Remove an object from the environment"""
        try:
            parts = command.split(':')
            if len(parts) >= 2:
                obj_name = parts[1]

                # Remove the object from Gazebo
                req = DeleteEntity.Request()
                req.name = obj_name

                future = self.delete_client.call_async(req)
                future.add_done_callback(lambda f: self.delete_entity_callback(f, obj_name))

                if obj_name in self.environment_objects:
                    del self.environment_objects[obj_name]

                self.get_logger().info(f'Removed object: {obj_name}')

                # Publish environment state update
                state_msg = String()
                state_msg.data = f'OBJECT_REMOVED:{obj_name}'
                self.env_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(f'Error removing object: {e}')

    def modify_scenario(self, command):
        """Modify the current educational scenario"""
        try:
            parts = command.split(':')
            if len(parts) >= 2:
                scenario_name = parts[1]

                if scenario_name in self.educational_scenarios:
                    # Load the specific scenario
                    self.load_scenario(scenario_name)
                    self.get_logger().info(f'Loaded scenario: {scenario_name}')
                else:
                    self.get_logger().warn(f'Unknown scenario: {scenario_name}')
        except Exception as e:
            self.get_logger().error(f'Error modifying scenario: {e}')

    def reset_scenario(self, command):
        """Reset the environment to a default state"""
        try:
            parts = command.split(':')
            scenario_name = parts[1] if len(parts) > 1 else 'default'

            self.reset_environment_to_scenario(scenario_name)
            self.get_logger().info(f'Reset to {scenario_name} scenario')
        except Exception as e:
            self.get_logger().error(f'Error resetting scenario: {e}')

    def spawn_box_object(self, name, x, y, z):
        """Spawn a box object in the environment"""
        # Create a simple box model string
        box_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.5 0.5 0.5</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.5 0.5 0.5</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0.5 0.5 0.5 1</ambient>
                  <diffuse>0.5 0.5 0.5 1</diffuse>
                </material>
              </visual>
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.1</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0.1</iyy>
                  <iyz>0</iyz>
                  <izz>0.1</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = box_model
        req.robot_namespace = ""
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.spawn_entity_callback(f, name))

    def spawn_cylinder_object(self, name, x, y, z):
        """Spawn a cylinder object in the environment"""
        # Similar to box but with cylinder geometry
        cylinder_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>0.25</radius>
                    <length>0.5</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>0.25</radius>
                    <length>0.5</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>0.8 0.2 0.2 1</ambient>
                  <diffuse>0.8 0.2 0.2 1</diffuse>
                </material>
              </visual>
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.1</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0.1</iyy>
                  <iyz>0</iyz>
                  <izz>0.1</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = cylinder_model
        req.robot_namespace = ""
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.spawn_entity_callback(f, name))

    def spawn_sphere_object(self, name, x, y, z):
        """Spawn a sphere object in the environment"""
        # Similar to box but with sphere geometry
        sphere_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <sphere>
                    <radius>0.25</radius>
                  </sphere>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <sphere>
                    <radius>0.25</radius>
                  </sphere>
                </geometry>
                <material>
                  <ambient>0.2 0.2 0.8 1</ambient>
                  <diffuse>0.2 0.2 0.8 1</diffuse>
                </material>
              </visual>
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.1</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0.1</iyy>
                  <iyz>0</iyz>
                  <izz>0.1</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = sphere_model
        req.robot_namespace = ""
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.spawn_entity_callback(f, name))

    def add_obstacle(self):
        """Add an obstacle to the environment"""
        import random
        x = random.uniform(-3, 3)
        y = random.uniform(-3, 3)
        obj_name = f"dynamic_obstacle_{int(self.get_clock().now().nanoseconds / 1000000)}"
        self.spawn_box_object(obj_name, x, y, 0.5)

    def remove_obstacle(self):
        """Remove a random obstacle from the environment"""
        if self.environment_objects:
            # Find a dynamic obstacle to remove (not part of default scenario)
            dynamic_objects = [name for name in self.environment_objects.keys()
                              if not name.startswith(('obstacle_', 'goal_marker', 'sign_'))]
            if dynamic_objects:
                obj_to_remove = dynamic_objects[0]
                self.remove_object_from_environment(f"remove_object:{obj_to_remove}")

    def change_scenario(self):
        """Change to a different educational scenario"""
        import random
        scenarios = list(self.educational_scenarios.keys())
        if scenarios:
            new_scenario = random.choice(scenarios)
            self.load_scenario(new_scenario)

    def reset_environment(self):
        """Reset environment to default state"""
        self.reset_environment_to_scenario('default')

    def load_scenario(self, scenario_name):
        """Load a specific educational scenario"""
        # In a real implementation, this would load predefined scenarios
        # For now, we'll just log the action
        self.get_logger().info(f'Loading scenario: {scenario_name}')

    def reset_environment_to_scenario(self, scenario_name):
        """Reset environment to a specific scenario state"""
        # Remove all dynamic objects
        for obj_name in list(self.environment_objects.keys()):
            if not obj_name.startswith(('obstacle_', 'goal_marker', 'sign_')):  # Keep default objects
                self.remove_object_from_environment(f"remove_object:{obj_name}")

        # Log reset
        self.get_logger().info(f'Environment reset to {scenario_name} scenario')

    def spawn_entity_callback(self, future, name):
        """Callback for spawn entity service call"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned entity: {name}')
            else:
                self.get_logger().error(f'Failed to spawn entity: {name}, error: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Spawn entity service call failed: {e}')

    def delete_entity_callback(self, future, name):
        """Callback for delete entity service call"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully deleted entity: {name}')
            else:
                self.get_logger().error(f'Failed to delete entity: {name}, error: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Delete entity service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    environment_manager = EnvironmentManagerNode()

    try:
        rclpy.spin(environment_manager)
    except KeyboardInterrupt:
        pass
    finally:
        environment_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()