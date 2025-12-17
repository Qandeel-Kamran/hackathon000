#!/usr/bin/env python3

"""
Sensor Fusion Node for Humanoid Robotics Education

This node aggregates sensor data for AI awareness and provides fused sensor information.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu, PointCloud2, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from conversational_ai_core.msg import RobotState
import numpy as np
import time


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('sensor_topics', ['/joint_states', '/imu/data', '/camera/rgb/image_raw'])
        self.declare_parameter('fusion_frequency', 50.0)
        self.declare_parameter('confidence_threshold', 0.8)

        # Get parameters
        self.sensor_topics = self.get_parameter('sensor_topics').value
        self.fusion_frequency = self.get_parameter('fusion_frequency').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # Publishers
        self.fused_sensor_pub = self.create_publisher(RobotState, 'fused_sensor_data', 10)
        self.ai_awareness_pub = self.create_publisher(String, 'ai/awareness', 10)

        # Subscribers for various sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Internal state
        self.joint_states = JointState()
        self.imu_data = Imu()
        self.odom_data = Odometry()
        self.camera_info = CameraInfo()

        self.last_joint_update = time.time()
        self.last_imu_update = time.time()
        self.last_odom_update = time.time()
        self.last_camera_update = time.time()

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(1.0/self.fusion_frequency, self.fuse_sensor_data)

        self.get_logger().info('Sensor Fusion Node initialized')

    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.joint_states = msg
        self.last_joint_update = time.time()
        self.get_logger().debug('Received joint state update')

    def imu_callback(self, msg):
        """Handle IMU data updates"""
        self.imu_data = msg
        self.last_imu_update = time.time()
        self.get_logger().debug('Received IMU update')

    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.odom_data = msg
        self.last_odom_update = time.time()
        self.get_logger().debug('Received odometry update')

    def camera_info_callback(self, msg):
        """Handle camera info updates"""
        self.camera_info = msg
        self.last_camera_update = time.time()
        self.get_logger().debug('Received camera info update')

    def fuse_sensor_data(self):
        """Fuse data from multiple sensors to create a comprehensive state"""
        # Check if we have recent data from all sensors
        current_time = time.time()
        timeout = 1.0  # 1 second timeout

        # Only proceed if we have reasonably recent data
        if (current_time - self.last_joint_update > timeout or
            current_time - self.last_imu_update > timeout or
            current_time - self.last_odom_update > timeout):
            self.get_logger().warn('Some sensor data is outdated, skipping fusion')
            return

        # Create fused sensor state message
        fused_state = RobotState()
        fused_state.robot_id = "fused_sensor_data"
        fused_state.timestamp = self.get_clock().now().to_msg()

        # Set pose from odometry
        fused_state.pose = self.odom_data.pose.pose

        # Set joint states
        fused_state.joint_states = self.joint_states

        # Set velocity from odometry
        fused_state.velocity = self.odom_data.twist.twist

        # Set active behaviors (empty for now)
        fused_state.active_behaviors = []

        # Calculate battery level based on sensor activity (simulated)
        # In a real system, this would come from power sensors
        sensor_activity = self.calculate_sensor_activity()
        fused_state.battery_level = max(0.0, 100.0 - (sensor_activity * 10.0))

        # Set connection status
        fused_state.is_connected = True
        fused_state.is_simulated = True

        # Determine status based on sensor health
        fused_state.status = self.determine_robot_status()

        # Publish fused data
        self.fused_sensor_pub.publish(fused_state)

        # Also publish awareness information to AI
        awareness_msg = String()
        awareness_msg.data = self.create_awareness_description(fused_state)
        self.ai_awareness_pub.publish(awareness_msg)

        self.get_logger().debug('Published fused sensor data')

    def calculate_sensor_activity(self):
        """Calculate sensor activity level for battery estimation"""
        # This is a simplified calculation for simulation
        # In reality, you would have actual power consumption models
        activity_score = 0.0

        # Check if joint states are changing (indicating movement)
        if len(self.joint_states.velocity) > 0:
            max_velocity = max([abs(v) for v in self.joint_states.velocity], default=0.0)
            activity_score += max_velocity * 0.1

        # Check IMU for movement
        imu_magnitude = np.sqrt(
            self.imu_data.linear_acceleration.x**2 +
            self.imu_data.linear_acceleration.y**2 +
            self.imu_data.linear_acceleration.z**2
        )
        activity_score += min(imu_magnitude * 0.01, 1.0)

        return min(activity_score, 10.0)  # Cap at 10.0

    def determine_robot_status(self):
        """Determine robot status based on sensor data"""
        # Check for any sensor errors or anomalies
        if not self.joint_states.name:
            return "sensors_error"
        elif self.odom_data.pose.pose.position.z > 1.0:  # If z position is unusually high
            return "unstable"
        else:
            return "operational"

    def create_awareness_description(self, fused_state):
        """Create a text description of the robot's awareness for the AI"""
        description = f"Robot Status: {fused_state.status}, "
        description += f"Battery: {fused_state.battery_level:.1f}%, "
        description += f"Position: ({fused_state.pose.position.x:.2f}, {fused_state.pose.position.y:.2f}), "
        description += f"Joint Count: {len(fused_state.joint_states.name)}, "

        # Add information about recent movements
        if len(fused_state.joint_states.velocity) > 0:
            max_vel = max([abs(v) for v in fused_state.joint_states.velocity], default=0.0)
            if max_vel > 0.1:
                description += "Moving, "
            else:
                description += "Stationary, "

        # Add orientation information
        description += f"Orientation: ({fused_state.pose.orientation.x:.2f}, {fused_state.pose.orientation.y:.2f}, {fused_state.pose.orientation.z:.2f}, {fused_state.pose.orientation.w:.2f})"

        return description


def main(args=None):
    rclpy.init(args=args)

    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()