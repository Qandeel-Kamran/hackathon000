#!/usr/bin/env python3

"""
SLAM and Localization Node (Educational Implementation)

This node simulates SLAM and localization capabilities for educational purposes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math


class SLAMLocalizationNode(Node):
    def __init__(self):
        super().__init__('slam_localization_node')

        # Publishers
        self.amcl_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.map_to_odom_pub = self.create_publisher(String, 'map_to_odom_status', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
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
            '/ground_truth/odom',  # Using ground truth from simulation
            self.ground_truth_odom_callback,
            10
        )

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal state
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.pose_covariance = np.eye(6) * 0.1  # Initial uncertainty
        self.last_odom_time = self.get_clock().now()
        self.scan_data = None
        self.imu_data = None

        # Timer for periodic updates
        self.update_timer = self.create_timer(0.1, self.update_localization)

        self.get_logger().info('SLAM and Localization Node initialized')

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = msg
        # In a real implementation, this would be used for landmark detection and mapping
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} points')

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.imu_data = msg
        # In a real implementation, this would be used for orientation estimation

    def ground_truth_odom_callback(self, msg):
        """Handle ground truth odometry from simulation"""
        # In an educational context, we might use this to simulate SLAM performance
        # or to provide ground truth for comparison
        self.get_logger().debug('Received ground truth odometry')

    def update_localization(self):
        """Update robot localization estimate"""
        current_time = self.get_clock().now()

        # In a real SLAM system, this would integrate sensor data to update the pose estimate
        # For this educational implementation, we'll simulate the process

        # Create a simulated pose estimate with some uncertainty
        estimated_pose = PoseWithCovarianceStamped()
        estimated_pose.header.stamp = current_time.to_msg()
        estimated_pose.header.frame_id = 'map'

        # Set position (simulated with some noise relative to ground truth)
        estimated_pose.pose.pose.position.x = self.robot_pose[0] + np.random.normal(0, 0.05)
        estimated_pose.pose.pose.position.y = self.robot_pose[1] + np.random.normal(0, 0.05)
        estimated_pose.pose.pose.position.z = 0.0

        # Set orientation
        quat = self.euler_to_quaternion(0, 0, self.robot_pose[2] + np.random.normal(0, 0.02))
        estimated_pose.pose.pose.orientation.x = quat[0]
        estimated_pose.pose.pose.orientation.y = quat[1]
        estimated_pose.pose.pose.orientation.z = quat[2]
        estimated_pose.pose.pose.orientation.w = quat[3]

        # Set covariance (simulated uncertainty)
        estimated_pose.pose.covariance = self.pose_covariance.flatten().tolist()

        # Publish the estimated pose
        self.amcl_pose_pub.publish(estimated_pose)

        # Broadcast transform
        self.broadcast_transforms(estimated_pose)

        # Publish odometry (simulated)
        self.publish_odometry(estimated_pose, current_time)

    def broadcast_transforms(self, pose_msg):
        """Broadcast coordinate transforms"""
        t = TransformStamped()

        # Stamping
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Translation
        t.transform.translation.x = pose_msg.pose.pose.position.x
        t.transform.translation.y = pose_msg.pose.pose.position.y
        t.transform.translation.z = pose_msg.pose.pose.position.z

        # Rotation
        t.transform.rotation = pose_msg.pose.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def publish_odometry(self, pose_msg, current_time):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set position
        odom.pose.pose = pose_msg.pose.pose
        # Set velocity to zero (would come from actual odometry in real system)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Set covariances
        odom.pose.covariance = pose_msg.pose.covariance
        odom.twist.covariance = [0.0] * 36  # No velocity covariance in simulation

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    def get_robot_pose(self):
        """Get the current robot pose estimate"""
        return self.robot_pose.copy()

    def reset_localization(self):
        """Reset the localization estimate"""
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.pose_covariance = np.eye(6) * 0.1
        self.get_logger().info('Localization reset to origin')


def main(args=None):
    rclpy.init(args=args)

    slam_localization = SLAMLocalizationNode()

    try:
        rclpy.spin(slam_localization)
    except KeyboardInterrupt:
        pass
    finally:
        slam_localization.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()