---
title: Locomotion and Walking Algorithms
sidebar_position: 6
description: Dynamic walking principles, ZMP control, and gait generation for humanoid robots
---

# Locomotion and Walking Algorithms

Humanoid locomotion represents one of the most challenging problems in robotics, requiring the coordination of multiple systems to achieve stable, efficient, and adaptive walking. The fundamental challenge lies in controlling an underactuated system (the robot is only in contact with the ground at discrete points) while maintaining balance and moving forward. Dynamic walking principles leverage the natural dynamics of the system to achieve energy-efficient locomotion, contrasting with static walking approaches that maintain stability at every instant.

The Zero Moment Point (ZMP) is a fundamental concept in bipedal locomotion that defines the point on the ground where the sum of all moments due to external forces equals zero. For stable walking, the ZMP must remain within the support polygon defined by the feet. ZMP-based controllers generate walking patterns by planning ZMP trajectories that remain within the support region while achieving the desired motion.

Capture Point theory provides a more intuitive approach to balance control by identifying the point where the robot needs to step to come to a stop. This concept is particularly useful for reactive balance control, allowing the robot to recover from disturbances by stepping to the appropriate capture point. The relationship between ZMP and Capture Point provides a comprehensive framework for understanding and controlling bipedal balance.

Gait generation for humanoid robots involves creating coordinated patterns of motion for all joints that result in stable forward movement. This includes not just the leg movements but also coordinated arm swing, torso motion, and head stabilization. Advanced gait generation algorithms can adapt to different terrains, walking speeds, and even recover from disturbances by modifying the planned gait in real-time.

Terrain adaptation is crucial for practical humanoid robots that must navigate real-world environments. This includes handling uneven surfaces, stairs, slopes, and obstacles. Advanced locomotion algorithms incorporate perception data to modify gait parameters in real-time, adjusting step height, length, and timing based on the terrain ahead.

```python
import numpy as np
from scipy import signal
import math

class WalkingController:
    """
    Advanced walking controller for humanoid robots
    """
    def __init__(self, robot_height=0.8, step_length=0.3, step_height=0.05):
        self.robot_height = robot_height  # Center of mass height
        self.step_length = step_length
        self.step_height = step_height
        self.g = 9.81
        self.omega = math.sqrt(self.g / self.robot_height)  # Natural frequency of inverted pendulum
        self.support_foot = "left"  # Current support foot
        self.swing_trajectory = None

    def compute_zmp_trajectory(self, step_positions, step_times, dt=0.005):
        """
        Compute ZMP trajectory for a sequence of footsteps
        """
        total_time = sum(step_times)
        time_points = np.arange(0, total_time, dt)
        zmp_x = np.zeros_like(time_points)
        zmp_y = np.zeros_like(time_points)

        current_time = 0
        foot_x, foot_y = 0, 0  # Starting position

        for i, t in enumerate(time_points):
            if current_time >= total_time:
                break

            # Find which step we're currently executing
            step_idx = 0
            temp_time = 0
            for j, step_t in enumerate(step_times):
                if temp_time + step_t > t:
                    step_idx = j
                    break
                temp_time += step_t

            # Compute ZMP based on desired foot placement
            if step_idx < len(step_positions):
                target_x, target_y = step_positions[step_idx]
                # Simple ZMP planning: interpolate between current and target foot positions
                phase = min(1.0, (t - temp_time) / step_times[step_idx]) if step_times[step_idx] > 0 else 1.0
                zmp_x[i] = foot_x + phase * (target_x - foot_x)
                zmp_y[i] = foot_y + phase * (target_y - foot_y)
            else:
                zmp_x[i] = foot_x
                zmp_y[i] = foot_y

        return time_points, zmp_x, zmp_y

    def compute_com_trajectory_from_zmp(self, zmp_x, zmp_y, dt=0.005):
        """
        Compute CoM trajectory from ZMP using the linear inverted pendulum model
        """
        # Solve the differential equation: CoM_ddot = g/h * (CoM - ZMP)
        com_x = np.zeros_like(zmp_x)
        com_y = np.zeros_like(zmp_y)

        # Initial conditions
        com_x[0] = 0.0  # Start at origin
        com_y[0] = 0.0
        com_dx = 0.0  # Initial velocity
        com_dy = 0.0

        for i in range(1, len(zmp_x)):
            # Integrate: CoM_ddot = g/h * (CoM - ZMP)
            com_ddx = self.g / self.robot_height * (com_x[i-1] - zmp_x[i-1])
            com_ddy = self.g / self.robot_height * (com_y[i-1] - zmp_y[i-1])

            # Update velocities
            com_dx += com_ddx * dt
            com_dy += com_ddy * dt

            # Update positions
            com_x[i] = com_x[i-1] + com_dx * dt
            com_y[i] = com_y[i-1] + com_dy * dt

        return com_x, com_y

    def generate_footstep_pattern(self, walk_distance, step_width=0.2):
        """
        Generate a footstep pattern for walking forward
        """
        num_steps = int(walk_distance / self.step_length) + 1
        footsteps = []

        for i in range(num_steps):
            # Alternate between left and right foot
            if i % 2 == 0:  # Right foot step (if starting with left support)
                foot_x = (i + 1) * self.step_length
                foot_y = step_width / 2
            else:  # Left foot step
                foot_x = (i + 1) * self.step_length
                foot_y = -step_width / 2

            footsteps.append((foot_x, foot_y))

        return footsteps

    def generate_swing_trajectory(self, start_pos, end_pos, height, duration, dt=0.005):
        """
        Generate a smooth trajectory for the swing foot
        """
        time_points = np.arange(0, duration, dt)
        trajectory = []

        for t in time_points:
            phase = t / duration
            # Use a smooth interpolation (e.g., 5th order polynomial)
            blend = 10 * phase**3 - 15 * phase**4 + 6 * phase**5

            x = start_pos[0] + blend * (end_pos[0] - start_pos[0])
            y = start_pos[1] + blend * (end_pos[1] - start_pos[1])
            z = start_pos[2]

            # Add arc motion for step height
            if phase < 0.5:
                z += height * 4 * phase  # Upward arc
            else:
                z += height * 4 * (1 - phase)  # Downward arc

            trajectory.append((x, y, z))

        return trajectory

    def compute_balance_recovery(self, current_com, current_com_vel, current_zmp, support_polygon_radius=0.1):
        """
        Compute balance recovery strategy when ZMP is outside support polygon
        """
        # Calculate the capture point
        capture_point_x = current_com[0] + current_com_vel[0] / self.omega
        capture_point_y = current_com[1] + current_com_vel[1] / self.omega

        # Check if current ZMP is outside support polygon
        zmp_distance = math.sqrt((current_zmp[0] - current_com[0])**2 + (current_zmp[1] - current_com[1])**2)

        if zmp_distance > support_polygon_radius:
            # Need to take a recovery step
            recovery_step_x = capture_point_x
            recovery_step_y = capture_point_y
            return {
                'recovery_needed': True,
                'step_target': (recovery_step_x, recovery_step_y),
                'time_to_step': 0.5  # Time until next step
            }
        else:
            return {
                'recovery_needed': False,
                'step_target': None,
                'time_to_step': 1.0  # Normal step timing
            }

    def generate_full_walking_pattern(self, walk_distance, step_width=0.2):
        """
        Generate complete walking pattern including CoM, ZMP, and foot trajectories
        """
        footsteps = self.generate_footstep_pattern(walk_distance, step_width)
        step_times = [0.8] * len(footsteps)  # 0.8 seconds per step

        # Compute ZMP trajectory
        zmp_times, zmp_x, zmp_y = self.compute_zmp_trajectory(footsteps, step_times)

        # Compute CoM trajectory from ZMP
        com_x, com_y = self.compute_com_trajectory_from_zmp(zmp_x, zmp_y)

        # Generate swing foot trajectories
        swing_trajectories = []
        for i in range(len(footsteps)-1):
            start_pos = (0, step_width/2 if i%2==0 else -step_width/2, 0)  # Simplified
            end_pos = footsteps[i]
            swing_traj = self.generate_swing_trajectory(start_pos, end_pos, self.step_height, step_times[i])
            swing_trajectories.append(swing_traj)

        return {
            'footsteps': footsteps,
            'com_trajectory': list(zip(com_x, com_y)),
            'zmp_trajectory': list(zip(zmp_x, zmp_y)),
            'swing_trajectories': swing_trajectories,
            'time_points': zmp_times
        }
```

:::tip
For robust walking, implement preview control that looks ahead at the next few footsteps to plan CoM trajectories that ensure stability throughout the entire walking sequence, not just the current step.
:::

![Walking gait diagram showing ZMP trajectory, CoM movement, and footstep pattern](./assets/walking-gait.png)