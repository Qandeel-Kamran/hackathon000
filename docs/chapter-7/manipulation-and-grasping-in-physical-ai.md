---
title: Manipulation and Grasping in Physical AI
sidebar_position: 7
description: Dexterous manipulation strategies and grasp planning for embodied AI systems
---

# Manipulation and Grasping in Physical AI

Manipulation in Physical AI systems requires the integration of perception, planning, and control to interact with objects in the physical world. Unlike traditional robotic manipulation that often occurs in controlled environments, Physical AI manipulation must handle uncertainty, dynamic environments, and the need to coordinate with other behaviors like locomotion and balance. The tight coupling between perception and action is particularly important in manipulation, where tactile feedback and visual servoing guide the manipulation process in real-time.

Kinematics of manipulation focuses on the geometric relationships between the robot's end-effector, the object being manipulated, and the environment. Forward kinematics computes the position and orientation of the end-effector given joint angles, while inverse kinematics solves for the joint angles needed to achieve a desired end-effector pose. In manipulation tasks, these calculations must account for object poses, grasp points, and collision avoidance with the environment.

Grasp planning involves determining how to securely grasp an object while considering factors like object geometry, weight, fragility, and the intended manipulation task. A good grasp must provide sufficient friction and contact points to resist the forces and moments that will be applied during manipulation. Force control is crucial for establishing and maintaining grasps, especially when dealing with objects of unknown properties or in unstructured environments.

Dexterous manipulation goes beyond simple pick-and-place tasks to include complex manipulation behaviors like in-hand manipulation, tool use, and multi-finger coordination. These behaviors require sophisticated control strategies that can manage multiple contact points, switch between different grasp types, and coordinate finger movements for complex tasks.

Learning-based manipulation approaches leverage the robot's experience with physical interactions to improve manipulation performance over time. This includes learning object properties, grasp success probabilities, and manipulation strategies through interaction with the environment. Physical interaction provides rich sensory feedback that can be used to refine manipulation models and improve future performance.

```cpp
#include <vector>
#include <Eigen/Dense>
#include <memory>

struct ContactPoint {
    Eigen::Vector3d position;    // 3D position of contact
    Eigen::Vector3d normal;      // Surface normal at contact
    double friction_coeff;       // Friction coefficient
    bool active;                 // Whether contact is active

    ContactPoint(Eigen::Vector3d pos, Eigen::Vector3d n, double mu)
        : position(pos), normal(n), friction_coeff(mu), active(true) {}
};

class GraspPlanner {
private:
    double gravity = 9.81;

public:
    struct Grasp {
        std::vector<ContactPoint> contacts;
        Eigen::Vector6d wrench;  // Force and torque
        double quality;          // Grasp quality metric
        bool stable;             // Whether grasp is stable

        Grasp(std::vector<ContactPoint> c, Eigen::Vector6d w, double q, bool s)
            : contacts(c), wrench(w), quality(q), stable(s) {}
    };

    struct ObjectProperties {
        Eigen::Vector3d center_of_mass;
        double mass;
        Eigen::Matrix3d inertia;
        std::vector<Eigen::Vector3d> surface_points;
        std::vector<Eigen::Vector3d> surface_normals;
    };

    std::vector<Grasp> generateGraspCandidates(const ObjectProperties& obj,
                                              const Eigen::Vector3d& approach_dir,
                                              int num_candidates = 20) {
        std::vector<Grasp> candidates;

        // Generate grasp candidates by sampling contact points
        for (int i = 0; i < num_candidates; ++i) {
            // Sample two contact points on the object surface
            if (obj.surface_points.size() < 2) continue;

            int idx1 = i % obj.surface_points.size();
            int idx2 = (i + num_candidates / 2) % obj.surface_points.size();

            ContactPoint contact1(obj.surface_points[idx1], obj.surface_normals[idx1], 0.8);
            ContactPoint contact2(obj.surface_points[idx2], obj.surface_normals[idx2], 0.8);

            std::vector<ContactPoint> contacts = {contact1, contact2};

            // Calculate required wrench to resist gravity
            Eigen::Vector6d wrench;
            wrench.head(3) = Eigen::Vector3d(0, 0, obj.mass * gravity);  // Force to counter gravity
            wrench.tail(3) = Eigen::Vector3d(0, 0, 0);  // Torque (simplified)

            // Calculate grasp quality (simplified: based on contact geometry)
            double quality = calculateGraspQuality(contacts, obj.mass * gravity * Eigen::Vector3d(0, 0, -1));

            // Check if grasp is stable
            bool stable = isGraspStable(contacts, wrench);

            candidates.emplace_back(contacts, wrench, quality, stable);
        }

        return candidates;
    }

    double calculateGraspQuality(const std::vector<ContactPoint>& contacts,
                                const Eigen::Vector3d& external_force) {
        // Simplified grasp quality metric based on force closure
        if (contacts.size() < 2) return 0.0;

        // For a 2-finger grasp, check if the line between contacts
        // can resist the external force direction
        Eigen::Vector3d contact_line = contacts[1].position - contacts[0].position;
        double force_alignment = std::abs(contact_line.dot(external_force.normalized()));

        // Factor in friction cones
        double friction_factor = 0.0;
        for (const auto& contact : contacts) {
            friction_factor += contact.friction_coeff;
        }
        friction_factor /= contacts.size();

        // Combine factors (simplified)
        return force_alignment * friction_factor * 10.0;  // Scale factor
    }

    bool isGraspStable(const std::vector<ContactPoint>& contacts,
                      const Eigen::Vector6d& wrench) {
        // Check if the grasp can resist the applied wrench
        // Simplified: check if contact forces can balance external wrench
        if (contacts.size() < 2) return false;

        // In reality, would check force closure using convex hull of friction cones
        // This is a simplified check
        double total_normal_force = 0.0;
        for (const auto& contact : contacts) {
            total_normal_force += std::abs(contact.normal.dot(Eigen::Vector3d(0, 0, 1)));  // Z component
        }

        // Compare with required force to resist gravity
        double required_force = std::abs(wrench(2));  // Z component of force

        return total_normal_force * 0.8 > required_force;  // Friction factor of 0.8
    }

    Grasp selectBestGrasp(const std::vector<Grasp>& candidates) {
        // Select the grasp with highest quality that is stable
        Grasp best_grasp = candidates[0];
        double best_quality = -1.0;

        for (const auto& grasp : candidates) {
            if (grasp.stable && grasp.quality > best_quality) {
                best_grasp = grasp;
                best_quality = grasp.quality;
            }
        }

        return best_grasp;
    }
};

class ManipulationController {
private:
    GraspPlanner grasp_planner;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;

public:
    struct ManipulationTrajectory {
        std::vector<std::vector<double>> joint_angles;
        std::vector<Eigen::Vector3d> end_effector_positions;
        std::vector<Eigen::Vector3d> end_effector_orientations;
        std::vector<double> timestamps;
    };

    ManipulationTrajectory planReachingTrajectory(const Eigen::Vector3d& target_pos,
                                                 const Eigen::Vector3d& target_orient,
                                                 const std::vector<double>& current_joints) {
        // Simplified trajectory planning using inverse kinematics
        ManipulationTrajectory traj;

        // Interpolate from current to target position
        Eigen::Vector3d current_pos = computeEndEffectorPosition(current_joints);

        int num_waypoints = 50;
        for (int i = 0; i <= num_waypoints; ++i) {
            double t = static_cast<double>(i) / num_waypoints;
            Eigen::Vector3d pos = current_pos + t * (target_pos - current_pos);

            // Solve inverse kinematics for this position
            std::vector<double> joint_angles = inverseKinematics(pos, target_orient);

            traj.joint_angles.push_back(joint_angles);
            traj.end_effector_positions.push_back(pos);
            traj.end_effector_orientations.push_back(target_orient);
            traj.timestamps.push_back(t * 2.0);  // 2 seconds for reaching
        }

        return traj;
    }

    std::vector<double> inverseKinematics(const Eigen::Vector3d& target_pos,
                                         const Eigen::Vector3d& target_orient) {
        // Simplified inverse kinematics (in practice, would use more sophisticated methods)
        // This is a placeholder implementation
        std::vector<double> result(6, 0.0);  // 6 DOF arm

        // Calculate joint angles to reach target position
        // (In practice, would use Jacobian-based methods, Cyclic Coordinate Descent, etc.)
        result[0] = std::atan2(target_pos.y(), target_pos.x());  // Base rotation
        result[1] = std::atan2(target_pos.z(), std::sqrt(target_pos.x()*target_pos.x() + target_pos.y()*target_pos.y()));  // Shoulder

        // Additional joint calculations would go here
        return result;
    }

    Eigen::Vector3d computeEndEffectorPosition(const std::vector<double>& joint_angles) {
        // Forward kinematics to compute end-effector position
        // Simplified 2D planar arm calculation
        double l1 = 0.3, l2 = 0.25;  // Link lengths
        double q1 = joint_angles[0], q2 = joint_angles[1];

        double x = l1 * std::cos(q1) + l2 * std::cos(q1 + q2);
        double y = l1 * std::sin(q1) + l2 * std::sin(q1 + q2);
        double z = 0.0;  // Simplified to 2D

        return Eigen::Vector3d(x, y, z);
    }

    void executeGrasp(const GraspPlanner::Grasp& grasp, double grasp_force = 20.0) {
        // Execute the grasp by commanding finger positions and forces
        for (const auto& contact : grasp.contacts) {
            // Move fingers to contact positions and apply grasp force
            // (In practice, would involve tactile feedback and force control)
        }
    }
};
```

:::warning
Always implement force limiting and tactile feedback in manipulation systems to prevent damage to objects or the robot itself. Unexpected contact forces can cause joint damage or object breakage if not properly managed.
:::

![Manipulation diagram showing grasp planning, contact points, and dexterous manipulation](./assets/manipulation-grasping.png)