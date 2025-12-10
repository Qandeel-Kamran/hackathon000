---
title: Whole-Body Motion Planning and Optimization
sidebar_position: 12
description: Comprehensive motion planning algorithms that coordinate all degrees of freedom in humanoid robots
---

# Whole-Body Motion Planning and Optimization

Whole-body motion planning addresses the complex challenge of coordinating all degrees of freedom in a humanoid robot to achieve multiple simultaneous objectives. Unlike traditional robotic systems that plan motion for specific tasks in isolation, whole-body planning must consider the entire robot as an integrated system where movements of one part affect the entire structure. This requires solving high-dimensional optimization problems that balance competing objectives such as reaching a target, maintaining balance, avoiding obstacles, and satisfying dynamic constraints.

The complexity of whole-body motion planning stems from the high-dimensional configuration space of humanoid robots, which typically have 30 or more degrees of freedom. Each joint contributes to the overall motion, and the planner must consider the coupled dynamics between all joints. The challenge is compounded by the need to satisfy multiple constraints simultaneously: kinematic constraints (joint limits, end-effector positions), dynamic constraints (balance, momentum), and environmental constraints (obstacles, contacts).

Optimization-based approaches to whole-body motion planning formulate the problem as a mathematical optimization where the objective function represents the desired motion characteristics, and constraints enforce the various requirements. These approaches can handle complex, multi-objective problems by combining different cost terms with appropriate weights. The computational challenge lies in solving these high-dimensional optimization problems in real-time for dynamic tasks.

Hierarchical optimization strategies decompose the complex whole-body problem into more manageable subproblems. These approaches prioritize different objectives and solve them in sequence or in parallel, with higher-priority constraints taking precedence over lower-priority ones. This hierarchical structure allows for more efficient computation while still addressing the full complexity of whole-body motion.

Real-time optimization techniques are essential for dynamic whole-body control, where motion plans must be recomputed at high frequencies to respond to changing conditions. These techniques often use simplified models or pre-computed solutions that can be quickly adapted to new situations. The trade-off between solution quality and computational efficiency is critical for maintaining real-time performance.

```cpp
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <map>
#include <string>
#include <chrono>

struct JointState {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;

    JointState(int n_joints) : positions(n_joints, 0.0), velocities(n_joints, 0.0), accelerations(n_joints, 0.0) {}
};

struct Task {
    std::string type;  // "reaching", "balance", "posture", "avoidance"
    Eigen::VectorXd target;
    Eigen::MatrixXd jacobian;
    double weight;
    bool active;

    Task(std::string t, int dim) : type(t), target(Eigen::VectorXd::Zero(dim)),
                                   jacobian(Eigen::MatrixXd::Zero(dim, 1)),
                                   weight(1.0), active(true) {}
};

class QuadraticOptimizer {
private:
    Eigen::MatrixXd H;  // Hessian matrix
    Eigen::VectorXd g;  // Linear term
    Eigen::MatrixXd A;  // Equality constraints matrix
    Eigen::VectorXd b;  // Equality constraints vector
    Eigen::MatrixXd C;  // Inequality constraints matrix
    Eigen::VectorXd d;  // Inequality constraints vector

public:
    QuadraticOptimizer(int n_vars) {
        H = Eigen::MatrixXd::Identity(n_vars, n_vars);
        g = Eigen::VectorXd::Zero(n_vars);
        A = Eigen::MatrixXd::Zero(1, n_vars);
        b = Eigen::VectorXd::Zero(1);
        C = Eigen::MatrixXd::Zero(1, n_vars);
        d = Eigen::VectorXd::Zero(1);
    }

    void setHessian(const Eigen::MatrixXd& hessian) { H = hessian; }
    void setLinearTerm(const Eigen::VectorXd& linear) { g = linear; }
    void setEqualityConstraints(const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq) {
        A = A_eq;
        b = b_eq;
    }
    void setInequalityConstraints(const Eigen::MatrixXd& C_ineq, const Eigen::VectorXd& d_ineq) {
        C = C_ineq;
        d = d_ineq;
    }

    Eigen::VectorXd solve() {
        // Simplified quadratic programming solution
        // In practice, use a proper QP solver like OSQP or qpOASES

        // For unconstrained problem: x = -H^(-1) * g
        if (A.rows() == 1 && A.cols() == 1 && A(0,0) == 0 &&
            C.rows() == 1 && C.cols() == 1 && C(0,0) == 0) {
            return -H.inverse() * g;
        }

        // For constrained problems, this would require a proper QP solver
        // Here we implement a simplified approach using Lagrange multipliers
        // This is a placeholder implementation
        Eigen::VectorXd solution = -H.inverse() * g;
        return solution;
    }
};

class WholeBodyPlanner {
private:
    int num_joints;
    JointState current_state;
    std::vector<Task> tasks;
    std::vector<std::pair<double, double>> joint_limits;  // (min, max)
    Eigen::VectorXd joint_weights;  // Task space weights for each joint

public:
    WholeBodyPlanner(int n_joints) : num_joints(n_joints), current_state(n_joints) {
        joint_weights = Eigen::VectorXd::Ones(n_joints);

        // Initialize joint limits (example values)
        for (int i = 0; i < n_joints; ++i) {
            joint_limits.emplace_back(-3.14, 3.14);  // Example: ±π for all joints
        }
    }

    void addTask(const Task& task) {
        tasks.push_back(task);
    }

    void clearTasks() {
        tasks.clear();
    }

    struct OptimizationResult {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_accelerations;
        bool success;
        double computation_time;
    };

    OptimizationResult planMotion(const JointState& initial_state,
                                  const std::vector<Task>& active_tasks) {
        auto start_time = std::chrono::high_resolution_clock::now();

        current_state = initial_state;
        tasks = active_tasks;

        // Build optimization problem
        int n_vars = num_joints;  // Joint velocities for differential IK
        QuadraticOptimizer optimizer(n_vars);

        // Construct cost function: minimize weighted sum of task errors
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars);

        for (const auto& task : tasks) {
            if (!task.active) continue;

            // Add task cost: ||J * dq - error||^2
            Eigen::VectorXd error = task.jacobian * Eigen::Map<const Eigen::VectorXd>(current_state.velocities.data(), current_state.velocities.size()) - task.target;

            // Weighted task error
            Eigen::MatrixXd weighted_jacobian = task.weight * task.jacobian;
            H += weighted_jacobian.transpose() * weighted_jacobian;
            g += -weighted_jacobian.transpose() * task.weight * task.target;
        }

        // Add joint limit avoidance
        for (int i = 0; i < num_joints; ++i) {
            double current_pos = current_state.positions[i];
            double lower_limit = joint_limits[i].first;
            double upper_limit = joint_limits[i].second;

            // Add penalty for approaching joint limits
            double lower_margin = current_pos - lower_limit;
            double upper_margin = upper_limit - current_pos;

            if (lower_margin < 0.2) {  // Within 0.2 rad of limit
                double penalty = 1000.0 * (0.2 - lower_margin);
                H(i, i) += penalty;
                g(i) -= penalty * 0.1;  // Bias away from limit
            }

            if (upper_margin < 0.2) {  // Within 0.2 rad of limit
                double penalty = 1000.0 * (0.2 - upper_margin);
                H(i, i) += penalty;
                g(i) += penalty * 0.1;  // Bias away from limit
            }
        }

        optimizer.setHessian(H);
        optimizer.setLinearTerm(g);

        // Solve optimization problem
        Eigen::VectorXd solution = optimizer.solve();

        // Convert to joint state changes
        std::vector<double> joint_velocities(solution.data(), solution.data() + solution.size());

        // Integrate to get new positions (simplified Euler integration)
        std::vector<double> joint_positions = current_state.positions;
        for (int i = 0; i < num_joints; ++i) {
            joint_positions[i] += joint_velocities[i] * 0.001;  // dt = 1ms
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double computation_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        OptimizationResult result;
        result.joint_positions = joint_positions;
        result.joint_velocities = joint_velocities;
        result.joint_accelerations.resize(num_joints, 0.0);  // Simplified
        result.success = true;
        result.computation_time = computation_time;

        return result;
    }

    void setJointWeights(const std::vector<double>& weights) {
        if (weights.size() == num_joints) {
            joint_weights = Eigen::Map<const Eigen::VectorXd>(weights.data(), weights.size());
        }
    }

    void setJointLimits(int joint_idx, double min_val, double max_val) {
        if (joint_idx >= 0 && joint_idx < joint_limits.size()) {
            joint_limits[joint_idx] = std::make_pair(min_val, max_val);
        }
    }

    // Hierarchical optimization approach
    OptimizationResult planHierarchicalMotion(const JointState& initial_state,
                                            const std::vector<std::vector<Task>>& hierarchy) {
        auto start_time = std::chrono::high_resolution_clock::now();

        JointState current = initial_state;

        // Process each level of the hierarchy
        for (const auto& level_tasks : hierarchy) {
            if (level_tasks.empty()) continue;

            // Create optimization problem for this level
            int n_vars = num_joints;
            QuadraticOptimizer optimizer(n_vars);

            // Construct cost function for this level
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
            Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars);

            for (const auto& task : level_tasks) {
                if (!task.active) continue;

                Eigen::VectorXd error = task.jacobian * Eigen::Map<const Eigen::VectorXd>(current.velocities.data(), current.velocities.size()) - task.target;

                // Weighted task error
                Eigen::MatrixXd weighted_jacobian = task.weight * task.jacobian;
                H += weighted_jacobian.transpose() * weighted_jacobian;
                g += -weighted_jacobian.transpose() * task.weight * task.target;
            }

            optimizer.setHessian(H);
            optimizer.setLinearTerm(g);

            // Solve for this level
            Eigen::VectorXd solution = optimizer.solve();

            // Apply solution to current state
            for (int i = 0; i < num_joints; ++i) {
                current.velocities[i] += solution[i] * 0.1;  // Blend with previous solution
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double computation_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // Convert to result format
        OptimizationResult result;
        result.joint_positions = current.positions;
        result.joint_velocities = current.velocities;
        result.joint_accelerations.resize(num_joints, 0.0);
        result.success = true;
        result.computation_time = computation_time;

        return result;
    }
};

class MotionOptimizer {
private:
    std::shared_ptr<WholeBodyPlanner> planner;
    std::vector<double> previous_solution;
    bool use_warm_start;

public:
    MotionOptimizer(int num_joints) : planner(std::make_shared<WholeBodyPlanner>(num_joints)),
                                     previous_solution(num_joints, 0.0), use_warm_start(true) {}

    WholeBodyPlanner::OptimizationResult optimizeMotion(const JointState& current_state,
                                                       const std::vector<Task>& tasks) {
        // Use previous solution as warm start if available
        if (use_warm_start && !previous_solution.empty()) {
            // Initialize with previous solution to speed up convergence
        }

        auto result = planner->planMotion(current_state, tasks);

        // Store solution for next iteration
        previous_solution = result.joint_velocities;

        return result;
    }

    // Multi-step trajectory optimization
    std::vector<WholeBodyPlanner::OptimizationResult> optimizeTrajectory(
        const JointState& start_state,
        const std::vector<std::vector<Task>>& task_sequence,
        int num_steps = 10) {

        std::vector<WholeBodyPlanner::OptimizationResult> trajectory;
        JointState current_state = start_state;

        for (int step = 0; step < num_steps; ++step) {
            // Get tasks for this step (cycling through task_sequence if needed)
            const auto& step_tasks = task_sequence[step % task_sequence.size()];

            auto result = optimizeMotion(current_state, step_tasks);

            if (!result.success) {
                break;  // Stop if optimization fails
            }

            trajectory.push_back(result);

            // Update current state for next step
            for (int i = 0; i < current_state.positions.size(); ++i) {
                current_state.positions[i] = result.joint_positions[i];
                current_state.velocities[i] = result.joint_velocities[i];
            }
        }

        return trajectory;
    }
};
```

:::tip
Implement task prioritization in your whole-body motion planner by organizing tasks into hierarchical levels. Higher-priority tasks (like balance) should be satisfied first, with lower-priority tasks (like reaching) being achieved only if they don't compromise higher-priority objectives.
:::

![Whole-body motion planning diagram showing task hierarchy and optimization constraints](./assets/whole-body-motion-planning.png)