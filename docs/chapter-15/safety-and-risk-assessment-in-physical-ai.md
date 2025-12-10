---
title: Safety and Risk Assessment in Physical AI
sidebar_position: 15
description: Risk evaluation frameworks and safety mechanisms for physical AI systems operating in human environments
---

# Safety and Risk Assessment in Physical AI

Safety and risk assessment form the cornerstone of responsible Physical AI development, particularly as these systems increasingly operate in close proximity to humans in shared environments. Unlike traditional industrial robots that operate behind safety barriers, humanoid robots must navigate complex social and physical spaces while ensuring the safety of both themselves and nearby humans. This requires comprehensive risk assessment frameworks that evaluate potential hazards across multiple domains: mechanical, electrical, thermal, and behavioral. The challenge lies in creating safety systems that are both comprehensive and responsive enough to handle the dynamic nature of physical AI interactions.

Risk assessment in Physical AI must account for the inherent uncertainties in real-world environments, including unpredictable human behavior, varying environmental conditions, and system degradation over time. These systems must continuously evaluate their operational state and environment to identify potential risks before they materialize into actual hazards. The assessment process involves modeling potential failure modes, analyzing the consequences of different failure scenarios, and implementing mitigation strategies that can adapt to changing conditions.

Safety-critical control systems for Physical AI implement multiple layers of protection, from low-level hardware safety mechanisms to high-level behavioral constraints. These systems must be able to detect and respond to safety violations within milliseconds, often requiring dedicated safety processors that operate independently of the main control system. The challenge is to maintain safety without overly constraining the robot's capabilities or responsiveness.

Fail-safe mechanisms ensure that when a system failure occurs, the robot transitions to a safe state that minimizes risk to humans and property. This includes emergency stop procedures, controlled shutdown sequences, and backup systems that can maintain basic safety functions even when primary systems fail. The design of fail-safe mechanisms must consider all possible failure modes and ensure that the robot can safely handle each scenario.

Human-aware safety protocols recognize that humans are the most important consideration in Physical AI safety. These protocols must account for human unpredictability, varying physical capabilities, and the need for graceful interaction even during safety interventions. The challenge is to create safety systems that protect humans without creating fear or anxiety that could impede natural human-robot interaction.

```cpp
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <map>
#include <string>
#include <functional>
#include <chrono>
#include <set>
#include <algorithm>

enum class RiskLevel {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

enum class SafetyViolation {
    COLLISION_IMMINENT,
    EXCEEDING_FORCE_LIMITS,
    UNSTABLE_BALANCE,
    HARDWARE_FAULT,
    BEHAVIOR_MISSED,
    ENVIRONMENT_HAZARD
};

struct SafetyConstraint {
    std::string name;
    std::function<bool()> check_function;
    RiskLevel risk_level;
    std::string description;

    SafetyConstraint(const std::string& n, std::function<bool()> f, RiskLevel level, const std::string& desc)
        : name(n), check_function(f), risk_level(level), description(desc) {}
};

struct SafetyEvent {
    SafetyViolation violation_type;
    RiskLevel risk_level;
    double severity_score;
    std::string description;
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
    std::vector<std::string> affected_systems;

    SafetyEvent(SafetyViolation type, RiskLevel level, double score, const std::string& desc)
        : violation_type(type), risk_level(level), severity_score(score), description(desc),
          timestamp(std::chrono::steady_clock::now()) {}
};

class RiskAssessmentEngine {
private:
    std::vector<SafetyConstraint> safety_constraints;
    std::vector<SafetyEvent> safety_log;
    std::map<SafetyViolation, RiskLevel> violation_risk_mapping;
    double current_risk_score;
    RiskLevel overall_risk_level;

public:
    RiskAssessmentEngine() : current_risk_score(0.0), overall_risk_level(RiskLevel::LOW) {
        // Initialize violation risk mapping
        violation_risk_mapping[SafetyViolation::COLLISION_IMMINENT] = RiskLevel::CRITICAL;
        violation_risk_mapping[SafetyViolation::EXCEEDING_FORCE_LIMITS] = RiskLevel::HIGH;
        violation_risk_mapping[SafetyViolation::UNSTABLE_BALANCE] = RiskLevel::MEDIUM;
        violation_risk_mapping[SafetyViolation::HARDWARE_FAULT] = RiskLevel::HIGH;
        violation_risk_mapping[SafetyViolation::BEHAVIOR_MISSED] = RiskLevel::MEDIUM;
        violation_risk_mapping[SafetyViolation::ENVIRONMENT_HAZARD] = RiskLevel::MEDIUM;
    }

    void addSafetyConstraint(const SafetyConstraint& constraint) {
        safety_constraints.push_back(constraint);
    }

    RiskLevel evaluateCurrentRisk() {
        current_risk_score = 0.0;
        overall_risk_level = RiskLevel::LOW;

        for (const auto& constraint : safety_constraints) {
            if (!constraint.check_function()) {
                // Constraint violated
                SafetyEvent event(SafetyViolation::BEHAVIOR_MISSED, constraint.risk_level, 1.0, constraint.description);
                safety_log.push_back(event);

                // Update risk score based on violation level
                double violation_score = 0.0;
                switch (constraint.risk_level) {
                    case RiskLevel::LOW: violation_score = 0.1; break;
                    case RiskLevel::MEDIUM: violation_score = 0.3; break;
                    case RiskLevel::HIGH: violation_score = 0.7; break;
                    case RiskLevel::CRITICAL: violation_score = 1.0; break;
                }

                current_risk_score += violation_score;

                // Update overall risk level
                if (static_cast<int>(constraint.risk_level) > static_cast<int>(overall_risk_level)) {
                    overall_risk_level = constraint.risk_level;
                }
            }
        }

        // Cap risk score at 10.0
        current_risk_score = std::min(current_risk_score, 10.0);

        return overall_risk_level;
    }

    double calculateCollisionRisk(const Eigen::Vector3d& robot_pos, const Eigen::Vector3d& human_pos,
                                double robot_velocity, double human_velocity) {
        double distance = (robot_pos - human_pos).norm();
        double relative_velocity = std::abs(robot_velocity - human_velocity);

        // Collision risk increases as distance decreases and relative velocity increases
        if (distance < 0.5) {  // Less than 50cm
            return std::min(1.0, (0.5 - distance) * 10.0 * (1.0 + relative_velocity));
        }
        return 0.0;
    }

    double calculateForceRisk(double applied_force, double maximum_safe_force) {
        if (applied_force > maximum_safe_force) {
            return std::min(1.0, (applied_force - maximum_safe_force) / maximum_safe_force);
        }
        return 0.0;
    }

    double calculateBalanceRisk(const Eigen::Vector3d& com_position, const Eigen::Vector3d& zmp_position,
                              double support_polygon_radius) {
        double zmp_distance = (com_position.head(2) - zmp_position.head(2)).norm();
        if (zmp_distance > support_polygon_radius) {
            return std::min(1.0, (zmp_distance - support_polygon_radius) / support_polygon_radius);
        }
        return 0.0;
    }

    std::vector<SafetyEvent> getRecentSafetyEvents(double time_window_seconds = 10.0) {
        auto now = std::chrono::steady_clock::now();
        std::vector<SafetyEvent> recent_events;

        for (const auto& event : safety_log) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - event.timestamp);
            if (duration.count() <= time_window_seconds) {
                recent_events.push_back(event);
            }
        }

        return recent_events;
    }

    void resetSafetyLog() {
        safety_log.clear();
    }
};

class SafetyController {
private:
    RiskAssessmentEngine risk_engine;
    std::set<SafetyViolation> active_violations;
    bool emergency_stop_activated;
    std::chrono::time_point<std::chrono::steady_clock> last_emergency_time;

public:
    SafetyController() : emergency_stop_activated(false) {
        last_emergency_time = std::chrono::steady_clock::now();
    }

    enum class SafetyResponse {
        CONTINUE_NORMAL,
        REDUCED_SPEED,
        PAUSE_OPERATION,
        EMERGENCY_STOP
    };

    SafetyResponse assessAndRespond() {
        RiskLevel current_risk = risk_engine.evaluateCurrentRisk();
        auto recent_events = risk_engine.getRecentSafetyEvents(1.0); // Check last second

        // Count critical violations in recent events
        int critical_violations = 0;
        for (const auto& event : recent_events) {
            if (event.risk_level == RiskLevel::CRITICAL) {
                critical_violations++;
            }
        }

        if (current_risk == RiskLevel::CRITICAL || critical_violations > 0) {
            activateEmergencyStop();
            return SafetyResponse::EMERGENCY_STOP;
        } else if (current_risk == RiskLevel::HIGH || critical_violations > 0) {
            return SafetyResponse::PAUSE_OPERATION;
        } else if (current_risk == RiskLevel::MEDIUM) {
            return SafetyResponse::REDUCED_SPEED;
        } else {
            deactivateEmergencyStop();
            return SafetyResponse::CONTINUE_NORMAL;
        }
    }

    void activateEmergencyStop() {
        if (!emergency_stop_activated) {
            emergency_stop_activated = true;
            last_emergency_time = std::chrono::steady_clock::now();
            // In real system, would send emergency stop commands to all actuators
        }
    }

    void deactivateEmergencyStop() {
        emergency_stop_activated = false;
        // In real system, would resume normal operation gradually
    }

    bool isEmergencyStopActive() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_emergency_time);
        // Keep emergency stop active for at least 100ms to ensure safety
        return emergency_stop_activated && duration.count() < 100;
    }

    void addSafetyConstraint(const SafetyConstraint& constraint) {
        risk_engine.addSafetyConstraint(constraint);
    }

    double getRiskScore() const {
        // This would interface with the risk engine to get current risk
        return 0.0; // Simplified
    }
};

class HumanAwareSafety {
private:
    std::vector<Eigen::Vector3d> detected_humans;
    std::vector<double> human_confidence;
    std::vector<Eigen::Vector3d> human_velocities;
    double personal_space_radius;
    double safety_buffer;

public:
    HumanAwareSafety() : personal_space_radius(0.8), safety_buffer(0.2) {} // 80cm personal space + 20cm buffer

    struct HumanSafetyAssessment {
        int human_id;
        double collision_risk;
        double proximity_risk;
        double movement_uncertainty;
        RiskLevel overall_risk;
    };

    std::vector<HumanSafetyAssessment> assessHumanSafety(
        const Eigen::Vector3d& robot_position,
        const Eigen::Vector3d& robot_velocity) {

        std::vector<HumanSafetyAssessment> assessments;

        for (size_t i = 0; i < detected_humans.size(); ++i) {
            HumanSafetyAssessment assessment;
            assessment.human_id = i;

            // Calculate collision risk
            double distance = (robot_position - detected_humans[i]).norm();
            double relative_speed = (robot_velocity - human_velocities[i]).norm();

            assessment.collision_risk = risk_engine.calculateCollisionRisk(
                robot_position, detected_humans[i],
                robot_velocity.norm(), human_velocities[i].norm()
            );

            // Calculate proximity risk (how close we are to human)
            double min_safe_distance = personal_space_radius + safety_buffer;
            if (distance < min_safe_distance) {
                assessment.proximity_risk = (min_safe_distance - distance) / min_safe_distance;
            } else {
                assessment.proximity_risk = 0.0;
            }

            // Movement uncertainty (how unpredictable the human's movement is)
            assessment.movement_uncertainty = 1.0 - human_confidence[i];

            // Overall risk is maximum of individual risks
            double max_risk = std::max({assessment.collision_risk,
                                      assessment.proximity_risk,
                                      assessment.movement_uncertainty});

            if (max_risk > 0.8) {
                assessment.overall_risk = RiskLevel::CRITICAL;
            } else if (max_risk > 0.5) {
                assessment.overall_risk = RiskLevel::HIGH;
            } else if (max_risk > 0.2) {
                assessment.overall_risk = RiskLevel::MEDIUM;
            } else {
                assessment.overall_risk = RiskLevel::LOW;
            }

            assessments.push_back(assessment);
        }

        return assessments;
    }

    void updateHumanDetection(const std::vector<Eigen::Vector3d>& humans,
                            const std::vector<double>& confidences,
                            const std::vector<Eigen::Vector3d>& velocities) {
        detected_humans = humans;
        human_confidence = confidences;
        human_velocities = velocities;
    }

    Eigen::Vector3d calculateSafeDirection(const Eigen::Vector3d& robot_position,
                                         const std::vector<HumanSafetyAssessment>& assessments) {
        // Calculate repulsive forces from humans to determine safe direction
        Eigen::Vector3d repulsive_force = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < detected_humans.size(); ++i) {
            Eigen::Vector3d to_human = detected_humans[i] - robot_position;
            double distance = to_human.norm();

            if (distance < personal_space_radius + safety_buffer) {
                // Strong repulsive force when too close
                Eigen::Vector3d force_dir = to_human.normalized();
                double force_magnitude = 1.0 / (distance * distance + 0.01); // Inverse square law
                repulsive_force += force_dir * force_magnitude * assessments[i].collision_risk;
            }
        }

        // If no significant repulsive force, return a default safe direction
        if (repulsive_force.norm() < 0.01) {
            return Eigen::Vector3d(0, 0, 1); // Move upward (or any default direction)
        }

        return -repulsive_force.normalized(); // Opposite to repulsive force
    }
};

class SafetyMonitor {
private:
    SafetyController safety_controller;
    HumanAwareSafety human_safety;
    std::vector<std::function<void(const SafetyEvent&)>> event_callbacks;
    bool monitoring_active;

public:
    SafetyMonitor() : monitoring_active(true) {}

    void registerSafetyCallback(std::function<void(const SafetyEvent&)> callback) {
        event_callbacks.push_back(callback);
    }

    struct SafetyStatus {
        RiskLevel current_risk_level;
        bool emergency_stop_active;
        std::vector<HumanAwareSafety::HumanSafetyAssessment> human_safety_assessments;
        double system_health_score;
        std::vector<SafetyViolation> active_violations;
    };

    SafetyStatus getSafetyStatus(
        const Eigen::Vector3d& robot_position,
        const Eigen::Vector3d& robot_velocity,
        const std::vector<Eigen::Vector3d>& humans,
        const std::vector<double>& human_confidences,
        const std::vector<Eigen::Vector3d>& human_velocities) {

        // Update human detection
        human_safety.updateHumanDetection(humans, human_confidences, human_velocities);

        // Perform human safety assessment
        auto human_assessments = human_safety.assessHumanSafety(robot_position, robot_velocity);

        // Get safety controller response
        auto response = safety_controller.assessAndRespond();

        SafetyStatus status;
        status.current_risk_level = RiskLevel::LOW; // Would be determined by risk engine
        status.emergency_stop_active = safety_controller.isEmergencyStopActive();
        status.human_safety_assessments = human_assessments;
        status.system_health_score = 0.9; // Simplified health score
        // status.active_violations would be populated from risk engine

        return status;
    }

    void setPersonalSpaceRadius(double radius) {
        human_safety.personal_space_radius = radius;
    }

    void setSafetyBuffer(double buffer) {
        human_safety.safety_buffer = buffer;
    }

    void startMonitoring() {
        monitoring_active = true;
    }

    void stopMonitoring() {
        monitoring_active = false;
    }
};
```

:::warning
Always implement redundant safety systems that can operate independently of the main control system. A single point of failure in safety-critical systems can have catastrophic consequences. Regular safety system testing and validation should be performed to ensure all safety mechanisms function correctly under various operating conditions.
:::

![Safety assessment diagram showing risk evaluation, human detection, and emergency response systems](./assets/safety-risk-assessment.png)