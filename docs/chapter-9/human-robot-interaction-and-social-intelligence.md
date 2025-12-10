---
title: Human-Robot Interaction and Social Intelligence
sidebar_position: 9
description: Social cognition principles and natural interaction methods for humanoid robots
---

# Human-Robot Interaction and Social Intelligence

Human-Robot Interaction (HRI) in the context of physical AI involves the design of robots that can communicate, collaborate, and coexist with humans in shared physical spaces. Social intelligence for robots encompasses not only the ability to recognize and respond to human social cues but also to generate appropriate social behaviors that facilitate natural and effective interaction. This requires understanding of human psychology, social norms, and communication modalities that go beyond traditional command-based interfaces.

Social cognition in humanoid robots involves the ability to interpret human intentions, emotions, and social context from multiple cues including facial expressions, body language, voice tone, and situational context. This interpretation enables robots to respond appropriately to human social signals and adjust their behavior accordingly. The challenge lies in processing these multimodal social signals in real-time and generating contextually appropriate responses.

Natural communication modalities for humanoid robots extend beyond speech to include gestures, gaze direction, proxemics (use of space), and even touch when appropriate. The robot's physical form enables it to use human-like communication channels, making interactions more intuitive for human users. However, this also introduces the challenge of avoiding the uncanny valley effect, where robots that appear almost human but not quite trigger negative emotional responses.

Trust and acceptance in HRI are critical for successful deployment of social robots. Trust is built through consistent, predictable behavior, appropriate response to social cues, and transparency in the robot's capabilities and limitations. The robot's ability to explain its actions and communicate its internal state when needed can significantly improve human acceptance and trust.

Collaborative task execution requires robots to understand human intentions, anticipate needs, and coordinate actions seamlessly with human partners. This involves shared attention mechanisms, intention recognition, and adaptive behavior that can accommodate different human working styles and preferences.

```cpp
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Dense>

enum class SocialState {
    NEUTRAL,
    ATTENTIVE,
    ENGAGED,
    COLLABORATIVE,
    SOCIAL_NORM_VIOLATION
};

enum class EmotionalState {
    NEUTRAL,
    HAPPY,
    SAD,
    ANGRY,
    SURPRISED,
    FEARFUL
};

struct HumanPerception {
    Eigen::Vector3d position;           // World position of human
    Eigen::Vector3d head_orientation;   // Head direction vector
    std::string facial_expression;      // Detected facial expression
    double attention_level;             // How much attention human is paying to robot
    EmotionalState emotional_state;     // Perceived emotional state
    std::vector<std::string> gestures;  // Detected gestures
    std::string speech_content;         // Speech content (if speech recognition available)
    double confidence;                  // Confidence in perception

    HumanPerception() : attention_level(0.0), emotional_state(EmotionalState::NEUTRAL), confidence(0.0) {}
};

class SocialInteractionManager {
private:
    SocialState current_state;
    std::vector<HumanPerception> perceived_humans;
    double personal_space_radius;       // Radius of personal space (meters)
    double social_space_radius;         // Radius of social space (meters)
    double gaze_timeout;                // Time to maintain gaze before looking away (seconds)
    double last_gaze_change_time;
    std::string interaction_mode;       // Current interaction mode

public:
    SocialInteractionManager() {
        current_state = SocialState::NEUTRAL;
        personal_space_radius = 0.5;     // 50cm personal space
        social_space_radius = 1.2;       // 1.2m social space
        gaze_timeout = 3.0;              // Look away after 3 seconds
        last_gaze_change_time = 0.0;
        interaction_mode = "casual";
    }

    SocialState processSocialSituation(const std::vector<HumanPerception>& humans) {
        perceived_humans = humans;

        // Evaluate social context
        int engaged_humans = 0;
        double total_attention = 0.0;

        for (const auto& human : humans) {
            if (human.attention_level > 0.5) {
                engaged_humans++;
                total_attention += human.attention_level;
            }
        }

        // Update social state based on context
        if (engaged_humans == 0) {
            current_state = SocialState::NEUTRAL;
        } else if (engaged_humans == 1 && total_attention / engaged_humans > 0.7) {
            current_state = SocialState::ATTENTIVE;
        } else if (engaged_humans >= 1 && isCollaborationDetected()) {
            current_state = SocialState::COLLABORATIVE;
        } else {
            current_state = SocialState::ENGAGED;
        }

        return current_state;
    }

    bool isPersonalSpaceViolated(const Eigen::Vector3d& robot_pos) {
        for (const auto& human : perceived_humans) {
            double distance = (human.position - robot_pos).norm();
            if (distance < personal_space_radius) {
                return true;
            }
        }
        return false;
    }

    std::vector<Eigen::Vector3d> computeGazeTargets(const Eigen::Vector3d& robot_pos) {
        std::vector<Eigen::Vector3d> gaze_targets;

        for (const auto& human : perceived_humans) {
            // Prioritize gaze toward eyes for engagement
            Eigen::Vector3d eye_position = human.position;
            eye_position.z() += 0.15;  // Approximate eye height above position

            // Don't stare continuously - use social gaze patterns
            if (current_state == SocialState::ATTENTIVE || current_state == SocialState::ENGAGED) {
                gaze_targets.push_back(eye_position);
            }
        }

        return gaze_targets;
    }

    bool isCollaborationDetected() {
        // Simplified collaboration detection based on spatial and temporal coordination
        if (perceived_humans.size() < 1) return false;

        // Check if humans are performing coordinated actions near robot
        // This would involve more complex analysis in practice
        for (const auto& human : perceived_humans) {
            if (human.gestures.size() > 0 &&
                (human.gestures[0] == "pointing_at_robot" ||
                 human.gestures[0] == "beckoning" ||
                 human.gestures[0] == "hand_over_object")) {
                return true;
            }
        }

        return false;
    }

    std::vector<std::string> generateAppropriateBehaviors() {
        std::vector<std::string> behaviors;

        switch (current_state) {
            case SocialState::NEUTRAL:
                behaviors.push_back("idle_pose");
                behaviors.push_back("environment_scanning");
                break;

            case SocialState::ATTENTIVE:
                behaviors.push_back("direct_gaze");
                behaviors.push_back("attentive_posture");
                if (isPersonalSpaceViolated(Eigen::Vector3d(0,0,0))) {  // Simplified position check
                    behaviors.push_back("respectful_distance");
                }
                break;

            case SocialState::ENGAGED:
                behaviors.push_back("responsive_gestures");
                behaviors.push_back("appropriate_backchanneling");
                behaviors.push_back("contextual_replies");
                break;

            case SocialState::COLLABORATIVE:
                behaviors.push_back("collaborative_pose");
                behaviors.push_back("joint_attention");
                behaviors.push_back("turn_taking_signals");
                behaviors.push_back("task_coordination");
                break;

            case SocialState::SOCIAL_NORM_VIOLATION:
                behaviors.push_back("apology_behavior");
                behaviors.push_back("corrective_action");
                behaviors.push_back("social_norm_acknowledgment");
                break;
        }

        return behaviors;
    }

    std::string selectAppropriateResponse(const HumanPerception& human, double elapsed_time) {
        // Select response based on human's emotional state and context
        std::string response;

        switch (human.emotional_state) {
            case EmotionalState::HAPPY:
                response = "positive_acknowledgment";
                break;
            case EmotionalState::SAD:
                response = "empathetic_response";
                break;
            case EmotionalState::ANGRY:
                response = "deescalation_behavior";
                break;
            case EmotionalState::SURPRISED:
                response = "acknowledgment_of_surprise";
                break;
            case EmotionalState::FEARFUL:
                response = "reassurance_behavior";
                break;
            case EmotionalState::NEUTRAL:
            default:
                if (current_state == SocialState::ATTENTIVE) {
                    response = "engagement_initiation";
                } else {
                    response = "polite_acknowledgment";
                }
                break;
        }

        return response;
    }

    void updateTrustModel(const std::string& robot_action, bool human_reaction_positive) {
        // Update internal trust model based on interaction outcomes
        // In practice, this would maintain a more complex trust model
        if (human_reaction_positive) {
            // Increase trust for this type of action
        } else {
            // Decrease trust for this type of action
        }
    }
};

class CulturalAdaptationModule {
private:
    std::map<std::string, std::vector<std::string>> cultural_norms;  // culture -> acceptable behaviors
    std::string current_culture;

public:
    CulturalAdaptationModule() {
        // Define basic cultural norms (in practice, this would be more extensive)
        cultural_norms["american"] = {"direct_eye_contact", "firm_handshake", "respectful_distance"};
        cultural_norms["japanese"] = {"avoid_direct_eye_contact", "bow_greeting", "greater_personal_space"};
        cultural_norms["middle_eastern"] = {"avoid_opposite_gender_contact", "respectful_gestures", "moderate_eye_contact"};

        current_culture = "universal";  // Default to universal norms
    }

    void setCulture(const std::string& culture) {
        if (cultural_norms.find(culture) != cultural_norms.end()) {
            current_culture = culture;
        }
    }

    std::vector<std::string> getAcceptableBehaviors() {
        if (cultural_norms.find(current_culture) != cultural_norms.end()) {
            return cultural_norms[current_culture];
        }
        return cultural_norms["american"];  // Fallback
    }

    bool isBehaviorAcceptable(const std::string& behavior) {
        auto norms = getAcceptableBehaviors();
        return std::find(norms.begin(), norms.end(), behavior) != norms.end();
    }
};

class SocialRobotController {
private:
    SocialInteractionManager interaction_manager;
    CulturalAdaptationModule culture_module;
    double last_interaction_time;

public:
    struct SocialAction {
        std::string action_type;  // "gaze", "gesture", "speech", etc.
        std::string content;      // Specific action content
        double priority;          // Action priority (0.0-1.0)
        bool executable;          // Whether action can be performed
    };

    std::vector<SocialAction> computeSocialResponse(const std::vector<HumanPerception>& humans,
                                                   double current_time) {
        // Process social situation
        SocialState current_state = interaction_manager.processSocialSituation(humans);

        std::vector<SocialAction> actions;

        // Generate appropriate behaviors based on social state
        std::vector<std::string> behaviors = interaction_manager.generateAppropriateBehaviors();

        for (const auto& behavior : behaviors) {
            SocialAction action;
            action.executable = true;

            if (behavior == "direct_gaze") {
                action.action_type = "gaze";
                auto targets = interaction_manager.computeGazeTargets(Eigen::Vector3d(0,0,0));
                if (!targets.empty()) {
                    action.content = "look_at_human";
                    action.priority = 0.8;
                }
            } else if (behavior == "responsive_gestures") {
                action.action_type = "gesture";
                action.content = "nodding";
                action.priority = 0.7;
            } else if (behavior == "respectful_distance") {
                action.action_type = "navigation";
                action.content = "move_away";
                action.priority = 0.9;  // High priority for personal space violation
            } else {
                action.action_type = "posture";
                action.content = behavior;
                action.priority = 0.5;
            }

            // Check if behavior is culturally acceptable
            if (!culture_module.isBehaviorAcceptable(behavior)) {
                action.executable = false;
                action.priority = 0.0;
            }

            actions.push_back(action);
        }

        // Sort actions by priority (highest first)
        std::sort(actions.begin(), actions.end(),
                  [](const SocialAction& a, const SocialAction& b) {
                      return a.priority > b.priority;
                  });

        return actions;
    }

    void setTargetCulture(const std::string& culture) {
        culture_module.setCulture(culture);
    }
};
```

:::tip
When designing social robots, consider implementing a "social pause" feature that allows humans to initiate interaction at their comfort level rather than having the robot approach immediately. This can significantly improve acceptance and trust.
:::

![Human-robot interaction diagram showing social spaces, communication modalities, and collaborative behaviors](./assets/hri-social-intelligence.png)