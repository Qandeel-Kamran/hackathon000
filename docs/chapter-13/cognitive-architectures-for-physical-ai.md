---
title: Cognitive Architectures for Physical AI
sidebar_position: 13
description: Intelligent decision-making frameworks and cognitive systems for embodied AI
---

# Cognitive Architectures for Physical AI

Cognitive architectures for Physical AI represent the integration of high-level reasoning and decision-making capabilities with the real-time control requirements of embodied systems. Unlike traditional AI systems that operate in virtual environments, Physical AI cognitive architectures must bridge the gap between abstract reasoning and physical interaction, managing the complexity of continuous, dynamic environments while maintaining real-time performance. These architectures must handle uncertainty, adapt to changing conditions, and coordinate multiple behavioral systems simultaneously while ensuring safety and efficiency.

The challenge in designing cognitive architectures for Physical AI lies in creating systems that can perform abstract reasoning while remaining tightly coupled with sensory-motor processes. Traditional symbolic AI approaches often struggle with the continuous, noisy nature of physical environments, while purely reactive systems lack the planning and reasoning capabilities needed for complex tasks. Successful architectures combine multiple approaches, using symbolic reasoning for high-level planning and reactive systems for low-level control.

Subsumption architectures provide a framework for organizing behaviors at different levels of abstraction, where higher-level behaviors can inhibit or modify lower-level ones. This approach is particularly effective for Physical AI because it allows for robust, reactive behaviors at the lowest levels while enabling more sophisticated decision-making at higher levels. The architecture naturally handles conflicts between different behavioral goals through priority-based inhibition.

Memory systems in Physical AI cognitive architectures must handle both episodic memory (specific experiences) and semantic memory (general knowledge) while managing the continuous flow of sensory information. Working memory systems temporarily store relevant information for ongoing tasks, while long-term memory stores learned behaviors, environmental models, and task knowledge. The challenge is to maintain efficient access to relevant information while filtering out irrelevant data in real-time.

Learning mechanisms within cognitive architectures enable robots to improve their performance over time through experience. This includes reinforcement learning for optimizing behaviors, imitation learning for acquiring new skills, and continual learning that allows the system to adapt without forgetting previous knowledge. The integration of learning with existing cognitive structures requires careful design to ensure that learned behaviors integrate smoothly with the overall architecture.

```python
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from enum import Enum
import time
import threading
from dataclasses import dataclass

class BehaviorPriority(Enum):
    EMERGENCY = 0
    SAFETY = 1
    BALANCE = 2
    LOCOMOTION = 3
    MANIPULATION = 4
    INTERACTION = 5
    EXPLORATION = 6
    IDLE = 7

class CognitiveState(Enum):
    PLANNING = "planning"
    EXECUTING = "executing"
    REASONING = "reasoning"
    LEARNING = "learning"
    REACTIVE = "reactive"

@dataclass
class SensoryInput:
    """Container for sensory information"""
    timestamp: float
    vision_data: Optional[Dict] = None
    tactile_data: Optional[Dict] = None
    proprioceptive_data: Optional[Dict] = None
    auditory_data: Optional[Dict] = None
    force_torque_data: Optional[Dict] = None

@dataclass
class MotorCommand:
    """Container for motor commands"""
    joint_positions: List[float]
    joint_velocities: List[float]
    gripper_commands: Optional[List[float]] = None
    priority: BehaviorPriority = BehaviorPriority.IDLE

@dataclass
class TaskGoal:
    """Representation of a task goal"""
    name: str
    description: str
    preconditions: Dict[str, Any]
    effects: Dict[str, Any]
    priority: int
    deadline: Optional[float] = None

class WorkingMemory:
    """Short-term memory for ongoing cognitive processes"""
    def __init__(self, capacity: int = 100):
        self.capacity = capacity
        self.items: Dict[str, Tuple[Any, float]] = {}  # (data, timestamp)
        self.access_counts: Dict[str, int] = {}

    def store(self, key: str, data: Any):
        """Store data in working memory"""
        current_time = time.time()
        self.items[key] = (data, current_time)
        self.access_counts[key] = self.access_counts.get(key, 0) + 1

        # Evict oldest items if capacity exceeded
        if len(self.items) > self.capacity:
            oldest_key = min(self.items.keys(), key=lambda k: self.items[k][1])
            del self.items[oldest_key]
            self.access_counts.pop(oldest_key, None)

    def retrieve(self, key: str) -> Optional[Any]:
        """Retrieve data from working memory"""
        if key in self.items:
            self.access_counts[key] = self.access_counts.get(key, 0) + 1
            return self.items[key][0]
        return None

    def update(self, key: str, data: Any):
        """Update existing data in working memory"""
        if key in self.items:
            self.items[key] = (data, time.time())
            self.access_counts[key] = self.access_counts.get(key, 0) + 1

    def get_recent_items(self, time_window: float = 1.0) -> Dict[str, Any]:
        """Get items accessed within a time window"""
        current_time = time.time()
        recent = {}
        for key, (data, timestamp) in self.items.items():
            if current_time - timestamp <= time_window:
                recent[key] = data
        return recent

class LongTermMemory:
    """Long-term storage for learned knowledge and experiences"""
    def __init__(self):
        self.episodic_memory = []  # List of experiences
        self.semantic_memory = {}  # General knowledge
        self.procedural_memory = {}  # Learned procedures/skills

    def store_episode(self, episode_data: Dict):
        """Store an episodic memory"""
        episode_data['timestamp'] = time.time()
        self.episodic_memory.append(episode_data)

    def store_semantic(self, key: str, value: Any):
        """Store semantic knowledge"""
        self.semantic_memory[key] = value

    def store_procedure(self, name: str, procedure: Dict):
        """Store a learned procedure"""
        self.procedural_memory[name] = procedure

    def retrieve_episode(self, query: Dict) -> List[Dict]:
        """Retrieve relevant episodic memories"""
        # Simple matching based on query
        matches = []
        for episode in self.episodic_memory:
            match = True
            for key, value in query.items():
                if key not in episode or episode[key] != value:
                    match = False
                    break
            if match:
                matches.append(episode)
        return matches

    def retrieve_semantic(self, key: str) -> Optional[Any]:
        """Retrieve semantic knowledge"""
        return self.semantic_memory.get(key)

    def retrieve_procedure(self, name: str) -> Optional[Dict]:
        """Retrieve a learned procedure"""
        return self.procedural_memory.get(name)

class Behavior:
    """Base class for cognitive behaviors"""
    def __init__(self, name: str, priority: BehaviorPriority):
        self.name = name
        self.priority = priority
        self.active = False
        self.last_execution_time = 0.0

    def can_activate(self, sensory_input: SensoryInput, working_memory: WorkingMemory) -> bool:
        """Check if behavior can be activated"""
        raise NotImplementedError

    def execute(self, sensory_input: SensoryInput, working_memory: WorkingMemory) -> Optional[MotorCommand]:
        """Execute the behavior and return motor command"""
        raise NotImplementedError

    def interrupt(self):
        """Handle interruption of behavior"""
        self.active = False

class BalanceBehavior(Behavior):
    """Low-level balance maintenance behavior"""
    def __init__(self):
        super().__init__("balance", BehaviorPriority.BALANCE)
        self.com_position = np.array([0.0, 0.0, 0.8])
        self.com_velocity = np.array([0.0, 0.0, 0.0])

    def can_activate(self, sensory_input: SensoryInput, working_memory: WorkingMemory) -> bool:
        # Always available for balance
        return True

    def execute(self, sensory_input: SensoryInput, working_memory: WorkingMemory) -> Optional[MotorCommand]:
        # Simplified balance control
        if sensory_input.proprioceptive_data:
            # Calculate required adjustments to maintain balance
            joint_positions = sensory_input.proprioceptive_data.get('joint_positions', [0.0] * 12)
            joint_velocities = [0.0] * len(joint_positions)  # Simplified

            # Add small adjustments for balance
            com_offset = working_memory.retrieve('com_offset') or np.array([0.0, 0.0])
            if np.linalg.norm(com_offset) > 0.05:
                # Apply balance correction
                joint_positions[0] += com_offset[0] * 0.1  # Simplified
                joint_positions[1] += com_offset[1] * 0.1

            return MotorCommand(joint_positions, joint_velocities, priority=self.priority)
        return None

class GoalPlanner:
    """High-level goal planning and reasoning"""
    def __init__(self):
        self.current_goals: List[TaskGoal] = []
        self.goal_stack: List[TaskGoal] = []

    def add_goal(self, goal: TaskGoal):
        """Add a new goal to the planner"""
        self.current_goals.append(goal)
        self.current_goals.sort(key=lambda g: g.priority)

    def plan_to_goal(self, goal: TaskGoal, current_state: Dict) -> Optional[List[Dict]]:
        """Plan a sequence of actions to achieve a goal"""
        # Simplified planning algorithm
        # In practice, this would use more sophisticated planning like PDDL, STRIPS, or HTN
        if self.check_preconditions(goal, current_state):
            # Simple action sequence for demonstration
            plan = [
                {'action': 'move_to', 'target': goal.name + '_location'},
                {'action': 'perform_task', 'task': goal.name},
                {'action': 'verify_success', 'goal': goal.name}
            ]
            return plan
        return None

    def check_preconditions(self, goal: TaskGoal, state: Dict) -> bool:
        """Check if preconditions for a goal are met"""
        for key, value in goal.preconditions.items():
            if key not in state or state[key] != value:
                return False
        return True

    def update_goals(self, current_state: Dict):
        """Update goal list based on current state"""
        completed_goals = []
        for goal in self.current_goals:
            if self.check_goal_completion(goal, current_state):
                completed_goals.append(goal)

        for goal in completed_goals:
            self.current_goals.remove(goal)

    def check_goal_completion(self, goal: TaskGoal, state: Dict) -> bool:
        """Check if a goal has been completed"""
        for key, value in goal.effects.items():
            if key not in state or state[key] != value:
                return False
        return True

class CognitiveArchitecture:
    """Main cognitive architecture integrating all components"""
    def __init__(self, num_joints: int = 24):
        self.num_joints = num_joints
        self.working_memory = WorkingMemory()
        self.long_term_memory = LongTermMemory()
        self.goal_planner = GoalPlanner()

        # Initialize behaviors
        self.behaviors: Dict[str, Behavior] = {
            'balance': BalanceBehavior()
        }

        # Current cognitive state
        self.state = CognitiveState.REACTIVE
        self.current_plan = []
        self.current_step = 0

        # Learning components
        self.learning_enabled = True
        self.performance_history = []

    def process_sensory_input(self, sensory_data: Dict) -> SensoryInput:
        """Process raw sensory data into structured format"""
        input_struct = SensoryInput(timestamp=time.time())

        if 'vision' in sensory_data:
            input_struct.vision_data = sensory_data['vision']
        if 'tactile' in sensory_data:
            input_struct.tactile_data = sensory_data['tactile']
        if 'proprioceptive' in sensory_data:
            input_struct.proprioceptive_data = sensory_data['proprioceptive']
        if 'auditory' in sensory_data:
            input_struct.auditory_data = sensory_data['auditory']
        if 'force_torque' in sensory_data:
            input_struct.force_torque_data = sensory_data['force_torque']

        return input_struct

    def select_behavior(self, sensory_input: SensoryInput) -> Optional[Behavior]:
        """Select the most appropriate behavior based on current state"""
        available_behaviors = []

        for name, behavior in self.behaviors.items():
            if behavior.can_activate(sensory_input, self.working_memory):
                available_behaviors.append(behavior)

        if not available_behaviors:
            return None

        # Select behavior with highest priority
        return max(available_behaviors, key=lambda b: b.priority.value)

    def execute_cognitive_cycle(self, sensory_data: Dict) -> Optional[MotorCommand]:
        """Main cognitive cycle: perceive, reason, act"""
        # Process sensory input
        sensory_input = self.process_sensory_input(sensory_data)

        # Update working memory with current sensory state
        self.working_memory.store('sensory_input', sensory_input)
        self.working_memory.store('timestamp', sensory_input.timestamp)

        # Select and execute behavior
        selected_behavior = self.select_behavior(sensory_input)

        if selected_behavior:
            command = selected_behavior.execute(sensory_input, self.working_memory)

            if command:
                # Store execution result in memory
                self.working_memory.store('last_command', command)
                self.working_memory.store('execution_time', time.time())

                return command

        # If no behavior selected, return neutral position
        neutral_positions = [0.0] * self.num_joints
        return MotorCommand(neutral_positions, [0.0] * self.num_joints, priority=BehaviorPriority.IDLE)

    def update_cognitive_state(self):
        """Update the cognitive state based on current situation"""
        # Check if we're in a planning situation
        pending_goals = self.goal_planner.current_goals
        if pending_goals and not self.current_plan:
            self.state = CognitiveState.PLANNING
        elif self.current_plan:
            self.state = CognitiveState.EXECUTING
        else:
            self.state = CognitiveState.REACTIVE

    def learn_from_experience(self, experience: Dict):
        """Learn from experience and update cognitive structures"""
        if not self.learning_enabled:
            return

        # Store experience in episodic memory
        self.long_term_memory.store_episode(experience)

        # Update performance metrics
        performance = experience.get('performance', 0.0)
        self.performance_history.append(performance)

        # Adaptive learning based on experience
        if len(self.performance_history) > 10:
            recent_performance = np.mean(self.performance_history[-10:])
            if recent_performance < 0.6:  # Below threshold
                # Adjust behavior parameters or planning strategies
                self.working_memory.store('learning_signal', 'performance_degradation_detected')

    def add_behavior(self, behavior: Behavior):
        """Add a new behavior to the architecture"""
        self.behaviors[behavior.name] = behavior

    def set_goal(self, goal: TaskGoal):
        """Set a new goal for the cognitive system"""
        self.goal_planner.add_goal(goal)
        self.update_cognitive_state()

class ReactiveCognitiveSystem:
    """A reactive cognitive system for real-time operation"""
    def __init__(self, architecture: CognitiveArchitecture):
        self.architecture = architecture
        self.running = False
        self.sensory_buffer = []
        self.command_buffer = []
        self.lock = threading.Lock()

    def start(self):
        """Start the reactive cognitive system"""
        self.running = True
        self.cognitive_thread = threading.Thread(target=self._cognitive_loop)
        self.cognitive_thread.start()

    def stop(self):
        """Stop the reactive cognitive system"""
        self.running = False
        if hasattr(self, 'cognitive_thread'):
            self.cognitive_thread.join()

    def _cognitive_loop(self):
        """Main cognitive processing loop"""
        while self.running:
            with self.lock:
                if self.sensory_buffer:
                    # Process most recent sensory data
                    sensory_data = self.sensory_buffer[-1]
                    self.sensory_buffer.clear()

                    # Execute cognitive cycle
                    command = self.architecture.execute_cognitive_cycle(sensory_data)

                    if command:
                        self.command_buffer.append(command)

            time.sleep(0.01)  # 100Hz processing rate

    def process_sensory_data(self, sensory_data: Dict):
        """Add sensory data to the processing queue"""
        with self.lock:
            self.sensory_buffer.append(sensory_data)

    def get_command(self) -> Optional[MotorCommand]:
        """Get the latest command from the cognitive system"""
        with self.lock:
            if self.command_buffer:
                return self.command_buffer[-1]
            return None
```

:::tip
Implement a dual-processing architecture that combines fast, reactive behaviors for immediate responses with slower, deliberative reasoning for complex planning. This allows the system to respond quickly to urgent situations while still being able to plan for long-term goals.
:::

![Cognitive architecture diagram showing memory systems, behavior hierarchy, and reasoning components](./assets/cognitive-architectures.png)