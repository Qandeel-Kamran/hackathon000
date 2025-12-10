---
title: Learning in Physical Environments
sidebar_position: 8
description: Reinforcement learning and adaptive algorithms for embodied systems
---

# Learning in Physical Environments

Learning in physical environments presents unique challenges and opportunities that distinguish it from traditional machine learning approaches. In embodied systems, learning occurs through direct interaction with the physical world, where the agent receives rich sensory feedback and must adapt its behavior based on the consequences of its actions. This interaction-driven learning paradigm leverages the physical properties of the system and environment to facilitate more efficient and robust learning outcomes.

Reinforcement learning for physical systems must account for the continuous, high-dimensional state and action spaces that characterize embodied agents. Unlike discrete environments where states can be easily enumerated, physical systems operate in continuous spaces where small changes in joint angles or positions can significantly affect outcomes. This requires specialized RL algorithms that can handle the complexity and real-time constraints of physical interaction.

Safety is paramount when implementing learning algorithms on physical robots. Unlike simulation environments, physical systems can cause damage to themselves or their surroundings if learning behaviors go wrong. Safe learning approaches include constrained optimization, learning from demonstration, and careful design of reward functions that prevent dangerous behaviors.

Transfer learning between simulation and reality addresses the "reality gap" problem, where policies learned in simulation often fail when transferred to real robots due to modeling inaccuracies. Domain randomization, system identification, and adaptive control techniques help bridge this gap by making policies more robust to modeling errors and environmental variations.

Sample efficiency is crucial in physical learning scenarios where each interaction has real costs in terms of time, energy, and potential wear on the system. Efficient learning algorithms maximize the information gained from each physical interaction, often by leveraging prior knowledge, using model-based approaches, or employing curriculum learning strategies.

```python
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import random
from collections import deque
import copy

class PhysicalEnvironment:
    """
    Simulated physical environment for learning experiments
    """
    def __init__(self):
        self.state = np.zeros(12)  # 6 joint angles + 6 joint velocities
        self.target = np.array([0.5, 0.3, 0.2, -0.2, -0.3, -0.5])  # Target joint angles
        self.max_steps = 1000
        self.step_count = 0
        self.dt = 0.01  # Time step

    def reset(self):
        """Reset the environment to initial state"""
        self.state = np.random.uniform(-0.1, 0.1, size=12)  # Small random initialization
        self.step_count = 0
        return self.state

    def step(self, action):
        """Execute action and return (next_state, reward, done, info)"""
        # Apply action (torque) to the system
        torques = action

        # Simplified physics simulation (double integrator model)
        joint_angles = self.state[:6]
        joint_velocities = self.state[6:]

        # Update velocities based on torques (simplified dynamics)
        joint_acc = torques * 0.1  # Simplified: acceleration proportional to torque
        new_velocities = joint_velocities + joint_acc * self.dt

        # Update positions
        new_angles = joint_angles + new_velocities * self.dt

        # Apply joint limits
        new_angles = np.clip(new_angles, -2.0, 2.0)
        new_velocities = np.clip(new_velocities, -5.0, 5.0)

        # Calculate reward (negative distance to target + penalty for large torques)
        distance_to_target = np.linalg.norm(new_angles - self.target)
        torque_penalty = 0.01 * np.sum(np.square(torques))
        velocity_penalty = 0.001 * np.sum(np.square(new_velocities))

        reward = -distance_to_target - torque_penalty - velocity_penalty

        # Check if done
        done = self.step_count >= self.max_steps or distance_to_target < 0.01
        self.step_count += 1

        # Update state
        self.state = np.concatenate([new_angles, new_velocities])

        return self.state, reward, done, {}

    def get_observation(self):
        """Get current observation (may include additional sensors)"""
        return self.state

class ActorNetwork(nn.Module):
    """Actor network for policy learning"""
    def __init__(self, state_dim, action_dim, max_action):
        super(ActorNetwork, self).__init__()

        self.l1 = nn.Linear(state_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, action_dim)

        self.max_action = max_action

    def forward(self, state):
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))
        return self.max_action * torch.tanh(self.l3(a))

class CriticNetwork(nn.Module):
    """Critic network for value estimation"""
    def __init__(self, state_dim, action_dim):
        super(CriticNetwork, self).__init__()

        # Q1 architecture
        self.l1 = nn.Linear(state_dim + action_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, 1)

        # Q2 architecture
        self.l4 = nn.Linear(state_dim + action_dim, 256)
        self.l5 = nn.Linear(256, 256)
        self.l6 = nn.Linear(256, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)

        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)

        q2 = F.relu(self.l4(sa))
        q2 = F.relu(self.l5(q2))
        q2 = self.l6(q2)
        return q1, q2

    def Q1(self, state, action):
        sa = torch.cat([state, action], 1)

        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
        return q1

class SafeDDPGAgent:
    """
    Safe Deep Deterministic Policy Gradient agent for physical systems
    """
    def __init__(self, state_dim, action_dim, max_action, lr=1e-4, gamma=0.99, tau=0.005):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.actor = ActorNetwork(state_dim, action_dim, max_action).to(self.device)
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=lr)

        self.critic = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=lr)

        self.gamma = gamma  # Discount factor
        self.tau = tau      # Soft update parameter
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_action = max_action

        # Experience replay buffer
        self.replay_buffer = deque(maxlen=100000)
        self.batch_size = 100

        # Noise for exploration (simplified Ornstein-Uhlenbeck process)
        self.noise_scale = 0.1
        self.noise_decay = 0.999

    def select_action(self, state, add_noise=True):
        """Select action using the current policy"""
        state = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        action = self.actor(state).cpu().data.numpy().flatten()

        if add_noise:
            noise = np.random.normal(0, self.noise_scale, size=self.action_dim)
            action = action + noise
            self.noise_scale *= self.noise_decay  # Decay exploration over time

        return np.clip(action, -self.max_action, self.max_action)

    def store_transition(self, state, action, reward, next_state, done):
        """Store transition in replay buffer"""
        self.replay_buffer.append((state, action, reward, next_state, done))

    def train(self):
        """Train the agent using a batch of experiences"""
        if len(self.replay_buffer) < self.batch_size:
            return

        # Sample batch
        batch = random.sample(self.replay_buffer, self.batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))

        # Convert to tensors
        state = torch.FloatTensor(state).to(self.device)
        action = torch.FloatTensor(action).to(self.device)
        reward = torch.FloatTensor(reward).to(self.device).unsqueeze(1)
        next_state = torch.FloatTensor(next_state).to(self.device)
        done = torch.BoolTensor(done).to(self.device).unsqueeze(1)

        # Compute target Q-value
        with torch.no_grad():
            next_action = self.actor_target(next_state)
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + (self.gamma * target_Q * ~done)

        # Get current Q-value estimates
        current_Q1, current_Q2 = self.critic(state, action)

        # Compute critic loss
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

        # Optimize critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)  # Gradient clipping for stability
        self.critic_optimizer.step()

        # Compute actor loss
        actor_loss = -self.critic.Q1(state, self.actor(state)).mean()

        # Optimize actor
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)  # Gradient clipping for stability
        self.actor_optimizer.step()

        # Soft update target networks
        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

    def save_model(self, filename):
        """Save the model parameters"""
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'actor_optimizer_state_dict': self.actor_optimizer.state_dict(),
            'critic_optimizer_state_dict': self.critic_optimizer.state_dict()
        }, filename)

    def load_model(self, filename):
        """Load the model parameters"""
        checkpoint = torch.load(filename)
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer_state_dict'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer_state_dict'])

class LearningCurriculum:
    """
    Curriculum learning for safe skill acquisition
    """
    def __init__(self):
        self.current_level = 0
        self.levels = [
            {"task": "basic_reaching", "difficulty": 0.1, "success_threshold": 0.8},
            {"task": "precise_positioning", "difficulty": 0.3, "success_threshold": 0.85},
            {"task": "object_manipulation", "difficulty": 0.5, "success_threshold": 0.75},
            {"task": "dynamic_control", "difficulty": 0.7, "success_threshold": 0.7},
            {"task": "complex_task", "difficulty": 1.0, "success_threshold": 0.6}
        ]

    def evaluate_performance(self, agent, env, num_episodes=10):
        """Evaluate agent performance on current curriculum level"""
        total_success = 0
        total_reward = 0

        for episode in range(num_episodes):
            state = env.reset()
            episode_reward = 0
            done = False

            while not done:
                action = agent.select_action(state, add_noise=False)  # No exploration during evaluation
                next_state, reward, done, _ = env.step(action)
                episode_reward += reward
                state = next_state

            # Determine success based on task completion
            if episode_reward > -50:  # Threshold for considering episode successful
                total_success += 1
            total_reward += episode_reward

        success_rate = total_success / num_episodes
        avg_reward = total_reward / num_episodes

        return success_rate, avg_reward

    def advance_curriculum(self, agent, env):
        """Advance to next curriculum level if current level is mastered"""
        if self.current_level >= len(self.levels) - 1:
            return False  # Already at final level

        success_rate, avg_reward = self.evaluate_performance(agent, env)
        current_level = self.levels[self.current_level]

        if success_rate >= current_level["success_threshold"]:
            self.current_level += 1
            print(f"Advancing to curriculum level: {self.levels[self.current_level]['task']}")
            return True

        return False
```

:::tip
When implementing learning algorithms on physical systems, start with simulation and gradually transfer to hardware using system identification to match simulation parameters to real-world dynamics. This approach significantly reduces the risk of damage during initial learning phases.
:::

![Learning in physical environments diagram showing reinforcement learning loop, safety constraints, and curriculum progression](./assets/physical-learning.png)