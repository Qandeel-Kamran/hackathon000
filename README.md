# Conversational AI for Humanoid Robotics and Physical AI Education

A conversational AI system designed to facilitate education in humanoid robotics and Physical AI, integrating with ROS 2, Gazebo simulation, Isaac Sim, and textbook-based learning content. The system provides an interactive interface for students and researchers to learn about and experiment with humanoid robot control, perception, and behavior.

## Project Overview

This educational platform combines:
- ROS 2 architecture for robot communication
- Gazebo simulation for robot modeling
- Isaac Sim for perception pipeline
- Textbook-based learning content
- Music-based interactive examples for teaching robot movements

## Setup and Installation

### Prerequisites
- ROS 2 Humble Hawksbill or later
- Gazebo simulation environment
- Isaac Sim (optional)
- Python 3.8 or higher
- Appropriate humanoid robot models for simulation

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd conversational-ai-humanoid-robotics
   ```

2. Install dependencies:
   ```bash
   # Install ROS 2 dependencies
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the System

1. Launch the main system:
   ```bash
   ros2 launch conversational_ai_core ai_system.launch.py
   ```

2. Interact with the conversational AI through the command line interface.

### Educational Scenarios

The system includes various educational scenarios that demonstrate robotics concepts through simulation and interactive examples.

## Architecture

The system is organized into several ROS 2 packages:

- `conversational_ai_core`: Main conversational interface and NLP functionality
- `robot_interfaces`: ROS 2 interfaces for different robot models
- `simulation_interfaces`: Gazebo and Isaac Sim integration
- `educational_content`: Textbook content management
- `music_integration`: Music-based movement system
- `utils`: Common utilities and helper functions

## Contributing

Please read the contributing guidelines for more information on how to contribute to this project.

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.