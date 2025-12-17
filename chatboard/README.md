# Conversational AI Chatboard

A web-based chat interface for the Conversational AI system designed for humanoid robotics and Physical AI education.

## Overview

This chatboard provides a user-friendly web interface that connects to the conversational AI system for humanoid robotics. It allows users to interact with the AI through a browser-based chat interface, supporting educational conversations about robotics concepts.

## Features

- Web-based chat interface accessible through any browser
- Integration with conversational AI for humanoid robotics education
- Educational content about robotics, kinematics, dynamics, control systems, and humanoid robotics
- Support for music-based movement commands (when connected to full system)
- Responsive design for various screen sizes

## Prerequisites

- Python 3.8 or higher
- ROS 2 (Humble Hawksbill or later) - optional, for full functionality
- pip package manager

## Installation

1. Clone or navigate to the chatboard directory
2. Install dependencies:
   ```bash
   python setup.py
   ```

This will install all required Python packages and create necessary directories.

## Usage

1. Start the chatboard server:
   ```bash
   python chat_interface.py
   ```

2. Open your web browser and navigate to `http://localhost:5000`

3. You can now interact with the conversational AI through the web interface

## How It Works

The chatboard operates in two modes:

### Web-Only Mode (Default)
- Runs when ROS 2 is not installed or available
- Provides educational responses based on predefined rules
- Simulates interaction with the conversational AI system

### Full Integration Mode
- Activated when ROS 2 is properly installed and configured
- Connects directly to the conversational AI nodes
- Provides full access to robot control and educational content

## API Endpoints

- `GET /` - Main chat interface
- `POST /send_message` - Send a message to the AI
- `GET /get_history` - Retrieve chat history

## Project Structure

```
chatboard/
├── chat_interface.py     # Main chatboard application
├── setup.py             # Installation script
├── package.json         # Package information
├── templates/           # HTML templates
│   └── index.html       # Main chat interface template
├── static/              # Static assets (empty, available for expansion)
└── README.md            # This file
```

## Integration with ROS 2

When running with ROS 2 available, the chatboard connects to:

- `ai/commands` topic for sending user commands
- `ai/responses` topic for receiving AI responses
- `get_lesson_content` service for educational content

## Safety and Educational Focus

This system is designed for educational purposes only:

- All robot interactions are simulated in a safe environment
- No real hardware control is possible through this interface
- Educational content focuses on robotics fundamentals

## Troubleshooting

- If you get import errors, make sure you've run `python setup.py` to install dependencies
- If the server won't start, check that port 5000 is available
- For ROS 2 integration issues, ensure your ROS 2 environment is properly sourced

## Next Steps

To integrate with the full conversational AI system:

1. Ensure ROS 2 is installed and properly configured
2. Build and run the conversational AI nodes
3. Start the chatboard - it will automatically detect and connect to ROS 2 if available

## License

This project is part of the Conversational AI for Humanoid Robotics educational system.