#!/usr/bin/env python3

"""
Setup script for Chatboard Interface
"""

import subprocess
import sys
import os

def install_dependencies():
    """Install required Python packages"""
    print("Installing required dependencies...")

    # Required packages
    packages = [
        'flask',
        'websockets'
    ]

    for package in packages:
        try:
            print(f"Installing {package}...")
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', package])
            print(f"Successfully installed {package}")
        except subprocess.CalledProcessError:
            print(f"Failed to install {package}")
            return False

    # Optional ROS 2 package
    try:
        print("Installing rclpy (ROS 2 Python client library)...")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'rclpy'])
        print("Successfully installed rclpy")
    except subprocess.CalledProcessError:
        print("rclpy not installed (ROS 2 not available) - this is OK for web interface only")

    print("Dependencies installed successfully!")
    return True

def main():
    """Main setup function"""
    print("Setting up Chatboard Interface...")

    # Create necessary directories
    if not os.path.exists('templates'):
        os.makedirs('templates')
        print("Created templates directory")

    if not os.path.exists('static'):
        os.makedirs('static')
        print("Created static directory")

    # Install dependencies
    if install_dependencies():
        print("\nSetup completed successfully!")
        print("To start the chatboard, run: python chat_interface.py")
    else:
        print("\nSetup failed. Please install dependencies manually.")
        return 1

    return 0

if __name__ == '__main__':
    sys.exit(main())