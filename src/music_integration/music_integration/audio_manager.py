#!/usr/bin/env python3

"""
Audio Manager for Music-Based Robot Control

This node handles multiple audio formats and provides audio processing capabilities
for music-based robot movement control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Twist
from conversational_ai_core.msg import AICommand
import numpy as np
import threading
import time
from collections import deque


class AudioManagerNode(Node):
    def __init__(self):
        super().__init__('audio_manager_node')

        # Publishers
        self.audio_status_pub = self.create_publisher(String, 'music/audio_status', 10)
        self.beat_info_pub = self.create_publisher(Float32MultiArray, 'music/beat_info', 10)
        self.rhythm_commands_pub = self.create_publisher(Twist, 'music/rhythm_commands', 10)

        # Subscribers
        self.ai_commands_sub = self.create_subscription(
            AICommand,
            'ai/commands',
            self.ai_command_callback,
            10
        )

        self.audio_input_sub = self.create_subscription(
            AudioData,
            'audio/input',
            self.audio_input_callback,
            10
        )

        # Internal state
        self.supported_formats = ['wav', 'mp3', 'ogg', 'flac', 'm4a']
        self.current_audio_file = None
        self.audio_buffer = deque(maxlen=100)  # Store recent audio data
        self.is_playing = False
        self.is_listening = False
        self.beat_threshold = 0.5
        self.tempo = 120.0  # BPM

        # Audio processing parameters
        self.sample_rate = 44100
        self.buffer_size = 1024
        self.fft_size = 1024
        self.overlap = 0.5

        # Timer for periodic audio processing
        self.audio_timer = self.create_timer(0.01, self.process_audio)

        self.get_logger().info('Audio Manager Node initialized')

    def ai_command_callback(self, msg):
        """Handle AI commands related to audio"""
        cmd_text = msg.command_text.lower()

        if 'music' in cmd_text or 'audio' in cmd_text or 'beat' in cmd_text:
            self.handle_music_command(msg)

    def handle_music_command(self, ai_cmd):
        """Handle music-related commands from AI"""
        cmd_text = ai_cmd.command_text.lower()

        if 'play' in cmd_text:
            self.play_audio_file(cmd_text)
        elif 'stop' in cmd_text:
            self.stop_audio()
        elif 'load' in cmd_text:
            self.load_audio_file(cmd_text)
        elif 'tempo' in cmd_text:
            self.adjust_tempo(cmd_text)
        elif 'beat' in cmd_text:
            self.analyze_beats()

    def load_audio_file(self, command):
        """Load an audio file (simulated)"""
        # Extract filename from command
        parts = command.split()
        if len(parts) > 1:
            filename = parts[-1]
            file_ext = filename.split('.')[-1].lower()

            if file_ext in self.supported_formats:
                self.current_audio_file = filename
                self.get_logger().info(f'Loaded audio file: {filename}')

                status_msg = String()
                status_msg.data = f"AUDIO_LOADED:{filename}"
                self.audio_status_pub.publish(status_msg)
            else:
                self.get_logger().warn(f'Unsupported audio format: {file_ext}')

                status_msg = String()
                status_msg.data = f"AUDIO_ERROR:Unsupported format {file_ext}"
                self.audio_status_pub.publish(status_msg)
        else:
            self.get_logger().warn('No filename specified in command')

    def play_audio_file(self, command):
        """Play an audio file (simulated)"""
        if self.current_audio_file:
            self.is_playing = True
            self.get_logger().info(f'Playing audio: {self.current_audio_file}')

            status_msg = String()
            status_msg.data = f"AUDIO_PLAYING:{self.current_audio_file}"
            self.audio_status_pub.publish(status_msg)
        else:
            self.get_logger().warn('No audio file loaded to play')

    def stop_audio(self):
        """Stop audio playback"""
        self.is_playing = False
        self.get_logger().info('Audio playback stopped')

        status_msg = String()
        status_msg.data = "AUDIO_STOPPED"
        self.audio_status_pub.publish(status_msg)

    def adjust_tempo(self, command):
        """Adjust playback tempo"""
        # Extract tempo value from command
        parts = command.split()
        for part in parts:
            try:
                tempo_val = float(part)
                if 20 <= tempo_val <= 300:  # Reasonable tempo range
                    self.tempo = tempo_val
                    self.get_logger().info(f'Adjusted tempo to: {tempo_val} BPM')

                    status_msg = String()
                    status_msg.data = f"TEMPO_ADJUSTED:{tempo_val}"
                    self.audio_status_pub.publish(status_msg)
                    return
            except ValueError:
                continue

        self.get_logger().warn(f'Could not extract tempo from command: {command}')

    def audio_input_callback(self, msg):
        """Handle incoming audio data"""
        # Store audio data in buffer
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_buffer.extend(audio_data)

        if not self.is_listening:
            self.is_listening = True
            self.get_logger().info('Started listening to audio input')

    def process_audio(self):
        """Process audio data for beat detection and rhythm analysis"""
        if not self.audio_buffer:
            return

        # Get a chunk of audio data
        chunk_size = min(self.buffer_size, len(self.audio_buffer))
        if chunk_size < self.buffer_size:
            return  # Not enough data yet

        # Extract audio chunk
        audio_chunk = np.array(list(self.audio_buffer)[-chunk_size:])

        # Perform FFT for frequency analysis
        fft_data = np.fft.rfft(audio_chunk, self.fft_size)
        magnitude_spectrum = np.abs(fft_data)

        # Simple beat detection based on energy changes
        energy = np.sum(audio_chunk ** 2)
        beat_detected = energy > self.beat_threshold

        if beat_detected:
            # Publish beat information
            beat_info = Float32MultiArray()
            beat_info.data = [energy, float(beat_detected), self.tempo]
            self.beat_info_pub.publish(beat_info)

            # Generate rhythm-based movement commands
            self.generate_rhythm_commands(energy)

    def analyze_beats(self):
        """Analyze beats in the current audio"""
        if self.current_audio_file:
            # Simulate beat analysis
            beat_info = Float32MultiArray()
            beat_info.data = [0.8, 1.0, self.tempo]  # [energy, beat_detected, tempo]
            self.beat_info_pub.publish(beat_info)

            self.get_logger().info(f'Analyzed beats for: {self.current_audio_file}')
        else:
            self.get_logger().warn('No audio file loaded for beat analysis')

    def generate_rhythm_commands(self, energy):
        """Generate robot movement commands based on rhythm"""
        # Create a Twist message for robot movement
        twist_cmd = Twist()

        # Map audio energy to movement intensity
        linear_speed = min(0.5, energy * 0.5)  # Scale energy to 0-0.5 m/s
        angular_speed = (energy - 0.5) * 0.8  # Use energy difference for turning

        twist_cmd.linear.x = linear_speed
        twist_cmd.angular.z = angular_speed

        # Publish the rhythm-based movement command
        self.rhythm_commands_pub.publish(twist_cmd)

        self.get_logger().debug(f'Generated rhythm command: linear={linear_speed:.2f}, angular={angular_speed:.2f}')


def main(args=None):
    rclpy.init(args=args)

    audio_manager = AudioManagerNode()

    try:
        rclpy.spin(audio_manager)
    except KeyboardInterrupt:
        pass
    finally:
        audio_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()