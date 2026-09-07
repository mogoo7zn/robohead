"""core: ROS-independent robot software framework for RoboGame 2026.

Runs on Mac (pytest, mock mission) and on the Raspberry Pi (via ROS 2
adapters in ros2_ws/). No module here imports rclpy or touches hardware
directly; all hardware access goes through interfaces with Mock/STM32
backends selected by configuration.
"""
__version__ = "0.1.0"
