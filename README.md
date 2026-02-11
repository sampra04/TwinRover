# TwinRover
## Overview

This project is a senior design autonomous mobile robot built using ROS2.  
The system integrates perception, SLAM, navigation, and low-level motor control on an embedded platform.

The goal of this project is to develop a fully autonomous ground robot capable of:

- Mapping unknown environments
- Performing real-time path planning
- Avoiding obstacles
- Detecting objects using computer vision
- Operating on embedded hardware (Raspberry Pi / Jetson)

---

## System Architecture

High-Level Stack:

- **Onboard Computer:** Raspberry Pi 5 / Jetson Orin Nano  
- **Low-Level Controller:** Arduino (motor control + encoders)  
- **Framework:** ROS2 (Humble)  
- **Navigation:** Nav2  
- **SLAM:** TBD (SLAM Toolbox / Cartographer)  
- **Perception:** YOLOv8  
- **Sensors:**
  - LiDAR (SICK TiM561)
  - IMU (BNO085)
  - RGB Camera

---

## Repository Structure

