

# TwinRover – Autonomous Mobile Robot Platform

## Overview

TwinRover is a senior design autonomous mobile robot built using ROS2.  
The system integrates perception, SLAM, navigation, and low-level motor control on embedded hardware.

The goal of this project is to develop a fully autonomous ground robot capable of:

- Mapping unknown environments
- Performing real-time path planning
- Avoiding obstacles
- Detecting objects using computer vision
- Running on embedded hardware (Raspberry Pi / Jetson)

---

## System Architecture

**Onboard Computer**
- Raspberry Pi 5 or Jetson Orin Nano

**Low-Level Controller**
- Arduino (motor control + encoder feedback)

**Framework**
- ROS2 (Humble)

**Navigation**
- Nav2

**SLAM**
- SLAM Toolbox or Cartographer (TBD)

**Perception**
- YOLOv8

**Sensors**
- LiDAR (SICK TiM561)
- IMU (BNO085)
- RGB Camera

---

## Initial Setup

### 1. Install Git (if needed)

Ubuntu:

```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```
### 2. Set up SSH
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
cat ~/.ssh/id_ed25519.pub
```
Go to:

GitHub → Settings → SSH and GPG Keys → New SSH Key

Paste and save.

### 3. Clone Repo
```bash
git clone git@github.com:sampra04/TwinRover.git
cd TwinRover
```





