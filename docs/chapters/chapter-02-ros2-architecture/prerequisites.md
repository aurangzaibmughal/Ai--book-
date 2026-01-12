---
id: prerequisites
title: Prerequisites
sidebar_position: 3
---

# Prerequisites

Before starting this chapter, ensure you have:

## Required Knowledge

### From Previous Chapters
- **Chapter 1: Introduction to Physical AI** ✅
  - Understanding of Physical AI concepts
  - Familiarity with embodied intelligence
  - Basic robotics terminology

### Programming
- **Python 3.x**: Intermediate level
  - Object-oriented programming
  - Async/await concepts (helpful)
  - Exception handling
- **C++** (Optional but recommended)
  - Basic C++11/14 syntax
  - Classes and inheritance
  - Smart pointers

### System Knowledge
- **Linux/Ubuntu**: Basic command-line proficiency
  - File system navigation
  - Package management (apt)
  - Environment variables
- **Networking**: Basic concepts
  - TCP/IP fundamentals
  - Localhost vs remote connections
  - Ports and protocols

## Required Software

### Operating System
- **Ubuntu 22.04 LTS** (Jammy Jellyfish) - Recommended
- **Ubuntu 20.04 LTS** (Focal Fossa) - Supported
- **Windows 11 with WSL2** - Alternative
- **macOS** - Limited support (Docker recommended)

### ROS 2 Installation
- **ROS 2 Humble Hawksbill** (LTS) - Recommended
- **ROS 2 Iron Irwini** - Latest
- **ROS 2 Galactic Geochelone** - Older LTS

### Development Tools
- **Text Editor/IDE**: VS Code, PyCharm, or CLion
- **Git**: Version control
- **Python 3.10+**: Installed with ROS 2
- **GCC/G++**: C++ compiler (for C++ examples)

## Hardware Requirements

### Minimum
- **CPU**: Dual-core processor
- **RAM**: 4GB (8GB recommended)
- **Storage**: 20GB free space
- **Network**: Internet connection for package downloads

### Recommended
- **CPU**: Quad-core processor
- **RAM**: 8GB or more
- **Storage**: 50GB free space (for multiple ROS 2 distributions)
- **Network**: Stable broadband connection

## Pre-Chapter Setup

### 1. Verify ROS 2 Installation

```bash
# Check ROS 2 version
ros2 --version

# Expected output: ros2 cli version 0.x.x
```

### 2. Source ROS 2 Environment

```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify environment
echo $ROS_DISTRO
# Expected output: humble
```

### 3. Install Additional Tools

```bash
# Install colcon build tool
sudo apt install python3-colcon-common-extensions

# Install ROS 2 development tools
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-xacro

# Verify installation
colcon version-check
```

### 4. Create Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build empty workspace
colcon build

# Source workspace
source install/setup.bash
```

## Optional but Helpful

### Distributed Systems Knowledge
- Understanding of client-server architecture
- Familiarity with message queues
- Basic knowledge of middleware concepts

### Robotics Background
- Kinematics and dynamics (basic)
- Sensor types and characteristics
- Control theory fundamentals

### Tools Experience
- Docker/containers
- CMake build system
- Unit testing frameworks

## Troubleshooting Common Issues

### ROS 2 Not Found
```bash
# If ros2 command not found
source /opt/ros/humble/setup.bash

# Make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Permission Denied
```bash
# If permission issues with packages
sudo chown -R $USER:$USER ~/ros2_ws
```

### Network Issues
```bash
# If DDS discovery fails
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
```

## Verification Checklist

Before proceeding, verify:
- ✅ ROS 2 Humble (or Iron) installed
- ✅ `ros2` command works
- ✅ Environment sourced (`$ROS_DISTRO` set)
- ✅ Workspace created and built
- ✅ Python 3.10+ available
- ✅ Text editor/IDE configured

## Need Help?

If you're missing prerequisites:
- **ROS 2 Installation**: Follow [official installation guide](https://docs.ros.org/en/humble/Installation.html)
- **Ubuntu Setup**: Use [Ubuntu installation tutorial](https://ubuntu.com/tutorials/install-ubuntu-desktop)
- **WSL2 Setup**: Follow [Microsoft WSL2 guide](https://docs.microsoft.com/en-us/windows/wsl/install)

---

**Ready?** Once all prerequisites are met, proceed to [Estimated Time](./estimated-time) to plan your learning session.
