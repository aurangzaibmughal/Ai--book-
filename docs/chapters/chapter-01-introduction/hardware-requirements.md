---
id: hardware-requirements
title: Hardware Requirements
sidebar_position: 11
---

# Hardware Requirements

This chapter can be completed using different hardware tiers based on your budget and learning goals. All core concepts can be learned using the **Minimum Tier** (cloud-based).

## Tier Comparison

| Feature | Minimum | Recommended | Premium |
|---------|---------|-------------|---------|
| **Cost** | $0-5/month | $1,500-2,500 | $5,000-15,000 |
| **Setup Time** | 15 minutes | 2-4 hours | 1-2 days |
| **Portability** | High (cloud) | Medium | Low |
| **Real Hardware** | No | Partial | Yes |
| **Learning Completeness** | 100% | 100% | 100% + Physical |

---

## Minimum Tier (Cloud-Based) ☁️

**Perfect for**: Learning concepts, running simulations, completing all labs

### Requirements
- **Computer**: Any modern laptop/desktop
  - OS: Windows 10+, macOS 11+, or Linux
  - RAM: 8GB minimum
  - Storage: 10GB free space
  - Internet: 10 Mbps+ stable connection

- **Cloud Instance**: AWS, GCP, or Azure
  - Instance Type: t3.medium (AWS) or equivalent
  - vCPUs: 2
  - RAM: 4GB
  - Storage: 30GB SSD
  - **Estimated Cost**: $0.04/hour (~$30/month if running 24/7, or $5/month for 5 hours/week)

### Setup Instructions

#### AWS EC2 Setup
```bash
# 1. Launch EC2 instance
Instance Type: t3.medium
AMI: Ubuntu 22.04 LTS
Storage: 30GB gp3
Security Group: Allow SSH (port 22)

# 2. Connect via SSH
ssh -i your-key.pem ubuntu@your-instance-ip

# 3. Install dependencies
sudo apt update
sudo apt install -y python3-pip python3-venv
pip3 install numpy matplotlib
```

#### Google Cloud Setup
```bash
# 1. Create Compute Engine instance
Machine Type: e2-medium
Boot Disk: Ubuntu 22.04 LTS, 30GB
Firewall: Allow HTTP/HTTPS

# 2. Connect via SSH (browser-based)
# Click "SSH" button in console

# 3. Install dependencies
sudo apt update
sudo apt install -y python3-pip
pip3 install numpy matplotlib
```

### Advantages
- ✅ No hardware investment
- ✅ Quick setup
- ✅ Access from anywhere
- ✅ Easy to reset/restart
- ✅ Pay only for what you use

### Limitations
- ❌ No physical robot interaction
- ❌ Requires internet connection
- ❌ Ongoing costs (though minimal)

---

## Recommended Tier (Local Development + Edge Device) 💻

**Perfect for**: Serious learners, developers, those wanting local control

### Requirements

#### Development Computer
- **CPU**: Intel i5/i7 or AMD Ryzen 5/7 (8+ cores recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA RTX 4060 Ti / 4070 / 4070 Ti
  - VRAM: 8GB minimum, 12GB+ recommended
  - CUDA Compute Capability: 7.5+
- **Storage**: 256GB SSD minimum, 512GB+ recommended
- **OS**: Ubuntu 22.04 LTS (native or dual-boot)

#### Edge Computing Device
- **NVIDIA Jetson Orin Nano** (8GB)
  - CPU: 6-core ARM Cortex-A78AE
  - GPU: 1024-core NVIDIA Ampere
  - RAM: 8GB
  - Storage: 64GB eMMC + microSD
  - **Price**: ~$499

**OR**

- **NVIDIA Jetson Orin NX** (16GB)
  - CPU: 8-core ARM Cortex-A78AE
  - GPU: 1024-core NVIDIA Ampere
  - RAM: 16GB
  - Storage: 64GB eMMC
  - **Price**: ~$699

### Total Investment
- Development PC: $1,000-1,800
- Jetson Device: $500-700
- Accessories: $100-200
- **Total**: $1,600-2,700

### Setup Instructions

#### Ubuntu 22.04 Installation
```bash
# 1. Download Ubuntu 22.04 LTS
# https://ubuntu.com/download/desktop

# 2. Create bootable USB
# Use Rufus (Windows) or Etcher (macOS/Linux)

# 3. Install Ubuntu
# Follow installation wizard
# Choose "Install alongside Windows" for dual-boot

# 4. Post-installation setup
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git python3-pip
```

#### NVIDIA Driver Installation
```bash
# Install NVIDIA drivers
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify installation
nvidia-smi
```

#### Jetson Orin Setup
```bash
# 1. Flash JetPack 5.1+ using NVIDIA SDK Manager
# https://developer.nvidia.com/sdk-manager

# 2. Initial setup on Jetson
sudo apt update
sudo apt install -y python3-pip
pip3 install jetson-stats

# 3. Verify installation
jtop  # Monitor Jetson resources
```

### Advantages
- ✅ No ongoing cloud costs
- ✅ Faster development cycle
- ✅ Offline capability
- ✅ Real edge computing experience
- ✅ GPU acceleration for simulations

### Limitations
- ❌ Higher upfront cost
- ❌ Requires technical setup
- ❌ Still no physical robot

---

## Premium Tier (Full Physical Robotics) 🤖

**Perfect for**: Advanced learners, researchers, those building real robots

### Requirements

#### All Recommended Tier Components PLUS:

#### High-End Development Workstation
- **CPU**: Intel i9 or AMD Ryzen 9 (16+ cores)
- **RAM**: 64GB
- **GPU**: NVIDIA RTX 4090
  - VRAM: 24GB
  - CUDA Compute Capability: 8.9
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

#### Physical Robot Platform
Choose one based on your focus:

**For Manipulation:**
- **Universal Robots UR5e** (~$35,000)
- **Franka Emika Panda** (~$25,000)
- **Kinova Gen3** (~$40,000)

**For Humanoid Robotics:**
- **Unitree G1** (~$16,000)
  - Height: 127cm
  - Weight: 35kg
  - DOF: 23-43 (depending on configuration)
  - Payload: 2kg per arm
- **Unitree H1** (~$90,000)
  - Height: 180cm
  - Weight: 47kg
  - DOF: 25
  - Walking speed: 3.3 m/s

**For Mobile Robotics:**
- **TurtleBot 4** (~$1,500)
- **Clearpath Jackal** (~$20,000)

#### Additional Hardware
- **Cameras**: Intel RealSense D435i (~$300)
- **LiDAR**: RPLiDAR A1 (~$100) or Velodyne VLP-16 (~$4,000)
- **Force/Torque Sensors**: ATI Mini40 (~$3,000)
- **Grippers**: Robotiq 2F-85 (~$10,000)

### Total Investment
- Workstation: $3,000-5,000
- Jetson Orin NX: $700
- Robot Platform: $1,500-90,000
- Sensors & Accessories: $500-10,000
- **Total**: $5,700-105,700

### Setup Requirements
- Dedicated workspace (10+ square meters)
- Safety equipment (emergency stops, barriers)
- Power supply (standard 110V/220V)
- Network infrastructure
- Insurance (for high-value robots)

### Advantages
- ✅ Complete Physical AI experience
- ✅ Real-world testing
- ✅ Hardware-software integration
- ✅ Research-grade capabilities
- ✅ Portfolio-building projects

### Limitations
- ❌ Very high cost
- ❌ Requires dedicated space
- ❌ Maintenance and repairs
- ❌ Safety considerations
- ❌ Steep learning curve

---

## Choosing Your Tier

### Start with Minimum Tier if:
- You're new to robotics
- You want to learn concepts first
- Budget is limited
- You don't have dedicated workspace

### Upgrade to Recommended Tier if:
- You're serious about robotics development
- You want faster iteration cycles
- You need offline capability
- You have budget for initial investment

### Consider Premium Tier if:
- You're building real robotic systems
- You're doing research
- You need physical validation
- You have significant budget and space

---

## Chapter 1 Specific Requirements

For this chapter specifically, you only need:

### Minimum
- ✅ Cloud instance (t3.medium or equivalent)
- ✅ Python 3.11+
- ✅ 2GB RAM available
- ✅ Internet connection

### Recommended
- ✅ Local Ubuntu 22.04 machine
- ✅ 8GB RAM
- ✅ Python 3.11+

### Premium
- Same as Recommended (physical hardware not needed for Chapter 1)

---

## Cost Optimization Tips

1. **Use Free Tiers**: AWS, GCP, and Azure offer free tiers for new users
2. **Stop Instances**: Stop cloud instances when not in use
3. **Spot Instances**: Use spot/preemptible instances for 60-90% savings
4. **Student Discounts**: Many cloud providers offer student credits
5. **Open Source**: Use open-source tools and simulators
6. **Community Resources**: Join robotics communities for shared resources

---

**Next**: Review the [Resources](./resources.md) section for additional learning materials and community support.
