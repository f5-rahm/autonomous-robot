# autonomous-robot
Side project building out a physical representation of connected robotic infrastructure

# Teleop Controller

A comprehensive teleoperation system for MakersPet autonomous robots supporting keyboard, gamepad, and programmatic command control through ROS2.

## Overview

This project provides multiple ways to control your MakersPet robot:

- **Keyboard Control**: Real-time WASD-style movement with incremental speed adjustments
- **Gamepad Control**: F710 Logitech gamepad support with analog stick control and button mapping
- **Command Script**: Precise programmatic movements with duration and distance specifications

## Supported Robots

This teleop system is designed for **MakersPet robot platforms** including:

- **Loki** (200mm round base)
- **Snoopy** (300mm round base) 
- **Fido** (250mm round base)

All robots must be running the [Kaia.ai software platform](https://kaia.ai) with ROS2 support and be equipped with:
- Arduino ESP32 microcontroller
- Compatible LiDAR sensor (YDLIDAR X4/X3/X3-PRO/X2/X2L, Neato XV11, LDS02RR, or RPLIDAR A1)
- Proper motor control hardware

## Prerequisites

- Docker and Docker Compose
- Linux system (tested on Ubuntu 20.04/22.04)
- MakersPet robot on the same network
- Optional: Logitech F710 gamepad for gamepad control

## Installation & Setup

### 1. Clone the Repository
```bash
git clone https://github.com/your-username/autonomous-robot.git
cd autonomous-robot
```

### 2. Gamepad Setup (Optional)
If using a Logitech F710 gamepad:

```bash
# Connect F710 gamepad via USB receiver
# Verify device detection
ls -la /dev/input/js*

# Should show: /dev/input/js0
# Grant permissions (if needed)
sudo chmod 666 /dev/input/js0
```

**Important**: Set your F710 to **D-mode** (not X-mode) using the switch on the back.

### 3. Configure Robot Connection
Edit the `compose.yaml` file if needed to match your network setup:

```yaml
environment:
  - ROS_DOMAIN_ID=0  # Match your robot's ROS domain ID
```

### 4. Start the System
```bash
# Launch both containers
docker-compose up -d

# Verify containers are running
docker-compose ps
```

## Usage

### Method 1: Real-time Keyboard Control

Access the interactive keyboard teleop interface:

```bash
# Enter the makerspet container
docker exec -it makerspet bash

# Launch the ros system
ros2 launch kaiaai_bringup physical.launch.py

# Start keyboard teleop
ros2 run kaiaai_teleop teleop_keyboard
```

**Keyboard Controls:**
- `w` - Move forward (incremental)
- `x` - Move backward (incremental)
- `a` - Turn left (incremental)
- `d` - Turn right (incremental)
- `s` - Stop rotation
- `SPACE` - Emergency stop (all movement)
- `CTRL+C` - Quit

**Capital letters** (W/X/A/D) provide larger increments for faster adjustments.

### Method 2: Gamepad Control

With F710 gamepad connected and containers running, the gamepad is automatically detected:

**Gamepad Controls:**
- **Left Analog Stick**: 
  - Forward/Backward: Y-axis controls linear movement
  - Left/Right: X-axis controls turning
- **D-Pad**: Discrete movement steps (same as WASD keys)
- **Face Buttons**:
  - Y button: Move forward
  - A button: Move backward  
  - X button: Turn left
  - B button: Turn right
- **Triggers**:
  - Left Trigger: 90° right turn
  - Right Trigger: 90° left turn
- **Right Bumper**: Emergency stop

### Method 3: Programmatic Commands

Execute precise movement commands using the command script:

```bash
# Enter the controller container
docker exec -it ros_controller bash

# Source ROS2 environment
source /opt/ros/iron/setup.bash

# Execute movement commands
python3 robot_commands.py move forward 1.5 50    # Forward 1.5 seconds at 50% speed
python3 robot_commands.py move backward 0.8 25   # Backward 0.8 seconds at 25% speed  
python3 robot_commands.py turn left 90           # Turn left 90 degrees
python3 robot_commands.py turn right 45          # Turn right 45 degrees
python3 robot_commands.py stop                   # Emergency stop
```

**Command Parameters:**
- **Movement**: `direction` (forward/backward), `duration` (max 2.0 seconds), `speed` (25/50/100%)
- **Turning**: `direction` (left/right), `degrees` (0-360)

## Configuration

### Robot Parameters
The system uses these default velocity limits (defined in `teleop_keyboard.yaml`):

```yaml
max_lin_vel: 0.22      # m/s - Maximum linear velocity
max_ang_vel: 12.84     # rad/s - Maximum angular velocity
lin_vel_step: 0.05     # Linear velocity increment
ang_vel_step: 0.1      # Angular velocity increment
```

### Network Ports
The system exposes these ports:

- `8888/udp` - Robot communication
- `4430/tcp` - Control interface
- `5901/tcp` - VNC access (optional)
- `8080/tcp` - Web interface

## Troubleshooting

### Gamepad Issues

**Gamepad not detected:**
```bash
# Check if device exists
ls -la /dev/input/js*

# Check permissions
sudo chmod 666 /dev/input/js0

# Verify gamepad is in D-mode (not X-mode)
```

**No gamepad response:**
- Ensure F710 is set to D-mode using the back switch
- Check battery levels
- Verify USB receiver is connected
- Restart containers: `docker-compose restart`

### Connection Issues

**Robot not responding:**
```bash
# Check ROS2 topic communication
docker exec -it makerspet bash
ros2 topic list
ros2 topic echo /cmd_vel

# Verify robot is on same ROS_DOMAIN_ID
```

**Container connection problems:**
```bash
# Check container status
docker-compose ps

# View logs
docker-compose logs makerspet
docker-compose logs ros_controller

# Restart system
docker-compose down && docker-compose up -d
```

### Command Script Issues

**Commands ignored:**
- Ensure robot is powered on and connected
- Check ROS2 communication: `ros2 topic list`
- Verify odometry topic: `ros2 topic echo /odom`
- Check for error messages in script output

## Safety Notes

- **Maximum duration**: Movement commands are limited to 2.0 seconds for safety
- **Emergency stop**: Always available via SPACE key, right bumper, or `stop` command
- **Speed limits**: System enforces velocity limits defined in configuration
- **Turn protection**: Overlapping turn commands are prevented to avoid erratic behavior

## Hardware Requirements

### Gamepad Compatibility
- **Recommended**: Logitech F710 Wireless Gamepad
- **Mode**: Must be set to D-mode (DirectInput)
- **Connection**: USB receiver required
- **Other gamepads**: May work but button mappings might differ

### System Requirements
- Linux host system (Ubuntu 20.04+ recommended)
- Docker Engine 20.10+
- Docker Compose 2.0+
- At least 2GB RAM for containers
- Network connection to MakersPet robot

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test with actual MakersPet hardware
4. Submit a pull request

## License

Licensed under the Apache License, Version 2.0. See the individual source files for details.

## Support

- **MakersPet Community**: [Facebook Group](https://www.facebook.com/groups/243730868651472/)
- **Technical Support**: [Support Forum](https://github.com/makerspet/support/discussions/)
- **Documentation**: [MakersPet Website](https://makerspet.com)
- **Kaia.ai Platform**: [Kaia.ai Documentation](https://kaia.ai)

---

**Note**: This project requires a physical MakersPet robot running the Kaia.ai software platform. The teleop system communicates via ROS2 topics and will not function without proper robot hardware and software setup.
