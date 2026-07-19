# M.A.R.S — Multi Agent Robot System for Warehouse Automation

Modular multi-robot warehouse automation system using sensor fusion (encoders + overhead camera/ArUco) for localization, with ROS 2 based central coordination and ESP32-based robot firmware.

## Overview

- Differential-drive robots with ESP32 microcontrollers, encoder-based odometry, and servo grippers
- Overhead camera + ArUco markers for global pose estimation of robots, loads, and dropzones
- ROS 2 central controller handles task allocation, path planning, collision avoidance
- Robot ↔ central controller communication over UDP (JSON commands)
- Supports single/multi-robot operation, priority-based and thread-based task execution, single/double dropzone configurations

## System Architecture

```
Overhead Camera → aruco_detector (OpenCV) → /bot_poses, /loads, /dropzone
                                                      ↓
                                              task_manager (ROS 2)
                                        (task allocation, collision avoidance, navigation)
                                                      ↓
                                          UDP (JSON) ↔ ESP32 robots
```

## Hardware

- ESP32-WROOM / ESP32-S3
- N20 DC motors with encoders
- L293D dual H-bridge motor driver
- Servo gripper
- Li-Po battery
- Custom double-sided PCB (KiCad design, in-house etched)
- 3D printed chassis (Fusion 360 + Bambu Lab printer)

## Software Stack

- **Firmware**: C++ (Arduino/ESP32), non-blocking UDP + Telnet handlers, PID for straight-line motion
- **Central Controller**: ROS 2 (Python/C++), OpenCV for ArUco detection
- **Communication**: ROS 2 topics (internal), UDP/WiFi (robot control)

## Repository Structure

```
/firmware        # ESP32 robot firmware (motor control, encoder PID, UDP command handling)
/ros2_ws          # ROS 2 packages: camera_node, aruco_detector, task_manager
/pcb              # KiCad PCB design files
/cad              # Fusion 360 chassis design files
/docs             # Report, diagrams, results
```

## Firmware Command Protocol

Robots listen on a dedicated UDP port for JSON commands:

```json
{"id": 1, "cmd": "FORWARD", "dist": 200}
{"id": 1, "cmd": "TURN_L", "angle": 90}
{"id": 1, "cmd": "GRAB"}
```

Supported commands: `FORWARD`, `BACKWARD`, `TURN_L`, `TURN_R`, `GRAB`, `RELEASE`, `STOP`
Each command receives an `ack` on receipt and a `done` reply on completion. Odometry is broadcast at 10 Hz.

## Setup

### Firmware
1. Open `/firmware` in Arduino IDE / PlatformIO
2. Set `BOT_ID`, WiFi credentials, and target UDP port per robot
3. Flash to ESP32

### Central Controller
1. Requires ROS 2 (tested on Humble/Iron), OpenCV, `opencv-contrib` (ArUco module)
2. Build workspace:
   ```
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```
3. Launch nodes:
   ```
   ros2 launch mars_bringup mars.launch.py
   ```

## Results Summary

| Configuration | Avg. time (2 loads) |
|---|---|
| Single robot | 38.14 s |
| Two robots (priority-based) | 30.44 s |
| Two robots (thread-based) | 15.45 s |

Thread-based multi-robot execution with double dropzone gave the best throughput; single dropzone increases collision risk and coordination overhead.

## Team

- Amrit Kumar Banjade
- Laxmi Prasad Upadhyaya
- Siddartha Gupta
- Sumit Sigdel

Department of Electronics and Computer Engineering, Pashchimanchal Campus, IOE, Tribhuvan University
Supervisor: Asst. Prof. Er. Hom Nath Tiwari

## License

See institutional copyright notice in `/docs/report.pdf`. For reuse permissions, contact the Department of Electronics and Computer Engineering, IOE, Pashchimanchal Campus.
