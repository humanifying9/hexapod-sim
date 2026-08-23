# Hexapod Robot 🦾

An autonomous six-legged robot built from scratch — 3D printed chassis, custom PCB power distribution, and a full ROS2 software stack. Designed to walk, see, listen, and eventually think for itself.

---

## What It Does

- **Walks** using a tripod gait with 18 independently controlled servos across 6 legs
- **Sees** with a Pi Camera Module 3, streaming live video and running real-time object detection
- **Listens** to voice commands via a laptop microphone, processed with Whisper speech-to-text
- **Talks back** using Piper TTS over the laptop speaker
- **Responds to a controller** for manual teleoperation
- **Simulates** in Gazebo Harmonic before any code touches the real hardware

---

## Hardware

| Component | Role |
|-----------|------|
| Raspberry Pi 4 | Main brain — runs ROS2, perception, voice, navigation |
| ESP32 | Real-time servo controller — generates PWM for all 18 servos |
| Pi Camera Module 3 (IMX708) | 12MP camera, CSI ribbon, live MJPEG stream |
| 18× DFS 45 servos | 45 Nm torque, 270° range, 3 per leg |
| Laptop | Development, simulation, voice I/O, controller input |
| LiPo + BEC/PDB | Power management and distribution |
| Tailscale | Secure networking between laptop and Pi from anywhere |

### Leg Design

Each leg has 3 degrees of freedom:
- **Coxa** — horizontal rotation at the body mount (40° splay from centerline)
- **Femur** — main lifting joint
- **Tibia** — ground contact, curved foot profile for stable walking

The coxa mount has a built-in 40° yaw angle, which is accounted for in the inverse kinematics solver.

---

## Software Stack

```
Voice Input (Whisper STT)
        ↓
  LLM / Command Parser
        ↓
   ROS2 (Jazzy)
   ┌────────────────────────────────┐
   │  hexapod_audio    (voice I/O) │
   │  hexapod_perception (camera)  │
   │  hexapod_control  (gait + IK) │
   │  hexapod_teleop   (gamepad)   │
   │  hexapod_face     (display)   │
   └────────────────────────────────┘
        ↓ serial (USB/UART)
      ESP32
        ↓ PWM
   18 × DFS 45 Servos
```

---

## ROS2 Packages

### `hexapod_description`
URDF/xacro robot model with all 6 legs, joints, sensors, and Gazebo plugins. Built from a tested single-leg design and mirrored across the body with correct 40° splay angles.

### `hexapod_gazebo`
Gazebo Harmonic simulation world, `ros2_control` configuration for all 18 joints with effort controllers, and launch files for the full sim stack.

### `hexapod_control`
Core locomotion: tripod gait controller, forward and inverse kinematics accounting for the coxa tilt geometry, and velocity command subscriber.

### `hexapod_bringup`
Top-level launch files for both simulation (`hexapod_sim.launch.py`) and real robot deployment (`hexapod_real.launch.py`).

### `hexapod_perception`
Pi Camera node using Picamera2, MJPEG stream server, and OpenCV + MobileNet-SSD object detection publishing to `/hexapod/detections` as `vision_msgs/Detection2DArray`.

### `hexapod_audio`
Microphone capture via WSLg PulseAudio (laptop) or Pi audio, Whisper STT, command parsing, and Piper TTS for spoken responses. Publishes to `/hexapod/heard_text` and subscribes to `/hexapod/speak`.

### `hexapod_teleop`
Gamepad/controller input mapped to `/cmd_vel` twist messages for manual teleoperation.

### `hexapod_face`
Expressive LED or display output for the robot — reacts to what it hears and sees.

---

## Getting Started

### Prerequisites

- ROS2 Jazzy
- Gazebo Harmonic
- Python 3.12
- `catkin_pkg`, `colcon`, `ros-jazzy-gz-ros2-control`

### Build

```bash
git clone https://github.com/humanifying9/hexapod-sim.git
cd hexapod-sim
pip install catkin_pkg --break-system-packages
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Run the Simulation

```bash
ros2 launch hexapod_bringup hexapod_sim.launch.py
```

### Run on Real Hardware (from Pi via SSH/Tailscale)

```bash
ssh humanifying9@100.76.67.26
source /opt/ros/jazzy/setup.bash
ros2 launch hexapod_bringup hexapod_real.launch.py
```

### Camera Stream

The Pi streams live video over Tailscale. Open in any browser:
```
http://100.76.67.26:8080/
```

---

## Development Setup

- **Coding**: VS Code + Aider with DeepSeek V4-Pro via OpenRouter
- **Agent**: Hermes (Nous Research) connected via Discord — handles GitHub, email, task automation
- **Pi access**: Tailscale VPN + SSH (no static IP needed)
- **Simulation**: Gazebo Harmonic on laptop RTX 5070 GPU

---

## Roadmap

- [x] Single leg URDF and IK solver
- [x] Full 6-leg robot model in Gazebo
- [x] Camera stream over Tailscale
- [x] Object detection node (MobileNet-SSD)
- [ ] Tripod gait tuning in simulation
- [ ] ESP32 serial communication bridge
- [ ] Voice command pipeline (Whisper → command → action)
- [ ] Real robot first walk
- [ ] Autonomous navigation with object avoidance
- [ ] Voice personality and responses

---

## Project Structure

```
hexapod-sim/
├── hexapod_bringup/       # Launch files
├── hexapod_control/       # Gait + IK
├── hexapod_description/   # URDF/xacro model
├── hexapod_gazebo/        # Simulation world
├── hexapod_audio/         # Voice I/O
├── hexapod_face/          # Display/LED
├── hexapod_perception/    # Camera + detection
└── hexapod_teleop/        # Controller input
```

---

*Built by Tia Parekh*
