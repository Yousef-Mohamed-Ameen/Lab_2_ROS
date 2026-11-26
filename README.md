# Robotic Arm – URDF, Gazebo Simulation and Hardware Control (ROS 2 Jazzy)

One command on `/arm_controller/joint_trajectory` moves the arm in **Gazebo (gz sim)**, in **RViz2** and on **real servos**.

```
                      /arm_controller/joint_trajectory
 slider_gui /  ───────────────┬────────────────────────────┐
 trajectory_publisher         │                            │
                              ▼                            ▼
              joint_trajectory_controller         hardware_bridge (Python)
              (gz_ros2_control in gz sim)                  │ USB serial, "J,a1,a2,a3,a4\n"
                      │                                    ▼
          /joint_states + TF                       Arduino firmware ──PWM──► servos
                      │
                      ▼
                    RViz2
```

## Repository layout
```
src/arm_description/   URDF (xacro), RViz config, controller YAML, display launch
src/arm_gazebo/        gz sim world + sim.launch.py (spawn, ros2_control, RViz2)
src/arm_control/       trajectory_publisher, slider_gui, hardware_bridge (+ launch/config)
firmware/arm_servo/    Arduino sketch
```

## Model (Figure 1 dimensions)
| Link | Size | Joint (parent → child) | Axis | Limits |
|---|---|---|---|---|
| `base_link` | 8 cm high box (10×10 cm footprint) | – (fixed to ground) | – | – |
| `L1` | 12 cm cylinder | `joint1` base_link → L1 | Z (base yaw) | ±90° |
| `L2` | 15 cm cylinder | `joint2` L1 → L2 | Y (pitch) | ±90° |
| `L3` | 9 cm cylinder | `joint3` L2 → L3 | Y (pitch) | ±90° |
| `end_effector` | 6 cm cylinder | `joint4` L3 → end_effector | Y (pitch) | ±90° |

At q = 0 the arm points straight up. Every link has `<visual>`, `<collision>` and `<inertial>`.
Joint limits of ±90° map to the 0–180° range of a hobby servo (0 rad = 90°).
With `sim:=true` the xacro adds a `world` anchor frame + fixed joint (Gazebo needs it to pin the base; it is not a physical link), the `ros2_control` block and the gz plugin.

## Dependencies (Ubuntu 24.04, ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui python3-serial python3-tk
```
Arduino IDE (or `arduino-cli`) with the built-in **Servo** library. Serial access: `sudo usermod -aG dialout $USER` (re-login).

## Build
```bash
git clone <your-repo-url> robotic_arm_ros2 && cd robotic_arm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run
**1. Check the URDF only (RViz2 + sliders):**
`ros2 launch arm_description display.launch.py`

**2. Simulation (Gazebo + RViz2):**
`ros2 launch arm_gazebo sim.launch.py`

**3. Publish trajectories** (new terminal, `source install/setup.bash`):
```bash
ros2 run arm_control trajectory_publisher     # looping demo
ros2 run arm_control slider_gui               # manual sliders
# or a single command:
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
 "{joint_names: ['joint1','joint2'], points: [{positions: [1.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}"
```

**4. Hardware:** upload `firmware/arm_servo/arm_servo.ino`, wire the servos, then
```bash
ros2 launch arm_control hardware.launch.py port:=/dev/ttyUSB0
# no board at hand? ->  ros2 launch arm_control hardware.launch.py dry_run:=true
```

**Full pipeline demo:** terminal 1 `sim.launch.py`, terminal 2 `hardware.launch.py`, terminal 3 `slider_gui`.
Moving a slider moves Gazebo, RViz2 and the servos.

## Hardware communication
* **Transport:** USB serial (pyserial), 115200 baud, 8N1.
* **Message format:** ASCII line `J,<a1>,<a2>,<a3>,<a4>\n`; angles in servo degrees with one decimal, e.g. `J,90.0,120.5,60.0,90.0`. The firmware also accepts fewer values (only the first N servos are updated).
* **Conversion (bridge):** `deg = servo_center_deg + servo_direction * degrees(rad)`, clamped to `[servo_min_deg, servo_max_deg]`. Per-servo parameters are in `src/arm_control/config/hardware_bridge.yaml` (use `servo_direction: -1.0` to mirror a servo).
* **Timing:** the bridge linearly interpolates between `time_from_start` points at 50 Hz, so the servos follow the same timing as the simulation. Joints missing in a message keep their last value.
* **Firmware:** parses the line, clamps to 0–180°, drives the servo with `writeMicroseconds(500…2500)`.

## Wiring (Arduino Uno/Nano)
| Board pin | Connects to |
|---|---|
| D3 | Servo 1 signal (joint1 – base rotation) |
| D5 | Servo 2 signal (joint2 – shoulder) |
| D6 | Servo 3 signal (joint3 – elbow, optional) |
| D9 | Servo 4 signal (joint4 – wrist, optional) |
| GND | Servo GND **and** external supply GND (common ground) |
| – | Servo V+ (red) → external 5–6 V supply (**not** the board's 5 V pin) |

Only 2 servos are required; unconnected channels are harmless.

## Troubleshooting
* Arm falls/does not move in Gazebo: check `ros2 control list_controllers` shows `arm_controller` and `joint_state_broadcaster` as *active*.
* RViz2 shows nothing: Fixed Frame must be `base_link`; `use_sim_time` is set by the launch file.
* Servo jitters: use a stronger external supply; add a 470 µF capacitor across the servo supply.
* Wrong direction on a servo: set its entry in `servo_direction` to `-1.0`.
