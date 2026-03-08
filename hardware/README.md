# SO-100 / SO-101 Hardware Bringup

Real-hardware support for the **leader-follower 2-DOF SO-100 arm** inside the
Modelfree repository.

This folder (`hardware/`) is a self-contained ROS2 Python package called
`so100_hardware_bringup`.  It adds real-hardware execution **without changing
any existing model-free or IOC code**.

---

## Folder structure

```
hardware/
├── nodes/                  ← ROS2 node scripts + Feetech serial driver
│   ├── feetech_driver.py   ←   low-level Feetech STS3215 serial protocol
│   ├── leader_hw_node.py   ←   leader arm  → /so100/joint_states
│   ├── follower_hw_node.py ←   /so101/arm_position_controller/commands → follower arm
│   ├── relay_node.py       ←   /so100/joint_states → /so101/.../commands
│   └── csv_logger_node.py  ←   records both arms to CSV
├── launch/
│   ├── leader_hw.launch.py   ← launch only leader
│   ├── follower_hw.launch.py ← launch only follower
│   └── dual_hw.launch.py     ← launch everything (main entry point)
├── scripts/
│   ├── setup_serial.sh       ← grant serial port permissions
│   └── check_connections.sh  ← verify devices are detected
├── config/
│   └── hardware_params.yaml  ← all tunable parameters
├── data/                     ← CSV log files saved here (one per run)
│   └── .gitkeep
├── package.xml
├── setup.py
└── README.md  (this file)
```

---

## Hardware wiring

| Arm      | Role     | Serial port (stable by-id path)                                    |
|----------|----------|---------------------------------------------------------------------|
| SO-100   | Leader   | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF218344-if00`     |
| SO-101   | Follower | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF219983-if00`     |

### Servo ID assignment (2-DOF subset of the full 5-DOF arm)

| Servo | Joint           | Default ID |
|-------|-----------------|------------|
| J1    | Shoulder_Pitch  | 2          |
| J2    | Elbow           | 3          |

Change these in `config/hardware_params.yaml` or as launch arguments.

---

## Prerequisites

### 1. ROS2 Jazzy (or Humble)

```bash
source /opt/ros/jazzy/setup.bash      # or humble
```

### 2. pyserial

```bash
pip install pyserial
# or
sudo apt install python3-serial
```

### 3. Serial port permissions

Run once after plugging in the USB adapters:

```bash
bash hardware/scripts/setup_serial.sh
```

Or add a permanent udev rule:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666"' | \
     sudo tee /etc/udev/rules.d/99-so100-serial.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. Build the package

Link or copy the `hardware/` folder into your ROS2 workspace `src/` directory,
then build:

```bash
# From your workspace root (e.g. ~/so100_ws)
ln -s ~/path/to/Modelfree/hardware  src/so100_hardware_bringup
colcon build --packages-select so100_hardware_bringup
source install/setup.bash
```

---

## Verify connections before running

```bash
bash hardware/scripts/check_connections.sh
```

Expected output:
```
[✓] Leader  arm found:  /dev/serial/by-id/...5AAF218344-if00  →  /dev/ttyUSB0
[✓] Follower arm found: /dev/serial/by-id/...5AAF219983-if00  →  /dev/ttyUSB1
```

---

## Commands to run

### ▶ Run only the leader arm

```bash
ros2 launch so100_hardware_bringup leader_hw.launch.py
```

This starts the **leader_hw_node** which:
- Connects to the leader arm over serial
- Reads Shoulder_Pitch and Elbow positions at 50 Hz
- Publishes to `/so100/joint_states`

### ▶ Run only the follower arm

```bash
ros2 launch so100_hardware_bringup follower_hw.launch.py
```

This starts the **follower_hw_node** which:
- Connects to the follower arm over serial
- Subscribes to `/so101/arm_position_controller/commands`
- Sends position commands to the servos
- Publishes `/so101/joint_states`

### ▶ Run the full leader-follower system (recommended)

```bash
ros2 launch so100_hardware_bringup dual_hw.launch.py
```

This starts **all four nodes** together:
1. `leader_hw_node`  — reads leader servos
2. `follower_hw_node` — drives follower servos
3. `relay_node`       — mirrors leader joints to follower commands
4. `csv_logger_node`  — records everything to CSV

---

## Where the CSV file will be saved

```
hardware/data/run_YYYYMMDD_HHMMSS.csv
```

For example:
```
hardware/data/run_20260308_103045.csv
```

### CSV columns

| Column          | Description                            |
|-----------------|----------------------------------------|
| `time`          | Elapsed time in seconds (float)        |
| `leader_joint1` | Leader Shoulder_Pitch angle (rad)      |
| `leader_joint2` | Leader Elbow angle (rad)               |
| `follower_joint1` | Follower Shoulder_Pitch angle (rad)  |
| `follower_joint2` | Follower Elbow angle (rad)           |

### Override the data directory

```bash
ros2 launch so100_hardware_bringup dual_hw.launch.py \
    data_dir:=/home/user/Modelfree/hardware/data
```

Or set the environment variable:

```bash
export HARDWARE_DATA_DIR=/home/user/Modelfree/hardware/data
ros2 launch so100_hardware_bringup dual_hw.launch.py
```

---

## Launch arguments reference

All `dual_hw.launch.py` arguments with defaults:

| Argument          | Default                                           | Description                        |
|-------------------|---------------------------------------------------|------------------------------------|
| `leader_port`     | `/dev/serial/by-id/...5AAF218344-if00`            | Leader serial device               |
| `follower_port`   | `/dev/serial/by-id/...5AAF219983-if00`            | Follower serial device             |
| `baud_rate`       | `1000000`                                         | Servo baud rate                    |
| `rate_hz`         | `50.0`                                            | Control / publish rate             |
| `goal_speed`      | `200`                                             | Servo speed limit (0 = max)        |
| `leader_servo_j1` | `2`                                               | Leader servo ID for Shoulder_Pitch |
| `leader_servo_j2` | `3`                                               | Leader servo ID for Elbow          |
| `follower_servo_j1`| `2`                                              | Follower servo ID for Shoulder_Pitch|
| `follower_servo_j2`| `3`                                              | Follower servo ID for Elbow        |
| `scale`           | `1.0`                                             | Relay angle scale factor           |
| `data_dir`        | `hardware/data/`                                  | CSV output directory               |

---

## ROS2 topic overview

```
/so100/joint_states                     ← published by leader_hw_node
       (sensor_msgs/JointState)

/so101/arm_position_controller/commands ← subscribed by follower_hw_node
       (std_msgs/Float64MultiArray)

/so101/joint_states                     ← published by follower_hw_node
       (sensor_msgs/JointState)
```

The relay node connects:
```
/so100/joint_states  →  relay_node  →  /so101/arm_position_controller/commands
```

---

## Connecting to the model-free pipeline

After `dual_hw.launch.py` is running, all existing model-free nodes will
work with the real hardware because they subscribe/publish the **same topics**
as the Gazebo simulation:

```bash
# Example: run the IOC leader log on real hardware (no changes needed)
ros2 run so100_ioc_pipeline leader_log --ros-args \
    -p ns:=/so100 -p duration_s:=30.0
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Failed to open serial port` | Port path wrong or permissions | Run `setup_serial.sh` and `check_connections.sh` |
| `Servo ID X did NOT respond to ping` | Wrong servo ID, wrong baud rate, loose cable | Check `servo_id_j1/j2` params and wiring |
| Follower jerks or overshoots | `goal_speed` too high | Reduce `goal_speed` (e.g. 100) |
| No CSV file created | `data_dir` does not exist | Run `mkdir -p hardware/data` |
| `ModuleNotFoundError: pyserial` | Missing dependency | `pip install pyserial` |
