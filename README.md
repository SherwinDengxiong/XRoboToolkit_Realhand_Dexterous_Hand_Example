# XRoboToolkit RealHand Dexterous Hand Example

Pico XR teleoperation demo, in Python, for the **RealHand L6** dexterous hand
in Placo visualization and on real hardware. A few additional arm-teleop demos
(UR5e, ARX R5, Galaxea R1 Lite) are kept from the parent project for reference.

## Overview

This project drives the RealHand L6 hand from XR (VR/AR) controller and hand
tracking input. Hand tracking is retargeted to the L6 joint space with
[`dex_retargeting`](https://github.com/dexsuite/dex-retargeting); commands are
sent to the hardware through the new
[`realhand`](https://github.com/RealHand-Robotics/realbot-python-sdk) Python SDK
(CAN-bus based).

## Installation
1. Download and install [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service). Run the installed program before running the following demo.

2.  **Clone the repository:**
    ```bash
    git clone <this-repo-url>
    cd XRoboToolkit_Realhand_Dexterous_Hand_Example
    ```

3.  **Installation**
    **Note:** The setup scripts are currently tested on Ubuntu 22.04 / 24.04.
    It is recommended to set up a Conda environment and install using the included script.
    ```bash
    bash setup_conda.sh --conda <optional_env_name>
    conda activate <env_name>
    bash setup_conda.sh --install
    ```

    If installing on system python:
    ```bash
    bash setup.sh
    ```

    The install scripts automatically:
    - Clone and build [`XRoboToolkit-PC-Service-Pybind`](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind) into `dependencies/`.
    - `pip install git+https://github.com/SherwinDengxiong/dex_retargeting_local.git`
      — the RealHand fork of `dex_retargeting` that adds the `real` robot entry
      and the RealHand YAML configs required here. The PyPI release does
      **not** have these entries, so do not replace this with
      `pip install dex_retargeting`.
    - `pip install git+https://github.com/RealHand-Robotics/realbot-python-sdk.git`
      — the new RealHand Python SDK.
    - `pip install -e .` the teleop package itself.

## Usage
Use the following instructions to run example scripts. For a more detailed description, please refer to [`teleop_details.md`](teleop_details.md).

### Running the MuJoCo Simulation Demo

To run the teleoperation demo with a UR5e robot in MuJoCo simulation:

```bash
python scripts/simulation/teleop_dual_ur5e_mujoco.py
```
This script initializes the [`MujocoTeleopController`](xrobotoolkit_teleop/simulation/mujoco_teleop_controller.py) with the UR5e model and starts the teleoperation loop.

### Running the Placo Visualization Demo

To run the teleoperation demo with a UR5e robot in Placo visualization:

```bash
python scripts/simulation/teleop_x7s_placo.py
```
This script initializes the [`PlacoTeleopController`](xrobotoolkit_teleop/simulation/placo_teleop_controller.py) with the X7S robot and starts the teleoperation loop.

### Running Dexterous Hand Teleop Simulation
- Shadow hand simulation in Mujoco
    ```bash
    python scripts/simulation/teleop_shadow_hand_mujoco.py
    ```

- **RealHand L6 in Placo visualization** (the main demo for this project)
    ```bash
    python scripts/simulation/teleop_realhand_l6_placo.py --hand-type right
    ```
    This loads the L6 URDF from `assets/real_hand/l6/`, builds a dex_retargeting
    optimizer for the `real` robot, and drives the L6 in a browser-based Placo
    viewer from live XR hand tracking.

- **RealHand L6 in MuJoCo visualization**
    ```bash
    python scripts/simulation/teleop_realhand_l6_mujoco.py --hand-type right
    ```
    This loads the same RealHand L6 URDF directly in MuJoCo and drives the
    simulated hand from live XR hand tracking.

- **Dual RealHand L6 in Placo visualization**
    ```bash
    python scripts/simulation/teleop_dual_realhand_l6_placo.py
    ```

- **Dual RealHand L6 in MuJoCo visualization**
    ```bash
    python scripts/simulation/teleop_dual_realhand_l6_mujoco.py
    ```

### Running RealHand L6 Hardware Demo

To drive a physical RealHand L6 via the new `realhand` SDK:

```bash
# Bring up the CAN interface (one-time setup before each run):
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Teleoperate the right hand on can0:
python scripts/hardware/teleop_realhand_l6_hardware.py \
    --hand-type right --interface-name can0

# Send an "open hand and exit" command (useful as a safe reset):
python scripts/hardware/teleop_realhand_l6_hardware.py --reset
```
This script initializes [`RealHandL6Controller`](xrobotoolkit_teleop/hardware/realhand_l6_controller.py),
which uses the new realhand SDK's `L6(side=..., interface_name=...)` interface
and sends `angle.set_angles(...)` commands on the 0-100 scale.

### Running the Hardware Demo (Dual UR5 Arms and Dynamixel-based Head)

To run the teleoperation demo with the physical dual UR arms and Dynamixel-based head:

1.  **Normal Operation:**
    ```bash
    python scripts/hardware/teleop_dual_ur5e_hardware.py
    ```
    This script initializes the [`DynamixelHeadController`](xrobotoolkit_teleop/hardware/dynamixel.py) and [`DualArmURController`](xrobotoolkit_teleop/hardware/ur.py) and starts the teleoperation loops for both head tracking and arm control.

2.  **Resetting Arm Positions:**
    If you need to reset the UR arms to their initial/home positions and initialize the robotiq grippers, you can run the script with the `--reset` flag:
    ```bash
    python scripts/hardware/teleop_dual_ur5e_hardware.py --reset
    ```
    This will execute the reset procedure defined in the [`DualArmURController`](xrobotoolkit_teleop/hardware/ur.py) and then exit.

3.  **Visualizing IK results:**
    To visualize the inverse kinematics solution with placo during teleoperation, run the script with the `--visualize_placo` flag.
    ```bash
    python scripts/hardware/teleop_dual_ur5e_hardware.py --visualize_placo
    ```

### Running ARX R5 Hardware Demo

To run the teleoperation demo with dual ARX R5 robotic arms:

```bash
python scripts/hardware/teleop_dual_arx_r5_hardware.py
```

This script initializes the [`ARXR5TeleopController`](xrobotoolkit_teleop/hardware/arx_r5_teleop_controller.py) for dual arm control with built-in grippers.

### Running Galaxea R1 Lite Humanoid Demo

To run the teleoperation demo with the Galaxea R1 Lite humanoid robot:

```bash
python scripts/hardware/teleop_r1lite_hardware.py
```

This script initializes the [`GalaxeaR1LiteTeleopController`](xrobotoolkit_teleop/hardware/galaxea_r1_lite_teleop_controller.py) for mobile manipulator control, the controller communicates with the robot hardware via ROS.

## Data Collection

### Collecting Teleoperation Data

The framework automatically logs teleoperation sessions when running hardware demos. Data collection includes:

- **Robot joint states** and end effector poses
- **Camera streams** from multiple viewpoints
- **User input data** from XR controllers
- **Timestamp synchronization** across all data streams

#### Starting Data Collection

1. **Run any hardware teleoperation script:**
   ```bash
   python scripts/hardware/teleop_dual_arx_r5_hardware.py
   ```

2. **Press B button** on the VR controller to start/stop logging
   - First press: Start data logging
   - Second press: Stop logging and save data to disk

3. **Emergency stop:** Press right joystick click to discard current session

#### Data Storage

Collected data is saved as `.pkl` files in the `logs/` directory with timestamps:
```
logs/
├── <robot_name>/
│   └── teleop_log_YYYYMMDD_HHMMSS_<session_id>.pkl
└── <another_robot>/
    ├── teleop_log_YYYYMMDD_HHMMSS_<session_id>.pkl
    └── teleop_log_YYYYMMDD_HHMMSS_<session_id>.pkl
```

### Validating Collected Data

Use the provided analysis script to verify data integrity and examine collected datasets:

```bash
python scripts/misc/test_data_log_analysis.py logs/<robot_name>/teleop_log_YYYYMMDD_HHMMSS_1.pkl
```

This script will:
- Display available data fields and their types
- Verify robot states and camera images are properly saved
- Show sample entries and data statistics
- Count total logged entries

### Converting to LeRobot Dataset

For training imitation learning models, convert collected data to [LeRobot](https://github.com/huggingface/lerobot) format using this example conversion script:

**Example:** [ARX Dual Arm Data Converter](https://github.com/zhigenzhao/openpi/blob/dev/finetuning/examples/arx_r5/arx_dual/convert_dual_arm_data_to_lerobot.py)

This conversion enables:
- Standardized dataset format for machine learning
- Integration with LeRobot training pipelines  
- Support for various imitation learning algorithms
- Easy data sharing and reproducibility

## Teleoperation Guide

### Tracking Modes

The teleoperation system supports multiple tracking modes for controlling robot end effectors:

#### 1. Controller Tracking (Default)
- **Description**: Uses VR/AR controller poses to control robot end effectors
- **Use Case**: Primary method for precise manipulation tasks
- **Configuration**: Set `pose_source` to `"left_controller"` or `"right_controller"`
- **Tracking**: Full 6DOF pose (position + orientation) or 3DOF position-only

#### 2. Hand Tracking
- **Description**: Uses hand pose estimation from XR cameras
- **Use Case**: Natural hand gesture control

#### 3. Head Tracking
- **Description**: Uses headset pose for controlling specific robot components
- **Use Case**: Head/neck control for humanoid robots or camera orientation

#### 4. Motion Tracker Tracking
- **Description**: Uses additional motion tracking devices for controlling auxiliary robot links
- **Use Case**: Multi-point control (e.g., elbow position while controlling end effector)
- **Configuration**: Add `motion_tracker` config with device serial and target link
- **Note**: Not recommended for 6DOF arms like UR5e; better suited for redundant arms

### Controller Button Functions

When using VR controllers for teleoperation, the following button mappings apply:

#### **Grip Buttons**
- **Left Grip** (`left_grip`): Activates left arm teleoperation
- **Right Grip** (`right_grip`): Activates right arm teleoperation
- **Function**: Hold to enable arm control, release to deactivate

#### **Trigger Buttons**
- **Left Trigger** (`left_trigger`): Controls left gripper/hand
- **Right Trigger** (`right_trigger`): Controls right gripper/hand
- **Function**: Analog control (0.0 = fully open, 1.0 = fully closed)

#### **System Buttons**
- **A Button**: Reserved for system functions
- **B Button**: Toggle data logging on/off
  - Press once: Start logging
  - Press again: Stop logging and save data

#### **Joysticks/Touchpads**
- **Left Joystick**: Linear velocity commands for mobile robots
- **Right Joystick**: Angular velocity commands for mobile robots
- **Right Axis Click**: stop data logging (discards current data)


## Dependencies
XR Robotics dependencies:
- [`xrobotoolkit_sdk`](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind): Python binding for XRoboToolkit PC Service SDK, MIT License

Robotics Simulation and Solver
- [`mujoco`](https://github.com/google-deepmind/mujoco): robotics simulation, Apache 2.0 License
- [`placo`](https://github.com/rhoban/placo): inverse kinematics, MIT License
- [`dex_retargeting`](https://github.com/dexsuite/dex-retargeting): XR hand → dexterous-hand retargeting. Vendored under `dependencies/dex_retargeting_local/` with custom `real` robot entries and RealHand L6/L20 configs.

Hardware Control
- [`realhand`](https://github.com/RealHand-Robotics/realbot-python-sdk): Python SDK for the RealHand L6 / L20 / L25 / O6 hands and A7 / A7 Lite arms. Installed from git by `setup.sh` / `setup_conda.sh`.
- [`dynamixel_sdk`](https://github.com/ROBOTIS-GIT/DynamixelSDK.git): Dynamixel control functions, Apache-2.0 License
- [`ur_rtde`](https://gitlab.com/sdurobotics/ur_rtde): interface for controlling and receiving data from a UR robot, MIT License
- [`ARX R5 SDK`](https://github.com/zhigenzhao/R5/tree/dev/python_pkg): Interface for controlling ARX R5 robotic arms

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
