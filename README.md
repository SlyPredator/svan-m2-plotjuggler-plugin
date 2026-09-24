# Svan M2 PlotJuggler Plugin Suite

[![Build & Test](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Platform](https://img.shields.io/badge/platform-Linux%20x86__64-blue.svg)]()
[![Middleware](https://img.shields.io/badge/middleware-CycloneDDS-orange.svg)]()
[![Qt](https://img.shields.io/badge/Qt-5.15+-green.svg)]()

High-performance real-time telemetry streaming, derived signal analytics, and interactive 3D kinematic visualization for the **Svan M2 Quadruped Robot** in **PlotJuggler 3**.

---

## Overview

This suite provides direct native CycloneDDS ingestion and 3D visualization inside PlotJuggler without requiring a running ROS 2 daemon or Python runtime. It deserializes and unpacks full robot telemetry at 500+ Hz, pairs actuator commands with state feedback to compute live physics metrics (closed-loop PD torque, tracking errors, mechanical power), and renders an interactive 3D URDF/STL model synchronized to the live data stream or recorded rosbags.

```
                  +----------------------------------------------+
                  |               Svan M2 Hardware               |
                  |  SensorData (500Hz) / JointCommand (200Hz)   |
                  +-----------------------+----------------------+
                                          | CycloneDDS (Native)
                                          v
+---------------------------------------------------------------------------------+
|                               PlotJuggler 3                                     |
|                                                                                 |
|  +---------------------------+           +-----------------------------------+  |
|  | Svan M2 DDS DataStreamer  |           |     Svan M2 3D Robot View         |  |
|  | (libplotjuggler_m2.so)    |           | (libplotjuggler_m2_robot_view.so) |  |
|  | - Direct IDL unpack       |           | - URDF + 17 Binary STL meshes     |  |
|  | - Canonical vs. Enhanced  |           | - Forward kinematics solver       |  |
|  | - PD Torques, Power, Err  |           | - IMU roll/pitch/yaw tracking     |  |
|  | - 1-Drag Multi-Curve tags |           | - Detachable PiP floating window  |  |
|  +-------------+-------------+           +-----------------+-----------------+  |
|                |                                           |                    |
|                +-------------------> PlotDataMap <---------+                    |
+---------------------------------------------------------------------------------+
```

---

## Key Features

### 1. Direct CycloneDDS DataStreamer (`libplotjuggler_m2.so`)
* **Zero-ROS-Daemon Ingestion**: Subscribes directly to native CycloneDDS topics over UDP/multicast with zero serialization overhead.
* **Full Svan M2 Topic Coverage**:
  * `SensorData` (500 Hz): Joint positions `q`, velocities `dq`, accelerations `ddq`, estimated torques `tau_est`, current `q_current`, motor/FET temperatures, bus voltages, power, and 6-DOF IMU attitude (`quat`, `gyro`, `accel`, `rpy`).
  * `JointData` (200 Hz): Command positions `q`, feedforward velocity `dq`, gains `kp`/`kd`, and feedforward torque `tau_ff`.
  * `JoyData` (20 Hz): Teleoperation axes (lateral, longitudinal, yaw, body/step height) and state machine button transitions.
  * Controller Telemetry: `QuadLog` (WBC, estimation, reference, ground truth), `SolverStats` (QP/NLP solver iterations & residuals), `Point3D` (`base_err`), `FloatScalar` (`mpc_time`), and `PowerData` (battery diagnostics).
* **Two Operating Modes**:
  * **Canonical Mode (Default)**: Clean 1:1 IDL schema (`sensor_data/q/0..11`, `driver_voltage/0..11`, etc.) with zero clutter.
  * **Enhanced Mode (`--enhanced`)**: Enables real-time math engines and structured flattened aliases:
    * **Closed-Loop Actuator PD Torques**
    * **Instantaneous Mechanical Power**
    * **Joint Tracking Errors**
    * **Leg Aliases**: `legs/FR/hip/q`, `legs/FL/thigh/q`, `legs/RR/calf/q`, etc.
    * **Metric-First Aliases**: `joints*/q/00..11` allowing 1-drag plotting of all 12 joints simultaneously onto a single plot window.

### 2. Interactive 3D Robot View Toolbox (`libplotjuggler_m2_robot_view.so`)
* **Accurate URDF & STL Meshes**: Automatically parses `m2_metal_description.urdf` and loads the robot.
* **Synchronized Kinematics & Attitude**: Live forward kinematics driven by incoming `q[12]` plus onboard IMU attitude (quaternion or Euler RPY) to orient the robot in 3D space.
* **Picture-in-Picture (PiP) Window**: Detach the 3D visualizer into a dedicated floating window (`⧉ Pop Out (PiP)` or <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>) so you can view plots and 3D kinematics side-by-side.
* **Global Visibility Toggle**: Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>R</kbd> anywhere in PlotJuggler to instantly toggle the 3D visualizer on or off.
* **Playback Scrubbing & Timeline**: Interactive timeline slider with scrubbing through recorded datasets.
* **Smart Buffer Management**: Automatically detects when bag playback is active and unlocks PlotJuggler's retention buffer to hold the entire dataset without truncation.

### 3. Standalone High-Speed MCAP Bag Player (`m2_bag_player`)
* Replays MCAP rosbags into CycloneDDS at configurable speeds (`--rate`), loop mode (`--loop`), or max speed (`--rate 0`).
* Runs independently without needing ROS 2 installed or sourced.

---

## Directory Layout

```text
m2-pj/
├── CMakeLists.txt                         # Root build configuration
├── bundle/plotjuggler_m2/                 # Self-contained deployment bundle
│   ├── libplotjuggler_m2.so               # DDS DataStreamer plugin
│   ├── libplotjuggler_m2_robot_view.so    # 3D Robot View Toolbox plugin
│   └── assets/m2_metal_description/       # URDF + STL meshes
├── include/plotjuggler_m2/
│   ├── m2_canonical_names.h               # Joint ordering, leg names, topic constants
│   └── m2_data_enhancement.h              # Physics pairing & telemetry engine
├── src/
│   ├── m2_canonical_names.cpp
│   ├── m2_data_enhancement.cpp            # PD math, power estimation, aliases
│   ├── m2_datastreamer.h / .cpp           # CycloneDDS subscriber & settings dialog
│   └── m2_robot_view_toolbox.h / .cpp     # OpenGL 3D viewer, FK solver & PiP dialog
├── tools/
│   └── m2_bag_player.cpp                  # Standalone high-speed MCAP player
├── tests/
│   ├── m2_message_flatten_smoke.cpp       # Unit tests for unpacking & physics calculations
│   ├── m2_dds_loopback_test.cpp           # Live CycloneDDS network loopback integration test
│   ├── m2_bag_player_smoke.cpp            # Test for direct MCAP extraction
│   └── m2_mock_publisher.cpp              # Synthetic 500 Hz telemetry generator
├── scripts/
│   ├── build_local_bundle.sh              # One-step build and dependency bootstrap
│   ├── run_plotjuggler.sh                 # Launcher script with environment presets
│   ├── play_bag.sh                        # MCAP / ROS 2 bag playback utility
│   └── package_bundle.sh                  # Tarball distributor for deployment
└── third_party/
    ├── m2_sdk/                            # Svan M2 IDL specifications & CycloneDDS headers
    ├── xterra_m2_assets/                  # Official M2 URDF and meshes
    └── mcap/                              # Standalone MCAP reader library
```

---

## Getting Started

### Prerequisites

Clone the repository with submodules:
```bash
git clone --recursive https://github.com/yourusername/m2-pj.git
```

Install dependencies:
```bash
./scripts/install_ubuntu_deps.sh
```

* **Operating System**: Ubuntu 22.04 or 24.04 (x86_64)
* **Compiler**: GCC 9+ / Clang 10+ with C++17 support
* **Core Dependencies**: CMake (3.16+), Qt5 (`Widgets`, `Xml`, `OpenGL`, `Network`, `Svg`)

To install standard development dependencies:
```bash
./scripts/install_ubuntu_deps.sh
```

### 1. Build the Plugin Suite
Run the automated build script, which handles submodules, dependencies, compilation, tests, and bundle assembly:
```bash
./scripts/build_local_bundle.sh
```

### 2. Launch PlotJuggler
Launch PlotJuggler with the plugin bundle loaded:
```bash
# Default Canonical Mode (clean raw IDL channels)
./scripts/run_plotjuggler.sh

# Enhanced Mode (enables PD torques, power calcs, leg & multi-curve aliases)
./scripts/run_plotjuggler.sh --enhanced
```
---

## Testing Workflows

### Option A: Live Robot Operation
1. Connect your workstation to the Svan M2 network (the above Plotjuggler run will auto-detect any active and correctly configured `ethernet` channels).
2. In PlotJuggler, click the gear/wrench icon next to **Svan M2 DDS** (or go to streamer options):
   * Select your robot's **DDS Domain ID** (default: `0`).
   * Choose your active network interface (e.g. `eth0` or `wlan0`).
3. Click **Start** to begin streaming.

---

### Option B: Replaying Recorded Rosbags
To replay an existing Svan M2 bag into PlotJuggler:

1. **Terminal 1**: Start PlotJuggler in bag mode (configures buffer to retain full recording):
   ```bash
   ./scripts/run_plotjuggler.sh --bag --enhanced
   ```
2. Click **Start** under **Svan M2 DDS** in PlotJuggler.
3. **Terminal 2**: Play the bag with the standalone player:
   ```bash
   ./scripts/play_bag.sh xtr_rosbags/testbag4/testbag4_0.mcap
   ```
   * *Play at 2x speed*: `./scripts/play_bag.sh <path_to_bag> 2.0`
   * *Loop continuously*: `./scripts/play_bag.sh <path_to_bag> 1.0 --loop`
4. Open the 3D Robot View (<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>R</kbd>). Use the slider at the bottom of the 3D window to scrub back and forth through time!

---

### Option C: Synthetic Telemetry (No Hardware Required)
You can verify the entire pipeline without a physical robot using the built-in mock publisher:

1. **Terminal 1**: Start the mock publisher (publishes 500 Hz `SensorData`, 200 Hz `JointData`, 20 Hz `JoyData`):
   ```bash
   ./build/m2_mock_publisher
   ```
2. **Terminal 2**: Start PlotJuggler:
   ```bash
   ./scripts/run_plotjuggler.sh --enhanced
   ```
3. In PlotJuggler:
   * Under **Streaming** in the left sidebar, select **Svan M2 DDS** and click **Start**.
   * Drag curves (e.g. `sensor_data/joint/00/q`, `enhanced/00/tau_des`, or `joints*/q`) onto the plot area.
   * Open **Tools** &rarr; **Svan M2 Robot View** (or press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>R</kbd>) to observe the 3D quadruped moving in real-time.

---

## User Interface & Controls

### 3D Robot View Controls

| Action | Control |
|---|---|
| **Orbit / Rotate View** | Left-click + Drag |
| **Pan Camera** | Right-click + Drag (or Middle-click + Drag) |
| **Zoom In / Out** | Mouse Wheel |
| **Reset Camera View** | Click **Reset Camera** button |
| **Toggle 3D View On / Off** | <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>R</kbd> |
| **Pop Out to Floating Window (PiP)** | <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>P</kbd> or <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>D</kbd> |
| **Close View / Return to Plots** | <kbd>Esc</kbd> |
| **IMU Toggle** | Checkbox: align 3D body with onboard IMU vs. fixed ground |
| **Mesh Toggle** | Checkbox: toggle 3D STL meshes vs. kinematic stick skeleton |
| **Live / Scrub Mode** | Toggle **Live** checkbox or drag timeline slider |
| **Buffer Management** | Click **Buffer** button to toggle 30s rolling vs. $\infty$ full bag |

---

## CLI Launcher Reference

`./scripts/run_plotjuggler.sh [OPTIONS] [PLOTJUGGLER_ARGS...]`

| Flag | Description |
|---|---|
| `--enhanced` | Enables derived mathematics (PD torques, instantaneous power, tracking errors, leg aliases). |
| `--bag` | Pre-sets PlotJuggler streaming buffer to maximum (retains entire bag without rolling discard). |
| `--live` | Pre-sets PlotJuggler buffer to 30-second rolling window (recommended for live testing). |
| `--buffer_size <sec>` | Manually specifies buffer retention window in seconds. |
| *Other arguments* | Transparently forwarded directly to the PlotJuggler executable (e.g. `--layout my_layout.xml`). |

---

## Running Automated Tests

Run the full automated test suite with CTest:
```bash
ctest --test-dir build --output-on-failure
```

Included test targets:
1. `m2_message_flatten_smoke`: Verifies deserialization and math accuracy of derived metrics ($\tau_{\text{des}}$, mechanical power, tracking errors).
2. `m2_dds_loopback_test`: Spawns a CycloneDDS participant, publishes live binary `SensorData`, and verifies end-to-end network loopback reception.
3. `m2_bag_player_smoke`: Verifies standalone MCAP extraction and DDS packet dispatch.

---

## License & Attribution

Developed for the **Svan M2** platform by Xterra Robotics. Built on top of [PlotJuggler](https://github.com/facontidavide/PlotJuggler) by Davide Faconti.
