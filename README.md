# Svan M2 PlotJuggler Plugin Suite (`m2-pj`)

High-performance real-time telemetry streaming and 3D visualization plugins for the **Svan M2 Quadruped Platform** in **PlotJuggler**.

---

## Features

- **Direct DDS Streaming Plugin (`plotjuggler_m2`)**:
  - Direct native CycloneDDS ingestion from the Svan M2 robot at 500 Hz.
  - Automatically unpacks:
    - Actuator Kinematics: `q[12]`, `dq[12]`, `ddq[12]`, `tau_est[12]`, `q_current[12]`
    - Electrical & Thermal Diagnostics: `driver_fault[12]`, `driver_voltage[12]`, `driver_power[12]`, `fet_temp[12]`, `motor_temp[12]`
    - Body IMU: Quaternion `quat[4]`, Angular velocity `gyro[3]`, Linear acceleration `accel[3]`, Euler angles `rpy[3]` (degrees and radians)
    - Actuator Targets: `q[12]`, `dq[12]`, `kp[12]`, `kd[12]`, `tau_ff[12]`
    - Joystick: Teleoperation axes and state machine button triggers
- **Real-Time Physics & Derived Telemetry Engine**:
  - **Zero-Order Hold Command/Feedback Pairing**: Computes motor driver closed-loop PD response matching Svan M2's onboard control law:
    $$\tau_{\text{des\_p}} = kp \cdot (q_{\text{ref}} - q_{\text{act}})$$
    $$\tau_{\text{des\_d}} = kd \cdot (\dot{q}_{\text{ref}} - \dot{q}_{\text{act}})$$
    $$\tau_{\text{des}} = \tau_{\text{ff}} + \tau_{\text{des\_p}} + \tau_{\text{des\_d}}$$
  - **Instantaneous Mechanical Power**: $P = \tau \cdot \dot{q}$ per joint and total quadruped mechanical vs. electrical power.
  - **Tracking Errors**: Continuous tracking error $e_q$ and $e_{\dot{q}}$.
  - **Convenience Aliases**:
    - Leg-first aliases: `legs/FR/hip/q`, `legs/FL/thigh/q`, `legs/RR/calf/q`, etc.
    - Metric-first aliases: `joints*/q/00..11` for 1-drag plotting of all 12 joints simultaneously.
- **3D Robot View Toolbox Plugin (`plotjuggler_m2_robot_view`)**:
  - Self-contained 3D OpenGL viewport embedded inside PlotJuggler (`Tools -> Svan M2 Robot View`).
  - Automatically parses the official URDF (`m2_metal_description.urdf`) and loads the 17 binary STL mesh components for the base, hips, thighs, calfs, and feet.
  - Forward kinematics solver transforms the robot model in real time or during playback scrubbing according to incoming `q[12]` and IMU orientation.

---

## Directory Structure

```text
m2-pj/
├── CMakeLists.txt                      # Root build configuration
├── bundle/plotjuggler_m2/              # Self-contained runtime bundle
│   ├── libplotjuggler_m2.so            # DataStreamer plugin
│   ├── libplotjuggler_m2_robot_view.so # 3D Toolbox plugin
│   └── assets/m2_metal_description/    # URDF + STL meshes
├── include/plotjuggler_m2/
│   ├── m2_canonical_names.h            # Joint indexing & topic conventions
│   └── m2_data_enhancement.h           # Derived physics & flattening engine
├── src/
│   ├── m2_canonical_names.cpp
│   ├── m2_data_enhancement.cpp
│   ├── m2_datastreamer.h / .cpp        # DataStreamer implementation & Qt GUI
│   └── m2_robot_view_toolbox.h / .cpp  # 3D Viewer & OpenGL renderer
├── tests/
│   ├── m2_message_flatten_smoke.cpp    # Unit smoke tests
│   ├── m2_dds_loopback_test.cpp        # Live CycloneDDS pub-sub test
│   └── m2_mock_publisher.cpp           # Synthetic DDS telemetry generator
├── scripts/
│   ├── build_local_bundle.sh           # Build & packaging script
│   └── run_plotjuggler.sh              # Launch script
└── third_party/
    ├── m2_sdk/                         # Svan M2 IDLs and CycloneDDS headers
    └── xterra_m2_assets/               # Official URDF and STL meshes
```

---

## Quick Start

### 1. Build the Plugin Bundle
```bash
./scripts/build_local_bundle.sh
```

### 2. Launch PlotJuggler with Plugins
```bash
./scripts/run_plotjuggler.sh
```

### 3. Testing Without a Physical Robot (Mock Generator)
In a separate terminal, run the mock telemetry publisher:
```bash
./build/m2_mock_publisher
```
In PlotJuggler:
1. Under **Streaming** on the left panel, select **Svan M2 DDS** and click **Start**.
2. Drag and drop curves (e.g. `sensor_data/joint/00/q`, `enhanced/00/tau_des`, `joints*/q`) onto the plot area.
3. Open **Tools -> Svan M2 Robot View** to see the 3D model moving in sync with the gait!

### 4. Testing with Recorded Rosbags
To replay a recorded ROS 2 bag into PlotJuggler over CycloneDDS:
```bash
./scripts/play_bag.sh xtr_rosbags/testbag
```
You can also specify playback speed (e.g. `0.5` for half speed, `2.0` for 2x) or loop playback:
```bash
./scripts/play_bag.sh xtr_rosbags/testbag 1.0 --loop
```
In PlotJuggler: Select **Svan M2 DDS** -> **Start**, and open **Tools -> Svan M2 Robot View** to visualize the recorded run in 3D!
