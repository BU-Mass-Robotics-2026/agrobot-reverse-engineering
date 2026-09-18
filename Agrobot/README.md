# Agrobot — Complete Codebase Guide for New Members

## What Is Agrobot?

**Agrobot (AgroBot TOM v2)** is the BU Robotics Club's autonomous tomato-picking robot. It consists of:
- A 7-joint robotic arm on a **linear rail** (the rail counts as joint 0)
- Multiple motor controllers from different vendors, all communicating over a **CAN bus**
- An **Intel RealSense** depth camera for vision
- An AI-powered **tomato detection pipeline** (camera → detect → pick)
- A **ROS 2 Jazzy** software stack for motion planning and hardware control

The robot rolls the rail down a row of tomato plants, detects tomatoes with AI vision, and commands the arm to pick them.

---

## PC Directory Map

```
/home/robotics-club/
├── AgrobotV2/              ← AI vision pipeline (perception only)
├── agrobot_ws/             ← ROS 2 workspace: arm control, hardware drivers
├── ingenia_overlay_ws/     ← Extra ROS 2 packages for the Ingenia J0 motor
├── robot_stack/            ← Startup/shutdown shell scripts (use these daily)
├── agrobot_ui/             ← Web dashboard (HTML/JS)
├── agrobot_dashboard/      ← (older dashboard files)
├── Copley/                 ← Copley motor drive manuals, tools, config files
├── dakota/                 ← Quick test scripts (CAN listener, gripper loop)
├── c/                      ← Low-level C code and DLL for CopleyCAN
├── librealsense/           ← Intel RealSense camera SDK source
├── robot_stack_audits/     ← Historical audit logs from startup scripts
├── j0_postmortem_*/        ← Debugging snapshots from J0 failures
├── tmp_epos_canopen_*/     ← Temporary CANopen recovery files
└── Downloads/              ← Manuals (EPOS2, Everest XCR, CANopen)
```

---

## 1. `AgrobotV2/` — AI Vision / Perception

**Purpose:** Detect tomatoes in camera images using state-of-the-art AI models. This is the "eyes" of the robot.

**Tech stack:** Python, Bazel, Docker, ROS 2, PyTorch

### How the detector works (4 stages)

```
Camera frame (518×518 px)
    │
    ▼
[Stage 1] SAM2 AMG
    Places 28×28 = 784 grid points across the image.
    For each point, SAM2 asks "what object is here?"
    → Produces up to 784 pixel-precise mask proposals
    (It doesn't know what is a tomato yet — just segments everything)
    │
    ▼
[Stage 2a] DINOv2 (Meta)
    Runs once per frame. Produces a 37×37 grid of "fingerprints"
    for every 14×14 pixel region. Compares each mask's fingerprints
    against 4 pre-built tomato prototype vectors (green, yellow, red,
    partially-occluded). Score = max cosine similarity − background similarity.
    │
[Stage 2b] SigLIP (Google)
    Crops each mask's bounding box, compares it to text:
    Positive: "a photograph of a ripe red tomato on a vine"
    Negative: "a green leaf", "a stem", "soil"
    → siglip_sim = best positive match − best negative match
    │
    ▼
[Stage 3] Fusion MLP (tiny neural net, <2KB)
    Takes 7 numbers per detection: dino_sim, siglip_sim, pred_iou,
    mask_area, circularity, mean_hue, mean_saturation.
    Outputs one number: "probability this is a real tomato"
    Trained on the detector's own outputs — no extra labeling needed.
    │
    ▼
[Stage 4] NMS + Cap
    Removes duplicate overlapping boxes (keep the higher-confidence one).
    Keeps at most 30 detections per frame.
    │
    ▼
ROS 2 topic: /agrobot/detections  (bounding boxes + confidence scores)
```

**Current accuracy:** mAP@0.5 = **0.492**, Precision = **0.87** (~21 seconds/frame on CPU NucBox)

### Key subdirectories

| Path | What it contains |
|---|---|
| `perception/agrobot_perception/` | The main Python package (detector nodes) |
| `perception/agrobot_perception/detectors/` | SAM2+DINOv2 detector, SAM2 AMG detector |
| `perception/eval/` | Evaluation scripts, SigLIP rescoring, Fusion MLP |
| `perception/tools/` | Training tools (finetune DINOv2, finetune SAM2, build prototypes) |
| `perception/launch/` | ROS 2 launch files (`perception.launch.py`, `perception_gpu.launch.py`) |
| `data/Laboro-Tomato/` | Training image dataset with annotations |
| `models/` | Model weight files (gitignored; see `models/README.md`) |
| `docs/` | Architecture docs, sprint notes, failure mode analysis |
| `deployment/docker/` | Dockerfiles for local dev and ROCm (AMD GPU) |

### Key ROS topics published by the detector

| Topic | Type | What it is |
|---|---|---|
| `/agrobot/detections` | `vision_msgs/Detection2DArray` | Bounding boxes of detected tomatoes |
| `/agrobot/detections_3d` | `vision_msgs/Detection3DArray` | 3D position of tomatoes (needs depth camera) |
| `/agrobot/safe_to_pick` | `std_msgs/Bool` | True = at least one tomato visible; False = do not pick |
| `/agrobot/debug_image` | `sensor_msgs/Image` | Camera frame with boxes drawn (for Foxglove) |

### To run the detector

```bash
cd ~/AgrobotV2
./deployment/docker/run_rocm.sh bash          # start Docker container (NucBox)
# Inside container:
ros2 run agrobot_perception tomato_detector
```

---

## 2. `agrobot_ws/` — ROS 2 Motion Workspace

**Purpose:** The full robot control stack — motion planning, hardware drivers, and the high-level "what do we do with the tomatoes we found?" logic.

### Package overview

| Package | Language | What it does |
|---|---|---|
| `robot_description` | Xacro/URDF | Defines the arm's geometry (joint positions, link lengths, meshes) |
| `moveit_config` | Config | MoveIt 2 setup: planning group, kinematics solver, joint limits, named poses |
| `robot_bringup` | XML launch | Launches robot_state_publisher, ros2_control, MoveIt, RViz for hardware |
| `robot_interfaces` | ROS 2 msgs | Custom messages: `JointCommand`, `PoseCommand`, `PositionCommand` |
| `robot_commander` | C++ + Python | Receives high-level commands → plans and executes arm motions via MoveIt |
| `agrobot_motion` | C++ | Lower-level movers: Cartesian, joint-space, and rail movers |
| `agrobot_supervisor` | Python | State machine that runs the full pick-and-sweep mission |
| `epos2_bridge` | Python | Translates ROS 2 trajectories → CANopen commands for **Maxon EPOS2** drives (J2–J6) |
| `epos2_bridge_interfaces` | ROS 2 srvs | Services: `MoveAbsolute`, `MoveAbsoluteTimed`, `MoveDelta` |
| `copley_bridge` | Python | Same as epos2_bridge but for **Copley APZ** drives (J1) |
| `motor_bringup` | Launch | Convenience launch to bring up all motor bridges |
| `ros2_canopen` | C++ | Third-party library that handles the low-level CANopen protocol (do not modify) |

### The arm joints

| Joint | Name | Drive Hardware | CAN Node ID | Bridge Package |
|---|---|---|---|---|
| J0 | Linear rail | Ingenia EVS-XCR-C | 10 | `ingenia_bridge` (separate workspace) |
| J1 | Shoulder rotate | Copley APZ-090-50 | 2 | `copley_bridge` |
| J2 | Shoulder pitch | Maxon EPOS2 70/10 | 3 | `epos2_bridge` |
| J3 | Elbow | Maxon EPOS2 70/10 | 7 | `epos2_bridge` |
| J4 | Wrist 1 | Maxon EPOS2 70/10 | 113 | `epos2_bridge` |
| J5 | Wrist 2 | Maxon EPOS2 70/10 | 3 | `epos2_bridge` |
| J6 | Wrist rotate | Maxon EPOS2 70/10 | 24 | `epos2_bridge` |

### How a pick command flows through the system

```
tomato_detector_node (vision)
    → /agrobot/tomato_tracks (JSON: positions of all tracked tomatoes)
        │
        ▼
agrobot_supervisor (supervisor_node.py)
    State machine: IDLE → STEP → SETTLE → CAPTURE → PROCESS_QUEUE → ...
    Steps the rail (J0) forward, captures a tomato snapshot at each stop.
    Builds a pick_queue (deduplicated by persistent_id).
        │
        ▼
robot_commander (commander.cpp)
    Receives /pick_targets (PoseArray — 3 poses per tomato: approach, grasp, retract)
    Calls MoveIt to plan and execute each pose.
        │
        ▼
MoveIt → arm_controller/follow_joint_trajectory (action)
        │
        ▼
arm_trajectory_fanout (ingenia_overlay_ws)
    Splits the full-arm trajectory into per-joint trajectories.
    Sends each joint's portion to its own bridge's action server.
        │
    ┌───┼──────────────┐
    ▼   ▼              ▼
epos2_bridge  copley_bridge  ingenia_bridge
    (J2-J6)       (J1)          (J0 rail)
        │
        ▼
CANopen PDO frames → CAN bus → Physical motor drives
```

### Named robot poses (defined in `moveit_config`)

| Name | Description |
|---|---|
| `attention` | All joints at 0° (home/rest position) |
| `crouch` | Compact transport pose |
| `vertical` | J3 at −90° |
| `bin` | Drop-off position for picked tomatoes |

### Building the workspace

```bash
cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 3. `ingenia_overlay_ws/` — Ingenia Motor Overlay Workspace

**Purpose:** ROS 2 packages specifically for the Ingenia EVS-XCR-C drive that controls the linear rail (J0). This is a separate workspace overlaid on top of `agrobot_ws`.

### Packages inside

| Package | What it does |
|---|---|
| `ingenia_bridge` | Translates ROS 2 FollowJointTrajectory actions → CANopen IPM segments for the Ingenia drive. Uses two RPDOs per segment (Ingenia's protocol differs from EPOS2 and Copley). |
| `ingenia_bringup` | Launch files and CANopen configuration (`.eds` file, `bus.yml`) for the Ingenia drive. |
| `arm_trajectory_fanout` | Sits between MoveIt and all the individual joint bridges. MoveIt sends one full-arm trajectory; fanout splits it and sends each joint's slice to the right bridge. |

### Why is this a separate workspace?

The Ingenia drive was added later and uses a different PDO layout than the EPOS2/Copley drives. Keeping it as an overlay means you can add/update it without rebuilding the entire `agrobot_ws`.

---

## 4. `robot_stack/` — System Startup/Shutdown Scripts

**Purpose:** The master control script for starting, stopping, and monitoring the entire robot software stack. **This is what you use every day.**

### Main script: `hw_moveit_stack.sh`

```bash
~/robot_stack/hw_moveit_stack.sh <command>
```

| Command | What it does |
|---|---|
| `start` | Starts everything in order: EPOS2 bridges → Copley bridge → fanout → fake joint filler → robot_state_publisher → MoveIt |
| `stop` | Gracefully stops all processes in reverse order |
| `restart` | stop + start |
| `hard-clean` | Nuclear option: kills ALL relevant processes by name, clears PID files. Use when things are stuck. |
| `home-j0` | Sends the linear rail (J0) to its home position |
| `estop` | Emergency soft-stop: immediately disarms the Ingenia J0 drive's IPM mode |
| `rviz` | Launches RViz with the MoveIt visualization |
| `status` | Shows which processes are running/stopped |
| `logs` | Tails all log files in real-time |
| `coverage` | Checks that all expected joint names are publishing on `/joint_states` |
| `action-info` | Shows which nodes own the FollowJointTrajectory action servers |
| `params` | Sets MoveIt trajectory execution tolerances |

### Typical startup sequence (every time you use the robot)

```bash
# 1. Make sure CAN bus is up at 1 Mbps
sudo ip link set can0 up type can bitrate 1000000

# 2. Start the full stack
~/robot_stack/hw_moveit_stack.sh start

# 3. (Optional) Home each joint — see agrobot_ws/src/Bringup_Commands.txt
~/robot_stack/home-j0-safe     # Home the rail

# 4. Open RViz to visualize
~/robot_stack/hw_moveit_stack.sh rviz

# When done:
~/robot_stack/hw_moveit_stack.sh stop
```

### Other files in `robot_stack/`

| File | Purpose |
|---|---|
| `home-j0-safe` | Script that safely homes the Ingenia J0 linear rail |
| `fake_unmapped_joint_state_filler.py` | Publishes dummy joint states for `fin_joint1` (a structural link with no encoder) so MoveIt doesn't complain about missing joints |
| `j0_postmortem_snapshot.sh` | Diagnostic script — takes a snapshot of J0 state for debugging after a failure |
| `SAFE_J0_J2_STAGED_BRINGUP_CHECKPOINT.md` | Documents the validated safe bringup configuration |
| `*.bak_*` files | Timestamped backups of scripts at key development milestones |
| `restore_baseline_report_*.csv` | Logs from hardware state restoration operations |
| `logs/` | Per-run log directory (created automatically, named by timestamp) |

---

## 5. `agrobot_ui/` — Web Dashboard

**Purpose:** A browser-based dashboard to monitor and control the robot over the local network.

### Files

| File | Role |
|---|---|
| `index.html` | Main dashboard page |
| `style.css` | Styling |
| `dashboard.js` | JavaScript that connects to ROS via `roslib` (rosbridge websocket) |

### What the dashboard shows

- **3D Robot View** — Button to open Foxglove at `ws://172.16.1.160:8765` (the robot's IP on the local network)
- **Camera Feed** — Live MJPEG stream from `/camera/camera/color/image_raw` via `web_video_server`
- **Emergency Stop** button
- **Controls** — Home, Pick, Reset buttons
- **Joint Positions** — Live readout of all 7 joints (J0–J6)
- **Motor Status** — EPOS2 J2 and Ingenia J0 status
- **System Status** — ROS bridge and MoveIt connection state

### To run the dashboard

```bash
~/start_dashboard.sh        # Starts the rosbridge and web server
# Then open http://172.16.1.160:8080 in a browser
```

---

## 6. `Copley/` — Copley Motor Drive Resources

**Purpose:** Reference materials for the Copley APZ-090-50 motor controller used on J1.

| File | What it is |
|---|---|
| `copley_control.py` / `copley_control2.py` / etc. | Python scripts for directly testing/commanding a Copley drive (bypasses ROS) |
| `copleycan.h`, `copleycan.dll`, `copleycan.exe` | Copley CAN API for Windows (for use with CME software) |
| `CME-8.2.1.zip`, `CME8.1.zip` | Copley Motion Editor — Windows GUI for configuring drives |
| `CANviewLinux.zip` | CAN bus sniffer/analyzer for Linux |
| `APV-Datasheet-rev13.pdf` | Datasheet for the Copley Accelnet APV drive |
| `CAN-ECAT-Programmers-Manual-rev06.pdf` | CANopen protocol guide for Copley drives |
| `ASCII_Programmers_Guide_Manual.pdf` | Copley ASCII command protocol reference |
| `parameter_dictionary_rev06.pdf` | Full list of all OD (Object Dictionary) parameters |

---

## 7. `dakota/` — Quick Test Scripts

Quick scripts for manual hardware testing:

| File | What it does |
|---|---|
| `can_listen.py` | Prints raw CAN frames — useful to verify the CAN bus is working and see what the drives are saying |
| `j0_test.sh` | Tests J0 (rail) motor commands directly |
| `gripper_loop.bash` | Repeatedly commands the gripper open/close for testing |

---

## 8. `c/` — Low-Level C Code

| File | What it is |
|---|---|
| `copleycan.h`, `copleycan.dll` | Copley's CAN API library (Windows) |
| `copleycan.c`, `test.c` | C source files for talking to Copley drives via CAN |
| `example.sln`, `example.vcproj` | Visual Studio project for building the C example |

---

## 9. CAN Bus & Hardware Overview

All motors communicate over a single **CAN bus** (Controller Area Network) at **1 Mbps**. The PC connects to the CAN bus via a **CANable2** USB adapter (`/dev/serial/by-id/usb-Openlight_Labs_CANable2_*`).

### CAN interface setup

```bash
# For gs_usb firmware (candleLight):
sudo ip link set can0 up type can bitrate 1000000

# For slcan firmware (legacy):
sudo slcand -o -c -f -s8 /dev/serial/by-id/<device> can0
sudo ip link set can0 up
```

### Protocol: CANopen + Interpolated Position Mode (IPM)

All drives use **CANopen** over CAN. For motion, they use **IPM (Interpolated Position Mode)**:
- The PC sends a stream of short position+velocity+time "segments" (typically every 10–100ms)
- The drive interpolates between segments for smooth motion
- This is how MoveIt trajectories are executed: the bridge slices up the planned path into segments and streams them to the drive in real-time

---

## 10. Key Log and Debug Files

| File/Dir | What it is |
|---|---|
| `can_trace.log` | Captured CAN bus traffic (from a previous debugging session) |
| `ingenia_launch.log` | Log from a previous Ingenia launch |
| `fanout.log` | Log from the arm_trajectory_fanout node |
| `move_group.log` | Log from MoveIt's move_group node |
| `j0_postmortem_20260516_163014/` | Diagnostic snapshots collected after a J0 motor failure |
| `robot_stack/logs/` | Per-run logs for each process (created at each `hw_moveit_stack.sh start`) |
| `debug_snapshots/` | Compressed tarballs of the full workspace state (for major debugging sessions) |

---

## 11. Configuration Files

### `cyclonedds.xml` & `fastdds_unicast.xml`
These configure the ROS 2 DDS middleware (how ROS nodes discover each other). The unicast config forces direct communication rather than multicast — important on networks where multicast is unreliable.

### `agrobot_ws/src/moveit_config/config/`

| File | Purpose |
|---|---|
| `joint_limits.yaml` | Max velocity and acceleration for each joint |
| `kinematics.yaml` | Kinematics solver (KDL) configuration |
| `moveit_controllers.yaml` | Tells MoveIt which controller action servers to use |
| `ros2_controllers.yaml` | ros2_control controller definitions |
| `initial_positions.yaml` | Default starting joint positions |
| `pilz_cartesian_limits.yaml` | Speed/accel limits for Cartesian planning |

### `tmp_epos_canopen_recovery/`
Contains CANopen bus configuration files (`bus.yml`, `master.dcf`, `.eds`) from a recovery attempt for the EPOS2 J3 joint. These were working test configurations used during debugging.

---

## 12. Where to Start as a New Member

**If you're working on vision/perception:**
1. Read `AgrobotV2/README.md` and `AgrobotV2/docs/SPRINT5_ARCHITECTURE.md`
2. Look at `AgrobotV2/perception/agrobot_perception/` — the Python package
3. The main detector node is `tomato_detector_node.py`

**If you're working on arm motion/control:**
1. Read `agrobot_ws/CLAUDE.md` — it's the best quick-reference for the ROS workspace
2. Look at `robot_commander/src/commander.cpp` — the top-level arm command dispatcher
3. Look at `epos2_bridge/epos2_bridge/epos2_joint_bridge.py` — how a motor bridge works

**If you're doing hardware bringup (starting the robot):**
1. Read `robot_stack/SAFE_J0_J2_STAGED_BRINGUP_CHECKPOINT.md`
2. Refer to `agrobot_ws/src/Bringup_Commands.txt` for homing sequences
3. Use `~/robot_stack/hw_moveit_stack.sh` as your main tool

**If something breaks:**
1. Run `~/robot_stack/hw_moveit_stack.sh hard-clean` to kill everything
2. Check `~/robot_stack/hw_moveit_stack.sh logs` to see what failed
3. The `j0_postmortem_snapshot.sh` script captures a full diagnostic snapshot
