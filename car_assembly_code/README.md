# 🤖 MechArm 270 Unified Robot Controller & Assembly Vision Cockpit

A high-fidelity single-process control and visual inspection system designed for the **Mouser Road Show**. This repository powers a **MechArm 270** robotic arm mounted on a stepper linear rail, integrated with an **OpenCV + AprilTags + HSV color contour** webcam inspection layer to perform automated, verified pick-and-place assembly of two toy cars.

The entire backend is hosted as a single process running on **Port 5000**, enabling **zero network latency** and absolute reliability.

---

## 📂 System File Architecture
```text
opencv vision code/
├── config.py             # Shared calibration values, HSV limits, and tag ID maps
├── server_unified.py     # Main Python Flask + Socket.IO server & Vision loop
├── scan_poses.json       # Dynamic, persistent file containing custom camera scan angles
├── waypoints_wifi.json   # Recorded rail & arm joint coordinates sequence database
├── README.md             # This comprehensive logic & installation manual
└── templates/
    ├── dashboard.html    # Unified Operator Cockpit (Telemetry, Stream, checklist status)
    └── waypoint_dashboard_wifi.html # Joint/Coordinate Waypoint Recording & Scan Pose Editor
```

---

## ⚡ Station Setup & Physical Component Mapping

The physical rail consists of four primary inspection stations (`P1`, `P2`, `P4`, `P5`) and one assembly station (`P3`) mapped to specific automotive components:

| Station | Preset Name | Detection Method | Physical Component Mapping & Target Rules |
| :--- | :--- | :--- | :--- |
| **P1** | `P1 (Chassis)` | **AprilTag (tag16h5)** | **Visit #1**: Chassis 1 (Tag ID `0`) <br>**Visit #2**: Chassis 2 (Tag ID `1`) |
| **P2** | `P2 (Wheelbase)`| **AprilTag (tag16h5)** | **Visit #1**: WheelBase 1a (Tag ID `2`) <br>**Visit #2**: WheelBase 2a (Tag ID `3`) <br>**Visit #3**: WheelBase 1b (Tag ID `4`) <br>**Visit #4**: WheelBase 2b (Tag ID `5`) |
| **P3** | `P3 (Assemble)` | *None* | Physical assembly zone (no vision verification required) |
| **P4** | `P4 (Blue)` | **HSV Contour** | Blue Color Inspection (Wheels Pick) |
| **P5** | `P5 (Green)` | **HSV Contour** | Green Color Inspection (Upper Body Pick) |

---

## 🧠 Dynamic Verification Logic & Visit Counter

Because waypoints are fully editable and user-configurable inside the editor, tracking parts using hardcoded step numbers is highly brittle. To address this, the **Playback Engine** utilizes a **transition-based visit counter**:

1. **Preset Transition Detection**: As the sequence runs, the Playback Engine monitors the rail's movements. Whenever the rail transitions to a different preset position (`P1`, `P2`, `P4`, `P5`), a transition event is registered.
2. **Visit Incrementor**: The engine increments the visit counter for that specific preset (e.g. `P1` has been visited 1 time, `P2` has been visited 2 times).
3. **Scan Pose Intervention**: 
   - Before completing the step, the arm is automatically commanded to the pre-recorded camera view angle (`SCAN_POSES[new_preset]`).
   - The playback thread halts for **1.5 seconds** to allow physical settling and camera focus stabilization.
4. **Zero-Latency Memory Evaluation**:
   - The engine queries the background OpenCV thread directly in-process.
   - For **P1/P2 (AprilTags)**, it checks if the specific tag ID (0 to 5) corresponding to that visit count is stable.
   - For **P3/P4 (HSV contours)**, it verifies if the red or green color signature is present.
5. **Enforcement or Continuation**:
   - If verified, the system emits a `'part_verified'` Socket.IO event to update the checklist UI instantly, and continues.
   - If missing or wrong, the engine halts the robot, triggers an **Emergency Stop**, enters `ERROR` state, and pushes a visual alarm to the Operator Cockpit.

---

## 🔍 OpenCV Vision Processing Pipeline

The background vision loop operates in a dedicated thread at 30 FPS:
* **AprilTags Detection (`pupil-apriltags`)**: Uses the highly robust `tag16h5` family. It is calibrated with high decimation values to detect small tag bounds (2cm) at viewing angles.
* **Color Inspection (HSV contour masking)**:
  * **Red Wheels**: Hue range `[0-10]`, Saturation `[120-255]`, Value `[120-255]`
  * **Green Body**: Hue range `[40-80]`, Saturation `[120-255]`, Value `[120-255]`
  * **Minimum Area Filter**: Contours must be larger than `500 pixels` to prevent background noise from triggering false positives.
* **Telemetry Smoothing Filters**:
  * **Confirmation Threshold**: Parts must be detected continuously for `5 frames` to trigger `"Detected"` status.
  * **Disappearance Threshold**: Parts must be missing for `8 consecutive frames` before being marked as `"Missing"`.

---

## 📸 Dynamic Scan Pose Recording

In addition to editing standard waypoints, the **Waypoint Editor** has a dedicated **📸 Inspection Scan Poses** console.

1. Jog the robot manually to the perfect viewing angle for a station (e.g., pointing the camera directly at P1's chassis slot).
2. Click the **Set P1 (Chassis)** button in the Scan Poses console.
3. The system captures the current **6 joint angles** of the arm and persistently writes them to `scan_poses.json` on the disk.
4. The system dynamically reloads this file, ensuring that the arm will automatically visit your custom, real-world camera inspection angle during the verification steps, without any hardcoded changes in Python code.

---

## 💻 Twin Dashboards

### 1. Unified Operator Cockpit (`/`)
* **Real-time Live Stream**: Embeds the processed webcam stream with highlighted contours, overlay grids, and detection markers.
* **Interactive checklists**: Tracks the build state for Car 1 and Car 2. Completed parts are marked with ✅ and progress bar updates from 0% to 100%.
* **8-Switch Hardware Simulator**: Allows complete offline dry-run testing of the assembly sequence by manually toggle-mocking any of the 8 parts.
* **Halt Diagnostic Panel**: Displays warning descriptions and prompts to restart in case of a verification fault.

### 2. Waypoint Editor Console (`/editor`)
* **Full Jog & Record Suite**: Record arm angles (ANG), coordinates (XYZ), speed, vacuum state, and delay times.
* **Inspection Scan Pose buttons**: Save camera inspection positions on the fly.
* **Calibration Controls**: Set joint power, lock/unlock individual motors, and configure Esp32-C6 Rail parameters.

---

## 🚀 Installation & Running

Ensure dependencies are installed:
```bash
pip install flask flask-socketio pymycobot opencv-python numpy pupil-apriltags
```

Start the server:
```bash
python server_unified.py
```

* Navigate to the Cockpit: `http://localhost:5000/`
* Navigate to the Editor: `http://localhost:5000/editor`
