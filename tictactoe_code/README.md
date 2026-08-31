# 🤖 Autonomous Tic-Tac-Toe Robotic Arm System

A fully offline, serial-driven, autonomous Tic-Tac-Toe playing robotic system featuring the **Elephant Robotics MechArm 270**, an **ESP32-S3 linear rail slider**, an **arm-mounted computer vision camera**, and a **45-LED WS2812B RGB under-rail lighting strip**.

---

## 📑 Table of Contents
1. [System Overview & Architecture](#-system-overview--architecture)
2. [Hardware Setup & Pinout](#-hardware-setup--pinout)
3. [Motion Model & Kinematics](#-motion-model--kinematics)
4. [Computer Vision & Token Detection](#-computer-vision--token-detection)
5. [Minimax AI Engine](#-minimax-ai-engine)
6. [Serial Communication Protocols](#-serial-communication-protocols)
7. [Directory Structure](#-directory-structure)
8. [Installation & Dependencies](#-installation--dependencies)
9. [Firmware Setup (ESP32)](#-firmware-setup-esp32)
10. [Calibration & Waypoint Teaching Workflow](#-calibration--waypoint-teaching-workflow)
11. [Running the Application](#-running-the-application)
12. [Web Dashboards & APIs](#-web-dashboards--apis)
13. [Troubleshooting & FAQ](#-troubleshooting--faq)

---

## 🌟 System Overview & Architecture

Unlike fixed-sequence assembly arms, this system operates in real-time closed-loop decision making:
1. **Vision Perception**: The arm positions itself at a calibrated `scan_pose`, analyzing the 3×3 grid using HSV color segmentation to detect human token placements (Red tokens).
2. **Game Reasoning**: An unbeatable Alpha-Beta Minimax AI calculates the optimal countermove.
3. **Motion Execution**: The arm transitions through a 3-phase pick-and-place routine (Tray Pick → Safe Rail Transit → Cell Placement → Vacuum Release → Scan Return).
4. **Dual-Board Support**: Supports simultaneous interleaved gameplay on two separate board stations (`Board 1` and `Board 2`) along the linear rail.
5. **Aesthetics & Lighting**: 45-pixel addressable WS2812B RGB strip provides real-time state animations (thinking, moving, turn alerts, win/draw flashes) alongside 12V safety relays.

```
                  +----------------------------------------------+
                  |         Host PC / Jetson (Python Server)     |
                  |                                              |
                  |  +----------------+     +-----------------+  |
                  |  |  Minimax AI    | <-> |  Game Manager   |  |
                  |  +----------------+     +-----------------+  |
                  |          ^                       |           |
                  |          |                       v           |
                  |  +----------------+     +-----------------+  |
                  |  |  OpenCV Vision |     | Move Executor   |  |
                  |  +----------------+     +-----------------+  |
                  +----------|-----------------------|-----------+
                             | USB Video             | USB Serial
                             v                       v
               +-----------------------+   +-----------------------+
               | Arm-Mounted Camera    |   | MechArm 270 (COM /    |
               | (Overhead Board Scan) |   | /dev/ttyACM1)         |
               +-----------------------+   +-----------------------+
                                                     |
                                                     | USB Serial (115200)
                                                     v
                                           +-----------------------+
                                           | ESP32-S3 Controller   |
                                           |  - TMC5160 Rail Stepper|
                                           |  - 45x WS2812B LEDs   |
                                           |  - 12V Relays/Buzzer  |
                                           |  - NTC Temp / Door Sw |
                                           +-----------------------+
```

---

## 🔌 Hardware Setup & Pinout

### 1. MechArm 270 6-DOF Robotic Arm
- **Interface**: USB Serial (115200 Baud, e.g., `COM3` on Windows / `/dev/ttyACM1` on Linux).
- **End-Effector**: Suction Vacuum Pump.
  - Pump Output Pin: Basic Output `5`
  - Valve Vent Pin: Basic Output `2`
- **Payload**: Token pick head with rubber suction cup.

### 2. ESP32-S3 Linear Rail Controller (`TTT_StepperControl.ino`)
- **Microcontroller**: ESP32-S3 Dev Board.
- **Motor Driver**: TMC5160 via hardware SPI.
- **Rail Travel**: 52.0 cm maximum physical length (12800 steps/cm at 256 microsteps).

#### Pin Assignments
| Function / Device | ESP32-S3 GPIO | Notes |
|---|---|---|
| **TMC5160 CS** | `GPIO 10` | SPI Chip Select (Active LOW) |
| **TMC5160 SCK** | `GPIO 12` | SPI Clock |
| **TMC5160 MOSI** | `GPIO 11` | SPI Master-Out-Slave-In |
| **TMC5160 MISO** | `GPIO 13` | SPI Master-In-Slave-Out |
| **TMC5160 EN** | `GPIO 9` | Driver Enable (Active LOW) |
| **Home Limit Switch (S2)** | `GPIO 41` | Active LOW, Internal Pull-Up |
| **12V Relay - Red LED** | `GPIO 14` | Active HIGH (Relay energized) |
| **12V Relay - Buzzer** | `GPIO 17` | Active HIGH (Pulsed beep) |
| **12V Relay - Green LED** | `GPIO 15` | Active HIGH (Ready / Safe) |
| **NTC Temp Sensor** | `GPIO 1` | ADC 11dB attenuation (10k NTC) |
| **45x WS2812B RGB Strip** | `GPIO 42` | Data signal to DIN (Line 79 in firmware) |

---

## 🦾 Motion Model & Kinematics

### 3-Phase Collision-Free Motion Strategy

To prevent arm collisions with boards, tray walls, or rail fixtures during movement, the execution is strictly decoupled:

1. **Same-Board Pick & Place (Stationary Rail)**:
   $$\text{scan\_pose} \longrightarrow \text{tray\_approach} \longrightarrow \text{tray\_pick (Vac ON)} \longrightarrow \text{tray\_lift} \longrightarrow \text{cell\_}N\text{\_approach} \longrightarrow \text{cell\_}N\text{\_place (Vac OFF)} \longrightarrow \text{cell\_}N\text{\_lift} \longrightarrow \text{scan\_pose}$$
   *No safe position retraction is needed if the rail does not translate.*

2. **Cross-Board Station Change (Rail Translating)**:
   $$\text{Current Board} \longrightarrow \mathbf{safe\_position} \text{ (Tucked In)} \longrightarrow \mathbf{RAIL\ MOVE\ (BOARD1 \leftrightarrow BOARD2)} \longrightarrow \text{Target Board} \longrightarrow \text{Execute Sequence}$$

### Keyed Waypoint Requirements (Per Board)
Every board station requires **25 calibrated waypoints** in `ttt_waypoints_b1.json` / `ttt_waypoints_b2.json`:
- `scan_pose`: Camera overhead viewing the entire 3×3 board.
- `safe_position`: Tucked arm posture with clear envelope for rail sliding.
- `tray_approach`, `tray_pick`, `tray_lift`: Pick sequence at the token reservoir.
- `cell_{0..8}_approach`: High hover above cell $i$.
- `cell_{0..8}_place`: Surface contact for suction release onto cell $i$.
- `cell_{0..8}_lift`: Vertical retreat after placement.

---

## 👁️ Computer Vision & Token Detection

The camera mounted on the robotic arm checks board state on-demand once the arm reaches `scan_pose`.

### Color Segmentation (HSV)
- **Human Token (Red)**: Uses double-range hue wrap-around:
  - Range 1: $H \in [0, 10], S \in [100, 255], V \in [80, 255]$
  - Range 2: $H \in [160, 180], S \in [100, 255], V \in [80, 255]$
- **Robot Token (Blue)**:
  - Range: $H \in [100, 130], S \in [100, 255], V \in [80, 255]$

### Temporal Stability Filter
A move is only registered when a cell retains color matching over `CONFIRM_FRAMES = 5` consecutive video frames, eliminating false positives caused by arm shadows or transient lighting changes.

---

## 🧠 Minimax AI Engine

- **Algorithm**: Minimax with Alpha-Beta Pruning.
- **Complexity**: $O(b^d)$ pruned; computes optimal move in under 2 milliseconds for any 3×3 depth.
- **Properties**: Zero hardware dependency (`ttt_engine.py` is standalone testable). Mathematically unbeatable — guarantees a win or draw under all opening book variations.

---

## 📡 Serial Communication Protocols

The entire system functions 100% offline via USB Serial connections.

### 1. ESP32 Serial Commands (PC $\rightarrow$ ESP32 at 115200 Baud)
All commands are newline (`\n`) terminated:

| Command | Description | Example |
|---|---|---|
| `MOVE:<PRESET>` | Move rail to named preset | `MOVE:BOARD1`, `MOVE:BOARD2`, `MOVE:HOME` |
| `STOP` | Immediate rail emergency stop | `STOP` |
| `SPEED:<VMAX>` | Set motor positioning speed | `SPEED:350000` (50,000 to 500,000) |
| `INDICATOR:<MODE>` | Set 12V indicator relays | `INDICATOR:alert`, `INDICATOR:green`, `INDICATOR:off` |
| `LEDRGB:<EFFECT>` | Trigger RGB strip animation | `LEDRGB:robot_thinking`, `LEDRGB:win_robot` |
| `LEDSOLID:<R>,<G>,<B>` | Set all 45 LEDs to solid RGB | `LEDSOLID:0,229,200` |
| `LEDBRIGHT:<VAL>` | Set LED strip brightness (0–255) | `LEDBRIGHT:180` |

### 2. ESP32 Telemetry Broadcast (ESP32 $\rightarrow$ PC every 500 ms)
```json
STATUS:{"running":false,"homed":true,"homing":false,"point":"BOARD1","absCm":9.00,"vmax":500000,"tempC":27.4,"doorOpen":false,"ledEffect":"idle","ledBrightness":128,"gpioPin":16}
```

---

## 📁 Directory Structure

```
tictactoe_code/
├── README.md                     # Comprehensive system documentation
├── arm_config.json               # Arm serial port & calibration offsets
├── config_ttt.py                 # System-wide configuration constants
├── rail_config.json              # Rail network & connection fallback config
├── server_ttt.py                 # Core Flask + Socket.IO server & orchestrator
├── ttt_engine.py                 # Minimax AI, Board & GameManager logic
├── ttt_move_executor.py          # 3-phase arm & rail motion orchestrator
├── ttt_settings.json             # Runtime dynamic settings & persistence
├── ttt_vision.py                 # OpenCV HSV token perception engine
├── ttt_waypoints_b1.json         # Board 1 keyed waypoint database
├── ttt_waypoints_b2.json         # Board 2 keyed waypoint database
├── static/
│   ├── socket.io.min.js          # Offline client websocket library
│   └── logos/                    # UI branding assets
├── templates/
│   ├── ttt_dashboard.html        # Game dashboard & live monitoring UI
│   └── ttt_waypoints.html        # Waypoint teaching & LED calibration UI
└── TTT_StepperControl/
    └── TTT_StepperControl.ino    # ESP32-S3 firmware for TMC5160 + RGB Strip
```

---

## 📦 Installation & Dependencies

### Python Environment Setup
Tested on Python 3.10 / 3.11 (Windows 10/11 & Linux).

```bash
cd c:/Users/Admin/Downloads/roadshow_wired-20260814T044334Z-1-001/roadshow_wired/tictactoe_code

# Install required Python packages
pip install pymycobot pyserial opencv-python numpy flask flask-socketio
```

---

## ⚡ Firmware Setup (ESP32)

1. Open `TTT_StepperControl/TTT_StepperControl.ino` in Arduino IDE or VS Code with ESP32 board support installed.
2. Install required Arduino libraries:
   - **TMCStepper** by teemuatlut
   - **FastLED** by Daniel Garcia
3. Configure `RGB_LED_PIN` at the top of `TTT_StepperControl.ino`:
   ```cpp
   #define RGB_LED_PIN  42   // GPIO 42 for WS2812B data line
   ```
4. Select board **ESP32-S3 Dev Module** and flash via USB-C.

---

## 🎯 Calibration & Waypoint Teaching Workflow

Before starting a game, both Board 1 (and optionally Board 2) must be calibrated.

1. **Start Server**:
   ```bash
   python server_ttt.py
   ```
2. **Open Waypoints Editor**: Navigate to `http://localhost:5000/waypoints`.
3. **Motion Sequence & Rules**:
   - **Rule 1: Human Always Plays First**: Humans make move 1 (Red token). The robot makes moves 2, 4, 6, 8 (Blue token).
   - **Rule 2: 4 Tray Slots**: The piece tray holds 4 robot tokens per game. The robot picks from:
     - Move 1 $\rightarrow$ **Slot 0**
     - Move 2 $\rightarrow$ **Slot 1**
     - Move 3 $\rightarrow$ **Slot 2**
     - Move 4 $\rightarrow$ **Slot 3**
   - **Trajectory Sequence**:
     ```
     Pick Sequence (tray_slot_S_approach -> tray_slot_S_pick [Vac ON] -> tray_slot_S_lift)
     -> Pick Safe Position (pick_safe)
     -> Place Safe Position (place_safe)
     -> Place Sequence (cell_i_approach -> cell_i_place [Vac OFF] -> cell_i_lift)
     -> Place Safe Position (place_safe)
     -> Scan Pose (scan_pose)
     ```

4. **Teach Positions**:
   - **System & Safe Positions**:
     - `scan_pose`: Camera overhead viewing the 3×3 grid.
     - `pick_safe`: Safe transit height on the pick (tray) side.
     - `place_safe`: Safe transit height on the place (board) side.
   - **4 Tray Slots (Slots 0 to 3)**:
     - `tray_slot_s_approach`: Hover 30–50mm above tray slot $s$.
     - `tray_slot_s_pick`: Contact token in slot $s$ (suction activates).
     - `tray_slot_s_lift`: Lift token cleanly from slot $s$.
   - **Place Sequence (Per Cell 0–8)**:
     - `cell_i_approach`: Hover 30–50mm above target cell $i$.
     - `cell_i_place`: Contact board cell (suction releases).
     - `cell_i_lift`: Retreat pose after placing token.

5. **Live Coordinates & Joint Angles**:
   - The editor displays **Live Joint Angles** (J1–J6 in °) and **Live Coordinates** (X, Y, Z, Rx, Ry, Rz).
   - Waypoints can be saved, inspected, and jogged using either **Angles Mode** or **Coords Mode**.

6. **LED Color Pad & Saved Palette**:
   - Visual color picker and **HEX input** (e.g. `#00E5C8`).
   - Click **💾 Save** to store custom favorite colors in your local palette.
   - Click any saved swatch to apply it instantly to the 45-LED strip.

7. **Test Trajectory**:
   - Select any slot ($0..3$) and cell ($0..8$), then click **▶ Run** to verify the full 2-safe-position pick-and-place cycle before starting actual games.

---

## 🚀 Running the Application

### 1. Launch Server
```bash
python server_ttt.py
```
Server runs locally on `http://0.0.0.0:5000`.

### 2. Start a Game
1. Open `http://localhost:5000` in any web browser.
2. Select **1 Game** (Single Board) or **2 Games** (Dual Board Rail Interleaving).
3. Click **▶ Start Game**.
4. The arm will automatically move to `scan_pose` and the under-rail strip will breathe red (`human_turn`), waiting for the player to place a red token.
5. Once a human move is detected, the robot computes its response, pulses blue (`robot_thinking`), moves the rail and arm (`robot_moving`), places its token, and returns to `scan_pose`.

---

## 🌐 Web Dashboards & APIs

### User Interfaces
- **`http://localhost:5000/`**: Live Game Dashboard (Real-time 3x3 interactive board, live OpenCV video stream, phase step tracker, score tallies, live LED controls, and move logs).
- **`http://localhost:5000/waypoints`**: Waypoint & Hardware Calibration Suite (Joint degree readouts, servo lock/release, 25-key trajectory database, solid color picker, and 11 LED effect testing chips).

### Core REST API Endpoints
- `POST /api/start_game`: `{"board_id": "B1", "mode": 1}`
- `POST /api/reset_game`: Reset active game states
- `POST /api/emergency_stop`: Immediate hardware freeze & alert state
- `POST /api/record_waypoint`: Persist current joint angles to a named key
- `POST /api/jog_waypoint`: Move robot arm to a named waypoint
- `POST /api/test_cell`: Execute single-cell pick-and-place dry run
- `POST /api/led/<effect>`: Trigger named animation on under-rail strip
- `POST /api/led/solid`: `{"r": 0, "g": 229, "b": 200}`
- `POST /api/led/brightness`: `{"value": 180}`
- `GET /api/game_state`: Full JSON snapshot of active games and scores
- `GET /video_feed`: MJPEG multipart live camera stream

---

## 🔧 Troubleshooting & FAQ

#### 1. "Arm Disconnected" in Dashboard
- Ensure USB serial cable is connected. Check `arm_config.json` for correct port (e.g. `COM3` on Windows or `/dev/ttyACM1` on Linux).
- Click **Reconnect** in settings or restart `server_ttt.py`.

#### 2. "Rail Disconnected"
- Ensure ESP32 is powered and flashed with `TTT_StepperControl.ino`.
- The server auto-detects ports streaming `STATUS:` strings.

#### 3. Vision Detection Tuning
- If tokens are not recognized under ambient light, open `config_ttt.py` and adjust:
  - `RED_HSV_LOWER1` / `RED_HSV_UPPER1`
  - `BLUE_HSV_LOWER` / `BLUE_HSV_UPPER`
  - `TOKEN_PIXEL_RATIO` (default: 0.10)

#### 4. Vacuum Suction Drops Token Early
- Increase `vacuum_on_delay_ms` in `ttt_settings.json` (e.g. from 1500 to 2000 ms) to ensure full negative pressure seal before lifting.

---

## 📜 License & Acknowledgments
Developed for the **Mouser RoadShow Interactive Robotic Showcase**. Built with Flask, Socket.IO, OpenCV, pymycobot, and FastLED.
