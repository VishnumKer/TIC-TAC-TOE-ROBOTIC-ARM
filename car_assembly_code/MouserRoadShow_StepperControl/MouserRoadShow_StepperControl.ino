/**
 * Mouser RoadShow - Professional Linear Rail Controller
 * Hardware: ESP32-S3 + TMC5160 (SPI Mode)
 *
 * FIXES APPLIED vs original:
 *   1. CS pin pulled HIGH before SPI.begin() to prevent garbage init
 *   2. VMAX/AMAX/DMAX scaled correctly for 256 microsteps
 *   3. VSTOP raised to safe value (was 10, now 200)
 *   4. VMAX correctly restored after homing (was wrong value)
 *   5. Homing guards against mid-move re-trigger
 *   6. moveToIndex() guards against calling before homing
 *   7. Homing uses large negative XTARGET (positioning mode) — direction-agnostic,
 *      no longer depends on shaft()/RAMPMODE velocity mode at all
 *   8. Live speed control via web dashboard slider + /speed?v=N endpoint
 *   9. Relay-controlled 12V indicators:
 *        GPIO 14 — Red  LED  relay  (constant ON when object NOT detected)
 *        GPIO 17 — Buzzer    relay  (beeps every 2 s when object NOT detected)
 *        GPIO 15 — Green LED relay  (ON when object is detected / system ready)
 *      "Object detected" = system is homed and at a valid preset position.
 *      Relays are active-HIGH (HIGH = relay energised = 12V device ON).
 */

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <WiFi.h>
#include <WebServer.h>

// ─── WiFi Credentials ──────────────────────────────────────────────────────
const char *WIFI_SSID = "MyCobotWiFi2.4G";
const char *WIFI_PASS = "mycobot123";

// ─── PCB Pin Map (Mouser RoadShow Board) ───────────────────────────────────
#define PIN_TMC_CS   10
#define PIN_TMC_SCK  12
#define PIN_TMC_MOSI 11
#define PIN_TMC_MISO 13
#define PIN_TMC_EN   9
#define PIN_BUTTON_S2 41  // Home switch (active LOW)

// ─── Relay / Indicator Pins (12V via relay, active HIGH) ───────────────────
#define PIN_RELAY_RED    14  // Red  LED  relay
#define PIN_RELAY_BUZZER 17  // Buzzer    relay
#define PIN_RELAY_GREEN  15  // Green LED relay

// ─── Sensor Pins ───────────────────────────────────────────────────────────
#define PIN_NTC_TEMP    1   // ADC — NTC thermistor (10kΩ B=3950, NTC to 3.3V, 10k pull-down to GND)
#define PIN_DOOR_SENSOR 2   // Digital — TMR 54140 (LOW = magnet present = door closed)

// ─── NTC Config ────────────────────────────────────────────────────────────
#define NTC_BETA        3950.0f   // Beta coefficient
#define NTC_NOMINAL    10000.0f   // Resistance at 25°C (10kΩ)
#define NTC_PULLUP     10000.0f   // Pull-up resistor value (10kΩ to 3.3V)
#define TEMP_NOMINAL    298.15f   // 25°C in Kelvin
#define NTC_SAMPLES         16   // Oversample count — averages out ADC noise

// ─── Indicator Timing ──────────────────────────────────────────────────────
#define BUZZER_INTERVAL_MS 2000UL  // Buzzer beep period (ms)
#define BUZZER_BEEP_MS      150UL  // Relay ON duration per beep (ms)

// ─── Door Alert Timing ─────────────────────────────────────────────────────
#define DOOR_ALERT_TIMEOUT_MS       0UL  // Red ON immediately when door opens (0 = instant)
#define DOOR_ALERT_ESCALATE_MS   5000UL  // After this long open → start blinking + beeping
#define DOOR_ALERT_BLINK_MS       1000UL  // Red LED blink period (1 Hz)
#define DOOR_ALERT_BEEP_INTERVAL  5000UL  // Double-beep cycle period
#define DOOR_ALERT_BEEP_MS         150UL  // Single beep ON duration
#define DOOR_ALERT_BEEP_GAP_MS     200UL  // Gap between the two beeps in a double-beep

// ─── TMC5160 Config ────────────────────────────────────────────────────────
#define TMC_R_SENSE     0.075f
#define MOTOR_RMS_MA    800       // Motor RMS current in mA

// ─── Motion Constants ──────────────────────────────────────────────────────
// TMC5160 at 256 microsteps = 51200 steps/rev
// Pulley: 20 teeth × 2mm belt pitch = 40mm/rev = 4cm/rev
#define STEPS_PER_REV   51200
#define PULLEY_TEETH    20
#define BELT_PITCH_MM   2
const float STEPS_PER_CM = (float)STEPS_PER_REV / ((PULLEY_TEETH * BELT_PITCH_MM) / 10.0);
// STEPS_PER_CM = 51200 / 4.0 = 12800 steps/cm

const float MAX_DISTANCE_CM = 52.0;

// ─── Ramp Parameters (tuned for 256 microsteps) ────────────────────────────
// At 256 microsteps, internal velocity unit ≈ 0.715 steps/s per count.
// VMAX=500000 ≈ 357,500 steps/s ≈ ~7 rev/s ≈ ~420 RPM — smooth, fast
// HOMING_VMAX is a slow creep to reliably trigger the limit switch
#define VMAX_POSITIONING  500000UL
#define AMAX_VALUE         50000UL
#define DMAX_VALUE         50000UL
#define VSTART_VALUE           10UL
#define VSTOP_VALUE           200UL   // Must be > VSTART; was 10 (too low)
#define VMAX_HOMING         30000UL   // Slow creep for homing

// ─── Presets ──────────────────────────────────────────────────────────────
struct Preset { String name; float cm; };
Preset presets[] = {
  {"HOME", 0.0},
  {"P1",   9.0},
  {"P2",  23.0},
  {"P3",  28.0},
  {"P4",  34.0},
  {"P5",  47.0}
};
const int NUM_PRESETS = 6;

// ─── Global Objects ────────────────────────────────────────────────────────
TMC5160Stepper tmc(PIN_TMC_CS, TMC_R_SENSE);
WebServer server(80);

// ─── Global State ──────────────────────────────────────────────────────────
String currentTargetName = "IDLE";
bool isHomed         = false;
bool triggerHoming   = true;             // Auto-home on boot
uint32_t currentVMAX = VMAX_POSITIONING; // Live speed, updated by /speed endpoint

// ─── Indicator State ───────────────────────────────────────────────────────
unsigned long lastBuzzerToggleMs = 0;    // Tracks last buzzer relay toggle time
bool          buzzerRelayOn      = false; // Current buzzer relay state

// ─── External Indicator Override (set by Python via /indicator or Serial) ──
// Modes: 0 = auto, 1 = alert (red+buzzer), 2 = green, 3 = off,
//        4 = play_started (green ON immediately), 5 = door_alert (internal)
int  indicatorOverrideMode = 0;
unsigned long overrideBuzzerToggleMs = 0;
bool          overrideBuzzerOn       = false;

// ─── Serial Command State ──────────────────────────────────────────────────
String        serialCmdBuffer        = "";   // Accumulates chars until newline
unsigned long lastSerialStatusMs     = 0;    // Tracks last STATUS: broadcast

// ─── Door Alert State ──────────────────────────────────────────────────────
bool          doorWasOpen         = false;
unsigned long doorOpenSinceMs     = 0;
int           doorAlertPriorMode  = 0;    // mode to restore when door closes
int           doorAlertPhase      = 0;    // 0 = solid red, 1 = blink+beep
unsigned long doorAlertLastBlink  = 0;
bool          doorAlertRedOn      = false;
unsigned long doorAlertLastBeep   = 0;
int           doorAlertBeepCount  = 0;    // 0 = waiting, 1 = first done, 2 = both done
bool          doorAlertBeepOn     = false;
unsigned long doorAlertBeepOnMs   = 0;

// ─── Web Dashboard HTML ────────────────────────────────────────────────────
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Mouser Rail Controller</title>
<style>
  body{background:#0b0b14;color:#e0e0ff;font-family:sans-serif;display:flex;flex-direction:column;align-items:center;padding:20px}
  .card{background:#16162a;border:1px solid #2a2a4a;border-radius:12px;padding:20px;width:100%;max-width:400px;margin-bottom:15px;box-shadow:0 8px 32px #0006}
  h1{background:linear-gradient(135deg,#f76ae2,#4fc3f7);-webkit-background-clip:text;-webkit-text-fill-color:transparent;font-size:1.5rem}
  .btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  button{padding:12px;border:none;border-radius:8px;background:#2a2a4a;color:#fff;font-weight:bold;cursor:pointer;transition:0.2s}
  button:hover{background:#3a3a6a}
  button:disabled{opacity:0.35;cursor:not-allowed}
  .btn-home{background:#3b82f6}
  .btn-stop{background:#ef4444;grid-column:span 2}
  .status{display:flex;justify-content:space-between;font-size:0.9rem;color:#a0a0c0;margin-bottom:10px}
  .val{color:#4fc3f7;font-weight:bold}
  .warn{color:#f59e0b;font-size:0.8rem;text-align:center;margin-top:4px}
  .speed-row{display:flex;align-items:center;gap:10px;margin-top:6px}
  .speed-row label{font-size:0.85rem;color:#a0a0c0;white-space:nowrap}
  .speed-row input[type=range]{flex:1;accent-color:#4fc3f7}
  .speed-val{color:#4fc3f7;font-weight:bold;min-width:44px;text-align:right;font-size:0.9rem}
  .speed-badges{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
  .badge{padding:4px 10px;border-radius:6px;background:#2a2a4a;color:#a0a0c0;font-size:0.78rem;cursor:pointer;border:1px solid #3a3a5a}
  .badge:hover{background:#3a3a6a;color:#fff}
</style>
</head><body>
  <h1>🚀 MOUSER RAIL</h1>

  <div class="card">
    <div class="status"><span>Position:</span><span class="val" id="pos">0.0 cm</span></div>
    <div class="status"><span>Status:</span><span class="val" id="stat">IDLE</span></div>
    <div class="status"><span>Homed:</span><span class="val" id="home">NO</span></div>
    <div class="status"><span>Speed:</span><span class="val" id="spd-disp">--</span></div>
  </div>

  <div class="card">
    <div class="status"><span>Speed Control</span></div>
    <div class="speed-row">
      <label>Slow</label>
      <input type="range" id="spd" min="1" max="100" value="50" oninput="updateSpeedLabel()">
      <label>Fast</label>
      <span class="speed-val" id="spd-label">50%</span>
    </div>
    <div class="speed-badges">
      <span class="badge" onclick="setSpeedPct(20)">20% — Slow</span>
      <span class="badge" onclick="setSpeedPct(50)">50% — Normal</span>
      <span class="badge" onclick="setSpeedPct(80)">80% — Fast</span>
      <span class="badge" onclick="setSpeedPct(100)">100% — Max</span>
    </div>
    <button style="width:100%;margin-top:12px" onclick="applySpeed()">Apply Speed</button>
  </div>

  <div class="card btn-grid">
    <button class="btn-home" onclick="cmd('home')">HOME</button>
    <button onclick="cmd('p1')" id="bp1">P1 (9cm)</button>
    <button onclick="cmd('p2')" id="bp2">P2 (23cm)</button>
    <button onclick="cmd('p3')" id="bp3">P3 (28cm)</button>
    <button onclick="cmd('p4')" id="bp4">P4 (34cm)</button>
    <button onclick="cmd('p5')" id="bp5">P5 (47cm)</button>
    <button class="btn-stop" onclick="cmd('stop')">🛑 STOP RAIL</button>
  </div>
  <p class="warn" id="warn"></p>

<script>
  // VMAX range: 50000 (min) to 500000 (max) — maps to slider 1–100%
  const VMAX_MIN = 50000, VMAX_MAX = 500000;

  function pctToVmax(pct){ return Math.round(VMAX_MIN + (VMAX_MAX - VMAX_MIN) * pct / 100); }
  function vmaxToPct(v)  { return Math.round((v - VMAX_MIN) / (VMAX_MAX - VMAX_MIN) * 100); }

  function updateSpeedLabel(){
    document.getElementById('spd-label').innerText = document.getElementById('spd').value + '%';
  }
  function setSpeedPct(pct){
    document.getElementById('spd').value = pct;
    updateSpeedLabel();
  }
  function applySpeed(){
    const pct  = parseInt(document.getElementById('spd').value);
    const vmax = pctToVmax(pct);
    fetch('/speed?v=' + vmax);
  }
  function cmd(c){ fetch('/'+c); }

  setInterval(async ()=>{
    try{
      const r = await fetch('/status');
      const d = await r.json();
      document.getElementById('pos').innerText  = d.absCm.toFixed(1) + ' cm';
      document.getElementById('stat').innerText = d.running ? 'MOVING...' : d.point;
      document.getElementById('home').innerText = d.homed ? 'YES' : (d.homing ? 'HOMING...' : 'NO');
      document.getElementById('spd-disp').innerText = vmaxToPct(d.vmax) + '% (' + d.vmax + ')';
      ['bp1','bp2','bp3','bp4','bp5'].forEach(id=>{
        document.getElementById(id).disabled = !d.homed;
      });
      document.getElementById('warn').innerText = d.homed ? '' : 'Preset moves locked until homed.';
    }catch(e){}
  }, 400);
</script>
</body></html>
)HTML";

// ─── Motor Control ─────────────────────────────────────────────────────────
void moveToIndex(int index) {
  // FIX 6: Guard — do not move to preset before homing
  if (!isHomed) {
    Serial.println("[WARN] Preset move requested before homing — ignored.");
    return;
  }
  if (index < 0 || index >= NUM_PRESETS) return;

  float targetCm    = presets[index].cm;
  int32_t targetPos = (int32_t)(targetCm * STEPS_PER_CM);

  // Clamp to physical travel limits
  int32_t maxSteps = (int32_t)(MAX_DISTANCE_CM * STEPS_PER_CM);
  targetPos = constrain(targetPos, 0, maxSteps);

  currentTargetName = presets[index].name;
  tmc.RAMPMODE(0);          // Positioning mode
  tmc.VMAX(currentVMAX);   // Use live speed setting
  tmc.XTARGET(targetPos);

  Serial.printf("[MOVE] → %s (%.1f cm = %d steps)\n",
                presets[index].name.c_str(), targetCm, targetPos);
}

void stopMotor() {
  // Stop by setting XTARGET = current position
  tmc.XTARGET(tmc.XACTUAL());
  currentTargetName = "STOPPED";
  Serial.println("[STOP] Motor halted.");
}

float readTemperatureC() {
  long sum = 0;
  for (int i = 0; i < NTC_SAMPLES; i++) {
    sum += analogRead(PIN_NTC_TEMP);
    delayMicroseconds(200);  // Brief gap between samples
  }
  int raw = (int)(sum / NTC_SAMPLES);

  if (raw <= 0 || raw >= 4095) {
    return -99.0f;  // Open circuit or shorted
  }

  // NTC to GND, pull-up (10k) to 3.3V
  // R_ntc = R_pullup * raw / (4095 - raw)
  float resistance = NTC_PULLUP * (4095.0f - (float)raw) / (float)raw;

  // Steinhart-Hart B-parameter equation
  float tempK = 1.0f / (1.0f / TEMP_NOMINAL + logf(resistance / NTC_NOMINAL) / NTC_BETA);
  return tempK - 273.15f;
}

bool isDoorClosed() {
  return digitalRead(PIN_DOOR_SENSOR) == LOW;  // 54140: LOW = magnet present = door closed
}

// ─── Status Handler ────────────────────────────────────────────────────────
void handleStatus() {
  int32_t currentSteps = tmc.XACTUAL();
  bool isMoving = (tmc.XACTUAL() != tmc.XTARGET());
  String json = "{";
  json += "\"running\":"  + String(isMoving      ? "true" : "false");
  json += ",\"homed\":"   + String(isHomed        ? "true" : "false");
  json += ",\"homing\":"  + String(triggerHoming  ? "true" : "false");
  json += ",\"point\":\"" + currentTargetName + "\"";
  json += ",\"absCm\":"   + String((float)currentSteps / STEPS_PER_CM, 2);
  json += ",\"steps\":"   + String(currentSteps);
  json += ",\"vmax\":"    + String(currentVMAX);
  json += ",\"tempC\":"   + String(readTemperatureC(), 1);
  json += ",\"doorOpen\":" + String(isDoorClosed() ? "false" : "true");
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ─── TMC5160 Initialization ────────────────────────────────────────────────
void initTMC() {
  // FIX 1: CS HIGH before any SPI activity to prevent garbage init
  pinMode(PIN_TMC_CS, OUTPUT);
  digitalWrite(PIN_TMC_CS, HIGH);

  // Disable driver during config
  pinMode(PIN_TMC_EN, OUTPUT);
  digitalWrite(PIN_TMC_EN, HIGH);

  delay(20);  // Let power rails settle

  SPI.begin(PIN_TMC_SCK, PIN_TMC_MISO, PIN_TMC_MOSI);
  tmc.begin();

  // Basic driver config
  tmc.toff(4);
  tmc.rms_current(MOTOR_RMS_MA);
  tmc.microsteps(256);

  // shaft(false): positive XTARGET = motor moves in its natural forward direction.
  // Homing now uses a large negative XTARGET (positioning mode) so this setting
  // no longer affects homing direction at all — only preset moves P1..P5.
  // If presets move LEFT instead of RIGHT after homing, flip this to shaft(true).
  tmc.shaft(true);

  // FIX 2: Ramp parameters scaled for 256 microsteps
  tmc.RAMPMODE(0);                   // Positioning mode
  tmc.VSTART(VSTART_VALUE);
  tmc.VSTOP(VSTOP_VALUE);            // FIX 3: Was 10 — too low, causes faults
  tmc.VMAX(VMAX_POSITIONING);
  tmc.AMAX(AMAX_VALUE);
  tmc.DMAX(DMAX_VALUE);

  // A1/D1/V1 for two-phase ramp (optional but smoother)
  tmc.v1(VMAX_POSITIONING / 2);
  tmc.a1(AMAX_VALUE * 2);
  tmc.d1(DMAX_VALUE * 2);

  // Enable driver
  digitalWrite(PIN_TMC_EN, LOW);

  Serial.println("[TMC] Driver initialised OK.");
  Serial.printf("[TMC] STEPS_PER_CM = %.1f\n", STEPS_PER_CM);
}

// ─── Homing Sequence (non-blocking) ───────────────────────────────────────
void runHomingSequence() {
  static bool homingStarted = false;

  if (!homingStarted) {
    // Stop any in-progress move first
    if (tmc.XACTUAL() != tmc.XTARGET()) {
      tmc.XTARGET(tmc.XACTUAL());
      delay(50);
    }

    Serial.println("[HOME] Sequence started — seeking switch...");

    // Use VELOCITY mode (RAMPMODE 2) to move negative toward the switch
    // matches the logic in stepper_control_v2.ino
    tmc.RAMPMODE(2);
    tmc.VMAX(VMAX_HOMING);

    homingStarted = true;
    currentTargetName = "HOMING";
  }

  // Check home switch (active LOW)
  if (digitalRead(PIN_BUTTON_S2) == LOW) {
    // Switch triggered — immediately stop and switch back to positioning mode
    tmc.RAMPMODE(0);
    tmc.VMAX(0);
    delay(50);
    tmc.XACTUAL(0);                       // Zero the position register
    tmc.XTARGET(0);
    tmc.VMAX(currentVMAX);               // Restore user speed for presets

    isHomed        = true;
    triggerHoming  = false;
    homingStarted  = false;
    currentTargetName = "HOME";

    Serial.println("[HOME] Success! Position zeroed.");
    Serial.printf("[HOME] STEPS_PER_CM=%.1f  max travel=%d steps\n",
                  STEPS_PER_CM, (int32_t)(MAX_DISTANCE_CM * STEPS_PER_CM));
  }
}

// ─── Speed Handler ─────────────────────────────────────────────────────────
void handleSpeed() {
  if (!server.hasArg("v")) {
    server.send(400, "text/plain", "Missing ?v= parameter");
    return;
  }
  uint32_t requested = (uint32_t)server.arg("v").toInt();
  // Clamp to safe range
  requested = constrain(requested, 50000UL, VMAX_POSITIONING);
  currentVMAX = requested;

  // Apply immediately if not homing
  if (!triggerHoming) {
    tmc.VMAX(currentVMAX);
  }

  Serial.printf("[SPEED] VMAX set to %u\n", currentVMAX);
  server.send(200, "text/plain", "OK");
}

// ─── Indicator Update (call every loop) ────────────────────────────────────
// "Object in place" = system is homed and not currently homing/moving to home.
// If NOT in place: Red LED ON constant, Buzzer beeps every BUZZER_INTERVAL_MS.
// If IN place:     Green LED ON, Red OFF, Buzzer OFF.
// If indicatorOverrideMode != 0: Python has taken control; skip the auto logic.
void updateIndicators() {
  unsigned long now = millis();

  // ── External override from Python (/indicator?mode=...) ──────────────────
  if (indicatorOverrideMode == 1) {
    // ALERT: Red constant ON, Green OFF, Buzzer beeps every 2 s
    digitalWrite(PIN_RELAY_RED,   HIGH);
    digitalWrite(PIN_RELAY_GREEN, LOW);
    if (!overrideBuzzerOn) {
      if (now - overrideBuzzerToggleMs >= BUZZER_INTERVAL_MS) {
        digitalWrite(PIN_RELAY_BUZZER, HIGH);
        overrideBuzzerOn       = true;
        overrideBuzzerToggleMs = now;
      }
    } else {
      if (now - overrideBuzzerToggleMs >= BUZZER_BEEP_MS) {
        digitalWrite(PIN_RELAY_BUZZER, LOW);
        overrideBuzzerOn       = false;
        overrideBuzzerToggleMs = now;
      }
    }
    return;  // Skip auto logic
  }

  if (indicatorOverrideMode == 2) {
    // GREEN: Object placed — all clear
    digitalWrite(PIN_RELAY_RED,    LOW);
    digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN,  HIGH);
    overrideBuzzerOn = false;
    return;  // Skip auto logic
  }

  if (indicatorOverrideMode == 3) {
    // ALL OFF: Idle / reset state
    digitalWrite(PIN_RELAY_RED,    LOW);
    digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN,  LOW);
    overrideBuzzerOn = false;
    return;  // Skip auto logic
  }

  if (indicatorOverrideMode == 4) {
    // PLAY STARTED: Green ON immediately, before object detection
    digitalWrite(PIN_RELAY_RED,    LOW);
    digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN,  HIGH);
    overrideBuzzerOn = false;
    return;
  }

  if (indicatorOverrideMode == 5) {
    digitalWrite(PIN_RELAY_GREEN, LOW);

    if (doorAlertPhase == 0) {
      // Phase 0: solid red ON, no beep — door just opened
      digitalWrite(PIN_RELAY_RED,    HIGH);
      digitalWrite(PIN_RELAY_BUZZER, LOW);
    } else {
      // Phase 1: red blinks 1 Hz + double-beep every 5s
      if (now - doorAlertLastBlink >= DOOR_ALERT_BLINK_MS) {
        doorAlertRedOn = !doorAlertRedOn;
        digitalWrite(PIN_RELAY_RED, doorAlertRedOn ? HIGH : LOW);
        doorAlertLastBlink = now;
      }

      // Double-beep state machine
      if (doorAlertBeepCount == 0) {
        if (now - doorAlertLastBeep >= DOOR_ALERT_BEEP_INTERVAL) {
          digitalWrite(PIN_RELAY_BUZZER, HIGH);
          doorAlertBeepOn    = true;
          doorAlertBeepOnMs  = now;
          doorAlertBeepCount = 1;
        }
      } else if (doorAlertBeepCount == 1 && doorAlertBeepOn) {
        if (now - doorAlertBeepOnMs >= DOOR_ALERT_BEEP_MS) {
          digitalWrite(PIN_RELAY_BUZZER, LOW);
          doorAlertBeepOn   = false;
          doorAlertBeepOnMs = now;
        }
      } else if (doorAlertBeepCount == 1 && !doorAlertBeepOn) {
        if (now - doorAlertBeepOnMs >= DOOR_ALERT_BEEP_GAP_MS) {
          digitalWrite(PIN_RELAY_BUZZER, HIGH);
          doorAlertBeepOn    = true;
          doorAlertBeepOnMs  = now;
          doorAlertBeepCount = 2;
        }
      } else if (doorAlertBeepCount == 2 && doorAlertBeepOn) {
        if (now - doorAlertBeepOnMs >= DOOR_ALERT_BEEP_MS) {
          digitalWrite(PIN_RELAY_BUZZER, LOW);
          doorAlertBeepOn    = false;
          doorAlertLastBeep  = now;
          doorAlertBeepCount = 0;
        }
      }
    }
    return;
  }

  // ── Auto logic (mode 0) — driven by homing state ─────────────────────────
  bool objectInPlace = isHomed && !triggerHoming;

  if (objectInPlace) {
    // ── Object detected ──────────────────────────────────────────────────
    digitalWrite(PIN_RELAY_RED,    LOW);   // Red  LED OFF
    digitalWrite(PIN_RELAY_BUZZER, LOW);   // Buzzer  OFF
    digitalWrite(PIN_RELAY_GREEN,  HIGH);  // Green LED ON
    buzzerRelayOn      = false;
    lastBuzzerToggleMs = now;
  } else {
    // ── Object NOT detected ──────────────────────────────────────────────
    digitalWrite(PIN_RELAY_RED,   HIGH);   // Red LED ON (constant)
    digitalWrite(PIN_RELAY_GREEN, LOW);    // Green LED OFF

    // Non-blocking buzzer beep
    if (!buzzerRelayOn) {
      if (now - lastBuzzerToggleMs >= BUZZER_INTERVAL_MS) {
        digitalWrite(PIN_RELAY_BUZZER, HIGH);
        buzzerRelayOn      = true;
        lastBuzzerToggleMs = now;
      }
    } else {
      if (now - lastBuzzerToggleMs >= BUZZER_BEEP_MS) {
        digitalWrite(PIN_RELAY_BUZZER, LOW);
        buzzerRelayOn      = false;
        lastBuzzerToggleMs = now;
      }
    }
  }
}

// ─── Indicator Mode Setter (shared by HTTP + Serial) ─────────────────────
void applyIndicatorMode(const String& mode) {
  if (mode == "alert") {
    indicatorOverrideMode  = 1;
    overrideBuzzerOn       = false;
    overrideBuzzerToggleMs = 0;
  } else if (mode == "green") {
    indicatorOverrideMode = 2;
  } else if (mode == "off") {
    indicatorOverrideMode = 3;
  } else if (mode == "play") {
    indicatorOverrideMode = 4;
  } else {
    indicatorOverrideMode = 0;  // auto
  }
  Serial.printf("[IND] Mode: %s\n", mode.c_str());
}

// ─── External Indicator Override Handler (HTTP) ────────────────────────────
void handleIndicator() {
  applyIndicatorMode(server.arg("mode"));
  server.send(200, "text/plain", "OK");
}

// ─── Serial Status Broadcast (every 500 ms) ────────────────────────────────
// Python reads lines prefixed with STATUS: and parses the JSON.
void broadcastSerialStatus() {
  if (millis() - lastSerialStatusMs < 500UL) return;
  lastSerialStatusMs = millis();
  int32_t steps = tmc.XACTUAL();
  bool moving   = (steps != tmc.XTARGET());
  Serial.print("STATUS:{\"running\":");
  Serial.print(moving ? "true" : "false");
  Serial.print(",\"homed\":");
  Serial.print(isHomed ? "true" : "false");
  Serial.print(",\"homing\":");
  Serial.print(triggerHoming ? "true" : "false");
  Serial.print(",\"point\":\"");
  Serial.print(currentTargetName);
  Serial.print("\",\"absCm\":");
  Serial.print((float)steps / STEPS_PER_CM, 2);
  Serial.print(",\"vmax\":");
  Serial.print(currentVMAX);
  Serial.print(",\"tempC\":");
  Serial.print(readTemperatureC(), 1);
  Serial.print(",\"doorOpen\":");
  Serial.print(isDoorClosed() ? "false" : "true");
  Serial.println("}");
}

// ─── Serial Command Handler ────────────────────────────────────────────────
// Commands (newline-terminated) sent by the Jetson Nano over USB serial:
//   MOVE:P1 .. MOVE:P5  → preset move
//   MOVE:HOME           → home
//   STOP                → emergency stop
//   SPEED:vmax_int      → set speed (raw VMAX count)
//   INDICATOR:mode      → same modes as HTTP /indicator?mode=
void processSerialCommand(const String& cmd) {
  if (cmd.startsWith("MOVE:")) {
    String target = cmd.substring(5);
    target.toUpperCase();
    if (target == "HOME") {
      if (isHomed) moveToIndex(0); else triggerHoming = true;
    } else {
      bool found = false;
      for (int i = 1; i < NUM_PRESETS; i++) {
        if (presets[i].name == target) { moveToIndex(i); found = true; break; }
      }
      if (!found) Serial.printf("[SERIAL] Unknown preset: %s\n", target.c_str());
    }
  } else if (cmd == "STOP") {
    stopMotor();
    triggerHoming = false;
  } else if (cmd.startsWith("SPEED:")) {
    uint32_t v = (uint32_t)cmd.substring(6).toInt();
    v = constrain(v, 50000UL, VMAX_POSITIONING);
    currentVMAX = v;
    if (!triggerHoming) tmc.VMAX(currentVMAX);
    Serial.printf("[SERIAL] Speed set to %u\n", currentVMAX);
  } else if (cmd.startsWith("INDICATOR:")) {
    applyIndicatorMode(cmd.substring(10));
  } else {
    Serial.printf("[SERIAL] Unknown cmd: %s\n", cmd.c_str());
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialCmdBuffer.trim();
      if (serialCmdBuffer.length() > 0) {
        processSerialCommand(serialCmdBuffer);
        serialCmdBuffer = "";
      }
    } else {
      serialCmdBuffer += c;
    }
  }
}

// ─── Door Alert Monitor (call every loop) ─────────────────────────────────
// Watches PIN_DOOR_SENSOR. If door stays open >= DOOR_ALERT_TIMEOUT_MS,
// forces indicatorOverrideMode=5 (red blink + double-beep). Restores prior
// mode when door closes. Ignores external Python overrides (modes 1-3) so
// that deliberate Python states are not clobbered.
void updateDoorAlert() {
  bool doorOpen = !isDoorClosed();
  unsigned long now = millis();

  if (doorOpen) {
    if (!doorWasOpen) {
      // Door just opened — enter phase 0 immediately (solid red)
      doorOpenSinceMs   = now;
      doorWasOpen       = true;
      if (indicatorOverrideMode != 1 &&
          indicatorOverrideMode != 2 &&
          indicatorOverrideMode != 3) {
        doorAlertPriorMode  = indicatorOverrideMode;
        indicatorOverrideMode = 5;
        doorAlertPhase      = 0;
        doorAlertBeepCount  = 0;
        doorAlertBeepOn     = false;
        doorAlertRedOn      = false;
        Serial.println("[DOOR] Alert phase 0 — solid red");
      }
    }
    // Escalate to phase 1 (blink + beep) after 5s
    if (indicatorOverrideMode == 5 && doorAlertPhase == 0) {
      if (now - doorOpenSinceMs >= DOOR_ALERT_ESCALATE_MS) {
        doorAlertPhase     = 1;
        doorAlertLastBlink = now;
        doorAlertLastBeep  = now;  // first double-beep fires after 5s
        doorAlertBeepCount = 0;
        doorAlertRedOn     = true;
        Serial.println("[DOOR] Alert phase 1 — blink + beep");
      }
    }
  } else {
    if (doorWasOpen) {
      // Door just closed — clear alert
      doorWasOpen = false;
      if (indicatorOverrideMode == 5) {
        digitalWrite(PIN_RELAY_RED,    LOW);
        digitalWrite(PIN_RELAY_BUZZER, LOW);
        indicatorOverrideMode = doorAlertPriorMode;
        doorAlertPhase        = 0;
        doorAlertBeepOn       = false;
        doorAlertBeepCount    = 0;
        Serial.println("[DOOR] Alert cleared — door closed");
      }
    }
  }
}

// ─── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] Mouser Rail Controller starting...");

  pinMode(PIN_BUTTON_S2, INPUT_PULLUP);

  // Sensor pins
  pinMode(PIN_NTC_TEMP,    INPUT);        // External voltage divider — no internal pull
  analogSetPinAttenuation(PIN_NTC_TEMP, ADC_11db);  // full 0–3.3V ADC range
  pinMode(PIN_DOOR_SENSOR, INPUT_PULLUP); // TMR 54140 push-pull; pullup as safety

  // Relay / indicator pins — default OFF (LOW)
  pinMode(PIN_RELAY_RED,    OUTPUT); digitalWrite(PIN_RELAY_RED,    LOW);
  pinMode(PIN_RELAY_BUZZER, OUTPUT); digitalWrite(PIN_RELAY_BUZZER, LOW);
  pinMode(PIN_RELAY_GREEN,  OUTPUT); digitalWrite(PIN_RELAY_GREEN,  LOW);

  initTMC();

  // WiFi — non-blocking: tries for 10 s then continues without network
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000UL) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Not connected — Serial-only mode.");
  }

  // HTTP routes
  server.on("/",      []() { server.send_P(200, "text/html", PAGE); });
  server.on("/status", handleStatus);

  server.on("/stop",  []() {
    stopMotor();
    triggerHoming = false;
    server.send(200, "text/plain", "OK");
  });

  server.on("/home",  []() {
    if (isHomed) {
      // If already calibrated, just go to the 0 position quickly
      moveToIndex(0);
    } else {
      // If not calibrated, start the slow switch-search sequence
      triggerHoming = true;
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/speed", handleSpeed);
  server.on("/indicator", handleIndicator);  // External indicator override from Python
  server.on("/p1", []() { moveToIndex(1); server.send(200, "text/plain", "OK"); });
  server.on("/p2", []() { moveToIndex(2); server.send(200, "text/plain", "OK"); });
  server.on("/p3", []() { moveToIndex(3); server.send(200, "text/plain", "OK"); });
  server.on("/p4", []() { moveToIndex(4); server.send(200, "text/plain", "OK"); });
  server.on("/p5", []() { moveToIndex(5); server.send(200, "text/plain", "OK"); });

  server.begin();
  Serial.println("[HTTP] Server started. Auto-homing on boot...");
}

// ─── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  // Handle USB serial commands from Jetson Nano
  handleSerialCommands();

  // Broadcast status over USB serial every 500 ms
  broadcastSerialStatus();

  // Run homing state machine if triggered
  if (triggerHoming) {
    runHomingSequence();
  }

  // Emergency stop ONLY if home switch triggered while moving TOWARDS home
  if (!triggerHoming && isHomed) {
    if (digitalRead(PIN_BUTTON_S2) == LOW && tmc.XTARGET() < tmc.XACTUAL()) {
      Serial.println("[SAFETY] Home switch triggered while moving negative — stopping.");
      stopMotor();
    }
  }

  // Monitor door sensor; escalate to alert if open too long
  updateDoorAlert();

  // Update 12V relay indicators every loop cycle (non-blocking)
  updateIndicators();

  yield();
}
