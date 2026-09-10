/**
 * TTT_StepperControl.ino
 * ============================================================
 * Tic Tac Toe Robot — ESP32-S3 Rail + LED Strip Controller
 * ============================================================
 * 100% Offline — Pure USB Serial Mode (No WiFi / No WebServer)
 *
 * Preserved Features:
 *   ✅ TMC5160 stepper motor control (SPI Mode)
 *   ✅ Homing sequence (home switch S2, non-blocking)
 *   ✅ Live speed control (SPEED: serial command)
 *   ✅ Preset position moves (MOVE:BOARD1, MOVE:BOARD2, MOVE:HOME)
 *   ✅ Emergency stop (STOP serial command)
 *   ✅ NTC thermistor temperature reading
 *   ✅ Door/safety sensor monitoring + escalating door alert
 *   ✅ 12V Relay indicators: Red LED, Buzzer, Green LED
 *   ✅ INDICATOR: serial command (alert / green / off / play / auto)
 *   ✅ STATUS: JSON broadcast every 500 ms over USB serial
 *   ✅ FastLED WS2812B strip driver (45 LEDs on GPIO 42)
 *   ✅ LEDRGB:<effect>, LEDSOLID:<R>,<G>,<B>, LEDBRIGHT:<0-255>
 *
 * Serial command set (all newline-terminated, sent by Python server):
 *   MOVE:HOME            → Home sequence / go to 0
 *   MOVE:BOARD1          → Move to Board 1 station (9.0 cm)
 *   MOVE:BOARD2          → Move to Board 2 station (43.0 cm)
 *   STOP                 → Emergency stop
 *   SPEED:<vmax>         → Raw VMAX count (50000 - 500000)
 *   INDICATOR:<mode>     → alert | green | off | play | auto
 *   LEDRGB:<effect>      → idle | scanning | human_turn | robot_thinking |
 *                           robot_moving | win_robot | win_human | draw |
 *                           alert | rainbow | off
 *   LEDSOLID:<R>,<G>,<B> → Solid color across all 45 LEDs (0-255)
 *   LEDBRIGHT:<0-255>    → Master LED brightness
 *
 * STATUS: JSON broadcast (every 500 ms over USB Serial):
 *   STATUS:{"running":bool,"homed":bool,"homing":bool,"point":"...",
 *           "absCm":float,"vmax":int,"tempC":float,"doorOpen":bool,
 *           "ledEffect":"...","ledBrightness":int,"gpioPin":42}
 */

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastLED.h>
#include <Preferences.h>

// ─── Forward Declarations ──────────────────────────────────────────────────
void loadPresets();
void applyIndicatorMode(const String& mode);
void applyLedEffect(const String& effect);
void applyLedSolid(int r, int g, int b);
void updateIndicators();
void updateDoorAlert();
void updateLedStrip();
void moveToIndex(int index);
void stopMotor();
void runHomingSequence();
void initTMC();
float readTemperatureC();
bool isDoorClosed();
void broadcastSerialStatus();
void processSerialCommand(const String& cmd);
void handleSerialCommands();

// ─── PCB Pin Map (Mouser RoadShow Board) ───────────────────────────────────
#define PIN_TMC_CS   10
#define PIN_TMC_SCK  12
#define PIN_TMC_MOSI 11
#define PIN_TMC_MISO 13
#define PIN_TMC_EN    9
#define PIN_BUTTON_S2 41  // Home switch (active LOW)

// ─── Relay / Indicator Pins (12V via relay, active HIGH) ───────────────────
#define PIN_RELAY_RED    14  // Red  LED  relay
#define PIN_RELAY_BUZZER 17  // Buzzer    relay
#define PIN_RELAY_GREEN  15  // Green LED relay

// ─── Sensor Pins ───────────────────────────────────────────────────────────
#define PIN_NTC_TEMP    1   // ADC — NTC thermistor
#define PIN_DOOR_SENSOR 2   // Digital — TMR 54140 (LOW = door closed)

// ─── RGB LED Strip (45 LEDs on GPIO 42) ───────────────────────────────────
#define RGB_LED_PIN    42   // Data pin for WS2812B strip
#define RGB_LED_COUNT  45
#define RGB_LED_TYPE   WS2812B
#define RGB_COLOR_ORDER BRG

// ─── NTC Thermistor Config ─────────────────────────────────────────────────
#define NTC_BETA       3950.0f
#define NTC_NOMINAL   10000.0f
#define NTC_PULLUP    10000.0f
#define TEMP_NOMINAL   298.15f
#define NTC_SAMPLES        16

// ─── Indicator Timing ──────────────────────────────────────────────────────
#define BUZZER_INTERVAL_MS 2000UL
#define BUZZER_BEEP_MS      150UL

// ─── Door Alert Timing ─────────────────────────────────────────────────────
#define DOOR_ALERT_TIMEOUT_MS       0UL
#define DOOR_ALERT_ESCALATE_MS   5000UL
#define DOOR_ALERT_BLINK_MS      1000UL
#define DOOR_ALERT_BEEP_INTERVAL 5000UL
#define DOOR_ALERT_BEEP_MS        150UL
#define DOOR_ALERT_BEEP_GAP_MS    200UL

// ─── TMC5160 Config ────────────────────────────────────────────────────────
#define TMC_R_SENSE     0.075f
#define MOTOR_RMS_MA    800

// ─── Motion Constants ──────────────────────────────────────────────────────
#define STEPS_PER_REV  51200
#define PULLEY_TEETH   20
#define BELT_PITCH_MM  2
const float STEPS_PER_CM = (float)STEPS_PER_REV / ((PULLEY_TEETH * BELT_PITCH_MM) / 10.0);
const float MAX_DISTANCE_CM = 52.0;

// ─── Ramp Parameters ───────────────────────────────────────────────────────
#define VMAX_POSITIONING  500000UL
#define AMAX_VALUE         50000UL
#define DMAX_VALUE         50000UL
#define VSTART_VALUE           10UL
#define VSTOP_VALUE           200UL
#define VMAX_HOMING         30000UL

// ─── Presets ───────────────────────────────────────────────────────────────
struct Preset { String name; float cm; };
Preset presets[] = {
  {"HOME",   0.0},   // Index 0 — Home position
  {"BOARD1", 9.0},   // Index 1 — Board 1 station (left end)
  {"BOARD2", 43.0},  // Index 2 — Board 2 station (right end)
};
const int NUM_PRESETS = 3;

// ─── Global Objects ────────────────────────────────────────────────────────
TMC5160Stepper tmc(PIN_TMC_CS, TMC_R_SENSE);
CRGB leds[RGB_LED_COUNT];

// ─── Motor State ───────────────────────────────────────────────────────────
String   currentTargetName = "IDLE";
bool     isHomed           = false;
bool     triggerHoming     = true;
uint32_t currentVMAX       = VMAX_POSITIONING;

// ─── Indicator State ───────────────────────────────────────────────────────
int           indicatorOverrideMode  = 0;
unsigned long overrideBuzzerToggleMs = 0;
bool          overrideBuzzerOn       = false;
unsigned long lastBuzzerToggleMs     = 0;
bool          buzzerRelayOn          = false;

// ─── Serial Command State ──────────────────────────────────────────────────
String        serialCmdBuffer    = "";
unsigned long lastSerialStatusMs = 0;

// ─── Door Alert State ──────────────────────────────────────────────────────
bool          doorWasOpen        = false;
unsigned long doorOpenSinceMs    = 0;
int           doorAlertPriorMode = 0;
int           doorAlertPhase     = 0;
unsigned long doorAlertLastBlink = 0;
bool          doorAlertRedOn     = false;
unsigned long doorAlertLastBeep  = 0;
int           doorAlertBeepCount = 0;
bool          doorAlertBeepOn    = false;
unsigned long doorAlertBeepOnMs  = 0;

// ─── RGB LED State ─────────────────────────────────────────────────────────
String        currentLedEffect   = "off";
uint8_t       ledBrightness      = 128;
unsigned long ledLastUpdateMs    = 0;
uint8_t       ledAnimIndex       = 0;
bool          ledBreathDir       = true;
uint8_t       ledBreathVal       = 0;

#define LED_OFF             0
#define LED_IDLE            1
#define LED_SCANNING        2
#define LED_HUMAN_TURN      3
#define LED_ROBOT_THINKING  4
#define LED_ROBOT_MOVING    5
#define LED_WIN_ROBOT       6
#define LED_WIN_HUMAN       7
#define LED_DRAW            8
#define LED_ALERT           9
#define LED_RAINBOW         10
#define LED_SOLID           11
#define LED_ROBOT_TURN      12

int     currentLedMode = LED_IDLE;
CRGB    solidColor     = CRGB::White;


// ═════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═════════════════════════════════════════════════════════════════════════

void moveToIndex(int index) {
  if (!isHomed) {
    Serial.println("[WARN] Preset move requested before homing — ignored.");
    return;
  }
  if (index < 0 || index >= NUM_PRESETS) return;

  float   targetCm  = presets[index].cm;
  int32_t targetPos = (int32_t)(targetCm * STEPS_PER_CM);
  int32_t maxSteps  = (int32_t)(MAX_DISTANCE_CM * STEPS_PER_CM);
  targetPos = constrain(targetPos, 0, maxSteps);

  currentTargetName = presets[index].name;
  tmc.RAMPMODE(0);
  tmc.VMAX(currentVMAX);
  tmc.XTARGET(targetPos);

  Serial.printf("[MOVE] -> %s (%.1f cm = %d steps)\n",
                presets[index].name.c_str(), targetCm, targetPos);
}

void stopMotor() {
  tmc.XTARGET(tmc.XACTUAL());
  currentTargetName = "STOPPED";
  Serial.println("[STOP] Motor halted.");
}

float readTemperatureC() {
  long sum = 0;
  for (int i = 0; i < NTC_SAMPLES; i++) {
    sum += analogRead(PIN_NTC_TEMP);
    delayMicroseconds(200);
  }
  int raw = (int)(sum / NTC_SAMPLES);
  if (raw <= 0 || raw >= 4095) return -99.0f;
  float resistance = NTC_PULLUP * (4095.0f - (float)raw) / (float)raw;
  float tempK = 1.0f / (1.0f / TEMP_NOMINAL + logf(resistance / NTC_NOMINAL) / NTC_BETA);
  return tempK - 273.15f;
}

bool isDoorClosed() {
  return digitalRead(PIN_DOOR_SENSOR) == LOW;
}


// ═════════════════════════════════════════════════════════════════════════
// 12V RELAY INDICATOR LOGIC
// ═════════════════════════════════════════════════════════════════════════

void applyIndicatorMode(const String& mode) {
  if      (mode == "alert") { indicatorOverrideMode = 1; overrideBuzzerOn = false; overrideBuzzerToggleMs = 0; }
  else if (mode == "green") { indicatorOverrideMode = 2; }
  else if (mode == "off")   { indicatorOverrideMode = 3; }
  else if (mode == "play")  { indicatorOverrideMode = 4; }
  else                      { indicatorOverrideMode = 0; }
  Serial.printf("[IND] Mode: %s\n", mode.c_str());
}

void updateIndicators() {
  unsigned long now = millis();

  if (indicatorOverrideMode == 1) {
    // ALERT: Red constant ON, Green OFF, Buzzer beeps every 2s
    digitalWrite(PIN_RELAY_RED,   HIGH);
    digitalWrite(PIN_RELAY_GREEN, LOW);
    if (!overrideBuzzerOn) {
      if (now - overrideBuzzerToggleMs >= BUZZER_INTERVAL_MS) {
        digitalWrite(PIN_RELAY_BUZZER, HIGH);
        overrideBuzzerOn = true; overrideBuzzerToggleMs = now;
      }
    } else {
      if (now - overrideBuzzerToggleMs >= BUZZER_BEEP_MS) {
        digitalWrite(PIN_RELAY_BUZZER, LOW);
        overrideBuzzerOn = false; overrideBuzzerToggleMs = now;
      }
    }
    return;
  }
  if (indicatorOverrideMode == 2) {
    digitalWrite(PIN_RELAY_RED, LOW); digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN, HIGH); overrideBuzzerOn = false; return;
  }
  if (indicatorOverrideMode == 3) {
    digitalWrite(PIN_RELAY_RED, LOW); digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN, LOW); overrideBuzzerOn = false; return;
  }
  if (indicatorOverrideMode == 4) {
    digitalWrite(PIN_RELAY_RED, LOW); digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN, HIGH); overrideBuzzerOn = false; return;
  }
  if (indicatorOverrideMode == 5) {
    digitalWrite(PIN_RELAY_GREEN, LOW);
    if (doorAlertPhase == 0) {
      digitalWrite(PIN_RELAY_RED, HIGH); digitalWrite(PIN_RELAY_BUZZER, LOW);
    } else {
      if (now - doorAlertLastBlink >= DOOR_ALERT_BLINK_MS) {
        doorAlertRedOn = !doorAlertRedOn;
        digitalWrite(PIN_RELAY_RED, doorAlertRedOn ? HIGH : LOW);
        doorAlertLastBlink = now;
      }
      if (doorAlertBeepCount == 0) {
        if (now - doorAlertLastBeep >= DOOR_ALERT_BEEP_INTERVAL) {
          digitalWrite(PIN_RELAY_BUZZER, HIGH); doorAlertBeepOn = true;
          doorAlertBeepOnMs = now; doorAlertBeepCount = 1;
        }
      } else if (doorAlertBeepCount == 1 && doorAlertBeepOn) {
        if (now - doorAlertBeepOnMs >= DOOR_ALERT_BEEP_MS) {
          digitalWrite(PIN_RELAY_BUZZER, LOW); doorAlertBeepOn = false; doorAlertBeepOnMs = now;
        }
      } else if (doorAlertBeepCount == 1 && !doorAlertBeepOn) {
        if (now - doorAlertBeepOnMs >= DOOR_ALERT_BEEP_GAP_MS) {
          digitalWrite(PIN_RELAY_BUZZER, HIGH); doorAlertBeepOn = true;
          doorAlertBeepOnMs = now; doorAlertBeepCount = 2;
        }
      } else if (doorAlertBeepCount == 2 && doorAlertBeepOn) {
        if (now - doorAlertBeepOnMs >= DOOR_ALERT_BEEP_MS) {
          digitalWrite(PIN_RELAY_BUZZER, LOW); doorAlertBeepOn = false;
          doorAlertLastBeep = now; doorAlertBeepCount = 0;
        }
      }
    }
    return;
  }
  // Auto mode
  bool objectInPlace = isHomed && !triggerHoming;
  if (objectInPlace) {
    digitalWrite(PIN_RELAY_RED, LOW); digitalWrite(PIN_RELAY_BUZZER, LOW);
    digitalWrite(PIN_RELAY_GREEN, HIGH); buzzerRelayOn = false; lastBuzzerToggleMs = now;
  } else {
    digitalWrite(PIN_RELAY_RED, HIGH); digitalWrite(PIN_RELAY_GREEN, LOW);
    if (!buzzerRelayOn) {
      if (now - lastBuzzerToggleMs >= BUZZER_INTERVAL_MS) {
        digitalWrite(PIN_RELAY_BUZZER, HIGH); buzzerRelayOn = true; lastBuzzerToggleMs = now;
      }
    } else {
      if (now - lastBuzzerToggleMs >= BUZZER_BEEP_MS) {
        digitalWrite(PIN_RELAY_BUZZER, LOW); buzzerRelayOn = false; lastBuzzerToggleMs = now;
      }
    }
  }
}


// ═════════════════════════════════════════════════════════════════════════
// SAFETY DOOR ALERT
// ═════════════════════════════════════════════════════════════════════════

void updateDoorAlert() {
  bool doorOpen = !isDoorClosed();
  unsigned long now = millis();
  if (doorOpen) {
    if (!doorWasOpen) {
      doorOpenSinceMs = now; doorWasOpen = true;
      if (indicatorOverrideMode != 1 && indicatorOverrideMode != 2 && indicatorOverrideMode != 3) {
        doorAlertPriorMode    = indicatorOverrideMode;
        indicatorOverrideMode = 5;
        doorAlertPhase        = 0;
        doorAlertBeepCount    = 0;
        doorAlertBeepOn       = false;
        doorAlertRedOn        = false;
        Serial.println("[DOOR] Alert phase 0 - solid red");
      }
    }
    if (indicatorOverrideMode == 5 && doorAlertPhase == 0) {
      if (now - doorOpenSinceMs >= DOOR_ALERT_ESCALATE_MS) {
        doorAlertPhase = 1; doorAlertLastBlink = now; doorAlertLastBeep = now;
        doorAlertBeepCount = 0; doorAlertRedOn = true;
        Serial.println("[DOOR] Alert phase 1 - blink + beep");
      }
    }
  } else {
    if (doorWasOpen) {
      doorWasOpen = false;
      if (indicatorOverrideMode == 5) {
        digitalWrite(PIN_RELAY_RED, LOW); digitalWrite(PIN_RELAY_BUZZER, LOW);
        indicatorOverrideMode = doorAlertPriorMode;
        doorAlertPhase = 0; doorAlertBeepOn = false; doorAlertBeepCount = 0;
        Serial.println("[DOOR] Alert cleared - door closed");
      }
    }
  }
}


// ═════════════════════════════════════════════════════════════════════════
// RGB LED STRIP (WS2812B)
// ═════════════════════════════════════════════════════════════════════════

void applyLedEffect(const String& effect) {
  currentLedEffect = effect;
  if      (effect == "off")            currentLedMode = LED_OFF;
  else if (effect == "idle")           currentLedMode = LED_IDLE;
  else if (effect == "scanning")       currentLedMode = LED_SCANNING;
  else if (effect == "human_turn")     currentLedMode = LED_HUMAN_TURN;
  else if (effect == "robot_turn")     currentLedMode = LED_ROBOT_TURN;
  else if (effect == "robot_thinking") currentLedMode = LED_ROBOT_THINKING;
  else if (effect == "robot_moving")   currentLedMode = LED_ROBOT_MOVING;
  else if (effect == "win_robot")      currentLedMode = LED_WIN_ROBOT;
  else if (effect == "win_human")      currentLedMode = LED_WIN_HUMAN;
  else if (effect == "draw")           currentLedMode = LED_DRAW;
  else if (effect == "alert")          currentLedMode = LED_ALERT;
  else if (effect == "rainbow")        currentLedMode = LED_RAINBOW;
  else                                 currentLedMode = LED_OFF;
  ledAnimIndex   = 0;
  ledBreathVal   = 0;
  ledBreathDir   = true;
  Serial.printf("[LED] Effect: %s\n", effect.c_str());
}

void applyLedSolid(int r, int g, int b) {
  solidColor       = CRGB(r, g, b);
  currentLedMode   = LED_SOLID;
  currentLedEffect = "solid";
  fill_solid(leds, RGB_LED_COUNT, solidColor);
  FastLED.show();
  Serial.printf("[LED] Solid rgb(%d,%d,%d)\n", r, g, b);
}

void updateLedStrip() {
  if (RGB_LED_PIN < 0) return;

  unsigned long now = millis();
  uint16_t interval = 33;
  if (currentLedMode == LED_ROBOT_THINKING || currentLedMode == LED_ALERT) interval = 20;

  if (now - ledLastUpdateMs < interval) return;
  ledLastUpdateMs = now;

  switch (currentLedMode) {

    case LED_OFF:
      fill_solid(leds, RGB_LED_COUNT, CRGB::Black);
      break;

    case LED_IDLE: {
      // Slow breathing cyan
      if (ledBreathDir) {
        ledBreathVal = (ledBreathVal >= 200) ? 200 : ledBreathVal + 2;
        if (ledBreathVal >= 200) ledBreathDir = false;
      } else {
        ledBreathVal = (ledBreathVal <= 10) ? 10 : ledBreathVal - 2;
        if (ledBreathVal <= 10) ledBreathDir = true;
      }
      fill_solid(leds, RGB_LED_COUNT, CHSV(128, 255, ledBreathVal));
      break;
    }

    case LED_SCANNING: {
      // Spinning standard blue dot
      fill_solid(leds, RGB_LED_COUNT, CRGB::Black);
      for (int j = -1; j <= 1; j++) {
        int idx = ((int)ledAnimIndex + j + RGB_LED_COUNT) % RGB_LED_COUNT;
        leds[idx] = (j == 0) ? CRGB(0, 100, 255) : CRGB(0, 25, 80);
      }
      ledAnimIndex = (ledAnimIndex + 2) % RGB_LED_COUNT;
      break;
    }

    case LED_HUMAN_TURN: {
      // Breathing standard blue (waiting for human move)
      if (ledBreathDir) {
        ledBreathVal = (ledBreathVal >= 220) ? 220 : ledBreathVal + 3;
        if (ledBreathVal >= 220) ledBreathDir = false;
      } else {
        ledBreathVal = (ledBreathVal <= 30) ? 30 : ledBreathVal - 3;
        if (ledBreathVal <= 30) ledBreathDir = true;
      }
      fill_solid(leds, RGB_LED_COUNT, CHSV(160, 255, ledBreathVal));
      break;
    }

    case LED_ROBOT_TURN: {
      // Robot turn: vibrant detection green
      fill_solid(leds, RGB_LED_COUNT, CRGB(0, 224, 122));
      break;
    }

    case LED_ROBOT_THINKING: {
      // Fast green pulse (coin detected / computing)
      uint8_t phase = (ledAnimIndex / 5) % 4;
      CRGB col = (phase == 0 || phase == 2) ? CRGB(0, 224, 122) : CRGB::Black;
      fill_solid(leds, RGB_LED_COUNT, col);
      ledAnimIndex++;
      break;
    }

    case LED_ROBOT_MOVING: {
      // Green chase along rail
      fill_solid(leds, RGB_LED_COUNT, CRGB(0, 25, 12));
      for (int j = 0; j < 8; j++) {
        int idx = ((int)ledAnimIndex + j) % RGB_LED_COUNT;
        uint8_t brightness = 255 - (j * 28);
        leds[idx] = CRGB(0, brightness, uint8_t(brightness / 2));
      }
      ledAnimIndex = (ledAnimIndex + 1) % RGB_LED_COUNT;
      break;
    }

    case LED_WIN_ROBOT: {
      uint8_t phase = (ledAnimIndex / 8) % 6;
      CRGB col = (phase % 2 == 0) ? CRGB(0, 80, 255) : CRGB::Black;
      if (ledAnimIndex >= 48) col = CRGB(0, 40, 200);
      fill_solid(leds, RGB_LED_COUNT, col);
      if (ledAnimIndex < 200) ledAnimIndex++;
      break;
    }

    case LED_WIN_HUMAN: {
      uint8_t phase = (ledAnimIndex / 8) % 6;
      CRGB col = (phase % 2 == 0) ? CRGB(255, 0, 0) : CRGB::Black;
      if (ledAnimIndex >= 48) col = CRGB(180, 0, 0);
      fill_solid(leds, RGB_LED_COUNT, col);
      if (ledAnimIndex < 200) ledAnimIndex++;
      break;
    }

    case LED_DRAW: {
      for (int i = 0; i < RGB_LED_COUNT; i++) {
        uint8_t pos = (i + ledAnimIndex) % RGB_LED_COUNT;
        leds[i] = (pos < RGB_LED_COUNT / 2) ? CRGB(120, 0, 120) : CRGB(40, 0, 120);
      }
      ledAnimIndex = (ledAnimIndex + 1) % RGB_LED_COUNT;
      break;
    }

    case LED_ALERT: {
      bool on = (ledAnimIndex / 5) % 2 == 0;
      fill_solid(leds, RGB_LED_COUNT, on ? CRGB::Red : CRGB::Black);
      ledAnimIndex++;
      break;
    }

    case LED_RAINBOW: {
      for (int i = 0; i < RGB_LED_COUNT; i++) {
        leds[i] = CHSV(ledAnimIndex + (i * 255 / RGB_LED_COUNT), 255, 200);
      }
      ledAnimIndex += 2;
      break;
    }

    case LED_SOLID:
      fill_solid(leds, RGB_LED_COUNT, solidColor);
      break;
  }

  FastLED.show();
}


// ═════════════════════════════════════════════════════════════════════════
// USB SERIAL STATUS BROADCAST (500 ms)
// ═════════════════════════════════════════════════════════════════════════

void broadcastSerialStatus() {
  if (millis() - lastSerialStatusMs < 500UL) return;
  lastSerialStatusMs = millis();
  int32_t steps  = tmc.XACTUAL();
  bool    moving = (steps != tmc.XTARGET());

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
  Serial.print(",\"ledEffect\":\"");
  Serial.print(currentLedEffect);
  Serial.print("\",\"ledBrightness\":");
  Serial.print(ledBrightness);
  Serial.print(",\"gpioPin\":");
  Serial.print(RGB_LED_PIN);
  Serial.print(",\"board1Cm\":");
  Serial.print(presets[1].cm, 2);
  Serial.print(",\"board2Cm\":");
  Serial.print(presets[2].cm, 2);
  Serial.println("}");
}

// ═════════════════════════════════════════════════════════════════════════
// USB SERIAL COMMAND PARSER
// ═════════════════════════════════════════════════════════════════════════


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
      if (!found) {
        // Try parsing target as raw float for arbitrary cm movements (e.g. MOVE:15.5)
        float val = target.toFloat();
        bool isNumeric = (target.length() > 0 && (isdigit(target[0]) || target[0] == '-' || target[0] == '.'));
        if (isNumeric) {
          if (!isHomed) {
            Serial.println("[WARN] Move requested before homing — ignored.");
          } else {
            float targetCm = val;
            int32_t targetPos = (int32_t)(targetCm * STEPS_PER_CM);
            int32_t maxSteps  = (int32_t)(MAX_DISTANCE_CM * STEPS_PER_CM);
            targetPos = constrain(targetPos, 0, maxSteps);
            currentTargetName = "CM_" + String(targetCm, 1);
            tmc.RAMPMODE(0);
            tmc.VMAX(currentVMAX);
            tmc.XTARGET(targetPos);
            Serial.printf("[MOVE] -> %.1f cm (%d steps)\n", targetCm, targetPos);
          }
          found = true;
        }
      }
      if (!found) Serial.printf("[SERIAL] Unknown target: %s\n", target.c_str());
    }
  }
  else if (cmd.startsWith("SETPRESET:")) {
    // Format: SETPRESET:BOARD1:12.5
    String payload = cmd.substring(10);
    int colonIdx = payload.indexOf(':');
    if (colonIdx > 0) {
      String name = payload.substring(0, colonIdx);
      name.toUpperCase();
      float cm = payload.substring(colonIdx + 1).toFloat();
      bool found = false;
      for (int i = 1; i < NUM_PRESETS; i++) {
        if (presets[i].name == name) {
          presets[i].cm = cm;
          found = true;
          
          // Persist to Preferences
          Preferences prefs;
          prefs.begin("rail_presets", false);
          if (name == "BOARD1") {
            prefs.putFloat("board1", cm);
          } else if (name == "BOARD2") {
            prefs.putFloat("board2", cm);
          }
          prefs.end();
          
          Serial.printf("[SERIAL] Preset %s set and saved to %.2f cm\n", name.c_str(), cm);
          break;
        }
      }
      if (!found) Serial.printf("[SERIAL] Unknown preset: %s\n", name.c_str());
    } else {
      Serial.println("[SERIAL] SETPRESET parse error");
    }
  }
  else if (cmd == "STOP") {
    stopMotor();
    triggerHoming = false;
  }
  else if (cmd.startsWith("SPEED:")) {
    uint32_t v = (uint32_t)cmd.substring(6).toInt();
    v = constrain(v, 50000UL, VMAX_POSITIONING);
    currentVMAX = v;
    if (!triggerHoming) tmc.VMAX(currentVMAX);
    Serial.printf("[SERIAL] Speed set to %u\n", currentVMAX);
  }
  else if (cmd.startsWith("INDICATOR:")) {
    applyIndicatorMode(cmd.substring(10));
  }
  else if (cmd.startsWith("LEDRGB:")) {
    applyLedEffect(cmd.substring(7));
  }
  else if (cmd.startsWith("LEDSOLID:")) {
    String vals = cmd.substring(9);
    int c1 = vals.indexOf(',');
    int c2 = vals.lastIndexOf(',');
    if (c1 > 0 && c2 > c1) {
      int r = constrain(vals.substring(0, c1).toInt(), 0, 255);
      int g = constrain(vals.substring(c1 + 1, c2).toInt(), 0, 255);
      int b = constrain(vals.substring(c2 + 1).toInt(), 0, 255);
      applyLedSolid(r, g, b);
    } else {
      Serial.println("[LED] LEDSOLID parse error — expected LEDSOLID:R,G,B");
    }
  }
  else if (cmd.startsWith("LEDBRIGHT:")) {
    int v = constrain(cmd.substring(10).toInt(), 0, 255);
    ledBrightness = (uint8_t)v;
    FastLED.setBrightness(ledBrightness);
    Serial.printf("[LED] Brightness set to %d\n", ledBrightness);
  }
  else {
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


// ═════════════════════════════════════════════════════════════════════════
// TMC5160 INIT
// ═════════════════════════════════════════════════════════════════════════

void initTMC() {
  pinMode(PIN_TMC_CS, OUTPUT);
  digitalWrite(PIN_TMC_CS, HIGH);
  pinMode(PIN_TMC_EN, OUTPUT);
  digitalWrite(PIN_TMC_EN, HIGH);
  delay(20);
  SPI.begin(PIN_TMC_SCK, PIN_TMC_MISO, PIN_TMC_MOSI);
  tmc.begin();
  tmc.toff(4);
  tmc.rms_current(MOTOR_RMS_MA);
  tmc.microsteps(256);
  tmc.shaft(true);
  tmc.RAMPMODE(0);
  tmc.VSTART(VSTART_VALUE);
  tmc.VSTOP(VSTOP_VALUE);
  tmc.VMAX(VMAX_POSITIONING);
  tmc.AMAX(AMAX_VALUE);
  tmc.DMAX(DMAX_VALUE);
  tmc.v1(VMAX_POSITIONING / 2);
  tmc.a1(AMAX_VALUE * 2);
  tmc.d1(DMAX_VALUE * 2);
  digitalWrite(PIN_TMC_EN, LOW);
  Serial.println("[TMC] Driver initialised OK.");
  Serial.printf("[TMC] STEPS_PER_CM = %.1f\n", STEPS_PER_CM);
}


// ═════════════════════════════════════════════════════════════════════════
// HOMING SEQUENCE
// ═════════════════════════════════════════════════════════════════════════

void runHomingSequence() {
  static bool homingStarted = false;
  if (!homingStarted) {
    if (tmc.XACTUAL() != tmc.XTARGET()) { tmc.XTARGET(tmc.XACTUAL()); delay(50); }
    Serial.println("[HOME] Sequence started - seeking switch...");
    tmc.RAMPMODE(2);
    tmc.VMAX(VMAX_HOMING);
    homingStarted     = true;
    currentTargetName = "HOMING";
  }
  if (digitalRead(PIN_BUTTON_S2) == LOW) {
    tmc.RAMPMODE(0);
    tmc.VMAX(0);
    delay(50);
    tmc.XACTUAL(0);
    tmc.XTARGET(0);
    tmc.VMAX(currentVMAX);
    isHomed           = true;
    triggerHoming     = false;
    homingStarted     = false;
    currentTargetName = "HOME";
    Serial.println("[HOME] Success! Position zeroed.");
    Serial.printf("[HOME] STEPS_PER_CM=%.1f  max travel=%d steps\n",
                  STEPS_PER_CM, (int32_t)(MAX_DISTANCE_CM * STEPS_PER_CM));
  }
}


void loadPresets() {
  Preferences prefs;
  prefs.begin("rail_presets", true);
  presets[1].cm = prefs.getFloat("board1", 9.0);
  presets[2].cm = prefs.getFloat("board2", 43.0);
  prefs.end();
}


// ═════════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  
  loadPresets();
  
  Serial.println("\n[BOOT] TTT Rail Controller starting (100% Offline Serial Mode)...");
  Serial.printf("[BOOT] Board presets: BOARD1=%.1fcm  BOARD2=%.1fcm\n",
                presets[1].cm, presets[2].cm);

  // Home switch
  pinMode(PIN_BUTTON_S2, INPUT_PULLUP);

  // Sensors
  pinMode(PIN_NTC_TEMP, INPUT);
  analogSetPinAttenuation(PIN_NTC_TEMP, ADC_11db);
  pinMode(PIN_DOOR_SENSOR, INPUT_PULLUP);

  // 12V Relay / indicator pins
  pinMode(PIN_RELAY_RED,    OUTPUT); digitalWrite(PIN_RELAY_RED,    LOW);
  pinMode(PIN_RELAY_BUZZER, OUTPUT); digitalWrite(PIN_RELAY_BUZZER, LOW);
  pinMode(PIN_RELAY_GREEN,  OUTPUT); digitalWrite(PIN_RELAY_GREEN,  LOW);

  // RGB LED strip (45 LEDs on GPIO 42)
  if (RGB_LED_PIN >= 0) {
    FastLED.addLeds<RGB_LED_TYPE, RGB_LED_PIN, RGB_COLOR_ORDER>(leds, RGB_LED_COUNT)
           .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(ledBrightness);
    fill_solid(leds, RGB_LED_COUNT, CRGB::Black);
    FastLED.show();
    Serial.printf("[LED] FastLED strip init: %d LEDs on GPIO %d\n",
                  RGB_LED_COUNT, RGB_LED_PIN);
  }

  // Stepper driver init
  initTMC();

  // Initial LED effect
  applyLedEffect("idle");
  Serial.println("[BOOT] Initialisation complete. Ready for serial commands.");
}


// ═════════════════════════════════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════════════════════════════════

void loop() {
  // Process incoming USB serial commands from Python server
  handleSerialCommands();

  // Broadcast STATUS: JSON telemetry every 500 ms
  broadcastSerialStatus();

  // Homing state machine
  if (triggerHoming) runHomingSequence();

  // Safety: stop if limit switch hit while moving in reverse
  if (!triggerHoming && isHomed) {
    if (digitalRead(PIN_BUTTON_S2) == LOW && tmc.XTARGET() < tmc.XACTUAL()) {
      Serial.println("[SAFETY] Home switch triggered while moving negative - stopping.");
      stopMotor();
    }
  }

  // Door sensor monitoring & alert escalation
  updateDoorAlert();

  // 12V indicator relays state machine
  updateIndicators();

  // RGB LED strip animations
  updateLedStrip();

  yield();
}
