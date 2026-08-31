#include <Arduino.h>

#define DEBUG 0

#if DEBUG
  #define DBG_PRINTLN(x) Serial.println(x)
  #define DBG_PRINT(x)   Serial.print(x)
#else
  #define DBG_PRINTLN(x)
  #define DBG_PRINT(x)
#endif

// ============ Joystick pins ============
// Joystick 1: throttle + yaw
#define VRX1 25   // yaw
#define VRY1 33   // throttle
#define SW1  32   // left switch

// Joystick 2: x/y tilt (roll/pitch)
#define VRX2 14   // roll (rx)
#define VRY2 27   // pitch (ry)
#define SW2  26   // right switch

// ============ Tunable parameters ============
const unsigned long HOLD_TIME_MS  = 500;

const unsigned long SEND_INTERVAL = 20;

const float MAX_VEL = 0.5f;
const float RATE_M_PER_SEC   = 0.5f;

// Low-pass filter time constant (seconds) applied to all four joystick
// channels — larger = smoother/more lag, smaller = snappier/noisier.
const float LPF_TAU = 0.05f;

// Deadband, in stick percent (post-mapJoystickF, so range is -100..100).
// Anything inside this band is clamped to exactly 0 so joystick center
// noise / ADC zero-offset never produces a nonzero target angle. Output
// outside the deadband is rescaled back to the full -100..100 range so
// you don't lose usable travel at the stick's outer edge.
const float STICK_DEADBAND = 3.0f;

// Expo blend: 0 = fully linear, 1 = fully cubic. A cubic-heavy blend
// compresses the response near center (fine control for small angles)
// while still reaching full deflection (MAX_VEL) at full stick travel —
// the two curves meet exactly at +-100%, only the shape in between changes.
// This is what makes 5 deg feel as controllable near-center as 2.5 deg did.
const float STICK_EXPO = 0.4f;

// Max rate at which the *target* velocity (velx_ref/vely_ref) is allowed to
// change, regardless of how fast the stick itself moves. This keeps sudden
// stick snaps (or a noisy LoRa/packet glitch) from slamming a step input
// into the velocity PID. pz_ref is already rate-integrated so it doesn't
// need this.
const float VEL_SLEW_MPS_PER_SEC = 40.0f;

// ============ Calibration storage ============
int vrx1_min, vrx1_center, vrx1_max;
int vry1_min, vry1_center, vry1_max;
int vrx2_min, vrx2_center, vrx2_max;
int vry2_min, vry2_center, vry2_max;

// ============ Low-pass filter state ============
struct LPF {
  float value = 0.0f;
  bool  init  = false;
};

LPF lpfX1, lpfY1, lpfX2, lpfY2;

float lpfUpdate(LPF &f, float raw, float dt, float tau) {
  if (!f.init) {
    f.value = raw;
    f.init  = true;
    return f.value;
  }
  float alpha = dt / (tau + dt);   // dt-normalized EMA — stays consistent even if loop rate jitters
  f.value += alpha * (raw - f.value);
  return f.value;
}

// ============ State machine ============
enum FlightState { STATE_IDLE, STATE_ARMED, STATE_FLYING };
FlightState state = STATE_IDLE;

unsigned long bothHoldStart  = 0;
unsigned long leftHoldStart  = 0;
unsigned long rightHoldStart = 0;
bool armFired    = false;
bool launchFired = false;
bool killFired   = false;

float velx_ref = 0.0f;
float vely_ref = 0.0f;
float pz_ref = 0.0f;

unsigned long lastSendTime = 0;
unsigned long lastLoopTime = 0;

int readCenter(int pin) {
  long sum = 0;
  for (int i = 0; i < 50; i++) { sum += analogReadMilliVolts(pin); delay(5); }
  return sum / 50;
}

// Fully floating-point stick mapping. Previously this used the integer
// map() and was also truncated to int by the caller, which quantized the
// stick reading to ~201 discrete levels (coarser still near center, since
// map() truncates internally). That quantized value fed directly into the
// pz_ref rate integration, producing a visibly stepped ramp even though
// the LPF output feeding it was smooth. Manual float lerp below preserves
// the LPF's precision all the way through.
float mapJoystickF(float raw, int minV, int centerV, int maxV) {
  if (raw < centerV) {
    if (centerV == minV) return 0.0f; // guard divide-by-zero from bad calibration
    return (raw - centerV) / (float)(centerV - minV) * 100.0f; // -> [-100, 0]
  } else {
    if (maxV == centerV) return 0.0f;
    return (raw - centerV) / (float)(maxV - centerV) * 100.0f; // -> [0, 100]
  }
}

// Clamps anything inside +-deadband to zero, then rescales the remainder
// so the usable range still spans the full -100..100 output. Without the
// rescale, deadband would just chop off the middle and leave a "dead" gap
// right where fine hover corrections are made.
float applyDeadband(float val, float deadband) {
  float mag = fabsf(val);
  if (mag < deadband) return 0.0f;
  float sign = (val > 0.0f) ? 1.0f : -1.0f;
  return sign * ((mag - deadband) / (100.0f - deadband)) * 100.0f;
}

// Blends linear and cubic response on a -100..100 input, output also
// -100..100. expo=0 -> identity (linear). expo=1 -> pure cube. Cubic
// compresses small inputs (fine control near center) while endpoints
// (+-100) are unchanged either way, so full-stick still means full angle.
float applyExpo(float val, float expo) {
  float x = val / 100.0f;
  float shaped = expo * (x * x * x) + (1.0f - expo) * x;
  return shaped * 100.0f;
}

// Limits how fast `current` is allowed to move toward `target`, in units
// of val/sec. Used on velx_ref/vely_ref since those are recomputed fresh
// from the stick every loop (not integrated), so without this a stick
// snap is a literal step input into the velocity PID.
float slewLimit(float current, float target, float maxRatePerSec, float dt) {
  float maxDelta = maxRatePerSec * dt;
  float delta = target - current;
  if (delta > maxDelta)  delta = maxDelta;
  if (delta < -maxDelta) delta = -maxDelta;
  return current + delta;
}

bool isPressed(int pin) {
  return digitalRead(pin) == LOW;
}

void sendCommand(const String &cmd) {
  Serial2.print(cmd);
  Serial2.print("\r\n");
  DBG_PRINT("[sent to FC] ");
  DBG_PRINTLN(cmd);
}

// ---------- Ground-station uplink snoop ----------
// The STM32's downlink replies (via com_send()) are binary length-prefixed
// frames with no delimiter — "ARM"/"armed!"/"LAUNCH"/"KILL" never carry a
// trailing terminator on the wire, which made parsing them from Serial2
// unreliable. Instead of decoding the downlink, we watch what's typed on
// this ESP32's own USB Serial and relayed out to the FC over Serial2 — that
// text is plain ASCII (whatever the ground-station operator types), so if
// someone arms/launches/kills via the passthrough terminal rather than the
// joystick switches, we catch it here, on the side we fully control the
// framing of, instead of trying to decode the STM32's reply format.
//
// scriptMode exists because the ground station's own autonomous LAUNCH:path
// playback also sends a literal "LAUNCH" line through this same passthrough
// (the STM32 requires that exact string) — without this flag we'd flip to
// STATE_FLYING and start streaming our own idle joystick readings at 50Hz
// right on top of the ground station's scripted stream. The ground station
// sends SCRIPT_MODE:ON immediately before an autonomous LAUNCH and
// SCRIPT_MODE:OFF once playback ends, so this only suppresses our own
// stream during a scripted run — manual joystick flight is unaffected.
String txLineBuf;
bool scriptMode = false;

void handleTypedCommand(const String &line) {
  if (line == "ARM") {
    state = STATE_ARMED;
  } else if (line == "LAUNCH") {
    state = STATE_FLYING;
  } else if (line == "KILL") {
    state = STATE_IDLE;
    scriptMode = false;
  } else if (line == "SCRIPT_MODE:ON") {
    scriptMode = true;
  } else if (line == "SCRIPT_MODE:OFF") {
    scriptMode = false;
  }
}

// Relay one byte from Serial -> Serial2, watching for a complete typed line.
void pumpTypedByte(char c) {
  Serial2.write(c);
  if (c == '\n') {
    txLineBuf.trim();
    handleTypedCommand(txLineBuf);
    txLineBuf = "";
  } else if (c != '\r') {
    txLineBuf += c;
  }
}

#define STATUS_LED 2

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  analogSetAttenuation(ADC_11db);
  pinMode(STATUS_LED, OUTPUT);

  pinMode(VRX1, INPUT);  pinMode(VRY1, INPUT);  pinMode(SW1, INPUT_PULLUP);
  pinMode(VRX2, INPUT);  pinMode(VRY2, INPUT);  pinMode(SW2, INPUT_PULLUP);

  digitalWrite(STATUS_LED, HIGH);
  DBG_PRINTLN("Leave both joysticks at rest...");

  vrx1_center = readCenter(VRX1);
  vry1_center = readCenter(VRY1);
  vrx2_center = readCenter(VRX2);
  vry2_center = readCenter(VRY2);

  DBG_PRINTLN("Sweep both joysticks fully in every direction for 5 seconds...");
  vrx1_min = vry1_min = vrx2_min = vry2_min = 4095;
  vrx1_max = vry1_max = vrx2_max = vry2_max = 0;

  unsigned long start = millis();
  unsigned long lastBlink = 0;
  bool ledState = false;

  while (millis() - start < 5000) {
    while (Serial.available())  { pumpTypedByte(Serial.read()); }
    while (Serial2.available()) { Serial.write(Serial2.read()); }

    if (millis() - lastBlink > 100) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED, ledState);
    }

    int x1 = analogReadMilliVolts(VRX1), y1 = analogReadMilliVolts(VRY1);
    int x2 = analogReadMilliVolts(VRX2), y2 = analogReadMilliVolts(VRY2);
    vrx1_min = min(vrx1_min, x1); vrx1_max = max(vrx1_max, x1);
    vry1_min = min(vry1_min, y1); vry1_max = max(vry1_max, y1);
    vrx2_min = min(vrx2_min, x2); vrx2_max = max(vrx2_max, x2);
    vry2_min = min(vry2_min, y2); vry2_max = max(vry2_max, y2);
    delay(10);
  }

  digitalWrite(STATUS_LED, LOW);
  delay(150);
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(100);
    digitalWrite(STATUS_LED, LOW);  delay(100);
  }
  DBG_PRINTLN("Calibration done. Ready.");

  lastLoopTime = millis();
}

void loop() {
  while (Serial.available())  { pumpTypedByte(Serial.read()); }
  while (Serial2.available()) { Serial.write(Serial2.read()); }

  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  // ---------- Filter all four analog channels every iteration ----------
  // Runs unconditionally (not just while flying) so the filters are already
  // settled by the time LAUNCH happens, avoiding a startup transient.
  float x1_f = lpfUpdate(lpfX1, analogReadMilliVolts(VRX1), dt, LPF_TAU);
  float y1_f = lpfUpdate(lpfY1, analogReadMilliVolts(VRY1), dt, LPF_TAU);
  float x2_f = lpfUpdate(lpfX2, analogReadMilliVolts(VRX2), dt, LPF_TAU);
  float y2_f = lpfUpdate(lpfY2, analogReadMilliVolts(VRY2), dt, LPF_TAU);

  bool left  = isPressed(SW1);
  bool right = isPressed(SW2);
  bool both  = left && right;

  // ---------- 1. ARM: both switches held 3s ----------
  if (both) {
    if (bothHoldStart == 0) bothHoldStart = now;
    if (!armFired && (now - bothHoldStart >= HOLD_TIME_MS) && state == STATE_IDLE) {
      sendCommand("ARM");
      state = STATE_ARMED;
      armFired = true;
    }
  } else {
    bothHoldStart = 0;
    armFired = false;
  }

  // ---------- 2. LAUNCH: left switch alone held 3s ----------
  if (left && !right) {
    if (leftHoldStart == 0) leftHoldStart = now;
    if (!launchFired && (now - leftHoldStart >= HOLD_TIME_MS) && state == STATE_ARMED) {
      sendCommand("LAUNCH");
      state = STATE_FLYING;
      launchFired = true;
    }
  } else {
    leftHoldStart = 0;
    launchFired = false;
  }

  // ---------- 4. KILL: right switch alone held 3s ----------
  if (right && !left) {
    if (rightHoldStart == 0) rightHoldStart = now;
    if (!killFired && (now - rightHoldStart >= HOLD_TIME_MS)) {
      sendCommand("KILL");
      state = STATE_IDLE;
      killFired = true;
    }
  } else {
    rightHoldStart = 0;
    killFired = false;
  }

  // ---------- 3. While flying: joystick 1 -> throttle/yaw, joystick 2 -> tilt ----------
  if (state == STATE_FLYING && !scriptMode) {
    // Float mapping preserved end-to-end — no int cast, no map() truncation.
    float x1 = mapJoystickF(x1_f, vrx1_min, vrx1_center, vrx1_max);   // yaw
    float y1 = mapJoystickF(y1_f, vry1_min, vry1_center, vry1_max);   // throttle
    float x2 = mapJoystickF(x2_f, vrx2_min, vrx2_center, vrx2_max);   // roll
    float y2 = mapJoystickF(y2_f, vry2_min, vry2_center, vry2_max);   // pitch

    // Deadband first (kill center noise), then expo (shape the remaining
    // travel). Order matters: expo on top of raw noise would still let
    // small jitter near center leak through as a small nonzero output.
    x1 = applyExpo(applyDeadband(x1, STICK_DEADBAND), STICK_EXPO);
    y1 = applyExpo(applyDeadband(y1, STICK_DEADBAND), STICK_EXPO);
    x2 = applyExpo(applyDeadband(x2, STICK_DEADBAND), STICK_EXPO);
    y2 = applyExpo(applyDeadband(y2, STICK_DEADBAND), STICK_EXPO);

    float velx_raw = (y1 / 100.0f) * MAX_VEL;
    float vely_raw = (x1 / 100.0f) * MAX_VEL;

    float vel_mag = sqrtf(velx_raw * velx_raw + vely_raw * vely_raw);
    if (vel_mag > MAX_VEL) {
        float scale = MAX_VEL / vel_mag;
        velx_raw *= scale;
        vely_raw *= scale;
    }

    // Slew-limit the target velocity itself, not just the raw stick, so a
    // sudden stick snap (or a glitchy reading) can't inject a step into
    // the velocity PID.
    velx_ref = slewLimit(velx_ref, velx_raw, VEL_SLEW_MPS_PER_SEC, dt);
    vely_ref = slewLimit(vely_ref, -vely_raw, VEL_SLEW_MPS_PER_SEC, dt);
    pz_ref -= (x2 / 100.0f) * RATE_M_PER_SEC   * dt;

    if (now - lastSendTime >= SEND_INTERVAL) {
      lastSendTime = now;
      char buf[64];
      snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f", velx_ref, vely_ref, pz_ref);
      sendCommand(String(buf));
    }
  }
}