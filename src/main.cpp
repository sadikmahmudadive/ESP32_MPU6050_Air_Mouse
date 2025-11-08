#include <Arduino.h>

// AirMouse_ESP32_MPU6050.ino
// Requires: Adafruit_MPU6050, Adafruit_Sensor, Wire, BleMouse
// Configure pins below as required for your board

#include <Wire.h>
#include <MPU6050_tockn.h>

// Compile-time feature flags
#ifndef USE_WIFI_MOUSE
#define USE_WIFI_MOUSE 1
#endif
#ifndef USE_BLE_MOUSE
#define USE_BLE_MOUSE 0
#endif

#if USE_BLE_MOUSE
#include <BleMouse.h>
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiUdp.h>

MPU6050 mpu(Wire);
#if USE_BLE_MOUSE
// BLE is still available but we will use Wi-Fi mouse by default
BleMouse bleMouse("ESP Air 1");
#endif

// Wi-Fi provisioning / UDP mouse globals
Preferences prefs;
WebServer server(80);
WiFiUDP udp;
const uint16_t UDP_PORT = 5555;
bool useWiFiMouse = true; // toggle to switch between BLE HID and Wi-Fi mouse
uint8_t udp_seq = 0;

const char* PREF_NS = "wifi";

// --- Double-Reset Detector (use ESP32 RESET button to enter config) ---
// If the device is reset twice within the window, the second boot enters config portal.
RTC_DATA_ATTR uint32_t DRD_FLAG = 0;              // persists across resets (RTC memory)
const uint32_t DRD_MAGIC = 0xA5A50F0Ful;          // magic value indicating armed state
unsigned long drdArmedMs = 0;                     // armed time (RAM only)
const unsigned long DRD_WINDOW_MS = 5000;         // 5s window for double reset

// HTML for captive portal
const char* cfg_html = R"rawliteral(
<!doctype html>
<html>
  <head><meta charset="utf-8"><title>ESP Air Wi-Fi Setup</title></head>
  <body>
    <h3>ESP Air Wi-Fi Setup</h3>
    <form method="POST" action="/save">
      <fieldset style="margin-bottom:10px;">
        <legend>Normal Wi‑Fi (via router)</legend>
        SSID:<br><input type="text" name="ssid"><br>
        Password:<br><input type="password" name="pass"><br>
      </fieldset>

      <fieldset style="margin-bottom:10px;">
        <legend>Direct AP mode (no router)</legend>
        <label><input type="checkbox" name="apmode"> Use Direct AP mode</label><br>
        AP Password (8+ chars, leave empty for open):<br>
        <input type="text" name="appass" placeholder="optional"><br>
        The AP SSID will be: <b>ESP-Air-Mouse</b>
      </fieldset>

      <input type="submit" value="Save & Reboot">
    </form>
    <p>After saving, the device will reboot and attempt to join the configured Wi‑Fi network.</p>
  </body>
</html>
)rawliteral";

// helper to send UDP mouse packet: header(0xA5), seq, dx(int8), dy(int8), buttons
void sendMousePacket(int8_t dx, int8_t dy, uint8_t buttons) {
  uint8_t buf[6];
  buf[0] = 0xA5;
  buf[1] = udp_seq++;
  buf[2] = (uint8_t)dx;
  buf[3] = (uint8_t)dy;
  buf[4] = buttons;
  buf[5] = 0; // reserved
  // use broadcast to reach the listening PC on the same network
  udp.beginPacket("255.255.255.255", UDP_PORT);
  udp.write(buf, sizeof(buf));
  udp.endPacket();
}
void detectFlick(float gx, float gy, float gz, unsigned long now);
float applyDeadzone(float valDeg, float dead);
float softDeadzone(float x, float dead, float width);
float mapTiltToSpeed(float tiltDeg, float sens);
float mapTiltToSpeedSimple(float tiltDeg, float sens, float gammaExp);
float readBatteryVoltage();
uint8_t voltageToPercent(float v);
void sendBrowserBack();
void sendBrowserForward();
void sendKeyPacket(uint8_t keyId); // presentation key packet (Wi-Fi)
void i2cScan();
uint8_t readWhoAmI();
uint8_t readRegisterNoRestart(uint8_t devaddr, uint8_t reg);
uint8_t readRegisterWithRestart(uint8_t devaddr, uint8_t reg);
uint8_t readRegisterEndThenRequest(uint8_t devaddr, uint8_t reg);
bool loadCalibration(float &outPitchOff, float &outRollOff);
void saveCalibration(float pitchOff, float rollOff);
void runCalibrationInteractive();
void checkLiveZeroing(float curPitch, float curRoll, unsigned long now);
bool loadMappingSettings();
void saveMappingSettings();

//
// === PIN CONFIG ===
const int PIN_LEFT_BTN  = 14; // left click
const int PIN_RIGHT_BTN = 27; // right click or mode (hold -> scroll)
const int PIN_BACK_BTN  = 26; // back button (or gesture)
const int PIN_LED       = 2;  // indicator
const int PIN_BAT_ADC   = 35; // ADC input (via voltage divider) for battery monitor

// I2C pins (change if your wiring uses other pins)
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

//
// === TIMING / FILTERS / TUNING ===
const unsigned long LOOP_INTERVAL_MS = 10; // target loop time ~10ms (100Hz)
// Reduce target loop interval to lower latency
const unsigned long LOOP_INTERVAL_TARGET_MS = 5; // aim for ~200Hz, but keep guard for CPU
const float alpha_comp = 0.98f; // complementary filter alpha (gyro trust)
float smoothing_alpha = 0.12f; // EMA smoothing for cursor deltas (0..1) - lower for smoother response
const float gyroToDegPerSec = 1.0f; // gyro reading is deg/s if library gives that
const float sensitivity = 1.6f; // base sensitivity multiplier
// Simple non-linear mapping exponent
const float gammaExp = 1.35f;   // exponent for non-linear pointer accel
const float deadzone_deg = 1.5f; // degrees of tilt to ignore (soft deadzone)
const float deadzone_soft_width = 1.0f; // transition width in degrees for soft deadzone
const float maxCursorSpeed = 12.0f; // max pixels per loop step
// Minimal output step to prevent micro jitter accumulation (pixels/loop)
const float minOutputStep = 0.25f;
// Adaptive smoothing
const float smooth_low_speed = 0.08f; // EMA alpha at low speed (more smoothing)
const float smooth_high_speed = 0.22f; // EMA alpha at high speed
const float smooth_speed_knee = 10.0f;  // speed where alpha transitions
const float scrollSensitivity = 0.8f; // sensitivity when in scroll mode

// Gesture detection thresholds
// Increase thresholds and add movement guard to avoid false positives while moving/hovering
const float flickGyroThreshold = 400.0f; // deg/s spike for flick gestures (raised)
const float flickAxisMin = 300.0f; // minimum dominant-axis magnitude required to consider direction
const unsigned long flickTimeWindow = 400; // ms between allowed flicks (debounce)
const float flickMoveGuard = 1.0f; // pixels per loop - ignore flicks when cursor is moving faster than this
// For peak detection
float lastGmag = 0.0f;
const float flickPrevMaxRatio = 0.5f; // previous gmag must be below this fraction of threshold

// Battery measurement
// ADC reference is nominally 3.3V (ESP32 ADC is not perfectly linear; consider calibration if needed)
const float ADC_REF = 3.3f;
// Configure your actual resistor divider values:
// R_TOP: from battery positive to ADC pin, R_BOTTOM: from ADC pin to GND
// Example: 100k/100k -> ratio 2.0
const float R_TOP = 100000.0f;    // ohms
const float R_BOTTOM = 100000.0f; // ohms
const float VOLT_DIVIDER_RATIO = (R_TOP + R_BOTTOM) / R_BOTTOM; // Vbatt = Vadc * ratio
const float LOW_BATT_THRESHOLD = 3.4f; // per-cell threshold (LiPo single cell)

// === STATE ===
float pitch = 0.0f; // degrees
float roll  = 0.0f; // degrees
unsigned long lastLoop = 0;
float smoothed_dx = 0.0f;
float smoothed_dy = 0.0f;

// fractional accumulators to preserve sub-pixel movement for smooth cursor motion
float cursorAccX = 0.0f;
float cursorAccY = 0.0f;

// For gyro integration
unsigned long lastFusionMs = 0;
// Calibration offsets for orientation (in degrees)
float pitchOffset = 0.0f;
float rollOffset  = 0.0f;
// Axis mapping settings
bool invertX = false;
bool invertY = false;
bool swapXY  = false;
// Non-blocking event blink state (for confirmations)
unsigned long eventBlinkEnd = 0;
unsigned long eventBlinkLast = 0;
bool eventBlinkOn = false;

// For gesture detection
unsigned long lastFlickTime = 0;
// Button previous-state tracking to avoid accidental continuous clicks
bool prevLeftPressed = false;
bool prevRightPressed = false;
bool prevBackPressed = false;
unsigned long rightPressStartMs = 0;
unsigned long leftPressStartMs = 0;
bool leftHoldDragging = false;

// Presentation control state
bool presentationStarted = false; // first single tap starts presentation (F5)
unsigned long lastBackTapMs = 0;  // time of last back single tap
const unsigned long DOUBLE_TAP_WINDOW_MS = 400; // within this window a second tap -> double tap (previous slide)
bool pendingSingleBack = false;  // awaiting possible second tap
// Long press end presentation
unsigned long backHoldStartMs = 0;
bool escHoldTriggered = false;
const unsigned long BACK_LONG_PRESS_MS = 1200;

// Debounce state for buttons
const unsigned long DEBOUNCE_MS = 50; // ms
bool lastRawLeft = false, lastRawRight = false, lastRawBack = false;
bool stableLeft = false, stableRight = false, stableBack = false;
unsigned long lastLeftChangeMs = 0, lastRightChangeMs = 0, lastBackChangeMs = 0;
// Click throttles and movement guard
unsigned long lastLeftClickMs = 0, lastRightClickMs = 0, lastBackClickMs = 0;
const unsigned long MIN_CLICK_INTERVAL_MS = 200; // ms between clicks to avoid bounce/auto-fire
const float CLICK_MOVE_THRESHOLD = 0.6f; // pixels per loop — if moving faster, suppress tap clicks

void setup(){
  Serial.begin(9600);
  // Ensure I2C is initialized on ESP32 pins. If your wiring uses different pins, change the constants above.
  // Some modules or cores work with Wire.begin() default pins, but explicitly specifying pins is more reliable.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Optional: increase I2C clock to 400kHz for more stable comms with some modules
  Wire.setClock(400000);
  // Run I2C scan to help debug device presence
  i2cScan();
  pinMode(PIN_LEFT_BTN, INPUT_PULLUP); // using INPUT_PULLUP; wire buttons to GND
  pinMode(PIN_RIGHT_BTN, INPUT_PULLUP);
  pinMode(PIN_BACK_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);


  // small delay to let the bus and sensor power settle
  delay(50);

  // Initialize debounce / button stable states
  unsigned long now = millis();
  lastRawLeft = (digitalRead(PIN_LEFT_BTN) == LOW);
  lastRawRight = (digitalRead(PIN_RIGHT_BTN) == LOW);
  lastRawBack = (digitalRead(PIN_BACK_BTN) == LOW);
  stableLeft = lastRawLeft;
  stableRight = lastRawRight;
  stableBack = lastRawBack;
  prevLeftPressed = stableLeft;
  prevRightPressed = stableRight;
  prevBackPressed = stableBack;
  lastLeftChangeMs = now;
  lastRightChangeMs = now;
  lastBackChangeMs = now;

  leftPressStartMs = now; // ensure initialized (will be overwritten on actual press)
  leftHoldDragging = false;

  // Initialize MPU6050_tockn (DMP). Use Wire.begin() earlier; now start the mpu and calibrate.
  Serial.println("Initializing MPU6050 (tockn)...");
  mpu.begin();
  Serial.println("Calibrating MPU6050 gyro offsets (this may take a few seconds). Do not move the sensor.");
  mpu.calcGyroOffsets(true);
  Serial.println("Calibration done.");

  // Get initial angles from DMP
  mpu.update();
  pitch = mpu.getAngleX();
  roll = mpu.getAngleY();
  // initialize previous angle timestamps for rate calculations
  lastFusionMs = millis();

  // Attempt to load saved orientation calibration; allow forcing recalibration by holding BACK at boot
  prefs.begin(PREF_NS, false);
  bool forceCal = (digitalRead(PIN_BACK_BTN) == LOW);
  if (!forceCal && loadCalibration(pitchOffset, rollOffset)) {
    Serial.print("Loaded calibration: pitchOff="); Serial.print(pitchOffset);
    Serial.print(" rollOff="); Serial.println(rollOffset);
  } else {
    Serial.println(forceCal ? "Forcing orientation calibration (BACK held)" : "No saved calibration found; running orientation calibration");
    runCalibrationInteractive();
  }

  // Load axis mapping settings (invert/swap)
  (void)loadMappingSettings();


  lastLoop = millis();
  lastFusionMs = millis();

  // Wi‑Fi provisioning and UDP setup
  // Double-reset detection: tap RESET twice quickly to enter config portal
  bool configMode = false;
  if (DRD_FLAG == DRD_MAGIC) {
    DRD_FLAG = 0; // consume
    configMode = true;
    Serial.println("Double reset detected -> entering config portal");
  } else {
    DRD_FLAG = DRD_MAGIC;     // arm DRD
    drdArmedMs = millis();    // start disarm timer
  }
  // prefs already begun above for calibration; keep open
  String stored_ssid = prefs.getString("ssid", "");
  String stored_pass = prefs.getString("pass", "");

  if (configMode || stored_ssid.length() == 0) {
    Serial.println("Entering config portal (AP mode). Tip: double-tap the RESET button within 5s to force this mode.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP-Air-Setup");
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP: "); Serial.println(ip);
    server.on("/", HTTP_GET, [](){ server.send(200, "text/html", cfg_html); });
    server.on("/save", HTTP_POST, [](){
      String ssid = server.arg("ssid");
      String pass = server.arg("pass");
      String apmode = server.arg("apmode");
      String appass = server.arg("appass");
      bool useAP = apmode.length() > 0;
      if (useAP) {
        // Store sentinel to request Direct AP mode
        prefs.putString("ssid", "__AP__");
        prefs.putString("pass", appass);
        server.send(200, "text/html", "Saved (Direct AP mode). Rebooting...");
      } else {
        prefs.putString("ssid", ssid);
        prefs.putString("pass", pass);
        server.send(200, "text/html", "Saved. Rebooting...");
      }
      delay(500);
      ESP.restart();
    });
    server.begin();
    Serial.println("Config server started at / (connect to AP 'ESP-Air-Setup')");
  } else {
    if (stored_ssid == "__AP__") {
      // Direct AP mode for mouse operation
      Serial.println("Starting Direct AP mode (ESP-Air-Mouse)");
      WiFi.mode(WIFI_AP);
      String apPass = stored_pass;
      if (apPass.length() >= 8) {
        WiFi.softAP("ESP-Air-Mouse", apPass.c_str());
      } else {
        WiFi.softAP("ESP-Air-Mouse");
      }
      IPAddress ip = WiFi.softAPIP();
      Serial.print("AP IP: "); Serial.println(ip);
      udp.begin(UDP_PORT);
      Serial.printf("UDP ready (broadcast on port %d)\n", UDP_PORT);
    } else {
      Serial.print("Connecting to Wi‑Fi SSID: "); Serial.println(stored_ssid);
      WiFi.mode(WIFI_STA);
      WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
      unsigned long startMs = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < 15000) {
        delay(200);
        Serial.print('.');
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("Connected. IP: "); Serial.println(WiFi.localIP());
        udp.begin(UDP_PORT);
        Serial.printf("UDP ready (broadcast on port %d)\n", UDP_PORT);
      } else {
        Serial.println();
        Serial.println("Failed to connect to Wi‑Fi. Entering AP config mode.");
        prefs.remove("ssid"); prefs.remove("pass");
        ESP.restart();
      }
    }
  }

    // Start BLE only if BLE mode selected and compiled in
  #if USE_BLE_MOUSE
    if (!useWiFiMouse) {
      bleMouse.begin();
      delay(200);
      Serial.println("BLE mouse begin called");
    }
  #endif
}

// Simple I2C scanner to print found addresses to Serial
void i2cScan() {
  Serial.println("I2C scan starting...");
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found");
  else Serial.println("I2C scan complete");
  Serial.println();
}

void loop(){
  unsigned long now = millis();
  if (now - lastLoop < LOOP_INTERVAL_TARGET_MS) return;
  unsigned long dt_ms = now - lastLoop;
  lastLoop = now;

  // Use MPU6050_tockn DMP update to get angles
  mpu.update();
  float newPitch = mpu.getAngleX();
  float newRoll  = mpu.getAngleY();
  float newYaw   = mpu.getAngleZ();

  // Compute angular rates (deg/s) from angle deltas
  float dt = dt_ms / 1000.0f;
  float gx = 0, gy = 0, gz = 0;
  if (dt > 0) {
    gy = (newPitch - pitch) / dt; // approximate pitch rate
    gx = (newRoll  - roll)  / dt; // approximate roll rate
    gz = (newYaw   - 0.0f)  / dt; // yaw rate not used for fusion here
  }

  // Complementary filter: fuse DMP angles with integrated deltas (DMP is already fused but keep tuning path)
  pitch = alpha_comp * (pitch + gy * dt) + (1.0f - alpha_comp) * newPitch;
  roll  = alpha_comp * (roll  + gx * dt) + (1.0f - alpha_comp) * newRoll;

  // Apply orientation calibration offsets to align neutral position
  float pitchCal = pitch - pitchOffset;
  float rollCal  = roll  - rollOffset;

  // Allow live zeroing and mapping toggles via button combos
  checkLiveZeroing(pitch, roll, now);

  // Gesture detection: flick if large angular rate
  detectFlick(gx, gy, gz, now);

  // Read buttons (active low because using INPUT_PULLUP) with debouncing
  bool rawLeft  = (digitalRead(PIN_LEFT_BTN)  == LOW);
  bool rawRight = (digitalRead(PIN_RIGHT_BTN) == LOW);
  bool rawBack  = (digitalRead(PIN_BACK_BTN)  == LOW);

  if (rawLeft != lastRawLeft) { lastRawLeft = rawLeft; lastLeftChangeMs = now; }
  if (rawRight != lastRawRight) { lastRawRight = rawRight; lastRightChangeMs = now; }
  if (rawBack != lastRawBack) { lastRawBack = rawBack; lastBackChangeMs = now; }

  if ((now - lastLeftChangeMs) > DEBOUNCE_MS && stableLeft != lastRawLeft) {
    stableLeft = lastRawLeft;
  }
  if ((now - lastRightChangeMs) > DEBOUNCE_MS && stableRight != lastRawRight) {
    stableRight = lastRawRight;
  }
  if ((now - lastBackChangeMs) > DEBOUNCE_MS && stableBack != lastRawBack) {
    stableBack = lastRawBack;
  }

  bool leftPressed  = stableLeft;
  bool rightPressed = stableRight;
  bool backPressed  = stableBack;

  // Map pitch & roll to cursor dx/dy
  float dx = 0, dy = 0;
  if (rightPressed) {
    // Scroll mode (tilt up/down to scroll)
    float tiltY = pitchCal; // up/down
    tiltY = softDeadzone(tiltY, deadzone_deg, deadzone_soft_width);
    // Scrolling can remain simpler
    dy = mapTiltToSpeed(tiltY, scrollSensitivity);
    dx = 0;
  } else {
    // Normal mode: roll -> X, pitch -> Y (will allow swap/invert below)
    float tiltX = rollCal;
    float tiltY = pitchCal;
    tiltX = softDeadzone(tiltX, deadzone_deg, deadzone_soft_width);
    tiltY = softDeadzone(tiltY, deadzone_deg, deadzone_soft_width);
    // Apply swap and invert preferences before mapping
    if (swapXY) {
      float tmp = tiltX; tiltX = tiltY; tiltY = tmp;
    }
    if (invertX) tiltX = -tiltX;
    if (invertY) tiltY = -tiltY;
    // Simple, stable non-linear pointer acceleration
    dx = mapTiltToSpeedSimple(tiltX, sensitivity, gammaExp);
    dy = mapTiltToSpeedSimple(tiltY, sensitivity, gammaExp);
  }

  // Adaptive smoothing: use higher alpha (more responsive) at higher speeds; when very fast, bypass smoothing more
  float speedMag = sqrt(dx*dx + dy*dy);
  float t = constrain(speedMag / smooth_speed_knee, 0.0f, 1.0f);
  smoothing_alpha = smooth_low_speed + t * (smooth_high_speed - smooth_low_speed);
  if (speedMag > (smooth_speed_knee * 1.6f)) {
    // Rapid motion: further increase alpha to reduce latency
    smoothing_alpha = min(0.45f, smoothing_alpha + 0.15f);
  }
  smoothed_dx = smoothing_alpha * dx + (1.0f - smoothing_alpha) * smoothed_dx;
  smoothed_dy = smoothing_alpha * dy + (1.0f - smoothing_alpha) * smoothed_dy;
  // Snap small outputs to zero to avoid visible jitter
  if (fabs(smoothed_dx) < minOutputStep) smoothed_dx = 0.0f;
  if (fabs(smoothed_dy) < minOutputStep) smoothed_dy = 0.0f;

  // Constrain max speed
  smoothed_dx = constrain(smoothed_dx, -maxCursorSpeed, maxCursorSpeed);
  smoothed_dy = constrain(smoothed_dy, -maxCursorSpeed, maxCursorSpeed);

  // Send mouse events over Wi‑Fi UDP or BLE
  if (useWiFiMouse) {
    digitalWrite(PIN_LED, HIGH);
    // Move cursor using fractional accumulators for smooth sub-pixel motion
    cursorAccX += smoothed_dx;
    cursorAccY += smoothed_dy;
    if (fabs(cursorAccX) < 0.01f) cursorAccX = 0.0f;
    if (fabs(cursorAccY) < 0.01f) cursorAccY = 0.0f;
    int moveX = (int)cursorAccX;
    int moveY = (int)cursorAccY;
    if (moveX != 0 || moveY != 0) {
      int8_t sx = (int8_t)constrain(moveX, -127, 127);
      int8_t sy = (int8_t)constrain(moveY, -127, 127);
      // Include left button state continuously to support drag on the host
      uint8_t btns = leftPressed ? 0x01 : 0x00;
      sendMousePacket(sx, sy, btns);
      cursorAccX -= moveX;
      cursorAccY -= moveY;
    }

    // Left button: stateful press/release to support drag
    if (leftPressed != prevLeftPressed) {
      // Emit a state update even if no movement
      sendMousePacket(0, 0, leftPressed ? 0x01 : 0x00);
    }
    prevLeftPressed = leftPressed;

    if (backPressed && !prevBackPressed) {
      // Back button tap for presentation control
      unsigned long sinceLastTap = now - lastBackTapMs;
      lastBackTapMs = now;
      if (pendingSingleBack && sinceLastTap <= DOUBLE_TAP_WINDOW_MS) {
        // Double tap -> previous slide (Left Arrow)
        pendingSingleBack = false;
        sendKeyPacket(3); // left
      } else {
        // Start new tap sequence
        pendingSingleBack = true;
      }
      // Start hold timing
      backHoldStartMs = now;
      escHoldTriggered = false;
    }
    if (backPressed) {
      // Check for long press to end presentation
      if (backHoldStartMs && presentationStarted && !escHoldTriggered && (now - backHoldStartMs) > BACK_LONG_PRESS_MS) {
        sendKeyPacket(4); // Esc end presentation
        presentationStarted = false;
        escHoldTriggered = true;
        pendingSingleBack = false; // cancel pending single tap
      }
    } else {
      backHoldStartMs = 0;
      escHoldTriggered = false;
    }
    // Resolve pending single tap if window expired
    if (pendingSingleBack && (now - lastBackTapMs) > DOUBLE_TAP_WINDOW_MS) {
      // Single tap action
      if (!presentationStarted) {
        sendKeyPacket(1); // F5 start presentation
        presentationStarted = true;
      } else {
        sendKeyPacket(2); // right (next slide)
      }
      pendingSingleBack = false;
    }
    prevBackPressed = backPressed;

    const unsigned long RIGHT_HOLD_MS = 500;
    if (rightPressed && !prevRightPressed) {
      rightPressStartMs = now;
    } else if (!rightPressed && prevRightPressed) {
      unsigned long held = now - rightPressStartMs;
      if (held < RIGHT_HOLD_MS) {
        unsigned long since = now - lastRightClickMs;
        float moveSpeed = sqrt(smoothed_dx*smoothed_dx + smoothed_dy*smoothed_dy);
        if (since >= MIN_CLICK_INTERVAL_MS && moveSpeed <= CLICK_MOVE_THRESHOLD) {
          sendMousePacket(0,0,0x02);
          lastRightClickMs = now;
        }
      }
    }
    prevRightPressed = rightPressed;

  }
#if USE_BLE_MOUSE
  else if (bleMouse.isConnected()) {
    digitalWrite(PIN_LED, HIGH);
    // Move cursor using fractional accumulators for smooth sub-pixel motion
    cursorAccX += smoothed_dx;
    cursorAccY += smoothed_dy;
    // eliminate tiny noise drift
    if (fabs(cursorAccX) < 0.01f) cursorAccX = 0.0f;
    if (fabs(cursorAccY) < 0.01f) cursorAccY = 0.0f;
    int moveX = (int)cursorAccX; // trunc towards zero
    int moveY = (int)cursorAccY;
    if (moveX != 0 || moveY != 0) {
      // send via Wi-Fi UDP if enabled, otherwise use BLE
      int8_t sx = (int8_t)constrain(moveX, -127, 127);
      int8_t sy = (int8_t)constrain(moveY, -127, 127);
      uint8_t btns = 0;
      sendMousePacket(sx, sy, btns);
      cursorAccX -= moveX;
      cursorAccY -= moveY;
    }

    // Buttons: use edge-detection to avoid accidental continuous clicks
    // Left button: support short-tap click and press-&-hold drag
    const unsigned long LEFT_HOLD_MS = 500; // ms to start drag (press)
    if (leftPressed && !prevLeftPressed) {
      // newly pressed
      leftPressStartMs = now;
    } else if (leftPressed && prevLeftPressed) {
      // still held -> check for hold threshold to start dragging
      if (!leftHoldDragging && (now - leftPressStartMs) >= LEFT_HOLD_MS) {
        Serial.println("Button: LEFT press (start drag)");
        if (bleMouse.isConnected()) bleMouse.press(MOUSE_LEFT);
        leftHoldDragging = true;
      }
    } else if (!leftPressed && prevLeftPressed) {
      // just released
      unsigned long held = now - leftPressStartMs;
      if (leftHoldDragging) {
        // end dragging
        Serial.println("Button: LEFT release (end drag)");
        if (bleMouse.isConnected()) bleMouse.release(MOUSE_LEFT);
        leftHoldDragging = false;
      } else {
        // short tap -> click (only if not moving and throttle interval passed)
        unsigned long since = now - lastLeftClickMs;
        float moveSpeed = sqrt(smoothed_dx*smoothed_dx + smoothed_dy*smoothed_dy);
        if (since >= MIN_CLICK_INTERVAL_MS && moveSpeed <= CLICK_MOVE_THRESHOLD) {
          Serial.println("Button: LEFT click (tap)");
          if (useWiFiMouse) {
            sendMousePacket(0, 0, 0x01);
          } else if (bleMouse.isConnected()) bleMouse.click(MOUSE_LEFT);
          lastLeftClickMs = now;
        } else {
          Serial.print("LEFT click suppressed: dt="); Serial.print(since);
          Serial.print("ms move="); Serial.println(moveSpeed);
        }
      }
    }
    prevLeftPressed = leftPressed;

    // Back button presentation control (BLE path simplified to Wi-Fi packet send for host handling)
    if (backPressed && !prevBackPressed) {
      unsigned long sinceLastTap = now - lastBackTapMs;
      lastBackTapMs = now;
      if (pendingSingleBack && sinceLastTap <= DOUBLE_TAP_WINDOW_MS) {
        pendingSingleBack = false;
        sendKeyPacket(3); // previous slide
      } else {
        pendingSingleBack = true;
      }
      backHoldStartMs = now;
      escHoldTriggered = false;
    }
    if (backPressed) {
      if (backHoldStartMs && presentationStarted && !escHoldTriggered && (now - backHoldStartMs) > BACK_LONG_PRESS_MS) {
        sendKeyPacket(4); // Esc end presentation
        presentationStarted = false;
        escHoldTriggered = true;
        pendingSingleBack = false;
      }
    } else {
      backHoldStartMs = 0;
      escHoldTriggered = false;
    }
    if (pendingSingleBack && (now - lastBackTapMs) > DOUBLE_TAP_WINDOW_MS) {
      if (!presentationStarted) {
        sendKeyPacket(1); presentationStarted = true; // start show
      } else {
        sendKeyPacket(2); // next slide
      }
      pendingSingleBack = false;
    }
    prevBackPressed = backPressed;

    // Right button: short tap -> right-click, long hold -> enable scroll mode (handled above by rightPressed state)
    const unsigned long RIGHT_HOLD_MS = 500; // hold threshold
    if (rightPressed && !prevRightPressed) {
      // just pressed
      rightPressStartMs = now;
    } else if (!rightPressed && prevRightPressed) {
      // just released
      unsigned long held = now - rightPressStartMs;
      if (held < RIGHT_HOLD_MS) {
        // short tap -> right click (with throttle & movement guard)
        unsigned long since = now - lastRightClickMs;
        float moveSpeed = sqrt(smoothed_dx*smoothed_dx + smoothed_dy*smoothed_dy);
        if (since >= MIN_CLICK_INTERVAL_MS && moveSpeed <= CLICK_MOVE_THRESHOLD) {
          Serial.println("Button: RIGHT click (tap)");
          if (useWiFiMouse) sendMousePacket(0,0,0x02);
          else bleMouse.click(MOUSE_RIGHT);
          lastRightClickMs = now;
        } else {
          Serial.print("RIGHT click suppressed: dt="); Serial.print(since);
          Serial.print("ms move="); Serial.println(moveSpeed);
        }
      } else {
        // long press released -> nothing to do (scroll mode is transient while held)
      }
    }
    prevRightPressed = rightPressed;

  } else {
    // Not connected
    digitalWrite(PIN_LED, now % 500 < 250 ? HIGH : LOW); // blink
  }
#endif

  // Handle captive portal HTTP when in AP mode
  if (WiFi.getMode() == WIFI_AP) {
    server.handleClient();
  }

  // Disarm DRD flag after the window if we didn't reset again
  if (DRD_FLAG == DRD_MAGIC) {
    unsigned long since = now - drdArmedMs;
    if (since > DRD_WINDOW_MS) {
      DRD_FLAG = 0;
    }
  }

  // Keep-alive for BLE advertising when BLE mode is enabled
#if USE_BLE_MOUSE
  static unsigned long lastAdvWatchdog = 0;
  const unsigned long ADV_WATCHDOG_MS = 5000;
  if (!useWiFiMouse && !bleMouse.isConnected() && (now - lastAdvWatchdog) > ADV_WATCHDOG_MS) {
    lastAdvWatchdog = now;
    Serial.println("BLE adv watchdog: ensuring advertising...");
    bleMouse.begin();
  }
#endif

  // Battery monitor (periodic, non-blocking)
  static unsigned long lastBatCheck = 0;
  if (now - lastBatCheck > 2000) {
    lastBatCheck = now;
    float v = readBatteryVoltage();
    Serial.print("Battery: "); Serial.println(v);
    // Broadcast battery voltage over UDP for desktop app (header 0xB0)
    if (useWiFiMouse) {
      uint16_t mv = (uint16_t)(v * 1000.0f + 0.5f); // millivolts approx
      uint8_t pct = voltageToPercent(v);
      uint8_t pkt[5];
      pkt[0] = 0xB0;           // battery packet header
      pkt[1] = udp_seq++;      // sequence
      pkt[2] = (uint8_t)(mv & 0xFF);
      pkt[3] = (uint8_t)((mv >> 8) & 0xFF);
      pkt[4] = pct;            // percent (0..100)
      udp.beginPacket("255.255.255.255", UDP_PORT);
      udp.write(pkt, sizeof(pkt));
      udp.endPacket();
    }
    // Non-blocking low-battery LED blink
    static bool lowBlink = false;
    static unsigned long lastBlink = 0;
    if (v < LOW_BATT_THRESHOLD && eventBlinkEnd == 0) {
      if (now - lastBlink > 120) {
        lastBlink = now;
        lowBlink = !lowBlink;
        digitalWrite(PIN_LED, lowBlink ? HIGH : LOW);
      }
    }
  }

  // Small yield to background tasks
  delay(0);

  // Service event blink (non-blocking confirmation)
  if (eventBlinkEnd) {
    if (now < eventBlinkEnd) {
      if (now - eventBlinkLast > 100) {
        eventBlinkLast = now;
        eventBlinkOn = !eventBlinkOn;
        digitalWrite(PIN_LED, eventBlinkOn ? HIGH : LOW);
      }
    } else {
      eventBlinkEnd = 0;
      eventBlinkOn = false;
      digitalWrite(PIN_LED, LOW);
    }
  }
}

// ---- Utility functions ----

// computeAccelAngles removed: using DMP-provided angles from MPU6050_tockn instead

float applyDeadzone(float valDeg, float dead) {
  if (fabs(valDeg) < dead) return 0.0f;
  return valDeg > 0 ? (valDeg - dead) : (valDeg + dead);
}

// Soft deadzone with smooth cubic transition to avoid a hard step at the threshold
float softDeadzone(float x, float dead, float width) {
  float ax = fabs(x);
  if (ax <= dead) return 0.0f;
  float sign = (x >= 0) ? 1.0f : -1.0f;
  if (ax >= dead + width) return sign * (ax - dead);
  // In the transition band [dead, dead+width], ease-in with cubic (Hermite)
  float u = (ax - dead) / width; // 0..1
  float eased = u * u * (3.0f - 2.0f * u); // smoothstep
  return sign * eased * (ax - dead);
}

float mapTiltToSpeed(float tiltDeg, float sens) {
  // Map tilt degrees to pixels per loop step. Exponential mapping gives finer low-end control
  float sign = (tiltDeg >= 0) ? 1.0f : -1.0f;
  float mag = fabs(tiltDeg);
  // A curve: speed = k * (mag^1.5)
  float k = sens * 0.08f; // tune this
  float speed = k * pow(mag, 1.5f);
  return sign * speed;
}

// Simple stable non-linear mapping
float mapTiltToSpeedSimple(float tiltDeg, float sens, float gammaExp) {
  float sign = (tiltDeg >= 0) ? 1.0f : -1.0f;
  float mag = fabs(tiltDeg);
  float speed = sens * 0.06f * pow(mag, gammaExp);
  return sign * speed;
}

void detectFlick(float gx, float gy, float gz, unsigned long now) {
  // If any gyro component magnitude > threshold -> flick
  float gmag = sqrt(gx*gx + gy*gy + gz*gz);
  // movement guard: if the cursor is moving, ignore flick gestures
  float moveSpeed = sqrt(smoothed_dx*smoothed_dx + smoothed_dy*smoothed_dy);
  if (moveSpeed > flickMoveGuard) {
    // suppressed due to movement
    lastGmag = gmag;
    return;
  }

  // simple peak detection: trigger only when current gmag exceeds threshold AND previous measured gmag
  // was substantially lower (avoids locking onto sustained noise)
  if ((now - lastFlickTime) > flickTimeWindow) {
    if (gmag > flickGyroThreshold && lastGmag < (flickGyroThreshold * flickPrevMaxRatio)) {
      // require a dominant axis to be above a per-axis minimum to avoid ambiguous small spikes
      if (!(abs(gx) > flickAxisMin || abs(gy) > flickAxisMin || abs(gz) > flickAxisMin)) {
        Serial.print("Flick suppressed: gmag="); Serial.print(gmag);
        Serial.print(" axis too small\n");
        lastGmag = gmag;
        return;
      }

      lastFlickTime = now;
      Serial.print("Flick detected! gmag="); Serial.println(gmag);
      // Basic mapping: left-right flick -> browser back/forward
      if (abs(gx) > abs(gy) && abs(gx) > flickAxisMin) {
        if (gx > 0) sendBrowserBack();
        else sendBrowserForward();
      } else if (abs(gy) > abs(gx) && abs(gy) > flickAxisMin) {
        if (gy > 0) sendBrowserBack();
        else sendBrowserForward();
      } else {
        // fallback: use gz if dominant
        if (abs(gz) > flickAxisMin) {
          if (gz > 0) sendBrowserBack();
          else sendBrowserForward();
        } else {
          Serial.println("Flick suppressed: no dominant axis");
        }
      }
    } else {
      // suppressed - either not a peak or within debounce window
      if (gmag > 0) {
        Serial.print("Flick ignored: gmag="); Serial.print(gmag);
        Serial.print(" lastGmag="); Serial.println(lastGmag);
      }
    }
  }
  lastGmag = gmag;
}

void sendBrowserBack() {
  Serial.println("Gesture: BACK");
  // Some BleMouse implementations support back/forward
  // If not supported, you can send keyboard shortcuts (Alt+Left, etc.). Here we try mouse extra buttons:
#if USE_BLE_MOUSE
  if (bleMouse.isConnected()) {
    bleMouse.press(MOUSE_BACK);
    delay(50);
    bleMouse.release(MOUSE_BACK);
  }
#endif
}

void sendBrowserForward() {
  Serial.println("Gesture: FORWARD");
#if USE_BLE_MOUSE
  if (bleMouse.isConnected()) {
    bleMouse.press(MOUSE_FORWARD);
    delay(50);
    bleMouse.release(MOUSE_FORWARD);
  }
#endif
}

// Send a presentation key packet over UDP: header 0xC0, seq, keyId
// keyId: 1=F5(start), 2=RIGHT(next), 3=LEFT(previous), 4=ESC(end presentation)
void sendKeyPacket(uint8_t keyId) {
  if (!useWiFiMouse) return; // only via Wi-Fi
  uint8_t pkt[4];
  pkt[0] = 0xC0; // key packet header
  pkt[1] = udp_seq++;
  pkt[2] = keyId;
  pkt[3] = 0; // reserved
  udp.beginPacket("255.255.255.255", UDP_PORT);
  udp.write(pkt, sizeof(pkt));
  udp.endPacket();
  Serial.print("Key packet sent id="); Serial.println(keyId);
}

float readBatteryVoltage() {
  // Read ADC and calculate battery voltage based on divider
  int raw = analogRead(PIN_BAT_ADC);
  // ESP32 ADC returns 0..4095 default (check your core & attenuation config)
  float adcMax = 4095.0f;
  float v_adc = (raw / adcMax) * ADC_REF;
  float batteryV = v_adc * VOLT_DIVIDER_RATIO;
  return batteryV;
}

// Improved LiPo single-cell SoC approximation for a typical 3.7V (1S) cell.
// Based on a blended rest/discharge curve (light load). Voltage under load will sag;
// treat this as an estimate. Below ~3.50V we rapidly approach empty; above 4.20V clamp.
// If you routinely measure under higher load, consider adding IR compensation.
uint8_t voltageToPercent(float v) {
  // Hard clamps
  if (v <= 3.50f) return 0;   // treat <3.50V as effectively empty to protect the cell
  if (v >= 4.20f) return 100;
  // Table of (voltage, percent). We'll interpolate between nearest lower & upper points.
  struct VP { float vv; uint8_t pp; };
  static const VP curve[] = {
    //   V      %
    {3.50f,  0},
    {3.55f,  3},
    {3.58f,  6},
    {3.60f,  9},
    {3.63f, 12},
    {3.66f, 16},
    {3.69f, 20},
    {3.71f, 24},
    {3.73f, 28},
    {3.75f, 33},
    {3.78f, 38},
    {3.80f, 43},
    {3.83f, 49},
    {3.85f, 54},
    {3.87f, 58},
    {3.89f, 62},
    {3.91f, 66},
    {3.94f, 70},
    {3.96f, 73},
    {3.98f, 76},
    {4.00f, 80},
    {4.03f, 84},
    {4.06f, 88},
    {4.09f, 92},
    {4.12f, 95},
    {4.15f, 97},
    {4.18f, 99},
    {4.20f,100}
  };
  // Find segment
  const int N = sizeof(curve)/sizeof(curve[0]);
  for (int i = 1; i < N; ++i) {
    if (v <= curve[i].vv) {
      float v0 = curve[i-1].vv, v1 = curve[i].vv;
      uint8_t p0 = curve[i-1].pp, p1 = curve[i].pp;
      float t = (v - v0) / (v1 - v0);
      float pf = p0 + t * (p1 - p0);
      if (pf < 0) pf = 0; if (pf > 100) pf = 100;
      return (uint8_t)(pf + 0.5f);
    }
  }
  return 0; // fallback (should not reach due to clamps)
}

// Read WHO_AM_I (0x75) from MPU6050 at default address 0x68
uint8_t readWhoAmI() {
  const uint8_t MPU_ADDR = 0x68;
  const uint8_t WHO_AM_I_REG = 0x75;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I_REG);
  if (Wire.endTransmission(false) != 0) return 0xFF; // NACK
  Wire.requestFrom((int)MPU_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

uint8_t readRegisterNoRestart(uint8_t devaddr, uint8_t reg) {
  // write register and endTransmission(true) which sends a stop
  Wire.beginTransmission(devaddr);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return 0xFF;
  Wire.requestFrom((int)devaddr, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

uint8_t readRegisterWithRestart(uint8_t devaddr, uint8_t reg) {
  // write register then repeated start (endTransmission(false)) and request
  Wire.beginTransmission(devaddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)devaddr, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

uint8_t readRegisterEndThenRequest(uint8_t devaddr, uint8_t reg) {
  // low-level: send stop then request (some devices behave differently)
  Wire.beginTransmission(devaddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((int)devaddr, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// ---- Calibration helpers ----

bool loadCalibration(float &outPitchOff, float &outRollOff) {
  if (!prefs.isKey("cal_pitch") || !prefs.isKey("cal_roll")) return false;
  outPitchOff = prefs.getFloat("cal_pitch", 0.0f);
  outRollOff  = prefs.getFloat("cal_roll", 0.0f);
  // Basic sanity: offsets within +/- 30 degrees
  if (fabs(outPitchOff) > 30.0f || fabs(outRollOff) > 30.0f) return false;
  return true;
}

void saveCalibration(float pitchOff, float rollOff) {
  prefs.putFloat("cal_pitch", pitchOff);
  prefs.putFloat("cal_roll", rollOff);
}

void runCalibrationInteractive() {
  Serial.println("Orientation calibration: keep device at natural neutral position and still...");
  const unsigned long CAL_MS = 2500;
  unsigned long start = millis();
  // Simple averaging; if large movement detected, extend window slightly
  float sumP = 0, sumR = 0; int n = 0;
  float lastP = mpu.getAngleX();
  float lastR = mpu.getAngleY();
  float moveAccum = 0;
  while (millis() - start < CAL_MS) {
    mpu.update();
    float p = mpu.getAngleX();
    float r = mpu.getAngleY();
    sumP += p; sumR += r; n++;
    moveAccum += fabs(p - lastP) + fabs(r - lastR);
    lastP = p; lastR = r;
    digitalWrite(PIN_LED, (millis() / 100) % 2);
    delay(10);
  }
  // If moved too much, warn but proceed
  if (moveAccum > 50.0f) {
    Serial.print("Calibration detected motion (score="); Serial.print(moveAccum); Serial.println("). Results may be less accurate.");
  }
  float offP = (n > 0) ? (sumP / n) : 0.0f;
  float offR = (n > 0) ? (sumR / n) : 0.0f;
  pitchOffset = offP; rollOffset = offR;
  saveCalibration(pitchOffset, rollOffset);
  Serial.print("Saved calibration: pitchOff="); Serial.print(pitchOffset);
  Serial.print(" rollOff="); Serial.println(rollOffset);
  digitalWrite(PIN_LED, LOW);
}

// Allow live zeroing: when the user holds all three buttons for ~1s, set current orientation as zero (pitch/roll)
void checkLiveZeroing(float curPitch, float curRoll, unsigned long now) {
  static unsigned long tripleStart = 0;
  static unsigned long pairLRStart = 0;
  static unsigned long pairRBStart = 0;
  static unsigned long pairLBStart = 0;
  bool left = (digitalRead(PIN_LEFT_BTN) == LOW);
  bool right = (digitalRead(PIN_RIGHT_BTN) == LOW);
  bool back = (digitalRead(PIN_BACK_BTN) == LOW);

  // Triple: live zero
  if (left && right && back) {
    if (tripleStart == 0) tripleStart = now;
    if (now - tripleStart > 900) {
      pitchOffset = curPitch;
      rollOffset  = curRoll;
      saveCalibration(pitchOffset, rollOffset);
      Serial.println("Live zero set: current orientation -> 0,0");
      eventBlinkEnd = now + 400; // short confirmation blink
      tripleStart = now + 60000; // prevent immediate retrigger
    }
  } else {
    tripleStart = 0;
  }

  // Pairs: mapping toggles (guarded so they don't trigger during triple)
  if (!back && left && right) {
    if (pairLRStart == 0) pairLRStart = now;
    if (now - pairLRStart > 900) {
      invertX = !invertX; saveMappingSettings();
      Serial.print("InvertX toggled -> "); Serial.println(invertX);
      eventBlinkEnd = now + 400;
      pairLRStart = now + 60000;
    }
  } else {
    pairLRStart = 0;
  }

  if (!left && right && back) {
    if (pairRBStart == 0) pairRBStart = now;
    if (now - pairRBStart > 900) {
      invertY = !invertY; saveMappingSettings();
      Serial.print("InvertY toggled -> "); Serial.println(invertY);
      eventBlinkEnd = now + 400;
      pairRBStart = now + 60000;
    }
  } else {
    pairRBStart = 0;
  }

  if (!right && left && back) {
    if (pairLBStart == 0) pairLBStart = now;
    if (now - pairLBStart > 900) {
      swapXY = !swapXY; saveMappingSettings();
      Serial.print("SwapXY toggled -> "); Serial.println(swapXY);
      eventBlinkEnd = now + 400;
      pairLBStart = now + 60000;
    }
  } else {
    pairLBStart = 0;
  }
}

bool loadMappingSettings() {
  if (!prefs.isKey("invX")) { return false; }
  invertX = prefs.getBool("invX", false);
  invertY = prefs.getBool("invY", false);
  swapXY  = prefs.getBool("swXY", false);
  return true;
}

void saveMappingSettings() {
  prefs.putBool("invX", invertX);
  prefs.putBool("invY", invertY);
  prefs.putBool("swXY", swapXY);
}
