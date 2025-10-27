#include <Arduino.h>

// AirMouse_ESP32_MPU6050.ino
// Requires: Adafruit_MPU6050, Adafruit_Sensor, Wire, BleMouse
// Configure pins below as required for your board

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <BleMouse.h>

MPU6050 mpu(Wire);
// Set BLE device name shown to hosts
BleMouse bleMouse("ESP Air 1");

// Forward declarations for functions defined later in this file
void detectFlick(float gx, float gy, float gz, unsigned long now);
float applyDeadzone(float valDeg, float dead);
float mapTiltToSpeed(float tiltDeg, float sens);
float readBatteryVoltage();
void sendBrowserBack();
void sendBrowserForward();
void i2cScan();
uint8_t readWhoAmI();
uint8_t readRegisterNoRestart(uint8_t devaddr, uint8_t reg);
uint8_t readRegisterWithRestart(uint8_t devaddr, uint8_t reg);
uint8_t readRegisterEndThenRequest(uint8_t devaddr, uint8_t reg);

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
const float alpha_comp = 0.98f; // complementary filter alpha (gyro trust)
const float smoothing_alpha = 0.12f; // EMA smoothing for cursor deltas (0..1) - lower for smoother response
const float gyroToDegPerSec = 1.0f; // gyro reading is deg/s if library gives that
const float sensitivity = 1.6f; // sensitivity multiplier for cursor movement
const float deadzone_deg = 2.0f; // degrees of tilt to ignore
const float maxCursorSpeed = 12.0f; // max pixels per loop step
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

// Battery
const float ADC_REF = 3.3f;
const float VOLT_DIVIDER_RATIO = 2.0f; // example: 100k/100k -> ratio 2.0 (adjust to your resistor values)
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

// For gesture detection
unsigned long lastFlickTime = 0;
// Button previous-state tracking to avoid accidental continuous clicks
bool prevLeftPressed = false;
bool prevRightPressed = false;
bool prevBackPressed = false;
unsigned long rightPressStartMs = 0;
unsigned long leftPressStartMs = 0;
bool leftHoldDragging = false;

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


  lastLoop = millis();
  lastFusionMs = millis();

  bleMouse.begin();
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
  if (now - lastLoop < LOOP_INTERVAL_MS) return;
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
    float tiltY = pitch; // up/down
    tiltY = applyDeadzone(tiltY, deadzone_deg);
    dy = mapTiltToSpeed(tiltY, scrollSensitivity);
    dx = 0;
  } else {
    // Normal mode: roll -> X, pitch -> Y
    float tiltX = roll;
    float tiltY = pitch;
    tiltX = applyDeadzone(tiltX, deadzone_deg);
    tiltY = applyDeadzone(tiltY, deadzone_deg);
    dx = mapTiltToSpeed(tiltX, sensitivity);
    dy = mapTiltToSpeed(tiltY, sensitivity);
  }

  // Smoothing EMA
  smoothed_dx = smoothing_alpha * dx + (1.0f - smoothing_alpha) * smoothed_dx;
  smoothed_dy = smoothing_alpha * dy + (1.0f - smoothing_alpha) * smoothed_dy;

  // Constrain max speed
  smoothed_dx = constrain(smoothed_dx, -maxCursorSpeed, maxCursorSpeed);
  smoothed_dy = constrain(smoothed_dy, -maxCursorSpeed, maxCursorSpeed);

  // Send BLE mouse events
  if (bleMouse.isConnected()) {
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
      bleMouse.move(moveX, moveY);
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
          if (bleMouse.isConnected()) bleMouse.click(MOUSE_LEFT);
          lastLeftClickMs = now;
        } else {
          Serial.print("LEFT click suppressed: dt="); Serial.print(since);
          Serial.print("ms move="); Serial.println(moveSpeed);
        }
      }
    }
    prevLeftPressed = leftPressed;

    // Back button: momentary click on press
    if (backPressed && !prevBackPressed) {
      // throttle/back movement guard
      unsigned long since = now - lastBackClickMs;
      float moveSpeed = sqrt(smoothed_dx*smoothed_dx + smoothed_dy*smoothed_dy);
      if (since >= MIN_CLICK_INTERVAL_MS && moveSpeed <= CLICK_MOVE_THRESHOLD) {
        Serial.println("Button: BACK click");
        bleMouse.click(MOUSE_BACK);
        lastBackClickMs = now;
      } else {
        Serial.print("BACK click suppressed: dt="); Serial.print(since);
        Serial.print("ms move="); Serial.println(moveSpeed);
      }
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
          bleMouse.click(MOUSE_RIGHT);
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

  // Battery monitor (periodic, non-blocking)
  static unsigned long lastBatCheck = 0;
  if (now - lastBatCheck > 2000) {
    lastBatCheck = now;
    float v = readBatteryVoltage();
    Serial.print("Battery: "); Serial.println(v);
    if (v < LOW_BATT_THRESHOLD) {
      // low battery warning: blink LED faster
      for (int i=0;i<2;i++){
        digitalWrite(PIN_LED, HIGH); delay(60);
        digitalWrite(PIN_LED, LOW);  delay(60);
      }
    }
  }

  // Small yield to background tasks
  delay(0);
}

// ---- Utility functions ----

// computeAccelAngles removed: using DMP-provided angles from MPU6050_tockn instead

float applyDeadzone(float valDeg, float dead) {
  if (fabs(valDeg) < dead) return 0.0f;
  return valDeg > 0 ? (valDeg - dead) : (valDeg + dead);
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
  if (bleMouse.isConnected()) {
    bleMouse.press(MOUSE_BACK);
    delay(50);
    bleMouse.release(MOUSE_BACK);
  }
}

void sendBrowserForward() {
  Serial.println("Gesture: FORWARD");
  if (bleMouse.isConnected()) {
    bleMouse.press(MOUSE_FORWARD);
    delay(50);
    bleMouse.release(MOUSE_FORWARD);
  }
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
