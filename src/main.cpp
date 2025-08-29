// ---------------- Lost line search timeout ----------------
const unsigned long LOST_LINE_TIMEOUT_MS = 3000; // Stop after 3 seconds of searching
/* ===============================================================================================
  Ohm's revenge Line Follower Robot prototype
  PASADENA CITY COLLEGE
  Hector Miranda, 2025

  STM32F103C8T6 (Blue Pill)
  Robot: 2x DC motors with L298N + 6x TCRT5000
  Framework: Arduino core for STM32 (STM32duino)

  Remarks:
  L298N is only used for the prototype version, not for actual bright manufacturing challenge
  Use runtime switch via serial:

  [l] line, [t] fake-line, [m] motor-test, [e] led-flash, [s] sensor-test"
  
  Fake-line: 'a'/'d' to step -/+, 's' zero, '1'..'5' presets ±{1,3,5}



  TODO:
    - Test and calibrate sensors
    - Test line following algorithm
    - Add switch for prototype mode and PCB BMF mode. (implement PCB mode)

  ================================================================================================ */

#include <Arduino.h>
#include <cstdint>

#define FEAT_LINE_FOLLOWER 1
#define FEAT_FAKE_LINE_TEST 1
#define FEAT_MOTOR_TEST 1

#define FEAT_SERIAL_SWITCH 1 // TODO: test the enable runtime switch via serial (l/t/m, ?)
#define FEAT_LED_TEST 1
#define LED_PIN PC13


// Modes for runtime switching
enum Mode
{
  MODE_LINE = 0,      // Line following
  MODE_TEST = 1,      // Fake line test (PID debug)
  MODE_MOTOR = 2,     // Motor test
  MODE_LED = 3,       // LED blink test
  MODE_SENSOR_TEST = 4 // Single TCRT5000 sensor test
};

Mode currentMode = MODE_LINE; // <--- START HERE (default)

// For sensor test mode
uint8_t sensorTestIndex = 2; // 0..5

const uint8_t ENA = PA5;                                       // Left PWM
const uint8_t IN1 = PA4;                                       // Left dir +
const uint8_t IN2 = PA3;                                       // Left dir -
const uint8_t ENB = PA0;                                       // Right PWM
const uint8_t IN3 = PA2;                                       // Right dir +
const uint8_t IN4 = PA1;                                       // Right dir -
const uint8_t SENSORS[6] = {PB3, PB4, PB5, PB6, PB7, PB8}; // 6x TCRT5000 (digital outputs)
const int8_t WEIGHTS[6] = {-5, -3, -1, 1, 3, 5};               // weights centered around 0 (TODO: depending on spacing we may need to adjust)
bool SENSOR_ACTIVE_LOW = false;                                // true: LOW=on-line; false: HIGH=on-line, TODO: If the TCRT boards output HIGH on black, we should flip this:

// ---------------- Motion / PWM -----------------------------------------
const int PWM_MAX = 1000;        // STM32 TIM1 default
const bool INVERT_LEFT = true;  // set true if left motor wired reversed
const bool INVERT_RIGHT = false; // set true if right motor wired reversed

// ---------------- PID (line follower) ----------------------------
float KP = 13.0f;
float KI = 0.0f;
float KD = 2.5f;
int BASE_SPEED = 200; // start slower to tune, e.g. 400..600
int MAX_SPEED = 1000;

float pidIntegral = 0.0f;
float pidLastErr = 0.0f;

// ---------------- Fake line test ---------------- TODO: test this.
int TEST_err_value = 0; // set at runtime with serial: 't' then +/- keys

// ---------------- Utils ---------------------------------------------
inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); } // this function clamps an integer value between a lower and upper bound, so that it cannot exceed these limits.

inline void motorLeft(int speed)
{
  if (INVERT_LEFT)
    speed = -speed;
  speed = clampi(speed, -PWM_MAX, PWM_MAX);
  if (speed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  }
}
inline void motorRight(int speed)
{
  if (INVERT_RIGHT)
    speed = -speed;
  speed = clampi(speed, -PWM_MAX, PWM_MAX);
  if (speed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  }
}
inline void motorsStop()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------------- Sensors ----------------
int readLineError(bool &seen)
{
  long sum = 0, wsum = 0;
  seen = false;

  Serial.print("SENSORS: [ ");
  for (uint8_t i = 0; i < 6; i++)
  {
    int raw = digitalRead(SENSORS[i]);
    int onLine = SENSOR_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
    Serial.print(onLine ? "1 " : "0 "); // Print raw ON/OFF state

    if (onLine)
    {
      seen = true;
      sum += 1;
      wsum += WEIGHTS[i];
    }
  }
  Serial.print("] ");

  if (!seen || sum == 0)
  {
    int fallback = (pidLastErr >= 0) ? 6 : -6;
    Serial.print("-> LOST LINE, returning ");
    Serial.println(fallback);
    return fallback;
  }

  int err = (int)(wsum / sum);
  Serial.print("-> ERROR = ");
  Serial.println(err);
  return err;
}


// --- Single TCRT5000 Sensor Test Mode ---
// Allows user to select a sensor (0..5) and view its state in real time
void loop_sensorTest()
{
  // Verbose sensor test: use same logic as line following
  long sum = 0, wsum = 0;
  int activeCount = 0;
  Serial.print("SENSORS: [ ");
  for (uint8_t i = 0; i < 6; i++) {
    int raw = digitalRead(SENSORS[i]);
    int onLine = SENSOR_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
    Serial.print(onLine ? "1 " : "0 ");
    if (onLine) {
      activeCount++;
      sum += 1;
      wsum += WEIGHTS[i];
    }
  }
  Serial.print("]  Active: ");
  Serial.print(activeCount);
  Serial.print("  SUM: ");
  Serial.print(sum);
  Serial.print("  WSUM: ");
  Serial.print(wsum);

  bool seen = (activeCount > 0);
  int err = 0;
  if (!seen || sum == 0) {
    int fallback = (pidLastErr >= 0) ? 6 : -6;
    Serial.print("  -> LOST LINE, fallback error: ");
    Serial.print(fallback);
    err = fallback;
  } else {
    err = (int)(wsum / sum);
    Serial.print("  -> ERROR: ");
    Serial.print(err);
  }

  // Simulate PID output as in line follower
  float P = err;
  float D = P - pidLastErr;
  float turn = KP * P + KI * pidIntegral + KD * D;
  Serial.print("  PID: ");
  Serial.print(turn, 2);
  int left = BASE_SPEED - (int)turn;
  int right = BASE_SPEED + (int)turn;
  left = clampi(left, -MAX_SPEED, MAX_SPEED);
  right = clampi(right, -MAX_SPEED, MAX_SPEED);
  Serial.print("  LEFT: ");
  Serial.print(left);
  Serial.print("  RIGHT: ");
  Serial.print(right);

  // Show what the robot would do
  if (!seen) {
    Serial.print("  ACTION: LOST LINE (would wiggle/stop)");
    motorsStop();
  } else {
    if (left > right) Serial.print("  ACTION: TURN LEFT");
    else if (right > left) Serial.print("  ACTION: TURN RIGHT");
    else Serial.print("  ACTION: FORWARD");
    motorLeft(left);
    motorRight(right);
  }
  Serial.println();
  pidLastErr = P;
  delay(300);
}




#if FEAT_LINE_FOLLOWER
void loop_lineFollower()
{
  bool seen = false;
  int err = readLineError(seen);

  static uint8_t lostStep = 0;
  static unsigned long lostTimer = 0;
  static unsigned long lostStart = 0;
  if (!seen) {
    unsigned long now = millis();
    if (lostStart == 0) lostStart = now;
    // If searching too long, stop motors
    if (now - lostStart > LOST_LINE_TIMEOUT_MS) {
      motorsStop();
      return;
    }
    // Wiggle search clockwise: right, back, left, forward
    if (now - lostTimer > 200) { // Change direction every 200ms
      lostStep = (lostStep + 1) % 4;
      lostTimer = now;
    }
    switch (lostStep) {
      case 0: // Turn right
        motorLeft(350); motorRight(-350); break;
      case 1: // Spin back
        motorLeft(-350); motorRight(-350); break;
      case 2: // Turn left
        motorLeft(-350); motorRight(350); break;
      case 3: // Forward
        motorLeft(350); motorRight(350); break;
    }
    delay(2);
    return;
  } else {
    lostStep = 0; // Reset wiggle when line found
    lostStart = 0; // Reset lost timer
  }
  // PID
  float P = err;
  pidIntegral += P;
  float D = P - pidLastErr;
  float turn = KP * P + KI * pidIntegral + KD * D;
  pidLastErr = P;

  int left = BASE_SPEED - (int)turn;
  int right = BASE_SPEED + (int)turn;

  left = clampi(left, -MAX_SPEED, MAX_SPEED);
  right = clampi(right, -MAX_SPEED, MAX_SPEED);

  motorLeft(left);
  motorRight(right);
  delay(2);
}
#endif

#if FEAT_FAKE_LINE_TEST
void loop_fakeLine()
{
  // Modify TEST_err_value at runtime with serial:
  // 'a'/'d' = -/+, 's' = zero; or use numbers '1'..'5' for presets.
  float P = TEST_err_value;
  pidIntegral += P;
  float D = P - pidLastErr;
  float turn = KP * P + KI * pidIntegral + KD * D;
  pidLastErr = P;

  int left = BASE_SPEED - (int)turn;
  int right = BASE_SPEED + (int)turn;

  left = clampi(left, -MAX_SPEED, MAX_SPEED);
  right = clampi(right, -MAX_SPEED, MAX_SPEED);

  motorLeft(left);
  motorRight(right);
  delay(5);
}
#endif

#if FEAT_MOTOR_TEST
void loop_motorTest()
{
  // Simple scripted test: fwd, stop, rev, left, right, stop
  const int S = 500; // PWM
  // forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, S);
  analogWrite(ENB, S);
  delay(1500);

  // stop
  motorsStop();
  delay(800);

  // reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, S);
  analogWrite(ENB, S);
  delay(1500);

  // stop
  motorsStop();
  delay(800);

  // left turn (tank)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, S);
  analogWrite(ENB, S);
  delay(1000);

  // right turn (tank)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, S);
  analogWrite(ENB, S);
  delay(1000);

  motorsStop();
  delay(1200);
}
#endif

#if FEAT_LED_TEST
void loop_ledTest()
{
  static bool ledState = false;
  static unsigned long lastToggle = 0;
  unsigned long now = millis();
  if (now - lastToggle >= 250)
  {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH); // ON = LOW, OFF = HIGH
    lastToggle = now;
  }
}
#endif

void setup()
{
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  motorsStop();

  // Sensors
  for (uint8_t i = 0; i < 6; i++)
    pinMode(SENSORS[i], INPUT_PULLUP); // TCRT boards usually open-collector with pullups

  // LED test mode
#if FEAT_LED_TEST
  pinMode(LED_PIN, OUTPUT);
#endif

#if FEAT_SERIAL_SWITCH
  Serial.begin(115200);
  delay(50);
  Serial.println("\n== Ohm's Revenge Robot Ready ==");
  Serial.println("Modes: [l] line  [t] fake-line  [m] motor-test  [e] led-flash  [s] sensor-test  [?] help");
  Serial.print("Boot mode: ");
  switch (currentMode) {
    case MODE_LINE: Serial.println("LINE"); break;
    case MODE_TEST: Serial.println("FAKE-LINE"); break;
    case MODE_MOTOR: Serial.println("MOTOR-TEST"); break;
    case MODE_LED: Serial.println("LED-TEST"); break;
    case MODE_SENSOR_TEST: Serial.println("SENSOR-TEST"); break;
    default: Serial.println("UNKNOWN"); break;
  }
  Serial.println("Fake-line keys: a/d=-/+  s=0  1..5 presets");
  Serial.println("Sensor-test keys: 0..5 to select sensor");
#endif
}

/* ---------------- Serial Control ----------------
Requires the USB to Serial USB to TTL CH340 Module
*/
#if FEAT_SERIAL_SWITCH
void handleSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == 'l') {
      currentMode = MODE_LINE;
      Serial.println("-> MODE_LINE");
    } else if (c == 't') {
      currentMode = MODE_TEST;
      Serial.println("-> MODE_FAKE_LINE");
    } else if (c == 'm') {
      currentMode = MODE_MOTOR;
      Serial.println("-> MODE_MOTOR_TEST");
    } else if (c == 'e') {
      currentMode = MODE_LED;
      Serial.println("-> MODE_LED_TEST");
    } else if (c == 's') {
      currentMode = MODE_SENSOR_TEST;
      Serial.println("-> MODE_SENSOR_TEST");
    } else if (c == '?') {
      Serial.println("[l] line, [t] fake-line, [m] motor-test, [e] led-flash, [s] sensor-test");
      Serial.println("Fake-line: 'a'/'d' to step -/+, 's' zero, '1'..'5' presets ±{1,3,5}");
      Serial.println("Sensor-test: 0..5 to select sensor");
    }
    // Fake-line test controls
#if FEAT_FAKE_LINE_TEST
    else if (currentMode == MODE_TEST && c == 'a') {
      TEST_err_value--;
      Serial.print("err=");
      Serial.println(TEST_err_value);
    } else if (currentMode == MODE_TEST && c == 'd') {
      TEST_err_value++;
      Serial.print("err=");
      Serial.println(TEST_err_value);
    } else if (currentMode == MODE_TEST && c == 's') {
      TEST_err_value = 0;
      Serial.print("err=");
      Serial.println(TEST_err_value);
    } else if (currentMode == MODE_TEST && c >= '1' && c <= '5') {
      int v = (c - '0');
      TEST_err_value = (pidLastErr >= 0) ? v : -v; // bias to last sign
      Serial.print("err=");
      Serial.println(TEST_err_value);
    }
#endif
    // Sensor-test controls: select sensor 0..5
    else if (currentMode == MODE_SENSOR_TEST && c >= '0' && c <= '5') {
      sensorTestIndex = c - '0';
      Serial.print("Selected sensor ");
      Serial.println(sensorTestIndex);
    }
  }
}
#endif

void loop()
{
#if FEAT_SERIAL_SWITCH
  handleSerial();
#endif

  switch (currentMode)
  {
#if FEAT_LINE_FOLLOWER
    case MODE_LINE:
      loop_lineFollower();
      break;
#endif
#if FEAT_FAKE_LINE_TEST
    case MODE_TEST:
      loop_fakeLine();
      break;
#endif
#if FEAT_MOTOR_TEST
    case MODE_MOTOR:
      loop_motorTest();
      break;
#endif
#if FEAT_LED_TEST
    case MODE_LED:
      loop_ledTest();
      break;
#endif
    case MODE_SENSOR_TEST:
      loop_sensorTest();
      break;
    default:
      motorsStop();
      delay(10);
      break;
  }
}
