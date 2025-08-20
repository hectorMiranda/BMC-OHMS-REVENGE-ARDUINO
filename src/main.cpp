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

  [l] line, [t] fake-line, [m] motor-test, [e] led-flash
  
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

enum Mode
{
  MODE_LINE = 0,
  MODE_TEST = 1,
  MODE_MOTOR = 2,
  MODE_LED = 3
};

Mode currentMode = MODE_LED; // <--- START HERE

const uint8_t ENA = PA5;                                       // Left PWM
const uint8_t IN1 = PA4;                                       // Left dir +
const uint8_t IN2 = PA3;                                       // Left dir -
const uint8_t ENB = PA0;                                       // Right PWM
const uint8_t IN3 = PA2;                                       // Right dir +
const uint8_t IN4 = PA1;                                       // Right dir -
const uint8_t SENSORS[6] = {PB0, PB1, PB10, PB11, PB12, PB13}; // 6x TCRT5000 (digital outputs)
const int8_t WEIGHTS[6] = {-5, -3, -1, 1, 3, 5};               // weights centered around 0 (TODO: depending on spacing we may need to adjust)
bool SENSOR_ACTIVE_LOW = true;                                 // true: LOW=on-line; false: HIGH=on-line, TODO: If the TCRT boards output HIGH on black, we should flip this:

// ---------------- Motion / PWM -----------------------------------------
const int PWM_MAX = 1000;        // STM32 TIM1 default
const bool INVERT_LEFT = false;  // set true if left motor wired reversed
const bool INVERT_RIGHT = false; // set true if right motor wired reversed

// ---------------- PID (line follower) ----------------------------
float KP = 13.0f;
float KI = 0.0f;
float KD = 2.5f;
int BASE_SPEED = 600; // start slower to tune, e.g. 400..600
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


// ---------------- Modes ----------------
#if FEAT_LINE_FOLLOWER
void loop_lineFollower()
{
  bool seen = false;
  int err = readLineError(seen);

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
  Serial.println("Modes: [l] line  [t] fake-line  [m] motor-test  [e] led-flash  [?] help");
  Serial.print("Boot mode: ");
  Serial.println(currentMode == MODE_LINE ? "LINE" : currentMode == MODE_TEST ? "FAKE-LINE"
                                                 : currentMode == MODE_MOTOR  ? "MOTOR-TEST"
                                                 : currentMode == MODE_LED    ? "LED-TEST"
                                                                              : "UNKNOWN");
  Serial.println("Fake-line keys: a/d=-/+  s=0  1..5 presets");
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
    if (c == 'l')
    {
      currentMode = MODE_LINE;
      Serial.println("-> MODE_LINE");
    }
    else if (c == 't')
    {
      currentMode = MODE_TEST;
      Serial.println("-> MODE_FAKE_LINE");
    }
    else if (c == 'm')
    {
      currentMode = MODE_MOTOR;
      Serial.println("-> MODE_MOTOR_TEST");
    }
    else if (c == 'e')
    {
      currentMode = MODE_LED;
      Serial.println("-> MODE_LED_TEST");
    }
    else if (c == '?')
    {
      Serial.println("[l] line, [t] fake-line, [m] motor-test, [e] led-flash");
      Serial.println("Fake-line: 'a'/'d' to step -/+, 's' zero, '1'..'5' presets ±{1,3,5}");
    }
#if FEAT_FAKE_LINE_TEST
    else if (c == 'a')
    {
      TEST_err_value--;
      Serial.print("err=");
      Serial.println(TEST_err_value);
    }
    else if (c == 'd')
    {
      TEST_err_value++;
      Serial.print("err=");
      Serial.println(TEST_err_value);
    }
    else if (c == 's')
    {
      TEST_err_value = 0;
      Serial.print("err=");
      Serial.println(TEST_err_value);
    }
    else if (c >= '1' && c <= '5')
    {
      int v = (c - '0');
      TEST_err_value = (pidLastErr >= 0) ? v : -v; // bias to last sign
      Serial.print("err=");
      Serial.println(TEST_err_value);
    }
#endif
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
    default:
      motorsStop();
      delay(10);
      break;
    }
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
  default:
    motorsStop();
    delay(10);
    break;
  }
}
