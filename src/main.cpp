/* =======================
  Ohm's revenge line follower robot
  PASADENA CITY COLLEGE

  STM32F103C8T6 (Blue Pill)
  Robot: 2x DC motors with L298N + 6x TCRT5000 + HW-504 Joystick
  Framework: Arduino core for STM32 (STM32duino)
  ======================= */

#include <Arduino.h>
#include <cstdint>

// Turn subsystems on/off at compile time (saves flash if needed).
#define FEAT_LINE_FOLLOWER   1
#define FEAT_JOYSTICK        1
#define FEAT_FAKE_LINE_TEST  1
#define FEAT_MOTOR_TEST      1

#define FEAT_SERIAL_SWITCH   1   // enable runtime switch via serial (l/j/t/m, ?)
#define FEAT_LED_TEST        1   // enable LED flash test mode

enum Mode { MODE_LINE=0, MODE_JOYSTICK=1, MODE_TEST=2, MODE_MOTOR=3, MODE_LED=4 };
Mode currentMode = MODE_LED;   // change default here

// ---------------- Pins ----------------
// L298N
const uint8_t ENA = PA8;    // Left PWM
const uint8_t IN1 = PB0;    // Left dir +
const uint8_t IN2 = PB1;    // Left dir -
const uint8_t ENB = PA9;    // Right PWM
const uint8_t IN3 = PB10;   // Right dir +
const uint8_t IN4 = PB11;   // Right dir -

// 6x TCRT5000 (digital outputs)
const uint8_t SENSORS[6] = {PA0, PA1, PA2, PA3, PA4, PA5};
// weights centered around 0 (adjust for your spacing)
const int8_t  WEIGHTS[6]  = {-5, -3, -1, 1, 3, 5};
// If your TCRT boards output HIGH on black, flip this:
bool SENSOR_ACTIVE_LOW = true;  // true: LOW=on-line; false: HIGH=on-line

// Joystick (power at 3.3V)
const uint8_t JOY_X = PA6;   // VRx (left/right)  ADC
const uint8_t JOY_Y = PA7;   // VRy (fwd/back)    ADC
const uint8_t JOY_SW = PB12; // Button (active LOW)

// ---------------- Motion / PWM ----------------
const int PWM_MAX = 1000;        // STM32 TIM1 default
const bool INVERT_LEFT  = false; // set true if left motor wired reversed
const bool INVERT_RIGHT = false; // set true if right motor wired reversed

// ---------------- PID (line follower) ----------------
float KP = 13.0f;
float KI = 0.0f;
float KD = 2.5f;
int   BASE_SPEED = 600;      // start slower to tune, e.g. 400..600
int   MAX_SPEED  = 1000;

float pidIntegral = 0.0f;
float pidLastErr  = 0.0f;

// ---------------- Joystick mapping ----------------
const int ADC_MIN = 0, ADC_MAX = 4095;
const int ADC_CEN = (ADC_MIN + ADC_MAX)/2;
const int DEAD    = 120;           // deadzone around center (ADC units)
const float TURN_GAIN  = 1.0f;     // >1 = stronger turning
const float SPEED_GAIN = 1.0f;     // 0..1.5 typical

// ---------------- Fake line test ----------------
int TEST_err_value = 0; // set at runtime with serial: 't' then +/- keys

// ---------------- Utils ----------------
inline int clampi(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }

inline void motorLeft(int speed){
  if (INVERT_LEFT) speed = -speed;
  speed = clampi(speed, -PWM_MAX, PWM_MAX);
  if (speed >= 0){ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, speed); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, -speed); }
}
inline void motorRight(int speed){
  if (INVERT_RIGHT) speed = -speed;
  speed = clampi(speed, -PWM_MAX, PWM_MAX);
  if (speed >= 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, speed); }
  else           { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, -speed); }
}
inline void motorsStop(){ analogWrite(ENA,0); analogWrite(ENB,0); }

// ADC center map to -PWM_MAX..+PWM_MAX with deadzone
int adcToCentered(int raw){
  int d = raw - ADC_CEN;
  if (abs(d) < DEAD) return 0;
  int span = (ADC_MAX - ADC_CEN - DEAD);
  long v = (long)(d > 0 ? (d - DEAD) : (d + DEAD));
  long out = (v * PWM_MAX) / span;
  return clampi((int)out, -PWM_MAX, PWM_MAX);
}

// ---------------- Sensors ----------------
int readLineError(bool &seen){
  long sum=0, wsum=0;
  seen = false;
  for(uint8_t i=0;i<6;i++){
    int raw = digitalRead(SENSORS[i]);
    int onLine = SENSOR_ACTIVE_LOW ? (raw==LOW) : (raw==HIGH);
    if(onLine) { seen=true; sum += 1; wsum += WEIGHTS[i]; }
  }
  if (!seen || sum==0){
    // Lost line: nudge toward last side
    return (pidLastErr >= 0) ? 6 : -6;
  }
  return (int)(wsum / sum);
}

// ---------------- Modes ----------------
#if FEAT_LINE_FOLLOWER
void loop_lineFollower(){
  bool seen=false;
  int err = readLineError(seen);

  // PID
  float P = err;
  pidIntegral += P;
  float D = P - pidLastErr;
  float turn = KP*P + KI*pidIntegral + KD*D;
  pidLastErr = P;

  int left  = BASE_SPEED - (int)turn;
  int right = BASE_SPEED + (int)turn;

  left  = clampi(left,  -MAX_SPEED, MAX_SPEED);
  right = clampi(right, -MAX_SPEED, MAX_SPEED);

  motorLeft(left);
  motorRight(right);
  delay(2);
}
#endif

#if FEAT_JOYSTICK
void loop_joystick(){
  int rx = analogRead(JOY_X);
  int ry = analogRead(JOY_Y);
  bool pressed = (digitalRead(JOY_SW)==LOW);

  int turn     = (int)(adcToCentered(rx) * TURN_GAIN);
  int throttle = (int)(adcToCentered(ry) * SPEED_GAIN);

  if (pressed){ motorsStop(); return; }

  int left  = throttle - turn;
  int right = throttle + turn;

  left  = clampi(left,  -PWM_MAX, PWM_MAX);
  right = clampi(right, -PWM_MAX, PWM_MAX);

  motorLeft(left);
  motorRight(right);
  delay(5);
}
#endif

#if FEAT_FAKE_LINE_TEST
void loop_fakeLine(){
  // Modify TEST_err_value at runtime with serial:
  // 'a'/'d' = -/+, 's' = zero; or use numbers '1'..'5' for presets.
  float P = TEST_err_value;
  pidIntegral += P;
  float D = P - pidLastErr;
  float turn = KP*P + KI*pidIntegral + KD*D;
  pidLastErr = P;

  int left  = BASE_SPEED - (int)turn;
  int right = BASE_SPEED + (int)turn;

  left  = clampi(left,  -MAX_SPEED, MAX_SPEED);
  right = clampi(right, -MAX_SPEED, MAX_SPEED);

  motorLeft(left);
  motorRight(right);
  delay(5);
}
#endif

#if FEAT_MOTOR_TEST
void loop_motorTest(){
  // Simple scripted test: fwd, stop, rev, left, right, stop
  const int S = 500; // PWM
  // forward
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA,S); analogWrite(ENB,S); delay(1500);

  // stop
  motorsStop(); delay(800);

  // reverse
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  analogWrite(ENA,S); analogWrite(ENB,S); delay(1500);

  // stop
  motorsStop(); delay(800);

  // left turn (tank)
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA,S); analogWrite(ENB,S); delay(1000);

  // right turn (tank)
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  analogWrite(ENA,S); analogWrite(ENB,S); delay(1000);

  motorsStop(); delay(1200);
}
#endif

#if FEAT_LED_TEST
void loop_ledTest(){
  static bool ledState = false;
  static unsigned long lastToggle = 0;
  unsigned long now = millis();
  if (now - lastToggle >= 250) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    lastToggle = now;
  }
}
#endif

// ---------------- Setup ----------------
void setup(){
  // Motor pins
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
    pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  motorsStop();

  // Sensors
  for(uint8_t i=0;i<6;i++) pinMode(SENSORS[i], INPUT_PULLUP); // TCRT boards usually open-collector with pullups

  // Joystick
  pinMode(JOY_SW, INPUT_PULLUP);
  // LED test mode
#if FEAT_LED_TEST
  pinMode(LED_BUILTIN, OUTPUT);
#endif

#if FEAT_SERIAL_SWITCH
  Serial.begin(115200);
  delay(50);
  Serial.println("\n== Robot Ready ==");
  Serial.println("Modes: [l] line  [j] joystick  [t] fake-line  [m] motor-test  [e] led-flash  [?] help");
  Serial.print("Boot mode: ");
  Serial.println(currentMode==MODE_LINE?"LINE":
                 currentMode==MODE_JOYSTICK?"JOYSTICK":
                 currentMode==MODE_TEST?"FAKE-LINE":
                 currentMode==MODE_MOTOR?"MOTOR-TEST":
                 currentMode==MODE_LED?"LED-TEST":"UNKNOWN");
  Serial.println("Fake-line keys: a/d=-/+  s=0  1..5 presets");
#endif
}

/* ---------------- Serial Control ----------------
Requires the USB to Serial USB to TTL CH340 Module 
*/
#if FEAT_SERIAL_SWITCH
void handleSerial(){
  while(Serial.available()){
    char c = Serial.read();
    if(c=='l'){ currentMode = MODE_LINE;      Serial.println("-> MODE_LINE");  }
    else if(c=='j'){ currentMode = MODE_JOYSTICK; Serial.println("-> MODE_JOYSTICK"); }
    else if(c=='t'){ currentMode = MODE_TEST;     Serial.println("-> MODE_FAKE_LINE"); }
    else if(c=='m'){ currentMode = MODE_MOTOR;    Serial.println("-> MODE_MOTOR_TEST"); }
    else if(c=='e'){ currentMode = MODE_LED;      Serial.println("-> MODE_LED_TEST"); }
    else if(c=='?'){
      Serial.println("[l] line, [j] joystick, [t] fake-line, [m] motor-test, [e] led-flash");
      Serial.println("Fake-line: 'a'/'d' to step -/+, 's' zero, '1'..'5' presets ±{1,3,5}");
    }
#if FEAT_FAKE_LINE_TEST
    else if(c=='a'){ TEST_err_value--; Serial.print("err="); Serial.println(TEST_err_value); }
    else if(c=='d'){ TEST_err_value++; Serial.print("err="); Serial.println(TEST_err_value); }
    else if(c=='s'){ TEST_err_value=0; Serial.print("err="); Serial.println(TEST_err_value); }
    else if(c>='1' && c<='5'){
      int v = (c-'0');
      TEST_err_value = (pidLastErr>=0)? v : -v; // bias to last sign
      Serial.print("err="); Serial.println(TEST_err_value);
    }
#endif
  }
}
#endif

void loop(){
#if FEAT_SERIAL_SWITCH
  handleSerial();
#endif

  switch(currentMode){
#if FEAT_LINE_FOLLOWER
    case MODE_LINE:     loop_lineFollower(); break;
#endif
#if FEAT_JOYSTICK
    case MODE_JOYSTICK: loop_joystick();     break;
#endif
#if FEAT_FAKE_LINE_TEST
    case MODE_TEST:     loop_fakeLine();     break;
#endif
#if FEAT_MOTOR_TEST
    case MODE_MOTOR:    loop_motorTest();    break;
#endif
#if FEAT_LED_TEST
    case MODE_LED:      loop_ledTest();      break;
#endif
    default: motorsStop(); delay(10); break;
  }
}
