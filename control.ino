/*
  servo_position_0_180.ino  —  Servo Joint (MTR5101), NUCLEO-G474RE
  The home switch (D4) is at the 180 deg end. After homing the arm returns to 0.
  Travel 0..180 deg (output angle, corrected by outRatio).

  Wiring
    AS5600 : VCC->3V3, GND->GND, SCL->PA15 (CN7 pin 17), SDA->PB7 (CN7 pin 21), DIR->GND
    IBT-2  : RPWM->D5, LPWM->D6, R_EN->D7, L_EN->D8, VCC->3V3, GND->GND
             B+/B- -> 12 V, M+/M- -> motor
    Switch : D4 <-> GND (NO contact, pressed = LOW)

  Serial Monitor: 115200, "New Line"
    h          home first: run to the switch (= 180 deg), then go to 0
    <number>   target angle 0..180 (only after homing)
    s          hold here (also clears a fault)
    bk         breakaway test: finds the PWM that starts the arm moving (arm at 30..150 deg)
    ?          print all settings
    ratio x    sensor turns per output turn (feedback gear ratio)
    kp x / ki x / kd x          PID gains
    minup x / mindn x           minimum % PWM to start moving (angle up / down)
    bmax x / brate x            stiction breaker: max % and rise rate (% per s)
    p          printing on/off
  Output: angle:<deg>,target:<deg>,err:<deg>,pwm:<%>,boost:<%>
*/

#include <Wire.h>

// ---------- pins ----------
const int RPWM    = D5;
const int LPWM    = D6;
const int R_EN    = D7;
const int L_EN    = D8;
const int HOME_SW = D4;

// ---------- AS5600 ----------
const uint8_t AS5600_ADDR   = 0x36;
const uint8_t REG_RAW_ANGLE = 0x0C;

// ---------- travel and homing ----------
const float ANGLE_MIN = 0.0;
const float ANGLE_MAX = 180.0;
const bool  SWITCH_AT_180 = true;     // true: switch at the 180 end, false: at the 0 end
const int   HOME_PWM_DIR  = -1;       // PWM sign that drives the arm toward the switch (-1 = LPWM)
const float HOME_SPEED    = 40.0;     // % PWM while homing
const unsigned long HOME_TIMEOUT_MS = 25000;
const float AFTER_HOME_TARGET = 0.0;  // where the arm goes after homing
const float HOME_ANGLE    = SWITCH_AT_180 ? ANGLE_MAX : ANGLE_MIN;
const int   PWM_ANGLE_DIR = SWITCH_AT_180 ? HOME_PWM_DIR : -HOME_PWM_DIR;   // PWM sign that increases the angle

// ---------- control settings ----------
const float OUT_MAX      = 100.0;     // lower it (e.g. 60) for gentler motion
const float I_ZONE       = 8.0;       // deg: integral works only inside this error
const float I_LIMIT      = 40.0;      // anti-windup (% PWM)
const float DB_ON        = 0.3;       // deg: start pushing when the error is bigger than this
const float DB_OFF       = 0.15;      // deg: stop pushing when the error is smaller than this
const float STUCK_SPEED  = 2.0;       // deg/s: below this the arm counts as "stuck"
const float SPEED_FILTER = 0.05;      // speed low-pass (smaller = smoother)
const unsigned long CTRL_US  = 2000;  // 500 Hz control loop
const unsigned long PRINT_MS = 100;   // 10 Hz output
const int PWM_BITS = 12;
const int PWM_MAX  = 4095;

float kp = 2.0;                       // % per deg
float ki = 3.0;                       // % per deg per s
float kd = 0.2;                       // % per (deg/s)
float outRatio = 2.0;                 // sensor turns per output turn (feedback gear ratio)
float minUp = 18.0;                   // min % PWM to start moving, angle increasing  (run bk to measure)
float minDn = 18.0;                   // min % PWM to start moving, angle decreasing
float bMax  = 25.0;                   // stiction breaker: max extra %
float bRate = 30.0;                   // stiction breaker: rise rate, % per second

// ---------- state ----------
long  totalCounts = 0, homeCounts = 0;
int   lastRaw = -1;
int   angleSign = 1;                  // found automatically during homing
bool  i2cOk = true, homed = false, fault = false, printOn = true, active = false;
float targetDeg = 0, pwmPct = 0, integ = 0, lastAngle = 0, speedF = 0, boost = 0;
float errWin = 0;
int   growCount = 0;
unsigned long tCtrl = 0, tPrint = 0, tWin = 0;
String line = "";

// ================= AS5600 =================
bool readRaw(int &raw) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_RAW_ANGLE);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = ((hi & 0x0F) << 8) | lo;          // 0..4095
  return true;
}

void updateEncoder() {
  int raw;
  if (!readRaw(raw)) { i2cOk = false; return; }
  i2cOk = true;
  if (lastRaw < 0) { lastRaw = raw; return; }
  int d = raw - lastRaw;
  if (d >  2048) d -= 4096;               // wrap at 0/360
  if (d < -2048) d += 4096;
  totalCounts += d;
  lastRaw = raw;
}

// output angle in degrees: HOME_ANGLE at the switch, 4096 counts = 360 sensor deg
float angleDeg() {
  return HOME_ANGLE + angleSign * (float)(totalCounts - homeCounts) * 360.0 / 4096.0 / outRatio;
}

// ================= motor =================
void setMotor(float pct) {
  pct = constrain(pct, -100.0, 100.0);
  pwmPct = pct;
  int duty = (int)(fabs(pct) / 100.0 * PWM_MAX);
  if (pct > 0)      { analogWrite(LPWM, 0); analogWrite(RPWM, duty); }
  else if (pct < 0) { analogWrite(RPWM, 0); analogWrite(LPWM, duty); }
  else              { analogWrite(RPWM, 0); analogWrite(LPWM, 0); }
}

bool switchPressed() {
  if (digitalRead(HOME_SW) != LOW) return false;
  delay(5);                               // simple debounce
  return digitalRead(HOME_SW) == LOW;
}

// ================= position control (500 Hz) =================
void controlTick(float dt) {
  if (!i2cOk || !homed) { setMotor(0); return; }

  float a = angleDeg();
  float target = constrain(targetDeg, ANGLE_MIN, ANGLE_MAX);
  float err = target - a;
  float absErr = fabs(err);
  float v = (a - lastAngle) / dt;         // deg/s
  lastAngle = a;
  speedF += SPEED_FILTER * (v - speedF);

  // runaway check every 250 ms: the error must not keep growing
  if (millis() - tWin >= 250) {
    tWin = millis();
    if (absErr > 10.0 && absErr > errWin + 5.0) growCount++; else growCount = 0;
    errWin = absErr;
    if (growCount >= 3) {
      fault = true;
      integ = 0; boost = 0;
      setMotor(0);
      Serial.println("!! FAULT: error keeps growing (wrong direction?). Type s to clear, then h to home again.");
      return;
    }
  }
  if (fault) { setMotor(0); return; }

  // deadband with hysteresis
  if (!active && absErr > DB_ON) active = true;
  else if (active && absErr < DB_OFF) active = false;

  // integral only near the target (no windup during big moves)
  if (absErr < I_ZONE) {
    integ += ki * err * dt;
    integ = constrain(integ, -I_LIMIT, I_LIMIT);
  } else {
    integ = 0;
  }

  // PID (derivative on the measured speed)
  float out = kp * err + integ - kd * speedF;
  out = constrain(out, -OUT_MAX, OUT_MAX);

  if (active) {
    // minimum push to overcome static friction
    float mn = (out >= 0) ? minUp : minDn;
    if (fabs(out) < mn) out = (out >= 0) ? mn : -mn;

    // stiction breaker: if the arm is stuck, add more force little by little
    if (fabs(speedF) < STUCK_SPEED) boost += bRate * dt;
    else boost -= 100.0 * dt;
    boost = constrain(boost, 0.0, bMax);
    out += (err > 0) ? boost : -boost;
    out = constrain(out, -OUT_MAX, OUT_MAX);
  } else {
    boost = 0;
  }

  // travel limits
  if (a >= ANGLE_MAX && out > 0) out = 0;
  if (a <= ANGLE_MIN && out < 0) out = 0;

  setMotor(PWM_ANGLE_DIR * out);          // out > 0 = increase the angle
}

// ================= homing =================
void homing() {
  fault = false; homed = false; integ = 0; boost = 0; active = false; growCount = 0;
  setMotor(0);
  Serial.println("homing...");

  // if the arm already sits on the switch, move off it first
  if (switchPressed()) {
    setMotor(-HOME_PWM_DIR * HOME_SPEED);
    unsigned long t1 = millis();
    while (switchPressed()) {
      updateEncoder();
      if (millis() - t1 > 3000) {
        setMotor(0);
        Serial.println("!! cannot leave the switch");
        return;
      }
    }
    setMotor(0);
    delay(300);
  }

  updateEncoder();
  long c0 = totalCounts;
  setMotor(HOME_PWM_DIR * HOME_SPEED);
  unsigned long t0 = millis();
  while (!switchPressed()) {
    updateEncoder();
    if (millis() - t0 > HOME_TIMEOUT_MS) {
      setMotor(0);
      Serial.println("!! homing timeout: switch not found (check D4 wiring, or set HOME_PWM_DIR = +1)");
      return;
    }
  }
  setMotor(0);
  delay(300);
  updateEncoder();

  // angle must increase when moving toward the switch if it is at 180 (decrease if at 0)
  long dc = totalCounts - c0;
  int towardAngleDir = SWITCH_AT_180 ? 1 : -1;
  if (labs(dc) > 40) angleSign = towardAngleDir * (dc > 0 ? 1 : -1);
  else Serial.println("!! arm barely moved, angle direction unchanged");

  homeCounts = totalCounts;               // the switch position = HOME_ANGLE
  lastAngle = HOME_ANGLE;
  speedF = 0;
  homed = true;
  targetDeg = AFTER_HOME_TARGET;
  tCtrl = micros();                       // avoid a huge dt after the blocking loop
  Serial.print("homed. angle = "); Serial.print(HOME_ANGLE, 1);
  Serial.print(" , going to "); Serial.println(targetDeg, 1);
}

// ================= breakaway test =================
// dirAngle = +1 (angle increasing) or -1 (angle decreasing); returns the % PWM at which the arm moved
float breakaway(int dirAngle) {
  updateEncoder();
  float a0 = angleDeg();
  for (float p = 0; p <= 60.0; p += 0.5) {
    setMotor(PWM_ANGLE_DIR * dirAngle * p);
    unsigned long t = millis();
    while (millis() - t < 100) updateEncoder();
    float a = angleDeg();
    if (fabs(a - a0) > 0.8) { setMotor(0); return p; }
    if (a >= ANGLE_MAX - 3 || a <= ANGLE_MIN + 3) break;
  }
  setMotor(0);
  return -1;
}

void breakawayTest() {
  if (!homed) { Serial.println("type h first (home)"); return; }
  float a = angleDeg();
  if (a < 20 || a > 160) { Serial.println("move the arm to 30..150 deg first"); return; }

  Serial.println("breakaway test: going up...");
  float up = breakaway(+1);
  delay(800);
  Serial.println("breakaway test: going down...");
  float dn = breakaway(-1);
  delay(300);

  if (up > 0) minUp = up + 2.0;
  if (dn > 0) minDn = dn + 2.0;
  Serial.print("breakaway up = ");   Serial.print(up, 1);
  Serial.print(" %, down = ");       Serial.print(dn, 1);
  Serial.print(" %  ->  minup = ");  Serial.print(minUp, 1);
  Serial.print(", mindn = ");        Serial.println(minDn, 1);

  targetDeg = angleDeg();                 // hold where it stopped
  integ = 0; boost = 0; active = false;
  lastAngle = targetDeg; speedF = 0;
  tCtrl = micros();
}

// ================= serial =================
void printSettings() {
  Serial.print("kp "); Serial.print(kp, 2);
  Serial.print(" | ki "); Serial.print(ki, 2);
  Serial.print(" | kd "); Serial.print(kd, 2);
  Serial.print(" | ratio "); Serial.print(outRatio, 3);
  Serial.print(" | minup "); Serial.print(minUp, 1);
  Serial.print(" | mindn "); Serial.print(minDn, 1);
  Serial.print(" | bmax "); Serial.print(bMax, 1);
  Serial.print(" | brate "); Serial.println(bRate, 1);
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { line += ch; continue; }
    line.trim();
    if (line.length() == 0) { line = ""; continue; }

    char c0 = line.charAt(0);
    if (line.startsWith("ratio")) {
      float r = line.substring(5).toFloat();
      if (r > 0.1) outRatio = r;
      Serial.print("ratio = "); Serial.println(outRatio, 3);
    }
    else if (line.startsWith("kp"))    { kp = line.substring(2).toFloat();    Serial.print("kp = ");    Serial.println(kp, 3); }
    else if (line.startsWith("ki"))    { ki = line.substring(2).toFloat();    Serial.print("ki = ");    Serial.println(ki, 3); }
    else if (line.startsWith("kd"))    { kd = line.substring(2).toFloat();    Serial.print("kd = ");    Serial.println(kd, 3); }
    else if (line.startsWith("minup")) { minUp = line.substring(5).toFloat(); Serial.print("minup = "); Serial.println(minUp, 1); }
    else if (line.startsWith("mindn")) { minDn = line.substring(5).toFloat(); Serial.print("mindn = "); Serial.println(minDn, 1); }
    else if (line.startsWith("bmax"))  { bMax = line.substring(4).toFloat();  Serial.print("bmax = ");  Serial.println(bMax, 1); }
    else if (line.startsWith("brate")) { bRate = line.substring(5).toFloat(); Serial.print("brate = "); Serial.println(bRate, 1); }
    else if (line.startsWith("bk"))    breakawayTest();
    else if (isDigit(c0) || c0 == '-' || c0 == '.') {
      if (!homed) Serial.println("type h first (home)");
      else {
        targetDeg = constrain(line.toFloat(), ANGLE_MIN, ANGLE_MAX);
        Serial.print("target "); Serial.print(targetDeg, 1); Serial.println(" deg");
      }
    }
    else if (c0 == 'h') homing();
    else if (c0 == 's') {
      if (homed) targetDeg = angleDeg();
      integ = 0; boost = 0; fault = false; growCount = 0;
      Serial.println("hold");
    }
    else if (c0 == 'p') printOn = !printOn;
    else if (c0 == '?') printSettings();
    else Serial.println("unknown command");
    line = "";
  }
}

// ================= setup / loop =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("boot");

  Wire.setSCL(PA15);
  Wire.setSDA(PB7);
  Wire.begin();
  Wire.setClock(400000);

  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(20000);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  setMotor(0);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  pinMode(HOME_SW, INPUT_PULLUP);

  delay(200);
  updateEncoder();
  homeCounts = totalCounts;
  tCtrl = micros();

  Serial.println("\n== Servo joint 0..180 deg (NUCLEO-G474RE) ==");
  Serial.println("h home | number 0..180 target | s hold | bk breakaway | ? settings | kp/ki/kd/ratio/minup/mindn/bmax/brate x | p print");
}

void loop() {
  updateEncoder();

  unsigned long now = micros();
  if (now - tCtrl >= CTRL_US) {
    float dt = (now - tCtrl) / 1000000.0;
    tCtrl = now;
    controlTick(dt);
  }

  handleSerial();

  unsigned long period = homed ? PRINT_MS : 1000;
  if (printOn && millis() - tPrint >= period) {
    tPrint = millis();
    if (!i2cOk) { Serial.println("!! I2C read error"); return; }
    if (!homed) { Serial.println("not homed - type h"); return; }
    float a = angleDeg();
    Serial.print("angle:");   Serial.print(a, 1);
    Serial.print(",target:"); Serial.print(targetDeg, 1);
    Serial.print(",err:");    Serial.print(targetDeg - a, 1);
    Serial.print(",pwm:");    Serial.print(pwmPct, 1);
    Serial.print(",boost:");  Serial.println(boost, 1);
  }
}
