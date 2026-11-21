// =====================================================================
//  ESS LFR Race - ESP32 + TB6612FNG + 8-channel IR sensor array
//  Handles: start/finish black box, crossings, 90-degree corners,
//           straight gaps, gap after a curve, circle touching the path.
//
//  BOOT button (on the ESP32 board):
//     short press  = start run (robot waits 1 s so you can move your hand)
//     long press (hold 1.5 s, LED blinks) = AUTO CALIBRATION
//                    (put robot centered on a straight line first)
//     press while running = STOP
//  Onboard blue LED: ON while running, 2 slow blinks = calibration OK,
//                    fast blinking = calibration failed / not calibrated.
//
//  Serial Monitor: 115200 baud, line ending "New Line". Send h for menu.
// =====================================================================

#include <Preferences.h>

// ---------------- Pins ----------------
const int SENSOR_PINS[8] = {36, 39, 34, 35, 32, 33, 25, 26};  // D1..D8 (left to right)
const int IR_PIN  = 27;     // IR enable on sensor board (leave unconnected if board has none)
const int PWMA = 4,  AIN1 = 16, AIN2 = 17;   // left motor
const int PWMB = 21, BIN1 = 18, BIN2 = 19;   // right motor
const int STBY = 23;
const int BTN_PIN = 0;      // BOOT button
const int LED_PIN = 2;      // onboard LED

// ---------------- Motor limits ----------------
// 255 if battery voltage = motor rating. 127 if 11.1V battery with 6V motors.
const int MAX_PWM  = 255;
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

// ---------------- Tunable parameters (saved with 's') ----------------
float Kp = 0.045f;
float Ki = 0.0f;
float Kd = 0.8f;
int   baseSpeed   = 110;   // straight speed
float curveSlow   = 0.015f;// slow down on curves: speed -= curveSlow * |error|
int   gapSpeed    = 100;   // speed while crossing a gap
int   gapTimeout  = 900;   // ms of straight driving allowed in a gap
int   turnSpeed   = 110;   // spin speed at 90-degree corners
int   turnTimeout = 1200;  // ms max spin while searching
int   crossSpeed  = 100;   // speed across crossings and the start box
int   finishMs    = 200;   // all-black this long (after minRun) = finish box
int   minRunMs    = 8000;  // ignore finish box for the first 8 s (start box)
int   calSpeed    = 85;    // spin speed during auto calibration
int   noiseFloor  = 150;   // normalized values below this count as white
int   cornerWidth = 4;     // black blob this wide touching an edge = 90-degree corner

const float CENTER   = 3500.0f;   // position 0..7000
const float TURN_ERR = 2400.0f;   // lost with |error| above this = turn, else gap

// ---------------- Calibration ----------------
int  sMin[8], sMax[8];
bool invertSensors = false;   // auto-detected during calibration
bool reverseOrder  = false;   // toggle with 'o' if D1 is on the RIGHT
Preferences prefs;

// ---------------- Runtime state ----------------
bool running = false, telemetry = false;
float lastError = 0, integral = 0, lastPos = CENTER;
bool  lost = false;
int   lostDir = 0, searchDir = 1;
unsigned long lostSince = 0, runStart = 0, allBlackSince = 0;
int   hintDir = 0;
unsigned long hintTime = 0;

// =====================================================================
//  PWM / motors
// =====================================================================
void pwmInit(int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  static int ch = 0;
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
  ch++;
#endif
}

void pwmWrite(int pin, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pin == PWMA ? 0 : 1, duty);
#endif
}

void setLeft(int s) {
  s = constrain(s, -MAX_PWM, MAX_PWM);
  digitalWrite(AIN1, s > 0);
  digitalWrite(AIN2, s < 0);
  pwmWrite(PWMA, abs(s));
}

void setRight(int s) {
  s = constrain(s, -MAX_PWM, MAX_PWM);
  digitalWrite(BIN1, s > 0);
  digitalWrite(BIN2, s < 0);
  pwmWrite(PWMB, abs(s));
}

void stopMotors() { setLeft(0); setRight(0); }

// dir > 0 spin right (clockwise), dir < 0 spin left
void spin(int dir, int spd) {
  if (dir > 0) { setLeft(spd);  setRight(-spd); }
  else         { setLeft(-spd); setRight(spd);  }
}

void blinkLed(int n, int ms) {
  for (int k = 0; k < n; k++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}

// =====================================================================
//  Sensors
// =====================================================================
int readRaw(int i) {
  int sum = 0;
  for (int k = 0; k < 4; k++) sum += analogRead(SENSOR_PINS[i]);
  return sum / 4;
}

// 0..1000, 1000 = black. i is the LOGICAL index (0 = leftmost)
int readNorm(int i) {
  int idx = reverseOrder ? 7 - i : i;
  int raw = readRaw(idx);
  int span = sMax[idx] - sMin[idx];
  if (span < 50) return 0;
  long n = (long)(raw - sMin[idx]) * 1000 / span;
  n = constrain(n, 0, 1000);
  if (invertSensors) n = 1000 - n;
  return (int)n;
}

// Finds black "blobs" and follows the one closest to where the line was last.
// This keeps the robot on the main path where the circle touches it.
bool readLine(float &pos, int &nBlack, int v[8], bool &corner, int &cornerDir) {
  nBlack = 0; corner = false; cornerDir = 0;
  for (int i = 0; i < 8; i++) {
    int x = readNorm(i);
    if (x < noiseFloor) x = 0;
    v[i] = x;
    if (x > 500) nBlack++;
  }

  bool found = false;
  float bestPos = 0, bestDist = 1e9;
  int bestStart = 0, bestEnd = 0;
  int i = 0;
  while (i < 8) {
    if (v[i] == 0) { i++; continue; }
    int s = i; long w = 0, t = 0;
    while (i < 8 && v[i] > 0) { w += (long)v[i] * (i * 1000L); t += v[i]; i++; }
    if (t < 300) continue;
    float p = (float)w / t;
    float d = fabsf(p - lastPos);
    if (d < bestDist) { bestDist = d; bestPos = p; bestStart = s; bestEnd = i - 1; found = true; }
  }
  if (!found) return false;

  pos = bestPos;
  int width = bestEnd - bestStart + 1;
  if (width >= cornerWidth) {
    if (bestStart == 0 && bestEnd < 7)      { corner = true; cornerDir = -1; }
    else if (bestEnd == 7 && bestStart > 0) { corner = true; cornerDir =  1; }
  }
  return true;
}

// =====================================================================
//  Calibration + parameter storage
// =====================================================================
void saveCal() {
  prefs.begin("lfr", false);
  for (int i = 0; i < 8; i++) {
    prefs.putInt(("mn" + String(i)).c_str(), sMin[i]);
    prefs.putInt(("mx" + String(i)).c_str(), sMax[i]);
  }
  prefs.putBool("inv", invertSensors);
  prefs.putBool("rev", reverseOrder);
  prefs.end();
}

void loadCal() {
  prefs.begin("lfr", true);
  for (int i = 0; i < 8; i++) {
    sMin[i] = prefs.getInt(("mn" + String(i)).c_str(), 0);
    sMax[i] = prefs.getInt(("mx" + String(i)).c_str(), 0);   // 0/0 = not calibrated
  }
  invertSensors = prefs.getBool("inv", false);
  reverseOrder  = prefs.getBool("rev", false);
  prefs.end();
}

void saveParams() {
  prefs.begin("lfrp", false);
  prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
  prefs.putFloat("slow", curveSlow);
  prefs.putInt("base", baseSpeed); prefs.putInt("gsp", gapSpeed); prefs.putInt("gto", gapTimeout);
  prefs.putInt("tsp", turnSpeed);  prefs.putInt("tto", turnTimeout); prefs.putInt("csp", crossSpeed);
  prefs.putInt("fin", finishMs);   prefs.putInt("mrun", minRunMs);  prefs.putInt("cal", calSpeed);
  prefs.putInt("flr", noiseFloor); prefs.putInt("cw", cornerWidth);
  prefs.end();
}

void loadParams() {
  prefs.begin("lfrp", true);
  Kp = prefs.getFloat("kp", Kp); Ki = prefs.getFloat("ki", Ki); Kd = prefs.getFloat("kd", Kd);
  curveSlow   = prefs.getFloat("slow", curveSlow);
  baseSpeed   = prefs.getInt("base", baseSpeed);
  gapSpeed    = prefs.getInt("gsp", gapSpeed);
  gapTimeout  = prefs.getInt("gto", gapTimeout);
  turnSpeed   = prefs.getInt("tsp", turnSpeed);
  turnTimeout = prefs.getInt("tto", turnTimeout);
  crossSpeed  = prefs.getInt("csp", crossSpeed);
  finishMs    = prefs.getInt("fin", finishMs);
  minRunMs    = prefs.getInt("mrun", minRunMs);
  calSpeed    = prefs.getInt("cal", calSpeed);
  noiseFloor  = prefs.getInt("flr", noiseFloor);
  cornerWidth = prefs.getInt("cw", cornerWidth);
  prefs.end();
}

int goodSensorCount() {
  int g = 0;
  for (int i = 0; i < 8; i++) if (sMax[i] - sMin[i] >= 300) g++;
  return g;
}

void sampleFor(unsigned long ms, long sum[8], long &cnt) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    for (int i = 0; i < 8; i++) {
      int r = readRaw(i);
      if (r < sMin[i]) sMin[i] = r;
      if (r > sMax[i]) sMax[i] = r;
      sum[i] += r;
    }
    cnt++;
    delay(1);
  }
}

// spinMode = true: robot sweeps itself over the line (no laptop needed)
// spinMode = false: you slide the robot by hand for 6 s
bool calibrate(bool spinMode) {
  running = false;
  stopMotors();
  digitalWrite(IR_PIN, HIGH);
  digitalWrite(STBY, HIGH);

  int oldMin[8], oldMax[8];
  for (int i = 0; i < 8; i++) { oldMin[i] = sMin[i]; oldMax[i] = sMax[i]; sMin[i] = 4095; sMax[i] = 0; }
  long sum[8] = {0}; long cnt = 0;

  if (spinMode) {
    Serial.println("AUTO CALIBRATION: robot sweeps left and right over the line...");
    const int seq[5][2] = {{-1, 250}, {1, 500}, {-1, 500}, {1, 500}, {-1, 250}};
    for (int k = 0; k < 5; k++) {
      spin(seq[k][0], calSpeed);
      sampleFor(seq[k][1], sum, cnt);
    }
    stopMotors();
  } else {
    Serial.println("MANUAL CALIBRATION: in 3 s, slide the robot sideways across the line");
    Serial.println("by hand for 6 s so every sensor sees black AND white.");
    delay(3000);
    Serial.println("Sliding now...");
    sampleFor(6000, sum, cnt);
  }

  Serial.println("Sensor  min   max   span");
  for (int i = 0; i < 8; i++) {
    int span = sMax[i] - sMin[i];
    Serial.printf("D%d     %4d  %4d  %4d%s\n", i + 1, sMin[i], sMax[i], span, span < 300 ? "  <-- WEAK" : "");
  }

  int good = goodSensorCount();
  if (good < 6) {
    for (int i = 0; i < 8; i++) { sMin[i] = oldMin[i]; sMax[i] = oldMax[i]; }
    Serial.println("CALIBRATION FAILED: too many weak sensors. Old calibration kept.");
    Serial.println("Check: IR LEDs on (phone camera), sensor VCC on 3.3V, height 3-8 mm,");
    Serial.println("robot placed on the line. Values stuck at 4095 or 0 = wiring problem.");
    blinkLed(10, 80);
    return false;
  }

  // Auto-detect polarity using the outer sensors (they see white most of the time)
  int votes = 0, blackHigh = 0;
  const int outer[4] = {0, 1, 6, 7};
  for (int k = 0; k < 4; k++) {
    int i = outer[k];
    if (sMax[i] - sMin[i] < 300) continue;
    float avg = (float)sum[i] / cnt;
    votes++;
    if (avg - sMin[i] < sMax[i] - avg) blackHigh++;   // mostly low = white is low = black is high
  }
  if (votes > 0) invertSensors = (blackHigh * 2 < votes);

  saveCal();
  Serial.printf("CALIBRATION OK (%d/8 good sensors). Black reads %s. Saved.\n",
                good, invertSensors ? "LOW (inverted)" : "HIGH");
  blinkLed(2, 300);
  return true;
}

void viewSensors() {
  running = false;
  stopMotors();
  digitalWrite(IR_PIN, HIGH);
  Serial.println("Normalized view: black ~1000, white ~0. Line on LEFT -> pos near 0. Send x to stop.");
  while (true) {
    if (Serial.available() && Serial.read() == 'x') break;
    float pos; int nb, v[8]; bool corner; int cdir;
    bool seen = readLine(pos, nb, v, corner, cdir);
    for (int i = 0; i < 8; i++) Serial.printf("%4d ", v[i]);
    if (seen) { lastPos = pos; Serial.printf("| pos %4.0f  black %d%s\n", pos, nb, corner ? (cdir < 0 ? "  CORNER-L" : "  CORNER-R") : ""); }
    else Serial.println("| no line");
    delay(150);
  }
  lastPos = CENTER;
}

void motorTest() {
  running = false;
  digitalWrite(STBY, HIGH);
  Serial.println("LEFT wheel forward 1 s");
  setLeft(120); setRight(0); delay(1000);
  stopMotors(); delay(300);
  Serial.println("RIGHT wheel forward 1 s");
  setLeft(0); setRight(120); delay(1000);
  stopMotors();
  Serial.println("Done. Wrong wheel or backwards? Fix the motor wires.");
}

// =====================================================================
//  Run
// =====================================================================
void startRun() {
  if (goodSensorCount() < 6) {
    Serial.println("NOT CALIBRATED. Hold BOOT 1.5 s (auto) or send 'a' / 'c' first.");
    blinkLed(10, 80);
    return;
  }
  integral = 0; lastError = 0; lastPos = CENTER;
  lost = false; lostDir = 0; hintDir = 0; hintTime = 0; allBlackSince = 0;
  digitalWrite(IR_PIN, HIGH);
  digitalWrite(STBY, HIGH);
  runStart = millis();
  running = true;
  digitalWrite(LED_PIN, HIGH);
  Serial.println("GO");
}

void stopRun(const char *msg) {
  running = false;
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  Serial.println(msg);
}

void runStep() {
  float pos; int nBlack, v[8]; bool corner; int cornerDir;
  bool seen = readLine(pos, nBlack, v, corner, cornerDir);
  unsigned long now = millis();
  static int tcount = 0;
  bool tprint = telemetry && (++tcount % 10 == 0);

  // ----- Almost all sensors black: start box, crossing, or finish box -----
  if (nBlack >= 7) {
    if (allBlackSince == 0) allBlackSince = now;
    if (now - runStart > (unsigned long)minRunMs && now - allBlackSince > (unsigned long)finishMs) {
      stopRun("FINISH box detected. Stopped.");
      return;
    }
    setLeft(crossSpeed); setRight(crossSpeed);   // go straight through
    lost = false;
    if (tprint) Serial.println("ALL BLACK: straight");
    return;
  }
  allBlackSince = 0;

  // ----- Line visible: PID -----
  if (seen) {
    if (corner) { hintDir = cornerDir; hintTime = now; }
    if (lost) { lost = false; integral = 0; lastError = pos - CENTER; }

    float error = pos - CENTER;
    integral = constrain(integral + error, -3000, 3000);
    float deriv = error - lastError;
    float corr = Kp * error + Ki * integral + Kd * deriv;
    lastError = error;
    lastPos = pos;

    int spd = baseSpeed - (int)(curveSlow * fabsf(error));
    int l = constrain(spd + (int)corr, -MAX_PWM, MAX_PWM);
    int r = constrain(spd - (int)corr, -MAX_PWM, MAX_PWM);
    setLeft(l); setRight(r);

    if (tprint) Serial.printf("pos %4.0f err %5.0f corr %4.0f L %4d R %4d%s\n",
                              pos, error, corr, l, r, corner ? " CORNER" : "");
    return;
  }

  // ----- Line lost: decide corner or gap -----
  if (!lost) {
    lost = true;
    lostSince = now;
    if (hintDir != 0 && now - hintTime < 250)  lostDir = hintDir;              // 90-degree corner
    else if (fabsf(lastError) > TURN_ERR)      lostDir = lastError > 0 ? 1 : -1; // line left the edge
    else                                       lostDir = 0;                     // gap
    searchDir = lastError >= 0 ? 1 : -1;
  }
  unsigned long elapsed = now - lostSince;

  if (lostDir == 0) {
    if (elapsed < (unsigned long)gapTimeout) {
      setLeft(gapSpeed); setRight(gapSpeed);
      if (tprint) Serial.println("GAP: straight");
    } else if (elapsed < (unsigned long)(gapTimeout + turnTimeout)) {
      spin(searchDir, turnSpeed);   // gap too long: search
      if (tprint) Serial.println("GAP timeout: searching");
    } else {
      stopRun("Line not found after gap. Stopped. Try: gapto 1200");
    }
  } else {
    if (elapsed < (unsigned long)turnTimeout) {
      spin(lostDir, turnSpeed);
      if (tprint) Serial.println(lostDir > 0 ? "CORNER: spin right" : "CORNER: spin left");
    } else {
      stopRun("Line lost at corner. Stopped. Try: turnto 1800");
    }
  }
}

// =====================================================================
//  Button + Serial
// =====================================================================
void checkButton() {
  if (digitalRead(BTN_PIN) == HIGH) return;
  delay(30);
  if (digitalRead(BTN_PIN) == HIGH) return;

  if (running) {
    stopRun("STOP (button)");
    while (digitalRead(BTN_PIN) == LOW) delay(10);
    delay(200);
    return;
  }

  unsigned long t = millis();
  bool longPress = false;
  while (digitalRead(BTN_PIN) == LOW) {
    if (!longPress && millis() - t > 1500) { longPress = true; blinkLed(3, 60); }
    delay(10);
  }
  delay(1000);   // time to move your hand away
  if (longPress) calibrate(true);
  else startRun();
}

void printParams() {
  Serial.printf("kp %.4f  ki %.4f  kd %.3f  base %d  slow %.3f\n", Kp, Ki, Kd, baseSpeed, curveSlow);
  Serial.printf("gapspd %d  gapto %d  turnspd %d  turnto %d  cross %d\n",
                gapSpeed, gapTimeout, turnSpeed, turnTimeout, crossSpeed);
  Serial.printf("finish %d  minrun %d  calspd %d  floor %d  cwidth %d\n",
                finishMs, minRunMs, calSpeed, noiseFloor, cornerWidth);
  Serial.printf("invert %d  reverse %d  calibrated sensors %d/8\n",
                invertSensors, reverseOrder, goodSensorCount());
}

void printMenu() {
  Serial.println();
  Serial.println("=== ESS LFR ===");
  Serial.println("a auto-calibrate (robot sweeps) | c manual calibrate (slide by hand)");
  Serial.println("v view sensors | m motor test | g go | x stop | t telemetry");
  Serial.println("o reverse sensor order | i invert sensors | p params | s SAVE params | h menu");
  Serial.println("Set: kp 0.05 | ki 0 | kd 1 | base 120 | slow 0.02 | gapspd 100 | gapto 900");
  Serial.println("     turnspd 110 | turnto 1200 | cross 100 | finish 200 | minrun 8000");
  Serial.println("     calspd 85 | floor 150 | cwidth 4");
  Serial.println("BOOT button: short = go/stop, hold 1.5 s = auto-calibrate");
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s.length() == 1) {
    switch (s[0]) {
      case 'a': calibrate(true); break;
      case 'c': calibrate(false); break;
      case 'v': viewSensors(); break;
      case 'm': motorTest(); break;
      case 'g': startRun(); break;
      case 'x': stopRun("STOP"); break;
      case 't': telemetry = !telemetry; Serial.println(telemetry ? "Telemetry on" : "Telemetry off"); break;
      case 'o': reverseOrder = !reverseOrder; saveCal(); Serial.println(reverseOrder ? "Order reversed" : "Order normal"); break;
      case 'i': invertSensors = !invertSensors; saveCal(); Serial.println(invertSensors ? "Inverted" : "Normal"); break;
      case 'p': printParams(); break;
      case 's': saveParams(); Serial.println("Parameters saved (kept after power off)."); break;
      case 'h': printMenu(); break;
      default: Serial.println("Unknown. Send h for menu.");
    }
    return;
  }

  char name[12]; float val;
  if (sscanf(s.c_str(), "%11s %f", name, &val) == 2) {
    String n = String(name);
    if      (n == "kp")      Kp = val;
    else if (n == "ki")      Ki = val;
    else if (n == "kd")      Kd = val;
    else if (n == "base")    baseSpeed = (int)val;
    else if (n == "slow")    curveSlow = val;
    else if (n == "gapspd")  gapSpeed = (int)val;
    else if (n == "gapto")   gapTimeout = (int)val;
    else if (n == "turnspd") turnSpeed = (int)val;
    else if (n == "turnto")  turnTimeout = (int)val;
    else if (n == "cross")   crossSpeed = (int)val;
    else if (n == "finish")  finishMs = (int)val;
    else if (n == "minrun")  minRunMs = (int)val;
    else if (n == "calspd")  calSpeed = (int)val;
    else if (n == "floor")   noiseFloor = (int)val;
    else if (n == "cwidth")  cornerWidth = (int)val;
    else { Serial.println("Unknown name. Send h for menu."); return; }
    printParams();
    Serial.println("(send s to save permanently)");
  }
}

// =====================================================================
//  Arduino
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  for (int i = 0; i < 8; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(IR_PIN, OUTPUT);  digitalWrite(IR_PIN, HIGH);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pwmInit(PWMA);
  pwmInit(PWMB);
  stopMotors();

  loadCal();
  loadParams();
  delay(500);
  printMenu();
  printParams();
  if (goodSensorCount() < 6) {
    Serial.println(">>> Not calibrated yet: hold BOOT 1.5 s with robot on the line, or send 'a'.");
    blinkLed(5, 80);
  }
}

void loop() {
  if (Serial.available()) handleLine(Serial.readStringUntil('\n'));
  checkButton();

  if (running) {
    static unsigned long last = 0;
    if (millis() - last >= 5) {   // 200 Hz control loop
      last = millis();
      runStep();
    }
  }
}
