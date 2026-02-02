/*
- V0 = temperature
- V1 = buzzer STOP switch (1 -> stop buzzer, 0 -> allow buzzer)
- V2 = move-to-zero switch (momentary: moves stepper to stored startAngle)
- Serial commands: MOV <angle>, AUTO, SET <angle>, ZERO, STOP, RESET
- ZERO command and V2 BOTH move motor to startAngle
*/

// ---- Blynk IoT required macros - MUST be defined BEFORE including Blynk library ----
#define BLYNK_TEMPLATE_ID   "TMPL3VtHsYk6W"
#define BLYNK_TEMPLATE_NAME "Thermal Device"
#define BLYNK_AUTH_TOKEN    "xqNJXB348lc08H-Pr6Ql2M26SZbAc_aF"

// create auth variable used by Blynk.begin()
char auth[] = BLYNK_AUTH_TOKEN;

#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BlynkSimpleEsp32.h>

// ----- CONFIG -----
const char* WIFI_SSID = "project";
const char* WIFI_PASS = "12345678";

// Pins
const int stepPins[4] = {19, 18, 5, 17}; // IN1, IN2, IN3, IN4 for ULN2003
const int buzzer1 = 25;
const int buzzer2 = 26;

// Stepper parameters (28BYJ-48 typical)
const float STEPS_PER_REV = 4096.0; // steps per 360 degrees (typical for 28BYJ-48)
const float STEPS_PER_DEG = STEPS_PER_REV / 360.0;

// Movement timings
const int stepDelayMs = 2; // ms between single steps -- adjust speed here

// Scanning and stops
const int stopIntervalDeg = 30; // stop every 30 degrees
const int stopHoldMs = 5000; // 5 seconds at each stop to scan temp

// Alert durations
const int alertBeepDurationMs = 10000; // 10 seconds of beep for Alert and Critical
const int alertStopMs = 10000; // 10 seconds stop for Alert

// Temperature thresholds
const float normalMin = 15.0;
const float normalMax = 35.0;
const float alert1Min = 36.0;
const float alert1Max = 45.0;
const float alert2Min = 10.0;
const float alert2Max = 19.0;
const float criticalMin = 45.0; // >45 critical

// Sweep limits (−90 .. +90)
float sweepMin = -90.0;
float sweepMax = 90.0;

// Blynk virtual pins
#define VP_TEMPERATURE V0
#define VP_BUZZER_STOP V1 // V1 switch: 1 -> stop buzzer, 0 -> allow buzzer
#define VP_MOVE_ZERO   V2 // V2 switch: momentary -> move to startAngle (zero)

// Globals
Adafruit_MLX90614 mlx;

long currentStep = 0; // software absolute step count
float startAngle = 0.0; // value set with SET (the "zero" or starting position)
bool startAngleSet = false;

bool buzzerEnabled = true;    // allowed to beep (true by default)
bool criticalHalted = false;

// mode flags
bool manualMode = true;      // true = manual-only (no auto sweeping)
bool autoEnabled = false;    // true when AUTO is active
bool stopRequested = false;  // request to stop ongoing motion

// stepper half-step sequence (8-step)
const int seqSteps = 8;
const int stepSequence[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

// low-level: write a half-step
void writeStep(int idx) {
  for (int i=0;i<4;i++) digitalWrite(stepPins[i], stepSequence[idx][i]);
}

// stepOnce(dir): dir = +1 (cw/right) or -1 (ccw/left)
void stepOnce(int dir) {
  static int seqIndex = 0;
  seqIndex += dir;
  while (seqIndex < 0) seqIndex += seqSteps;
  seqIndex %= seqSteps;
  writeStep(seqIndex);
  delay(stepDelayMs);
  currentStep += dir;
}

// Move relative degrees (positive => right, negative => left)
void moveRelativeDegrees(float deg) {
  long steps = (long)round(fabs(deg) * STEPS_PER_DEG);
  int dir = (deg >= 0) ? 1 : -1;
  stopRequested = false;
  for (long i=0;i<steps;i++) {
    if (criticalHalted || stopRequested) break;
    stepOnce(dir);
    Blynk.run();
    // small delay already in stepOnce
  }
}

// Move absolute to targetAngle (degrees)
void moveToAngle(float targetAngle) {
  long targetStep = (long)round(targetAngle * STEPS_PER_DEG);
  long delta = targetStep - currentStep;
  if (delta == 0) return;
  int dir = (delta > 0) ? 1 : -1;
  long stepsToMove = abs(delta);
  stopRequested = false;
  for (long i=0;i<stepsToMove;i++) {
    if (criticalHalted || stopRequested) break;
    stepOnce(dir);
    Blynk.run();
  }
}

// convert steps -> degrees
float getCurrentAngle() {
  return ((float)currentStep) / STEPS_PER_DEG;
}

// buzzer helpers
void buzzerOn(int b) { digitalWrite(b, HIGH); }
void buzzerOff(int b) { digitalWrite(b, LOW); }

void buzzPatternNormal() {
  if (!buzzerEnabled) return;
  buzzerOn(buzzer1); delay(150); buzzerOff(buzzer1);
}

void buzzPatternAlertContinuous(long durationMs) {
  if (!buzzerEnabled) return;
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)durationMs) {
    if (!buzzerEnabled || criticalHalted) break;
    buzzerOn(buzzer1); buzzerOn(buzzer2); delay(200);
    buzzerOff(buzzer1); buzzerOff(buzzer2); delay(200);
  }
}

void buzzPatternCriticalContinuous(long durationMs) {
  if (!buzzerEnabled) return;
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)durationMs) {
    if (!buzzerEnabled || criticalHalted) break;
    for (int i=0;i<5 && (millis()-t0 < (unsigned long)durationMs); ++i) {
      buzzerOn(buzzer1); buzzerOn(buzzer2); delay(600);
      buzzerOff(buzzer1); buzzerOff(buzzer2); delay(200);
    }
  }
}

// classify temperature: 0 normal, 1 alert, 2 critical
int classifyTemp(float t) {
  if (t > criticalMin) return 2;
  if ((t >= alert1Min && t <= alert1Max) || (t >= alert2Min && t <= alert2Max)) return 1;
  if (t >= normalMin && t <= normalMax) return 0;
  return 1; // fallback to alert
}

// Blynk handlers
BLYNK_WRITE(VP_BUZZER_STOP) {
  int v = param.asInt();
  // V1 switch: if 1 -> stop buzzer, if 0 -> allow buzzer
  if (v == 1) {
    buzzerEnabled = false;
    buzzerOff(buzzer1);
    buzzerOff(buzzer2);
    Serial.println("V1: Buzzer STOP (disabled).");
  } else {
    buzzerEnabled = true;
    Serial.println("V1: Buzzer ALLOWED (enabled).");
  }
}

BLYNK_WRITE(VP_MOVE_ZERO) {
  int v = param.asInt();
  if (v == 1) {
    // Move to stored startAngle (zero). If not set, default startAngle=0
    Serial.printf("V2 pressed: moving to zero/startAngle %.2f deg\n", startAngle);
    // ensure we go to manual mode and interrupt auto
    stopRequested = false;
    autoEnabled = false;
    manualMode = true;
    moveToAngle(startAngle);
    // update temperature virtual pin will be done at stops, we do not write V3/V4
    // clear the V2 switch back to 0 so it behaves like a momentary button
    Blynk.virtualWrite(VP_MOVE_ZERO, 0);
  }
}

// Serial command parser: MOV <angle>, AUTO, SET <angle>, ZERO, STOP, RESET
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  String u = line;
  u.toUpperCase();

  int sp = u.indexOf(' ');
  String cmd = (sp == -1) ? u : u.substring(0, sp);
  String arg = (sp == -1) ? "" : u.substring(sp + 1);
  arg.trim();

  if (cmd == "MOV") {
    // MOV <angle> -> relative move by angle (positive right, negative left)
    float deg = arg.toFloat();
    Serial.printf("MOV % .2f deg\n", deg);
    moveRelativeDegrees(deg);
    Serial.printf("Angle now: %.2f deg\n", getCurrentAngle());
    // update app temperature pin only when scanning; we keep V0 updates where needed
  }
  else if (cmd == "SET") {
    float a = arg.toFloat();
    startAngle = a;
    startAngleSet = true;
    Serial.printf("SET startAngle to %.2f (software starting/zero pos)\n", startAngle);
  }
  else if (cmd == "ZERO") {
    // Move to stored startAngle (do not remap)
    Serial.printf("ZERO command: moving to startAngle %.2f deg\n", startAngle);
    stopRequested = false;
    autoEnabled = false;
    manualMode = true;
    moveToAngle(startAngle);
    Serial.printf("Angle now: %.2f deg\n", getCurrentAngle());
  }
  else if (cmd == "STOP") {
    stopRequested = true;
    manualMode = true;
    autoEnabled = false;
    Serial.println("STOP: manual mode engaged, any movement stopped.");
  }
  else if (cmd == "AUTO") {
    if (criticalHalted) {
      Serial.println("Cannot start AUTO: device halted by critical. Use RESET first.");
    } else {
      manualMode = false;
      autoEnabled = true;
      stopRequested = false;
      Serial.println("AUTO mode started: automatic sweeping enabled.");
    }
  }
  else if (cmd == "RESET") {
    criticalHalted = false;
    stopRequested = false;
    Serial.println("Critical cleared via Serial RESET.");
  }
  else {
    Serial.println("Unknown command. Use MOV/AUTO/SET/ZERO/STOP/RESET");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- ESP32 MLX90614 + Stepper + Buzzers (Manual-first, V1/V2 updated) ---");
  // pins
  for (int i=0;i<4;i++) {
    pinMode(stepPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
  }
  pinMode(buzzer1, OUTPUT);
  pinMode(buzzer2, OUTPUT);
  buzzerOff(buzzer1);
  buzzerOff(buzzer2);

  Wire.begin();
  mlx = Adafruit_MLX90614();
  if (!mlx.begin()) {
    Serial.println("MLX90614 init failed. Check wiring.");
  } else {
    Serial.println("MLX90614 OK");
  }

  // connect to Blynk
  Blynk.begin(auth, WIFI_SSID, WIFI_PASS);

  // initial Blynk virtual state
  Blynk.virtualWrite(VP_TEMPERATURE, 0);
  Blynk.virtualWrite(VP_BUZZER_STOP, 0);
  Blynk.virtualWrite(VP_MOVE_ZERO, 0);

  Serial.println("Device started in MANUAL mode.");
  Serial.println("Serial Commands: MOV <angle>, AUTO, SET <angle>, ZERO, STOP, RESET");
  Serial.println("Example: SET -90  <ENTER>  ZERO  -> moves to -90 deg");
}

// helper: perform one auto sweep cycle (from currentStopIndex to next, reading temp etc.)
void performAutoSweepCycle(float stopAngles[], int &currentStopIndex) {
  static int sweepDirection = 1;
  int nextIndex = currentStopIndex + sweepDirection;
  // stopAngles array length is 7 (indices 0..6)
  if (nextIndex < 0 || nextIndex > 6) {
    sweepDirection *= -1;
    nextIndex = currentStopIndex + sweepDirection;
  }
  float targetAngle = stopAngles[nextIndex];

  // move to target
  moveToAngle(targetAngle);
  currentStopIndex = nextIndex;
  float measuredAngle = getCurrentAngle();
  Serial.printf("Reached stop angle: %.2f deg (steps: %ld)\n", measuredAngle, currentStep);

  // read temp
  float temp = mlx.readObjectTempC();
  if (isnan(temp)) temp = mlx.readAmbientTempC();
  Serial.printf("Temp at stop: %.2f C\n", temp);

  // update temperature to Blynk V0
  Blynk.virtualWrite(VP_TEMPERATURE, temp);

  int level = classifyTemp(temp);
  if (level == 0) {
    // Normal
    Serial.println("NORMAL range");
    buzzPatternNormal();
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)stopHoldMs) {
      if (criticalHalted || stopRequested) break;
      Blynk.run();
      handleSerial();
      delay(10);
    }
  } else if (level == 1) {
    // Alert: beep for 10s and stop for 10s, no Blynk notification
    Serial.println("ALERT LEVEL detected!");
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)alertBeepDurationMs) {
      if (criticalHalted || stopRequested) break;
      buzzPatternAlertContinuous(800);
      Blynk.run();
      handleSerial();
    }
    unsigned long t1 = millis();
    while (millis() - t1 < (unsigned long)alertStopMs) {
      if (criticalHalted || stopRequested) break;
      Blynk.run();
      handleSerial();
      delay(50);
    }
  } else if (level == 2) {
    // Critical: beep for 10s then halt indefinitely
    Serial.println("CRITICAL LEVEL detected!");
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)alertBeepDurationMs) {
      if (stopRequested) break;
      buzzPatternCriticalContinuous(800);
      Blynk.run();
      handleSerial();
      if (!buzzerEnabled) break;
    }
    // Halt indefinitely until RESET
    criticalHalted = true;
    autoEnabled = false;
    manualMode = true;
    Serial.println("Motor halted due to critical temperature. Use RESET to resume.");
  }
}

void loop() {
  Blynk.run();
  handleSerial();

  if (criticalHalted) {
    delay(200);
    return;
  }

  if (manualMode || !autoEnabled) {
    delay(10);
    return;
  }

  // Auto sweep mode execution
  // Prepare stop angles (−90..+90 step 30) => 7 stops
  static float stopAngles[7];
  static bool stopsPrepared = false;
  if (!stopsPrepared) {
    int idx = 0;
    for (float a = sweepMin; a <= sweepMax + 0.1; a += stopIntervalDeg) {
      stopAngles[idx++] = a;
    }
    stopsPrepared = true;
  }

  // find nearest stop index (on first run)
  static int currentStopIndex = -1;
  static bool initializedSweep = false;
  if (!initializedSweep) {
    float curAng = getCurrentAngle();
    int nearest = 0;
    float bestDiff = 1e9;
    for (int i=0;i<7;i++) {
      float d = fabs(curAng - stopAngles[i]);
      if (d < bestDiff) { bestDiff = d; nearest = i; }
    }
    currentStopIndex = nearest;
    initializedSweep = true;
  }

  // perform one sweep cycle (move to next stop, read temp, handle beeps)
  performAutoSweepCycle(stopAngles, currentStopIndex);

  // small idle
  delay(20);
}
