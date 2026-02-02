/*********************************************************************
 Project  : IoT-Based Thermal Monitoring & Scanning System
 Author   : Navee
 Platform : ESP32 + Blynk IoT
 
 Virtual Pins:
  V0 = Temperature display
  V1 = Buzzer STOP switch (1 = stop buzzer, 0 = allow buzzer)
  V2 = Move-to-zero switch (momentary)
 
 Serial Commands:
  MOV <angle>, AUTO, SET <angle>, ZERO, STOP, RESET
 
 *********************************************************************/

/*
- V0 = temperature
- V1 = buzzer STOP switch (1 -> stop buzzer, 0 -> allow buzzer)
- V2 = move-to-zero switch (momentary: moves stepper to stored startAngle)
- Serial commands: MOV <angle>, AUTO, SET <angle>, ZERO, STOP, RESET
- ZERO command and V2 BOTH move motor to startAngle
*/

// ---- Blynk IoT required macros (MUST be before Blynk include) ----
#define BLYNK_TEMPLATE_ID   "TMPL3VtHsYk6W"
#define BLYNK_TEMPLATE_NAME "Thermal Device"
#define BLYNK_AUTH_TOKEN    "xqNJXB348lc08H-Pr6Ql2M26SZbAc_aF"

// Auth token for Blynk
char auth[] = BLYNK_AUTH_TOKEN;

// -------------------- LIBRARIES --------------------
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BlynkSimpleEsp32.h>

// -------------------- WIFI CONFIG --------------------
const char* WIFI_SSID = "project";
const char* WIFI_PASS = "12345678";

// -------------------- PIN DEFINITIONS --------------------
const int stepPins[4] = {19, 18, 5, 17}; // ULN2003 IN1–IN4
const int buzzer1 = 25;
const int buzzer2 = 26;

// -------------------- STEPPER PARAMETERS --------------------
const float STEPS_PER_REV = 4096.0;     // 28BYJ-48
const float STEPS_PER_DEG = STEPS_PER_REV / 360.0;
const int stepDelayMs = 2;              // motor speed control

// -------------------- SCANNING PARAMETERS --------------------
const int stopIntervalDeg = 30;
const int stopHoldMs = 5000;

// -------------------- ALERT TIMINGS --------------------
const int alertBeepDurationMs = 10000;
const int alertStopMs = 10000;

// -------------------- TEMPERATURE THRESHOLDS --------------------
const float normalMin = 15.0;
const float normalMax = 35.0;
const float alert1Min = 36.0;
const float alert1Max = 45.0;
const float alert2Min = 10.0;
const float alert2Max = 19.0;
const float criticalMin = 45.0;

// -------------------- SWEEP LIMITS --------------------
float sweepMin = -90.0;
float sweepMax = 90.0;

// -------------------- BLYNK VIRTUAL PINS --------------------
#define VP_TEMPERATURE V0
#define VP_BUZZER_STOP V1
#define VP_MOVE_ZERO   V2

// -------------------- GLOBAL OBJECTS --------------------
Adafruit_MLX90614 mlx;

// -------------------- STATE VARIABLES --------------------
long currentStep = 0;
float startAngle = 0.0;
bool startAngleSet = false;

bool buzzerEnabled = true;
bool criticalHalted = false;

bool manualMode = true;
bool autoEnabled = false;
bool stopRequested = false;

// -------------------- STEPPER HALF-STEP SEQUENCE --------------------
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

// -------------------- LOW-LEVEL STEPPER CONTROL --------------------
void writeStep(int idx) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(stepPins[i], stepSequence[idx][i]);
  }
}

void stepOnce(int dir) {
  static int seqIndex = 0;
  seqIndex += dir;
  while (seqIndex < 0) seqIndex += seqSteps;
  seqIndex %= seqSteps;
  writeStep(seqIndex);
  delay(stepDelayMs);
  currentStep += dir;
}

// -------------------- MOTOR MOVEMENT --------------------
void moveRelativeDegrees(float deg) {
  long steps = (long)round(fabs(deg) * STEPS_PER_DEG);
  int dir = (deg >= 0) ? 1 : -1;
  stopRequested = false;

  for (long i = 0; i < steps; i++) {
    if (criticalHalted || stopRequested) break;
    stepOnce(dir);
    Blynk.run();
  }
}

void moveToAngle(float targetAngle) {
  long targetStep = (long)round(targetAngle * STEPS_PER_DEG);
  long delta = targetStep - currentStep;
  if (delta == 0) return;

  int dir = (delta > 0) ? 1 : -1;
  long stepsToMove = abs(delta);
  stopRequested = false;

  for (long i = 0; i < stepsToMove; i++) {
    if (criticalHalted || stopRequested) break;
    stepOnce(dir);
    Blynk.run();
  }
}

float getCurrentAngle() {
  return ((float)currentStep) / STEPS_PER_DEG;
}

// -------------------- BUZZER CONTROL --------------------
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
    for (int i = 0; i < 5 && (millis() - t0 < (unsigned long)durationMs); ++i) {
      buzzerOn(buzzer1); buzzerOn(buzzer2); delay(600);
      buzzerOff(buzzer1); buzzerOff(buzzer2); delay(200);
    }
  }
}

// -------------------- TEMPERATURE CLASSIFICATION --------------------
int classifyTemp(float t) {
  if (t > criticalMin) return 2;
  if ((t >= alert1Min && t <= alert1Max) ||
      (t >= alert2Min && t <= alert2Max)) return 1;
  if (t >= normalMin && t <= normalMax) return 0;
  return 1;
}

// -------------------- BLYNK CALLBACKS --------------------
BLYNK_WRITE(VP_BUZZER_STOP) {
  int v = param.asInt();
  if (v == 1) {
    buzzerEnabled = false;
    buzzerOff(buzzer1);
    buzzerOff(buzzer2);
  } else {
    buzzerEnabled = true;
  }
}

BLYNK_WRITE(VP_MOVE_ZERO) {
  if (param.asInt() == 1) {
    stopRequested = false;
    autoEnabled = false;
    manualMode = true;
    moveToAngle(startAngle);
    Blynk.virtualWrite(VP_MOVE_ZERO, 0);
  }
}

// -------------------- SERIAL COMMAND HANDLER --------------------
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

  if (cmd == "MOV") {
    moveRelativeDegrees(arg.toFloat());
  } else if (cmd == "SET") {
    startAngle = arg.toFloat();
    startAngleSet = true;
  } else if (cmd == "ZERO") {
    stopRequested = false;
    autoEnabled = false;
    manualMode = true;
    moveToAngle(startAngle);
  } else if (cmd == "STOP") {
    stopRequested = true;
    manualMode = true;
    autoEnabled = false;
  } else if (cmd == "AUTO") {
    if (!criticalHalted) {
      manualMode = false;
      autoEnabled = true;
      stopRequested = false;
    }
  } else if (cmd == "RESET") {
    criticalHalted = false;
    stopRequested = false;
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 4; i++) {
    pinMode(stepPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
  }

  pinMode(buzzer1, OUTPUT);
  pinMode(buzzer2, OUTPUT);
  buzzerOff(buzzer1);
  buzzerOff(buzzer2);

  Wire.begin();
  mlx.begin();

  Blynk.begin(auth, WIFI_SSID, WIFI_PASS);

  Blynk.virtualWrite(VP_TEMPERATURE, 0);
  Blynk.virtualWrite(VP_BUZZER_STOP, 0);
  Blynk.virtualWrite(VP_MOVE_ZERO, 0);
}

// -------------------- AUTO SWEEP FUNCTION --------------------
void performAutoSweepCycle(float stopAngles[], int &currentStopIndex) {
  static int sweepDirection = 1;
  int nextIndex = currentStopIndex + sweepDirection;

  if (nextIndex < 0 || nextIndex > 6) {
    sweepDirection *= -1;
    nextIndex = currentStopIndex + sweepDirection;
  }

  moveToAngle(stopAngles[nextIndex]);
  currentStopIndex = nextIndex;

  float temp = mlx.readObjectTempC();
  if (isnan(temp)) temp = mlx.readAmbientTempC();

  Blynk.virtualWrite(VP_TEMPERATURE, temp);

  int level = classifyTemp(temp);

  if (level == 0) {
    buzzPatternNormal();
    delay(stopHoldMs);
  } else if (level == 1) {
    buzzPatternAlertContinuous(alertBeepDurationMs);
    delay(alertStopMs);
  } else {
    buzzPatternCriticalContinuous(alertBeepDurationMs);
    criticalHalted = true;
    autoEnabled = false;
    manualMode = true;
  }
}

// -------------------- MAIN LOOP --------------------
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

  static float stopAngles[7];
  static bool stopsPrepared = false;
  if (!stopsPrepared) {
    int idx = 0;
    for (float a = sweepMin; a <= sweepMax + 0.1; a += stopIntervalDeg) {
      stopAngles[idx++] = a;
    }
    stopsPrepared = true;
  }

  static int currentStopIndex = -1;
  static bool initializedSweep = false;
  if (!initializedSweep) {
    currentStopIndex = 3;
    initializedSweep = true;
  }

  performAutoSweepCycle(stopAngles, currentStopIndex);
  delay(20);
}
