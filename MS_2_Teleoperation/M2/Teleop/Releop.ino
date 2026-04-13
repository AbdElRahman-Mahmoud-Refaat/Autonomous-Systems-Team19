#include <avr/wdt.h>

// =========================
// PIN DEFINITIONS
// =========================
const int SERVO_PIN   = 9;

const int MOTOR_IN1   = 5;
const int MOTOR_IN2   = 6;
const int MOTOR_ENA   = 10;

const int ENCODER_A   = 2;
const int ENCODER_B   = 3;

// =========================
// SERVO SETTINGS
// =========================
const int SERVO_CENTER       = 90;
const int SERVO_LEFT_LIMIT   = 60;
const int SERVO_RIGHT_LIMIT  = 120;
const int STEP_DEG           = 5;

// Servo pulse widths in microseconds
const int SERVO_MIN_US       = 1000;
const int SERVO_MAX_US       = 2000;
const unsigned long SERVO_FRAME_US = 20000UL;   // 20 ms

int currentAngle = SERVO_CENTER;
unsigned long lastServoFrameUs = 0;

// =========================
// MOTOR SETTINGS
// =========================
// Set this so UP means physical forward.
// If UP and DOWN are swapped after test, change true <-> false.
const bool FORWARD_IN1_HIGH = true;

const int STEP_CMD           = 14;
const int MAX_CMD            = 255;
const int RAMP_STEP          = 6;
const int RAMP_PERIOD_MS     = 10;
const int MIN_EFFECTIVE_PWM  = 115;
const int STOP_DEADBAND      = 3;

// =========================
// STATE
// =========================
volatile long encoderCount = 0;

int targetCmd = 0;
int currentCmd = 0;

String line = "";

unsigned long lastRampMs = 0;
unsigned long lastReportMs = 0;

// =========================
// ENCODER
// =========================
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  if (a == b) encoderCount++;
  else        encoderCount--;
}

// =========================
// HELPERS
// =========================
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void stopMotorHard() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}

void zeroEncoder() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

int angleToPulseUs(int angleDeg) {
  angleDeg = clampInt(angleDeg, 0, 180);
  long pulse = SERVO_MIN_US + ((long)angleDeg * (SERVO_MAX_US - SERVO_MIN_US)) / 180L;
  return (int)pulse;
}

void serviceServo() {
  unsigned long nowUs = micros();

  if (nowUs - lastServoFrameUs >= SERVO_FRAME_US) {
    lastServoFrameUs = nowUs;

    int pulseUs = angleToPulseUs(currentAngle);

    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(SERVO_PIN, LOW);
  }
}

void goCenter() {
  currentAngle = SERVO_CENTER;
}

void stepLeft() {
  currentAngle -= STEP_DEG;
  if (currentAngle < SERVO_LEFT_LIMIT) currentAngle = SERVO_LEFT_LIMIT;
}

void stepRight() {
  currentAngle += STEP_DEG;
  if (currentAngle > SERVO_RIGHT_LIMIT) currentAngle = SERVO_RIGHT_LIMIT;
}

void driveSignedCommand(int signedCmd) {
  int mag = abs(signedCmd);

  if (mag <= STOP_DEADBAND) {
    stopMotorHard();
    return;
  }

  int pwm = MIN_EFFECTIVE_PWM + (long)(255 - MIN_EFFECTIVE_PWM) * mag / 255;
  if (pwm > 255) pwm = 255;

  bool forward = (signedCmd > 0);

  if (FORWARD_IN1_HIGH) {
    if (forward) {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
    } else {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
    }
  } else {
    if (forward) {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
    } else {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
    }
  }

  analogWrite(MOTOR_ENA, pwm);
}

void serviceRamp() {
  unsigned long now = millis();
  if (now - lastRampMs < RAMP_PERIOD_MS) return;
  lastRampMs = now;

  if (currentCmd < targetCmd) {
    currentCmd += RAMP_STEP;
    if (currentCmd > targetCmd) currentCmd = targetCmd;
  } else if (currentCmd > targetCmd) {
    currentCmd -= RAMP_STEP;
    if (currentCmd < targetCmd) currentCmd = targetCmd;
  }

  driveSignedCommand(currentCmd);
}

void clearAllState() {
  targetCmd = 0;
  currentCmd = 0;
  stopMotorHard();
  zeroEncoder();
}

void softResetArduino() {
  goCenter();
  clearAllState();

  Serial.println("RESETTING");
  Serial.flush();
  delay(50);

  wdt_enable(WDTO_15MS);
  while (true) { }
}

// =========================
// COMMAND HANDLER
// =========================
void handleCommand(String cmd) {
  cmd.trim();

  // Servo logic
  if (cmd == "L") {
    stepLeft();
    Serial.print("Angle -> ");
    Serial.println(currentAngle);
  }
  else if (cmd == "R") {
    stepRight();
    Serial.print("Angle -> ");
    Serial.println(currentAngle);
  }
  else if (cmd == "CENTER") {
    goCenter();
    stopMotorHard();
    Serial.print("Angle -> ");
    Serial.println(currentAngle);
  }

  // Motor logic
  else if (cmd == "UP") {
    targetCmd += STEP_CMD;
    targetCmd = clampInt(targetCmd, -MAX_CMD, MAX_CMD);
  }
  else if (cmd == "DOWN") {
    targetCmd -= STEP_CMD;
    targetCmd = clampInt(targetCmd, -MAX_CMD, MAX_CMD);
  }
  else if (cmd == "CLEAR") {
    clearAllState();
    Serial.println("CLEARED");
  }
  else if (cmd == "ZEROENC") {
    zeroEncoder();
    Serial.println("ENC=0");
  }

  // Shared reset
  else if (cmd == "RESET") {
    softResetArduino();
  }
  else {
    Serial.print("Unknown cmd -> ");
    Serial.println(cmd);
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  pinMode(SERVO_PIN, OUTPUT);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  goCenter();
  clearAllState();

  Serial.println("READY");
  Serial.print("Initial angle -> ");
  Serial.println(currentAngle);
}

// =========================
// LOOP
// =========================
void loop() {
  serviceServo();
  serviceRamp();

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      if (line.length() > 0) {
        handleCommand(line);
      }
      line = "";
    } else {
      line += c;
    }
  }

  if (millis() - lastReportMs >= 700) {
    lastReportMs = millis();

    long enc;
    noInterrupts();
    enc = encoderCount;
    interrupts();

    Serial.print("T=");
    Serial.print(targetCmd);
    Serial.print(" C=");
    Serial.print(currentCmd);
    Serial.print(" E=");
    Serial.print(enc);
    Serial.print(" A=");
    Serial.println(currentAngle);
  }
}