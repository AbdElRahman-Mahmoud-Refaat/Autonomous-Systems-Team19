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

// manual servo pulse generation
const int SERVO_MIN_US       = 1000;
const int SERVO_MAX_US       = 2000;
const unsigned long SERVO_FRAME_US = 20000UL;

int currentAngle = SERVO_CENTER;
unsigned long lastServoFrameUs = 0;

// =========================
// MOTOR SETTINGS
// =========================
const bool FORWARD_IN1_HIGH = true;
const int MIN_EFFECTIVE_PWM = 115;
const int STOP_DEADBAND     = 3;

// =========================
// ENCODER
// =========================
volatile long encoderCount = 0;
unsigned long lastReportMs = 0;

// =========================
// SEQUENCE DEFINITION
// =========================
// angleDeg, signedCmd, durationMs, label
struct Step {
  int angleDeg;      // steering angle
  int signedCmd;     // +forward, -backward, 0 stop
  unsigned long durationMs;
  const char* label;
};

// Example open-loop sequence for milestone maneuvers
Step sequence[] = {
  {SERVO_CENTER,  140, 2500, "Forward"},
  {SERVO_CENTER,    0, 1000, "Stop"},
  {SERVO_CENTER, -140, 2500, "Backward"},
  {SERVO_CENTER,    0, 1000, "Stop"},
  {SERVO_LEFT_LIMIT, 140, 2500, "Turn Left"},
  {SERVO_CENTER,    0, 1000, "Stop"},
  {SERVO_RIGHT_LIMIT, 140, 2500, "Turn Right"},
  {SERVO_CENTER,    0, 2000, "Stop End"}
};

const int NUM_STEPS = sizeof(sequence) / sizeof(sequence[0]);
int currentStep = 0;
unsigned long stepStartMs = 0;
bool sequenceStarted = false;

// =========================
// HELPERS
// =========================
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  if (a == b) encoderCount++;
  else        encoderCount--;
}

void stopMotorHard() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
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

void applyStep(const Step& s) {
  currentAngle = s.angleDeg;
  driveSignedCommand(s.signedCmd);
}

void zeroEncoder() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

void startSequence() {
  currentStep = 0;
  stepStartMs = millis();
  zeroEncoder();
  applyStep(sequence[currentStep]);

  Serial.print("STEP -> ");
  Serial.println(sequence[currentStep].label);

  sequenceStarted = true;
}

void serviceSequence() {
  if (!sequenceStarted) {
    startSequence();
    return;
  }

  unsigned long now = millis();

  if (now - stepStartMs >= sequence[currentStep].durationMs) {
    currentStep++;

    if (currentStep >= NUM_STEPS) {
      // sequence finished; hold safe stop forever
      currentAngle = SERVO_CENTER;
      stopMotorHard();
      return;
    }

    stepStartMs = now;
    applyStep(sequence[currentStep]);

    Serial.print("STEP -> ");
    Serial.println(sequence[currentStep].label);
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

  currentAngle = SERVO_CENTER;
  stopMotorHard();

  Serial.println("Open-loop sequence ready");
}

// =========================
// LOOP
// =========================
void loop() {
  serviceServo();
  serviceSequence();

  if (millis() - lastReportMs >= 700) {
    lastReportMs = millis();

    long enc;
    noInterrupts();
    enc = encoderCount;
    interrupts();

    Serial.print("STEP_INDEX=");
    Serial.print(currentStep);
    Serial.print(" ANGLE=");
    Serial.print(currentAngle);
    Serial.print(" ENC=");
    Serial.println(enc);
  }
}