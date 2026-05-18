/*
  TEAM 19 - Low-Level Arduino Firmware
  Arduino only does hardware I/O:
  - Applies signed PWM to motor driver
  - Generates steering servo pulses
  - Counts encoder pulses
  - Reads raw MPU6050 IMU values
  - Sends raw state to Raspberry Pi

  All processing/control/filtering/odometry is done on Raspberry Pi.
*/

#include <Wire.h>
#include <avr/wdt.h>

// =========================
// PINS
// =========================
const int PWM_PIN   = 10;   // L298N ENA / PWM
const int IN1       = 5;
const int IN2       = 6;

const int ENCODER_A = 2;    // interrupt pin
const int ENCODER_B = 3;    // direction channel

const int SERVO_PIN = 9;

// Arduino UNO I2C:
// MPU6050 SDA -> A4
// MPU6050 SCL -> A5

// =========================
// CONFIG
// =========================
const bool FORWARD_IN1_HIGH = true;
const bool USE_ENCODER_B = true;   // keep true if encoder B is wired to D3

const int PWM_DEADBAND = 4;
const unsigned long COMMAND_TIMEOUT_MS = 700;

// Servo pulse limits. RPi sends servo pulse in microseconds.
const int SERVO_MIN_US = 1000;
const int SERVO_CENTER_US = 1500;
const int SERVO_MAX_US = 2000;
const unsigned long SERVO_FRAME_US = 20000UL;

// State output rate. Keep moderate because baud is 9600.
const unsigned long STATE_PERIOD_MS = 100;  // 10 Hz
const unsigned long IMU_PERIOD_MS   = 20;   // read IMU internally at 50 Hz

// =========================
// MPU6050
// =========================
const byte MPU_ADDR = 0x68;
const byte MPU_PWR_MGMT_1   = 0x6B;
const byte MPU_ACCEL_XOUT_H = 0x3B;
const byte MPU_WHO_AM_I     = 0x75;
const byte MPU_CONFIG       = 0x1A;
const byte MPU_GYRO_CONFIG  = 0x1B;
const byte MPU_ACCEL_CONFIG = 0x1C;

bool imuOK = false;
int16_t axRaw = 0, ayRaw = 0, azRaw = 0;
int16_t gxRaw = 0, gyRaw = 0, gzRaw = 0;

// =========================
// STATE
// =========================
volatile long encoderCount = 0;

int appliedPWM = 0;          // signed PWM from -255 to +255
int servoPulseUs = SERVO_CENTER_US;

unsigned long lastCommandMs = 0;
unsigned long lastStateMs = 0;
unsigned long lastImuMs = 0;

// Non-blocking servo pulse state
unsigned long servoFrameStartUs = 0;
bool servoHigh = false;

// Non-blocking serial command buffer
char cmdBuffer[48];
byte cmdIndex = 0;

// =========================
// HELPERS
// =========================
int clampInt(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

void stopMotor() {
  appliedPWM = 0;
  analogWrite(PWM_PIN, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void applySignedPWM(int signedPWM) {
  signedPWM = clampInt(signedPWM, -255, 255);
  appliedPWM = signedPWM;

  int mag = abs(signedPWM);

  if (mag <= PWM_DEADBAND) {
    stopMotor();
    return;
  }

  bool forward = signedPWM > 0;

  if (FORWARD_IN1_HIGH) {
    digitalWrite(IN1, forward ? HIGH : LOW);
    digitalWrite(IN2, forward ? LOW  : HIGH);
  } else {
    digitalWrite(IN1, forward ? LOW  : HIGH);
    digitalWrite(IN2, forward ? HIGH : LOW);
  }

  analogWrite(PWM_PIN, mag);
}

void zeroEncoder() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

// =========================
// ENCODER ISR
// =========================
void encoderISR() {
  if (USE_ENCODER_B) {
    int b = digitalRead(ENCODER_B);
    if (b == HIGH) encoderCount++;
    else           encoderCount--;
  } else {
    if (appliedPWM >= 0) encoderCount++;
    else                 encoderCount--;
  }
}

// =========================
// NON-BLOCKING SERVO
// =========================
void serviceServo() {
  unsigned long nowUs = micros();

  if (!servoHigh && (unsigned long)(nowUs - servoFrameStartUs) >= SERVO_FRAME_US) {
    servoFrameStartUs = nowUs;
    digitalWrite(SERVO_PIN, HIGH);
    servoHigh = true;
  }

  if (servoHigh && (unsigned long)(nowUs - servoFrameStartUs) >= (unsigned long)servoPulseUs) {
    digitalWrite(SERVO_PIN, LOW);
    servoHigh = false;
  }
}

// =========================
// IMU LOW-LEVEL FUNCTIONS
// =========================
bool mpuWrite(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

byte mpuReadByte(byte reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;

  Wire.requestFrom(MPU_ADDR, (byte)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void setupIMU() {
  Wire.begin();
  Wire.setClock(400000L);

  byte who = mpuReadByte(MPU_WHO_AM_I);

  if (who != 0x68) {
    imuOK = false;
    return;
  }

  mpuWrite(MPU_PWR_MGMT_1, 0x00);   // wake up
  mpuWrite(MPU_CONFIG, 0x03);       // DLPF
  mpuWrite(MPU_GYRO_CONFIG, 0x00);  // +/-250 dps
  mpuWrite(MPU_ACCEL_CONFIG, 0x00); // +/-2g

  imuOK = true;
}

bool readIMURaw() {
  if (!imuOK) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    imuOK = false;
    return false;
  }

  Wire.requestFrom(MPU_ADDR, (byte)14);

  if (Wire.available() < 14) {
    imuOK = false;
    return false;
  }

  axRaw = (Wire.read() << 8) | Wire.read();
  ayRaw = (Wire.read() << 8) | Wire.read();
  azRaw = (Wire.read() << 8) | Wire.read();

  // temperature ignored
  Wire.read();
  Wire.read();

  gxRaw = (Wire.read() << 8) | Wire.read();
  gyRaw = (Wire.read() << 8) | Wire.read();
  gzRaw = (Wire.read() << 8) | Wire.read();

  return true;
}

void serviceIMU() {
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastImuMs) < IMU_PERIOD_MS) return;
  lastImuMs = nowMs;
  readIMURaw();
}

// =========================
// SERIAL COMMANDS
// =========================
void handleCommand(char *cmd) {
  // Expected:
  // C,<signed_pwm>,<servo_us>
  // STOP
  // ZERO
  // PING
  // RESET

  if (cmd[0] == 'C' && cmd[1] == ',') {
    char *p1 = cmd + 2;
    char *comma = strchr(p1, ',');

    if (comma != NULL) {
      *comma = '\0';
      char *p2 = comma + 1;

      int pwm = atoi(p1);
      int servo = atoi(p2);

      pwm = clampInt(pwm, -255, 255);
      servo = clampInt(servo, SERVO_MIN_US, SERVO_MAX_US);

      applySignedPWM(pwm);
      servoPulseUs = servo;
      lastCommandMs = millis();
    }
    return;
  }

  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "S") == 0) {
    stopMotor();
    lastCommandMs = millis();
    Serial.println("A,STOP");
    return;
  }

  if (strcmp(cmd, "ZERO") == 0 || strcmp(cmd, "Z") == 0) {
    zeroEncoder();
    Serial.println("A,ZERO");
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    Serial.println("A,PONG");
    return;
  }

  if (strcmp(cmd, "RESET") == 0) {
    stopMotor();
    Serial.println("A,RESETTING");
    Serial.flush();
    wdt_enable(WDTO_15MS);
    while (true) { }
  }
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdIndex > 0) handleCommand(cmdBuffer);
      cmdIndex = 0;
    } else if (c != '\r') {
      if (cmdIndex < sizeof(cmdBuffer) - 1) {
        cmdBuffer[cmdIndex++] = c;
      } else {
        cmdIndex = 0; // overflow protection
      }
    }
  }
}

void serviceSafety() {
  if ((unsigned long)(millis() - lastCommandMs) > COMMAND_TIMEOUT_MS) {
    if (appliedPWM != 0) stopMotor();
  }
}

void printState() {
  unsigned long nowMs = millis();
  if ((unsigned long)(nowMs - lastStateMs) < STATE_PERIOD_MS) return;
  lastStateMs = nowMs;

  long enc;
  noInterrupts();
  enc = encoderCount;
  interrupts();

  // Compact CSV for 9600 baud:
  // S,time_ms,encoder,ax,ay,az,gx,gy,gz,pwm,servo_us,imu_ok
  Serial.print("S,");
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(enc);
  Serial.print(',');
  Serial.print(axRaw);
  Serial.print(',');
  Serial.print(ayRaw);
  Serial.print(',');
  Serial.print(azRaw);
  Serial.print(',');
  Serial.print(gxRaw);
  Serial.print(',');
  Serial.print(gyRaw);
  Serial.print(',');
  Serial.print(gzRaw);
  Serial.print(',');
  Serial.print(appliedPWM);
  Serial.print(',');
  Serial.print(servoPulseUs);
  Serial.print(',');
  Serial.println(imuOK ? 1 : 0);
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  stopMotor();
  servoPulseUs = SERVO_CENTER_US;

  Serial.begin(9600);
  Serial.setTimeout(2);

  setupIMU();

  zeroEncoder();
  lastCommandMs = millis();
  lastStateMs = millis();
  lastImuMs = millis();
  servoFrameStartUs = micros();

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  Serial.print("READY,LOW_LEVEL,IMU=");
  Serial.println(imuOK ? 1 : 0);
}

void loop() {
  serviceSerial();
  serviceServo();
  serviceIMU();
  serviceSafety();
  printState();
}
