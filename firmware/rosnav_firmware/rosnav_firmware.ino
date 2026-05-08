// ============================================================
//  ROSNAV — STM32F103C6 Firmware (Differential Drive) By D. Anudeep,K. Vinay 
//
//  Board   : Generic STM32F103C6 (Blue Pill — 32KB flash)
//  IDE     : Arduino IDE + stm32duino (STM32F1 core)
//  Version : 2.1 
//
//  WIRING:
//    L298N ENA   → PA0   (PWM left speed)
//    L298N IN1   → PB0   (left direction)
//    L298N IN2   → PB1   (left direction)
//    L298N ENB   → PA1   (PWM right speed)
//    L298N IN3   → PA8   (right direction)
//    L298N IN4   → PA9   (right direction)
//    Left  Enc A → PA6   (interrupt)
//    Left  Enc B → PA7   (quadrature sense)
//    Right Enc A → PB9   (interrupt)
//    Right Enc B → PB8   (quadrature sense)
//    MPU6050 SDA → PB7   (optional)
//    MPU6050 SCL → PB6   (optional)
//    All Grounds tie together
//  PROTOCOL (JSON over USB @ 115200 baud):
//    Receive  : {"v":0.30,"w":0.50}
//    Transmit : {"el":120,"er":118,"ax":0.01,"ay":-0.02,"gz":1.5}
//
// ============================================================

#include <Wire.h>

// ── MPU6050 REGISTER MAP ─────────────────────────────────
#define MPU_ADDR       0x68
#define MPU_PWR_MGMT   0x6B
#define MPU_ACCEL_OUT  0x3B

// ── PIN DEFINITIONS ──────────────────────────────────────
#define MOTOR_L_EN   PA0
#define MOTOR_L_IN1  PB0
#define MOTOR_L_IN2  PB1

#define MOTOR_R_EN   PA1
#define MOTOR_R_IN1  PA8
#define MOTOR_R_IN2  PA9

#define ENC_L_A      PA6
#define ENC_L_B      PA7
#define ENC_R_A      PB9
#define ENC_R_B      PB8

// ── ROBOT PARAMETERS ─────────────────────────────────────
#define WHEEL_BASE    0.15f     // distance between wheels (meters)
#define MAX_SPEED_MS  0.57f     // max wheel speed (m/s)

// ── PWM DEAD-ZONE COMPENSATION ───────────────────────────
// Raise to 160 if motors stall at low speed.
// Lower to 100 if start is jerky.
#define MIN_PWM       140

// ── TIMING ───────────────────────────────────────────────
#define SERIAL_BAUD   115200
#define PUBLISH_MS    50        // telemetry rate = 20 Hz
#define WATCHDOG_MS   1000      // stop if Pi silent for 1 s

// ── GLOBAL STATE ─────────────────────────────────────────
volatile long encL = 0;
volatile long encR = 0;

float cmdV  = 0.0f;
float cmdW  = 0.0f;
bool  mpuOk = false;

unsigned long lastPublish = 0;
unsigned long lastCmd     = 0;

void encL_ISR() {
    // FLIPPED — left motor is mirrored on chassis
    if (digitalRead(ENC_L_B) == HIGH)
        encL--;
    else
        encL++;
}

void encR_ISR() {
    // CORRECT — right encoder counts normally
    if (digitalRead(ENC_R_B) == HIGH)
        encR++;
    else
        encR--;
}

bool mpuInit() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_PWR_MGMT);
    Wire.write(0x00);   // clear sleep bit → wake chip
    return (Wire.endTransmission() == 0);
}

void mpuRead(float &ax, float &ay, float &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_OUT);
    if (Wire.endTransmission(false) != 0) { ax = ay = gz = 0.0f; return; }

    Wire.requestFrom(MPU_ADDR, 14, true);
    if (Wire.available() < 14)            { ax = ay = gz = 0.0f; return; }

    int16_t ax_r = ((int16_t)Wire.read() << 8) | Wire.read();
    int16_t ay_r = ((int16_t)Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();   // accel Z  — skip
    Wire.read(); Wire.read();   // temp     — skip
    Wire.read(); Wire.read();   // gyro X   — skip
    Wire.read(); Wire.read();   // gyro Y   — skip
    int16_t gz_r = ((int16_t)Wire.read() << 8) | Wire.read();

    ax = (float)ax_r / 16384.0f;   // ±2 g
    ay = (float)ay_r / 16384.0f;
    gz = (float)gz_r /   131.0f;   // ±250 °/s
}


// ╔══════════════════════════════════════════════════════╗
// ║  SETUP                                               ║
// ╚══════════════════════════════════════════════════════╝

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    // Motors — safe state first
    pinMode(MOTOR_L_EN,  OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_EN,  OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);
    stopMotors();

    // Encoders
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, RISING);

    // IMU (optional)
    Wire.begin();
    Wire.setClock(400000);
    mpuOk = mpuInit();

    lastCmd = millis();

    if (mpuOk)
        Serial.println("{\"status\":\"READY\",\"imu\":true}");
    else
        Serial.println("{\"status\":\"READY\",\"imu\":false}");
}


// ╔══════════════════════════════════════════════════════╗
// ║  MAIN LOOP                                           ║
// ╚══════════════════════════════════════════════════════╝

void loop() {
    parseSerial();

    // Watchdog — stop if Pi goes silent
    if (millis() - lastCmd > WATCHDOG_MS) {
        cmdV = 0.0f;
        cmdW = 0.0f;
    }

    applyDrive(cmdV, cmdW);

    if (millis() - lastPublish >= PUBLISH_MS) {
        lastPublish = millis();
        publishData();
    }
}


// ╔══════════════════════════════════════════════════════╗
// ║  SERIAL PARSER                                       ║
// ║  Expects: {"v":0.30,"w":0.50}                       ║
// ╚══════════════════════════════════════════════════════╝

void parseSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() < 5 || line.charAt(0) != '{') return;

    int vi = line.indexOf("\"v\":");
    if (vi >= 0) {
        String s = line.substring(vi + 4);
        int e = s.indexOf(','); if (e < 0) e = s.indexOf('}');
        if (e > 0) cmdV = s.substring(0, e).toFloat();
    }

    int wi = line.indexOf("\"w\":");
    if (wi >= 0) {
        String s = line.substring(wi + 4);
        int e = s.indexOf(','); if (e < 0) e = s.indexOf('}');
        if (e > 0) cmdW = s.substring(0, e).toFloat();
    }

    cmdV = constrain(cmdV, -MAX_SPEED_MS, MAX_SPEED_MS);
    cmdW = constrain(cmdW, -3.14f, 3.14f);
    lastCmd = millis();
}


// ╔══════════════════════════════════════════════════════╗
// ║  DIFFERENTIAL DRIVE KINEMATICS                       ║
// ║  vL = v - w*(B/2)                                   ║
// ║  vR = v + w*(B/2)                                   ║
// ╚══════════════════════════════════════════════════════╝

void applyDrive(float v, float w) {
    float vL = v - (w * WHEEL_BASE / 2.0f);
    float vR = v + (w * WHEEL_BASE / 2.0f);

    int pwmL = (int)constrain((vL / MAX_SPEED_MS) * 255.0f, -255.0f, 255.0f);
    int pwmR = (int)constrain((vR / MAX_SPEED_MS) * 255.0f, -255.0f, 255.0f);

    setMotor(MOTOR_L_EN, MOTOR_L_IN1, MOTOR_L_IN2, pwmL);
    setMotor(MOTOR_R_EN, MOTOR_R_IN1, MOTOR_R_IN2, pwmR);
}


// ╔══════════════════════════════════════════════════════╗
// ║  MOTOR DRIVER                                        ║
// ║  pwm > 0 → forward  pwm < 0 → reverse  0 → coast   ║
// ╚══════════════════════════════════════════════════════╝

void setMotor(int en, int in1, int in2, int pwm) {
    if (pwm > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        pwm = map(pwm, 1, 255, MIN_PWM, 255);
    }
    else if (pwm < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        pwm = map(-pwm, 1, 255, MIN_PWM, 255);
    }
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        pwm = 0;
    }

    analogWrite(en, (uint8_t)constrain(pwm, 0, 255));
}

void stopMotors() {
    analogWrite(MOTOR_L_EN, 0);
    analogWrite(MOTOR_R_EN, 0);
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
}


// ╔══════════════════════════════════════════════════════╗
// ║  TELEMETRY                                           ║
// ║  {"el":120,"er":118,"ax":0.01,"ay":-0.02,"gz":1.5} ║
// ╚══════════════════════════════════════════════════════╝

void publishData() {
    float ax = 0.0f, ay = 0.0f, gz = 0.0f;
    if (mpuOk) mpuRead(ax, ay, gz);

    noInterrupts();
    long el = encL;
    long er = encR;
    interrupts();

    Serial.print("{\"el\":");
    Serial.print(el);
    Serial.print(",\"er\":");
    Serial.print(er);
    Serial.print(",\"ax\":");
    Serial.print(ax, 4);
    Serial.print(",\"ay\":");
    Serial.print(ay, 4);
    Serial.print(",\"gz\":");
    Serial.print(gz, 3);
    Serial.println("}");
}
