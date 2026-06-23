/*
 * MARS - Bot Test Sequence
 * 1. Forward 20cm
 * 2. Turn Right 90°
 * 3. Forward 20cm
 *
 * Constants:
 *   PPR = 692, Wheel dia = 34mm, Wheelbase = 125mm
 *   Ticks/mm  = 6.48
 *   Ticks/deg = 7.067
 *
 *   20cm = 200mm → 200 * 6.48 = 1296 ticks
 *   90°         → 90 * 7.067 =  636 ticks
 */
#include<Arduino.h>
// ─── Motor Pins (L293D) ──────────────────────────────────────────
#define R_IN1   18
#define R_IN2   23
#define R_EN    19

#define L_IN1   16
#define L_IN2   17
#define L_EN    4

// ─── Encoder Pins ────────────────────────────────────────────────
#define ENC_L_A  26
#define ENC_L_B  25
#define ENC_R_A  32
#define ENC_R_B  33

// ─── PWM ─────────────────────────────────────────────────────────
#define L_PWM_CH  0
#define R_PWM_CH  1
#define PWM_FREQ  5000
#define PWM_RES   8

// ─── Constants ───────────────────────────────────────────────────
const float TICKS_PER_MM  = 6.48;
const float TICKS_PER_DEG = 7.067;

const long TICKS_20CM = (long)(200.0 * TICKS_PER_MM);   // 1296
const long TICKS_90DEG = (long)(90.0 * TICKS_PER_DEG);  // 636

// ─── PID Gains ───────────────────────────────────────────────────
const float Kp = 1.2;
const float Ki = 0.0;
const float Kd = 0.4;

// ─── Encoder ─────────────────────────────────────────────────────
volatile long encL = 0;
volatile long encR = 0;

void IRAM_ATTR encL_ISR() {
  if (digitalRead(ENC_L_B)) encL++; else encL--;
}
void IRAM_ATTR encR_ISR() {
  if (digitalRead(ENC_R_B)) encR++; else encR--;
}

// ─── PID State ───────────────────────────────────────────────────
float pid_integral = 0;
float pid_last_err = 0;
unsigned long pid_last_time = 0;

// ════════════════════════════════════════════════════════════════


void stopMotors();
void moveForward(long targetTicks, int speed);
void turnRight(long targetTicks, int speed);
void resetPID();
int pidCalc(float err);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Motor pins
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  // PWM
  ledcSetup(L_PWM_CH, PWM_FREQ, PWM_RES);
  ledcSetup(R_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_EN, L_PWM_CH);
  ledcAttachPin(R_EN, R_PWM_CH);

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, RISING);

  stopMotors();
  delay(2000);  // wait 2s before starting so you can place the bot

  Serial.println("=== MARS Test Sequence Starting ===");

  // ── Step 1: Forward 20cm ──
  Serial.println("[1] Forward 20cm");
  moveForward(TICKS_20CM, 150);
  stopMotors();
  delay(500);

  // ── Step 2: Turn Right 90° ──
  Serial.println("[2] Turn Right 90deg");
  turnRight(TICKS_90DEG, 120);
  stopMotors();
  delay(500);

  // ── Step 3: Forward 20cm ──
  Serial.println("[3] Forward 20cm");
  moveForward(TICKS_20CM, 150);
  stopMotors();

  Serial.println("=== Sequence Complete ===");
}

void loop() {
  // nothing
}

// ─── Move Forward with PID straight correction ───────────────────
void moveForward(long targetTicks, int speed) {
  encL = encR = 0;
  resetPID();

  while (true) {
    long avgTicks = (abs(encL) + abs(encR)) / 2;
    if (avgTicks >= targetTicks) break;

    float err = (float)(encL - encR);
    int correction = pidCalc(err);

    int leftSpd  = constrain(speed - correction, 0, 255);
    int rightSpd = constrain(speed + correction, 0, 255);

    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);
    ledcWrite(L_PWM_CH, leftSpd);
    ledcWrite(R_PWM_CH, rightSpd);

    Serial.printf("  encL:%ld encR:%ld avg:%ld target:%ld\n",
                  encL, encR, avgTicks, targetTicks);
    delay(20);
  }
}

// ─── Turn Right: left wheel forward, right wheel backward ────────
void turnRight(long targetTicks, int speed) {
  encL = encR = 0;
  resetPID();

  while (true) {
    long avgTicks = (abs(encL) + abs(encR)) / 2;
    if (avgTicks >= targetTicks) break;

    // Slow down in last 20% of turn for accuracy
    long remaining = targetTicks - avgTicks;
    int spd = speed;
    if (remaining < targetTicks * 0.2) {
      spd = max(80, (int)(speed * remaining / (targetTicks * 0.2)));
    }

    // Right turn: left fwd, right back
    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
    ledcWrite(L_PWM_CH, spd);
    ledcWrite(R_PWM_CH, spd);

    Serial.printf("  encL:%ld encR:%ld avg:%ld target:%ld\n",
                  encL, encR, avgTicks, targetTicks);
    delay(20);
  }
}

// ─── Stop ────────────────────────────────────────────────────────
void stopMotors() {
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  ledcWrite(L_PWM_CH, 0);
  ledcWrite(R_PWM_CH, 0);
}

// ─── PID ─────────────────────────────────────────────────────────
void resetPID() {
  pid_integral = 0;
  pid_last_err = 0;
  pid_last_time = millis();
}

int pidCalc(float err) {
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  pid_last_time = now;

  pid_integral += err * dt;
  float derivative = (err - pid_last_err) / dt;
  pid_last_err = err;

  float output = Kp * err + Ki * pid_integral + Kd * derivative;
  return (int)constrain(output, -80, 80);
}