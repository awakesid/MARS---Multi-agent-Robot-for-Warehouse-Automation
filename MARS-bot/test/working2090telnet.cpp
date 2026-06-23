#include <Arduino.h>
#include <WiFi.h>

// ─── WiFi Config ─────────────────────────────────────────────────
const char* SSID     = "Robotics";
const char* PASSWORD = "passwordxaina";

WiFiServer telnetServer(23);
WiFiClient telnetClient;

// ─── Motor Pins ───────────────────────────────────────────────────
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
const float TICKS_PER_MM  = 6.48f;
const float TICKS_PER_DEG = 7.067f;

const long TICKS_20CM  = (long)(200.0f * TICKS_PER_MM);
const long TICKS_90DEG = (long)(75.0f  * TICKS_PER_DEG);

// ─── PID Gains (tunable at runtime) ──────────────────────────────
float Kp = 1.2f;
float Ki = 0.8f;
float Kd = 0.8f;

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

// ─── Remote Log ──────────────────────────────────────────────────
void rlog(const char* format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  Serial.println(buf);
  if (telnetClient && telnetClient.connected())
    telnetClient.println(buf);
}

// ─── Motor Control ───────────────────────────────────────────────
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
  if (dt <= 0.001f) dt = 0.02f;
  pid_last_time = now;
  pid_integral += err * dt;
  float derivative = (err - pid_last_err) / dt;
  pid_last_err = err;
  float output = Kp * err + Ki * pid_integral + Kd * derivative;
  return (int)constrain(output, -80, 80);
}

// ─── Sequence forward declare ─────────────────────────────────────
void runSequence();

// ─── Telnet Command Handler ───────────────────────────────────────
/*
 * Commands:
 *   kp <val>   set Kp        e.g.  kp 1.5
 *   ki <val>   set Ki        e.g.  ki 0.01
 *   kd <val>   set Kd        e.g.  kd 0.3
 *   pid        print current PID values
 *   run        run test sequence again
 *   help       show this menu
 */
void printHelp() {
  rlog("─────────────────────────────────────");
  rlog(" kp <val>   set Kp      e.g. kp 1.5");
  rlog(" ki <val>   set Ki      e.g. ki 0.01");
  rlog(" kd <val>   set Kd      e.g. kd 0.3");
  rlog(" pid        show current PID values");
  rlog(" run        run test sequence again");
  rlog(" help       show this menu");
  rlog("─────────────────────────────────────");
}

void handleTelnet() {
  // Accept new client
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      telnetClient = telnetServer.available();
      rlog("=== MARS Bot Monitor Connected ===");
      rlog("Kp=%.4f  Ki=%.4f  Kd=%.4f", Kp, Ki, Kd);
      printHelp();
    }
  }

  if (!telnetClient || !telnetClient.connected()) return;
  if (!telnetClient.available()) return;

  String line = telnetClient.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("kp ")) {
    Kp = line.substring(3).toFloat();
    rlog("[PID] Kp=%.4f  Ki=%.4f  Kd=%.4f", Kp, Ki, Kd);
  }
  else if (line.startsWith("ki ")) {
    Ki = line.substring(3).toFloat();
    rlog("[PID] Kp=%.4f  Ki=%.4f  Kd=%.4f", Kp, Ki, Kd);
  }
  else if (line.startsWith("kd ")) {
    Kd = line.substring(3).toFloat();
    rlog("[PID] Kp=%.4f  Ki=%.4f  Kd=%.4f", Kp, Ki, Kd);
  }
  else if (line == "pid") {
    rlog("[PID] Kp=%.4f  Ki=%.4f  Kd=%.4f", Kp, Ki, Kd);
  }
  else if (line == "run") {
    rlog("[CMD] Running sequence...");
    runSequence();
  }
  else if (line == "help") {
    printHelp();
  }
  else {
    rlog("[?] Unknown: '%s'  (type help)", line.c_str());
  }
}

// ─── Move Forward ────────────────────────────────────────────────
void moveForward(long targetTicks, int speed) {
  encL = encR = 0;
  resetPID();
  rlog("[FWD] target:%ld ticks  speed:%d", targetTicks, speed);

  while (true) {
    handleTelnet();
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

    rlog("  L:%ld R:%ld avg:%ld/%ld err:%.1f pwm:%d/%d",
         encL, encR, avgTicks, targetTicks, err, leftSpd, rightSpd);
    delay(20);
  }
}

// ─── Turn Right ──────────────────────────────────────────────────
void turnRight(long targetTicks, int speed) {
  encL = encR = 0;
  resetPID();
  rlog("[TURN RIGHT] target:%ld ticks  speed:%d", targetTicks, speed);

  while (true) {
    handleTelnet();
    long avgTicks = (abs(encL) + abs(encR)) / 2;
    if (avgTicks >= targetTicks) break;

    long remaining = targetTicks - avgTicks;
    int spd = speed;
    if (remaining < targetTicks * 0.2f) {
      spd = max(80, (int)(speed * (float)remaining / (targetTicks * 0.2f)));
    }

    digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH);
    ledcWrite(L_PWM_CH, spd);
    ledcWrite(R_PWM_CH, spd);

    rlog("  L:%ld R:%ld avg:%ld/%ld spd:%d",
         encL, encR, avgTicks, targetTicks, spd);
    delay(20);
  }
}

// ─── Test Sequence ───────────────────────────────────────────────
void runSequence() {
  rlog("── Step 1: Forward 20cm ──");
  moveForward(TICKS_20CM, 150);
  stopMotors();
  rlog("   Done. encL:%ld encR:%ld", encL, encR);
  delay(500);

  rlog("── Step 2: Turn Right 90deg ──");
  turnRight(TICKS_90DEG, 120);
  stopMotors();
  rlog("   Done. encL:%ld encR:%ld", encL, encR);
  delay(500);

  rlog("── Step 3: Forward 20cm ──");
  moveForward(TICKS_20CM, 150);
  stopMotors();
  rlog("   Done. encL:%ld encR:%ld", encL, encR);

  rlog("=== Sequence Complete ===");
  rlog("Tune PID with kp/ki/kd then type 'run' to repeat");
}

// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  ledcSetup(L_PWM_CH, PWM_FREQ, PWM_RES);
  ledcSetup(R_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_EN, L_PWM_CH);
  ledcAttachPin(R_EN, R_PWM_CH);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, RISING);

  stopMotors();

  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
  Serial.println("Connect:  nc <IP> 23");

  telnetServer.begin();

  Serial.println("Waiting 8s for monitor to connect...");
  unsigned long wait = millis();
  while (millis() - wait < 8000) {
    handleTelnet();
    delay(100);
  }

  runSequence();
}

void loop() {
  handleTelnet();
  delay(50);
}