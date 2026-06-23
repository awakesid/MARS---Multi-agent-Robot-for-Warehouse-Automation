#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ─── WiFi Config ─────────────────────────────────────────────────
const char* SSID     = "Robotics";
const char* PASSWORD = "passwordxaina";
const int   UDP_PORT = 4210;

// ─── Bot Identity ────────────────────────────────────────────────
const int BOT_ID = 1;  // change per bot: 1, 2, 3

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

// ─── Robot Constants ─────────────────────────────────────────────
const float TICKS_PER_MM  = 6.48f;
const float TICKS_PER_DEG = 7.067f;
const int   BASE_SPEED    = 150;
const int   TURN_SPEED    = 120;
const int   MIN_SPEED     = 85;   // min PWM before motor stalls (L293D)

// ─── PID Gains (tunable via telnet) ──────────────────────────────
float Kp = 1.2f;
float Ki = 0.0f;
float Kd = 0.4f;

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

// ─── Network ─────────────────────────────────────────────────────
WiFiUDP     udp;
WiFiServer  telnetServer(23);
WiFiClient  telnetClient;

IPAddress   remoteIP;
uint16_t    remotePort = 0;

// ─── Log (telnet + serial) ────────────────────────────────────────
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

// ─── UDP Reply ────────────────────────────────────────────────────
void udpReply(const char* type, const char* extra = "") {
  if (remotePort == 0) return;
  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"id\":%d,\"type\":\"%s\",\"encL\":%ld,\"encR\":%ld%s}",
    BOT_ID, type, encL, encR, extra);
  udp.beginPacket(remoteIP, remotePort);
  udp.print(buf);
  udp.endPacket();
}

// ─── Motor Control ───────────────────────────────────────────────
void stopMotors() {
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  ledcWrite(L_PWM_CH, 0);
  ledcWrite(R_PWM_CH, 0);
}

void motorLeft(int spd) {
  if (spd >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
  else          { digitalWrite(L_IN1, LOW);  digitalWrite(L_IN2, HIGH); spd = -spd; }
  ledcWrite(L_PWM_CH, constrain(spd, 0, 255));
}

void motorRight(int spd) {
  if (spd >= 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); }
  else          { digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); spd = -spd; }
  ledcWrite(R_PWM_CH, constrain(spd, 0, 255));
}

// ─── PID ─────────────────────────────────────────────────────────
void resetPID() {
  pid_integral = 0; pid_last_err = 0; pid_last_time = millis();
}

int pidCalc(float err) {
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0f;
  if (dt < 0.001f) dt = 0.02f;
  pid_last_time = now;
  pid_integral += err * dt;
  float deriv = (err - pid_last_err) / dt;
  pid_last_err = err;
  return (int)constrain(Kp*err + Ki*pid_integral + Kd*deriv, -80, 80);
}

// ─── Odometry Send (10Hz) ─────────────────────────────────────────
unsigned long lastOdomMs = 0;
void sendOdom() {
  if (millis() - lastOdomMs < 100) return;
  lastOdomMs = millis();
  udpReply("odom");
}

// ════════════════════════════════════════════════════════════════
//  MOVE FORWARD
// ════════════════════════════════════════════════════════════════
void moveForward(float dist_mm, int speed) {
  long target = (long)(dist_mm * TICKS_PER_MM);
  encL = encR = 0;
  resetPID();
  rlog("[FWD] %.0fmm → %ld ticks", dist_mm, target);
  udpReply("ack");

  while (true) {
    long avg = (abs(encL) + abs(encR)) / 2;
    if (avg >= target) break;

    // slow down last 15%
    long rem = target - avg;
    int spd = speed;
    if (rem < target * 0.15f)
      spd = max(MIN_SPEED, (int)(speed * (float)rem / (target * 0.15f)));

    float err = (float)(encL - encR);
    int   cor = pidCalc(err);
    motorLeft (spd - cor);
    motorRight(spd + cor);

    sendOdom();
    delay(20);
  }
  stopMotors();
  rlog("[FWD] done encL:%ld encR:%ld", encL, encR);
  udpReply("done");
}

// ════════════════════════════════════════════════════════════════
//  MOVE BACKWARD
// ════════════════════════════════════════════════════════════════
void moveBackward(float dist_mm, int speed) {
  long target = (long)(dist_mm * TICKS_PER_MM);
  encL = encR = 0;
  resetPID();
  rlog("[BWD] %.0fmm → %ld ticks", dist_mm, target);
  udpReply("ack");

  while (true) {
    long avg = (abs(encL) + abs(encR)) / 2;
    if (avg >= target) break;

    long rem = target - avg;
    int spd = speed;
    if (rem < target * 0.15f)
      spd = max(MIN_SPEED, (int)(speed * (float)rem / (target * 0.15f)));

    float err = (float)(encL - encR);
    int   cor = pidCalc(err);
    motorLeft (-(spd - cor));
    motorRight(-(spd + cor));

    sendOdom();
    delay(20);
  }
  stopMotors();
  rlog("[BWD] done encL:%ld encR:%ld", encL, encR);
  udpReply("done");
}

// ════════════════════════════════════════════════════════════════
//  TURN  — with overshoot correction
//  dir: +1 = left,  -1 = right
// ════════════════════════════════════════════════════════════════
void executeTurn(float angle_deg, int dir) {
  long target = (long)(angle_deg * TICKS_PER_DEG);
  encL = encR = 0;

  // ── Phase 1: main turn ────────────────────────────────────────
  while (true) {
    long avg = (abs(encL) + abs(encR)) / 2;
    if (avg >= target) break;

    long rem = target - avg;
    int spd = TURN_SPEED;
    if (rem < target * 0.25f)
      spd = max(MIN_SPEED, (int)(TURN_SPEED * (float)rem / (target * 0.25f)));

    // dir +1 LEFT:  left back,  right fwd
    // dir -1 RIGHT: left fwd,   right back
    motorLeft (dir * (-spd));
    motorRight(dir * spd);

    sendOdom();
    delay(20);
  }
  stopMotors();
  delay(80);  // let bot settle mechanically

  // ── Phase 2: overshoot correction (±15 ticks tolerance) ───────
  long avg = (abs(encL) + abs(encR)) / 2;
  long error = avg - target;
  rlog("[TURN] landed:%ld target:%ld error:%ld", avg, target, error);

  if (abs(error) > 15) {
    // nudge back in opposite direction
    int nudgeDir = (error > 0) ? -dir : dir;
    encL = encR = 0;
    long nudgeTicks = abs(error);

    rlog("[TURN] correcting %ld ticks...", nudgeTicks);
    while (true) {
      long navg = (abs(encL) + abs(encR)) / 2;
      if (navg >= nudgeTicks) break;
      motorLeft (nudgeDir * (-MIN_SPEED));
      motorRight(nudgeDir *  MIN_SPEED);
      delay(10);
    }
    stopMotors();
    delay(50);
  }

  rlog("[TURN] done. encL:%ld encR:%ld", encL, encR);
  udpReply("done");
}

void turnLeft(float angle_deg)  { 
  rlog("[TURNL] %.1f deg", angle_deg);
  udpReply("ack");
  executeTurn(angle_deg, +1); 
}
void turnRight(float angle_deg) { 
  rlog("[TURNR] %.1f deg", angle_deg);
  udpReply("ack");
  executeTurn(angle_deg, -1); 
}

// ════════════════════════════════════════════════════════════════
//  UDP PACKET HANDLER
// ════════════════════════════════════════════════════════════════
/*
  Expected JSON from app/ROS:
  {"id":1, "cmd":"FORWARD",   "dist":200}
  {"id":1, "cmd":"BACKWARD",  "dist":150}
  {"id":1, "cmd":"TURN_L",    "angle":90}
  {"id":1, "cmd":"TURN_R",    "angle":90}
  {"id":1, "cmd":"STOP"}

  Replies:
  {"id":1, "type":"ack",  "encL":0,    "encR":0}
  {"id":1, "type":"odom", "encL":1234, "encR":1230}
  {"id":1, "type":"done", "encL":1296, "encR":1292}
*/
void handleUDP() {
  int sz = udp.parsePacket();
  if (!sz) return;

  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = '\0';

  remoteIP   = udp.remoteIP();
  remotePort = udp.remotePort();

  rlog("[UDP] %s", buf);

  JsonDocument doc;
  if (deserializeJson(doc, buf) != DeserializationError::Ok) {
    rlog("[UDP] bad JSON");
    return;
  }

  // extract first, then compare — ArduinoJson v7 MemberProxy is non-copyable
  int         id   = doc["id"]    | 0;
  const char* cmd  = doc["cmd"]   | "";
  float       dist = doc["dist"]  | 200.0f;
  float       ang  = doc["angle"] | 90.0f;

  if (id != BOT_ID) return;

  if      (strcmp(cmd, "FORWARD")  == 0) moveForward (dist, BASE_SPEED);
  else if (strcmp(cmd, "BACKWARD") == 0) moveBackward(dist, BASE_SPEED);
  else if (strcmp(cmd, "TURN_L")   == 0) turnLeft    (ang);
  else if (strcmp(cmd, "TURN_R")   == 0) turnRight   (ang);
  else if (strcmp(cmd, "STOP")     == 0) {
    stopMotors();
    rlog("[STOP]");
    udpReply("stopped");
  }
  else rlog("[UDP] unknown cmd: %s", cmd);
}

// ════════════════════════════════════════════════════════════════
//  TELNET HANDLER (PID tuning)
// ════════════════════════════════════════════════════════════════
void printHelp() {
  rlog("── Telnet Commands ──────────────────");
  rlog(" kp <v>   set Kp     e.g. kp 1.5");
  rlog(" ki <v>   set Ki     e.g. ki 0.01");
  rlog(" kd <v>   set Kd     e.g. kd 0.3");
  rlog(" pid      show PID values");
  rlog(" enc      show encoder counts");
  rlog(" stop     stop motors");
  rlog(" help     this menu");
  rlog("─────────────────────────────────────");
}

void handleTelnet() {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      telnetClient = telnetServer.available();
      rlog("=== MARS Monitor Connected ===");
      rlog("Bot ID:%d  IP:%s", BOT_ID, WiFi.localIP().toString().c_str());
      rlog("Kp=%.3f Ki=%.3f Kd=%.3f", Kp, Ki, Kd);
      printHelp();
    }
  }

  if (!telnetClient || !telnetClient.connected()) return;
  if (!telnetClient.available()) return;

  String line = telnetClient.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  if      (line.startsWith("kp ")) { Kp = line.substring(3).toFloat(); rlog("[PID] Kp=%.4f Ki=%.4f Kd=%.4f", Kp, Ki, Kd); }
  else if (line.startsWith("ki ")) { Ki = line.substring(3).toFloat(); rlog("[PID] Kp=%.4f Ki=%.4f Kd=%.4f", Kp, Ki, Kd); }
  else if (line.startsWith("kd ")) { Kd = line.substring(3).toFloat(); rlog("[PID] Kp=%.4f Ki=%.4f Kd=%.4f", Kp, Ki, Kd); }
  else if (line == "pid")          { rlog("[PID] Kp=%.4f Ki=%.4f Kd=%.4f", Kp, Ki, Kd); }
  else if (line == "enc")          { rlog("[ENC] L:%ld R:%ld", encL, encR); }
  else if (line == "stop")         { stopMotors(); rlog("[STOP]"); }
  else if (line == "help")         { printHelp(); }
  else                             { rlog("[?] unknown: %s", line.c_str()); }
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
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  udp.begin(UDP_PORT);
  telnetServer.begin();

  Serial.printf("UDP  port: %d\n", UDP_PORT);
  Serial.println("Telnet:  nc <IP> 23");
  Serial.println("Ready.");
}

void loop() {
  handleUDP();
  handleTelnet();
}