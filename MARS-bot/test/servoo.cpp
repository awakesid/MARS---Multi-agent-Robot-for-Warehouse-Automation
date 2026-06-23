#include <Arduino.h>
#include <ESP32Servo.h>

// Motor A
#define AIN1 18
#define AIN2 23
#define PWMA 19
// Motor B
#define BIN1 16
#define BIN2 17
#define PWMB 4
// Servo
#define SERVO_PIN 14

Servo myServo;

void setup()
{
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);

  // Place servo at 90 degrees at start
  myServo.write(90);
  delay(1000);
  Serial.println("Servo at 90° - Ready");
}

// Motor A forward
void motorA_forward(int speed)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed);
}

// Motor A backward
void motorA_backward(int speed)
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
}

// Motor A stop
void motorA_stop()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

// Motor B forward
void motorB_forward(int speed)
{
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speed);
}

// Motor B backward
void motorB_backward(int speed)
{
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed);
}

// Motor B stop
void motorB_stop()
{
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

void loop()
{
  // --- Phase 1: Servo at 90°, move forward ---
  Serial.println("Servo at 90° | Moving forward");
  myServo.write(90);
  delay(500);


  delay(1500);

  Serial.println("Stopped");
  delay(500);


  Serial.println("Servo at 0°");
  myServo.write(0);
  delay(1000);

  Serial.println("Turning 180°");

  delay(500); // tune this until robot turns exactly 180°


}

