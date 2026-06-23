#include <Arduino.h>

// Motor A
#define AIN1 18
#define AIN2 23
#define PWMA 19

// Motor B
#define BIN1 16
#define BIN2 17
#define PWMB 4

void setup()
{
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
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
    // Forward
    motorA_forward(200);
    motorB_forward(200);
    delay(2000);

    // Stop
    motorA_stop();
    motorB_stop();
    delay(1000);

    // Backward
    motorA_backward(200);
    motorB_backward(200);
    delay(2000);

    // Stop
    motorA_stop();
    motorB_stop();
    delay(1000);
}