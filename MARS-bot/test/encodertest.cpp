
#include<Arduino.h>

#define ENCODER1_A 25
#define ENCODER1_B 26
#define ENCODER2_A 32
#define ENCODER2_B 33

volatile long enc1 = 0;
volatile long enc2 = 0;


void IRAM_ATTR enc1ISR() {
  if (digitalRead(ENCODER1_B)) enc1--;
  else enc1++;
}

void IRAM_ATTR enc2ISR() {
  if (digitalRead(ENCODER2_B)) enc2++;
  else enc2--;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), enc2ISR, RISING);
}

void loop() {
  Serial.printf("ENC1: %ld  |  ENC2: %ld\n", enc1, enc2);
  delay(200);
}