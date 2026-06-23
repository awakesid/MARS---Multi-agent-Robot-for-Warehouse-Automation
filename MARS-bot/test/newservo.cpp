
#include<Arduino.h>
#define SERVO_CH   2
#define SERVO_FREQ 50    // 50Hz standard servo
#define SERVO_PIN 14
void servoSetup() {
  ledcSetup(SERVO_CH, SERVO_FREQ, 16);  // 16-bit for servo precision
  ledcAttachPin(SERVO_PIN, SERVO_CH);
}

void servoOpen() {   // 90 degrees
  ledcWrite(SERVO_CH, 7864);  // ~1.5ms pulse / 20ms * 65535
}

void servoClose() {  // 0 degrees  
  ledcWrite(SERVO_CH, 1738);  // ~0.5ms pulse / 20ms * 65535
}


void setup(){
    Serial.begin(115200);
    servoSetup();
    servoOpen(); 
}

void loop(){

     
     
    servoOpen(); 
    delay(3000);
    servoClose();
    delay(500);

}