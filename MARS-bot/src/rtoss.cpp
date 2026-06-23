#include<Arduino.h>
#include <Servo.h>


# define trigpin1 22
# define echopin1 23

# define trigpin2 44
# define echopin2 43

# define servopin1 33
# define servopin2 22


QueueHandle_t queue;

typedef struct{
   int trigpin;
   int echopin;
   const char * name;
}sensor_cfg;


typedef struct{
    int servopin;
    const char *name;

}servo_cfg;

typedef struct{
    int dist;
    int detected;
}sens_msg;


float getDistance(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, 30000);
    if (duration == 0) return -1;

    return (duration * 0.0343f) / 2.0f;
}



void sensorTask(void*pvParameters){

    sensor_cfg *scfg= (sensor_cfg*)pvParameters;

    pinMode(scfg->echopin,OUTPUT);
    pinMode(scfg->trigpin,OUTPUT);

    while(1){
        float distance = getDistance(scfg->trigpin, scfg->echopin);
        sens_msg msg{
            .dist=distance,
            .detected=(distance > 0 && distance < 20) ? 1 : 0
        };

        xQueueSend(queue,&msg,portMAX_DELAY);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(300));
    }



}

void servoTask(void *pvParameters){
    servo_cfg *ser=(servo_cfg*)pvParameters;
    sens_msg msg;
    Servo (ser->name).attach(ser->name);


    while(1){

        xQueueReceive(queue, &msg, portMAX_DELAY);

        if (msg.detected){
            (ser->name).write(90);
        }
        else {
            (ser->name).write(90);

        }


    }
}


static servo_cfg servo1={.name="servo1",.servopin=servopin1};
static servo_cfg servo2={.name="servo2",.servopin=servopin2};
static sensor_cfg sense1={.echopin=echopin1,.trigpin=trigpin1,.name="sensor1"};
static sensor_cfg sense2={.echopin=echopin2,.trigpin=trigpin2,.name="sensor2"};


}

void setup(){

     xQueue = xQueueCreate(5, sizeof(SensorMsg_t));
     xTaskCreate(sensorTask,"sensortask1",2048,&sense1,2,NULL);
     xTaskCreate(sensorTask,"sensortask2",2048,&sense2,2,NULL);
     xTaskCreate(servoTask,"sercotask1",2048,&servo1,1,NULL);
     xTaskCreate(servoTask,"servotask2",2048,&servo2,1,NULL);


    



}

void loop(){
    vTaskDelay(portMAX_DELAY);
}