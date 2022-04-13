/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Abeyta/Documents/IoT/ABQ-Smart-Bus-Stop/Fire_Sensor/src/Fire_Sensor.ino"
/*
 * Project Fire_Sensor
 * Description:
 * Author: Thomas abeyta
 * Date:
 */

void setup();
void loop();
#line 8 "/Users/Abeyta/Documents/IoT/ABQ-Smart-Bus-Stop/Fire_Sensor/src/Fire_Sensor.ino"
SYSTEM_MODE(SEMI_AUTOMATIC);
const int FLAMEPIN = A2;
const int FLAMEPINDIGITAL = D6;
int flameSensor;
int currentTime, lastTime;

void setup() {
    pinMode(FLAMEPIN, INPUT);
}

void loop() {
    currentTime = millis();
    if ((currentTime - lastTime) > 1000) {
        
        flameSensor = analogRead(FLAMEPIN);
        if (flameSensor<1500){
          Serial.printf("Flame Detected!");
        }
        Serial.printf("flame sensor:%i DIGITAL:%i\n", flameSensor, FLAMEPINDIGITAL);
        lastTime = millis();
    }}
