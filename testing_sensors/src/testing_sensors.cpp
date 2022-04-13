/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/DennisDavis/Documents/IoT/ABQ-Smart-Bus-Stop/testing_sensors/src/testing_sensors.ino"
/*
 * Project testing_sensors
 * Description:
 * Author:
 * Date:
 */

#include "Grove-Ultrasonic-Ranger.h"

void setup();
void loop();
#line 10 "c:/Users/DennisDavis/Documents/IoT/ABQ-Smart-Bus-Stop/testing_sensors/src/testing_sensors.ino"
SYSTEM_MODE(SEMI_AUTOMATIC)

Ultrasonic ultrasonic(A4);

int moSensor = A0;
int motion;
int lastTime;
int lastTime2;
int button=A1;
int oldButton;
int buttonState;
int AQpin=A3;
int AQSensor;
int flameSensor;
int flamePin= A5;



void setup() {

  Serial.begin(9600);
  pinMode(moSensor, INPUT);
  pinMode(button,INPUT);
  pinMode(AQpin,INPUT);
  pinMode(flamePin,INPUT);

}

void loop() {

  
  if(millis()-lastTime2>10000){
  long RangeInCentimeters;
  RangeInCentimeters=ultrasonic.MeasureInCentimeters();
  Serial.printf("RIC:%i\n",RangeInCentimeters);
  if(RangeInCentimeters<50){
    Serial.printf("The trash is full\n");
  }
  lastTime2=millis();
  }

motion = analogRead(moSensor);
buttonState = digitalRead(button);
AQSensor = analogRead(AQpin);

if(millis()-lastTime>1000){
Serial.printf("motion: %i\n", motion);
Serial.printf("button:%i\n",buttonState);
Serial.printf("AQSensor%i\n",AQSensor);
lastTime=millis();
if(buttonState==1){
  Serial.printf("Emergancy button has been pressed, please send a response unit to station 1\n",buttonState);
}

}

flameSensor=digitalRead(flamePin);
if(flameSensor==0){
  Serial.printf("Flame Detected.\n",flameSensor);
}

}

// bool isThereFire(){
// const int threshHold = 500;
// if(flameSensor>threshHold){
//   return true;
// }else{
//   return false;
// }

// }