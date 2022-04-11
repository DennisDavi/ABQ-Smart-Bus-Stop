/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Abeyta/Documents/IoT/ABQ-Smart-Bus-Stop/UltraSonic_Sensor/src/UltraSonic_Sensor.ino"
/*
 * Project UltraSonic_Sensor
 * Description: grove ultrasonic sensor testing
 * Author: Thomas Abeyta
 * Date: 4/11/22
 */

#include "Grove-Ultrasonic-Ranger.h"


void setup();
void loop();
#line 11 "/Users/Abeyta/Documents/IoT/ABQ-Smart-Bus-Stop/UltraSonic_Sensor/src/UltraSonic_Sensor.ino"
SYSTEM_MODE(SEMI_AUTOMATIC);
Ultrasonic ultrasonic(A2);
void setup()
{
	Serial.begin(9600);
}
void loop()
{
	long RangeInInches;
	long RangeInCentimeters;
	
	Serial.println("The distance to obstacles in front is: ");
	RangeInInches = ultrasonic.MeasureInInches();
	Serial.print(RangeInInches);//0~157 inches
	Serial.println(" inch");
	delay(1000);
	
	RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
	Serial.print(RangeInCentimeters);//0~400cm
	Serial.println(" cm");
	delay(1000);
}
