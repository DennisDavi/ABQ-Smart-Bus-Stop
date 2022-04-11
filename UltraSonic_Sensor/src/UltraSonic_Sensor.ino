/*
 * Project UltraSonic_Sensor
 * Description: grove ultrasonic sensor testing
 * Author: Thomas Abeyta
 * Date: 4/11/22
 */

#include "Grove-Ultrasonic-Ranger.h"


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
