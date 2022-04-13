/*
 * Project Master_Code_SmartBusStop
 * Description:
 * Author:
 * Date:
 */
#include "Grove-Ultrasonic-Ranger.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT_SPARK.h"
#include "credentials.h"

const int FLAMEPIN = A2;
const int FLAMEPINDIGITAL = D6;
const int MQ4ANALOGPIN = A3;
const int MQ4DIGITALPIN = D5;
const int DIODEPIN = A4;
const int LEDPIN = A5;

int flameSensor;
int currentTime1, currentTime2, currentTime3, currentTime4, currentTime5;
int lastTime1, lastTime2, lastTime3, lastTime4,lastTime5;
int mq4Digital, mq4Analog;
int diodeState;
int nightLed;

// delete after published is established
int value1 = 102; 
int ON_OFF;
struct geo{
    float lat;
    float lon;
    int alt;  
    };

geo myLoc;
geo locations[13];




Ultrasonic ultrasonic(A2);

TCPClient TheClient;

Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish mqttObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/abq_gps");
Adafruit_MQTT_Subscribe mqttON_OFFobject = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ON_OFF");

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
    Serial.begin(9600);
    pinMode(FLAMEPIN, INPUT);
    pinMode(MQ4ANALOGPIN, INPUT);
    pinMode(MQ4DIGITALPIN, INPUT);
    pinMode(DIODEPIN, INPUT);
    pinMode(LEDPIN, OUTPUT);

     WiFi.connect();
    while (WiFi.connecting()) {
        Serial.printf(".");
    }

     // Setup MQTT subscription for onoff feed.
    mqtt.subscribe(&mqttON_OFFobject);
}
void loop() {

   MQTT_connect();
  currentTime1 = millis();
  if((currentTime1-lastTime1) > 6000){
      if(mqtt.Update()) {
        mqttObj1.publish(value1);
        Serial.printf("Publishing %0.2f \n",value1);
        }
      lastTime1 = millis();
    }

    // this is our 'wait for incoming subscription packets' busy subloop
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) {
        if (subscription == &mqttON_OFFobject) {
            ON_OFF = atof((char *)mqttON_OFFobject.lastread);
            Serial.printf("Received %i from Adafruit.io feed FeedNameB \n", ON_OFF);
        }
    }

    long RangeInInches;
    long RangeInCentimeters;

    currentTime2 = millis();
    if ((currentTime2 - lastTime2) > 1000) {
        Serial.println("The distance to obstacles in front is: ");
        RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
        Serial.print(RangeInCentimeters);                       // 0~400cm
        Serial.println(" cm");
        lastTime2 = millis();
    }

    currentTime3 = millis();
    if ((currentTime3 - lastTime3) > 1000) {

        flameSensor = analogRead(FLAMEPIN);
        if (flameSensor < 1500) {
            Serial.printf("Flame Detected!");
        }
        Serial.printf("flame sensor:%i DIGITAL:%i\n", flameSensor, FLAMEPINDIGITAL);
        lastTime3 = millis();
    }
    currentTime4= millis();
    if ((currentTime4 - lastTime4) > 2000) {
        mq4Analog = analogRead(MQ4ANALOGPIN);
        mq4Digital = analogRead(MQ4DIGITALPIN);

        // indoor reading: 1800-2100 by an exhaust is around 3200-3400
        Serial.printf("mq4 Analog Read:%imq Digital Read:%i\n", mq4Analog, mq4Digital);
        lastTime4 = millis();
    }

    diodeState = analogRead(DIODEPIN);
    nightLed = map(diodeState,120,195,255,-0);

    currentTime5 = millis();
    if((currentTime5-lastTime5)>2000){
    Serial.printf("diode State:%i conversion:%i\n", diodeState,nightLed);
    lastTime5 =millis();
    }
     analogWrite(LEDPIN,nightLed);
}


void MQTT_connect() {
    int8_t ret;

    // Stop if already connected.
    if (mqtt.connected()) {
        return;
    }

    Serial.print("Connecting to MQTT... ");

    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.printf("%s\n", (char *)mqtt.connectErrorString(ret));
        Serial.printf("Retrying MQTT connection in 5 seconds..\n");
        mqtt.disconnect();
        delay(5000); // wait 5 seconds
    }
    Serial.printf("MQTT Connected!\n");
}
