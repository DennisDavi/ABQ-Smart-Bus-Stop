/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/Abeyta/Documents/IoT/ABQ-Smart-Bus-Stop/MQ4_Methane_Senosor/src/MQ4_Methane_Senosor.ino"
/*
 * Project MQ4_Methane_Senosor
 * Description:
 * Author:
 * Date:
 */

void setup();
void loop();
#line 8 "/Users/Abeyta/Documents/IoT/ABQ-Smart-Bus-Stop/MQ4_Methane_Senosor/src/MQ4_Methane_Senosor.ino"
const int MQ4ANALOGPIN = A2;
const int MQ4DIGITALPIN = D6;

int mq4Digital, mq4Analog;


SYSTEM_MODE(SEMI_AUTOMATIC);
void setup(){
 
  pinMode(MQ4ANALOGPIN, INPUT);
  pinMode (MQ4DIGITALPIN, INPUT);

}

  void loop() {


    mq4Analog = analogRead(MQ4ANALOGPIN);
    mq4Digital = analogRead(MQ4DIGITALPIN);

    Serial.printf("mq4 Analog Read:%imq Digita Read:%i\n",mq4Analog, mq4Digital);
    delay(2000);
  }


  