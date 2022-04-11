/*
 * Project MQ4_Methane_Senosor
 * Description:
 * Author:
 * Date:
 */

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

    //indoor reading: 1800-2100 by an exhaust is around 3200-3400
    Serial.printf("mq4 Analog Read:%imq Digita Read:%i\n",mq4Analog, mq4Digital); 
   
    delay(2000);
  }


  