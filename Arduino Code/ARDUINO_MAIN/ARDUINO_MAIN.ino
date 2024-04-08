#include <Adafruit_BNO055.h>

#include "Header.h"

void setup() {
 // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial2.begin(9600);
  Serial.begin(115200);
  Wire.begin();
  attachInterrupt(digitalPinToInterrupt(18), Interruptfunc, RISING);
  //write("init");
  bnoSetup();
}

void loop() {
  // put your main code here, to run repeatedly:

  read();
  Serial.println(msg);
  Serial.println("hi");
  if (msg[0] == '+' || msg[0] == '-') {
    int sign1 = 1 - (2 * (msg[0] == '-')), sign2 = 1 - (2 * (msg[4] == '-'));
    setMultipleMotors(getnum(&msg[1]) * sign1, getnum(&msg[5]) * sign2);
  }
  if (msg[0] == 'F' && msg[1] == 'S') {    //going forward (or backward)
    int sign = 1 - (2 * (msg[2] == '-'));  //sets sign to 1 or -1
    setMultipleMotors(getnum(&msg[3]) * sign, getnum(&msg[3]) * sign);
    Serial.println(getnum(&msg[3]));
  }
  if (msg[0] == 'F' && msg[1] == 'C') {  //going forward with cm
    forwardCm(getnum(&msg[2]), 120);
  }
  if (msg[0] == 'B' && msg[1] == 'C') {  //going backward with cm
    backwardCm(getnum(&msg[2]), 120);
  }
  if(msg[0] == 'R' && msg[1] == 'G'){
    forwardCm(9, 60);
    enc_turn_abs(90, 90);
    backwardCm(2, 60);
    setMultipleMotors(0, 0);
    delay(3000);
  }
  if(msg[0] == 'L' && msg[1] == 'G'){
    forwardCm(9, 60);
    enc_turn_abs(-90, 90);
    setMultipleMotors(0, 0);
    backwardCm(2, 60);
    delay(3000);
  }
  if(msg[0] == 'X' && msg[1] == 'Y' && msg[2] == 'Z')
  {
    enc_turn(-90, 75);
    setMultipleMotors(0, 0);
    delay(3000);
  }
  //enc_turn_abs(90, 75);
  //delay(2000);
  //while(true)
  //  setMultipleMotors(50, 50);
  write("!");
  //while(true)
    //setMultipleMotors(70, 70);*/
  /*delay(3000);
  enc_turn_abs(-90, 90);
  //*/
}


