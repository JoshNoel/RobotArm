#include <SSC32.h>

/*
  Tests the SSC32 library. By Martin Peris (http://www.martinperis.com)
  This example code is in the public domain.
 */

SSC32 myssc = SSC32();

void setup() {


  //Start comunications with the SSC32 device  
  myssc.begin(9600);



}

void loop() {

  //Move motor 0 to position 750
  //The first command should not define any speed or time, is used as initialization by SSC32
  myssc.servoMove(0,750);

  delay(1000);

  //Move motor 1 to position 750
  //The first command should not define any speed or time, is used as initialization by SSC32
  myssc.servoMove(1,750);

  delay(1000);

  //Move motor 0 to position 1500. It will take 5 seconds to finish the movement.
  myssc.servoMoveTime(0,1500,5000);

  delay(5500);

  //Move motor 1 to position 900. It will take 5 seconds to finish the movement
  myssc.servoMoveTime(1,900,5000);

  //Move both servos to position 750 at the same time. 
  //The movement will take 5 seconds to complete
  //Notice that currently motor 0 is at position 1500 and motor 1 is at position 900,
  //but they will reach position 750 at the same time
  myssc.beginGroupCommand(SSC32_CMDGRP_TYPE_SERVO_MOVEMENT);
  myssc.servoMoveTime(1,750,5000);
  myssc.servoMoveTime(5,750,5000);
  myssc.endGroupCommand();

  delay(5500);

}
