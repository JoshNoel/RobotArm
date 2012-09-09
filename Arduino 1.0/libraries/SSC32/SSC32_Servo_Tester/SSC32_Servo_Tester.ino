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
  myssc.servoMove(0,1500);

  delay(1000);

}
