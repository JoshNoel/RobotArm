#include <PS2X_lib.h>
#include <SPI.h>
#include <SSC32.h>

PS2X ps2x; // create PS2 Controller Class
SSC32 myssc = SSC32(); // SSC32 Servo Controller

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

//PS2 Controller Variables
int error = 0;
byte type = 0;
byte vibrate = 0;

void setup()
{
  myssc.begin(9600);
  myssc.servoMove(16, 1500);
  //Configuring PS2 controller and checking for errors
  //Serial.begin(57600);
  
  error = ps2x.config_gamepad(8,10,12,7, false, false);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

  /*if(error == 0){
    Serial.println("Found Controller, configured successful");
  }

  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
    */

  //Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType(); 
  switch(type) {
  case 0:
    //Serial.println("Unknown Controller type");
    break;
  case 1:
    //Serial.println("DualShock Controller Found");
    break;
  case 2:
    //Serial.println("GuitarHero Controller Found");
    break;
  }
}

void loop()
{
  ps2x.read_gamepad();
  ps2x.read_gamepad(false, vibrate);
  float LX = (ps2x.Analog(PSS_LX));
  if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
  {
  int bas_servopulse = map(LX, 0, 255, 1400, 1600);
  Serial.println(bas_servopulse);
  myssc.servoMove(16, bas_servopulse);
  }
}
