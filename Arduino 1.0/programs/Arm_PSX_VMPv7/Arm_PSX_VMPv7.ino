//AL5D robotic arm manual control using PSX (PS2) controller. Servo controller by Lynxmotion SSC32 servo controller.

/////////
// Setup 
/////////

//Enable SSC32 and PSX
   //Include the SSC32, SPI, and PS2X libraries
   #include <SSC32.h>
   #include <SPI.h>
   #include <PS2X_lib.h>

   //Enable Devices/Classes
   SSC32 myssc = SSC32(); // SSC32 Servo Controller
   PS2X ps2x;             // create PS2 Controller Class

//Set Default Values and constants
   //PSX Controller Variables
   int error = 0;
   byte type = 0;
   byte vibrate = 0;

   //Float to long conversion
   #define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

   //Pre-calculations
   float hum_sq = HUMERUS*HUMERUS;
   float uln_sq = ULNA*ULNA;

   //Servo names and numbers on SSC32
   #define BAS_SERVO 16    //Base servo HS-485HB
   #define SHL_SERVO 17    //Shoulder Servo HS-5745-MG
   #define ELB_SERVO 18    //Elbow Servo HS-5745-MG
   #define WRI_SERVO 19    //Wrist servo HS-645MG
   #define WRO_SERVO 20    //Wrist rotate servo HS-485HB
   #define GRI_SERVO 21    //Gripper servo HS-422

   //Arm data structure (not sure what this is for)
   struct {
     float x_coord;           // X coordinate of the gripper tip (Side to side position)
     float y_coord;           // Y coordinate of the gripper tip (Distance out from base)
     float z_coord;           //Z coordinate of the gripper tip (Height)
     float gripper_angle;     //gripper angle
     int16_t gripper_servo;   //gripper servo pulse duration 
     int16_t wrist_rotate;    //wrist rotate servo pulse duration
   } 
   armdata;

   //Arm dimensions (mm)
   #define BASE_HGT 98.31     //base hight 2.65"
   #define HUMERUS 265.50     //shoulder-to-elbow "bone" 5.75"        (350.00 for long, 265.50 for medium, 146.50 for none)
   #define ULNA 326.00        //elbow-to-wrist "bone"                 (Long: 326.39, Medium: 170.00)
   #define GRIPPER 0.00       //gripper + medium duty wrist 3.94"     (With Gripper: 11.00)


void setup()
{
//Begin communication with SSC32 and Computer
   myssc.begin(9600);  //Communicate with servo controller - Note: Servo Controller Library modified to use serial1
   Serial.begin(9600); //Communicate to computer via USB Serial - Used for position feedback to computer to find positions

//Configuring PS2 controller and checking for errors
   error = ps2x.config_gamepad(8,10,12,7, false, false);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
   //Error print removed because it iterferes with serial connection to SSC 32
   type = ps2x.readType();

//Start Position of gripper using X (rotation speed), Y cordinates (horizontal out from base), and Z (height)
   set_arm( armdata.x_coord = 0, armdata.y_coord = -76.00, armdata.z_coord = 652.00, armdata.gripper_angle = 0 ); //Use (y=-76, z=652) for long, (y=0, z=680) for medium, (y =-200, z=350) for no extension
   myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
   myssc.servoMove( GRI_SERVO, armdata.gripper_servo = 890 );

//Controller Ready Loop - operator prepares controller by turning on analog mode and hitting select. If not, the controller will report false values and the robot will spin around and be uncontrollable
  Setup:
    //Check PSX state
      ps2x.read_gamepad();
      ps2x.read_gamepad(false, vibrate);
    //Press select to unlock movement
      if(ps2x.Button(PSB_SELECT))
      {
      }
      else
        goto Setup;
  }


////////////////
// Main Program
////////////////

//Main loop of program - updates gripper cordinates according to new variable values and reports them to computer
void loop()
{
  if(error == 1) //skip loop if no controller found
    return; 

//Send gripper values to SSC32
  //Update arm to X, Y, and Z positions for Gripper Angle
  set_arm( armdata.x_coord, armdata.y_coord, armdata.z_coord, armdata.gripper_angle );
  //Update to Wrist and Grip positions
  myssc.servoMove( WRO_SERVO, armdata.wrist_rotate );
  myssc.servoMove( GRI_SERVO, armdata.gripper_servo );

//Send Computer Coordinates
  Serial.println();
  Serial.print("Y (Depth):");
  Serial.println( armdata.y_coord );
  Serial.print("Z (Height):");
  Serial.println( armdata.z_coord );
  Serial.print("Wrist Angle:");
  Serial.println( armdata.gripper_angle );

//Run PSX_poll to get controller readings
  PSX_poll();
}

//Poll PSX controller using Get Report and fill arm data structure
byte PSX_poll( void )
{
//Check PSX state
  ps2x.read_gamepad();
  ps2x.read_gamepad(false, vibrate);

//Base Rotate
  float LX = (ps2x.Analog(PSS_LX));  //Create variable LX for left stick analog values
  int bas_servopulse = map(LX, 0, 255, 1430, 1570); //Map the left analog stick values to the range of values the servo accepts and then save it as the base servo pulse value
  //If any of these values are true, check the D-Pad, if it is being pressed left or right, slowly rotate the base in that direction. If not, check the base servopulse value, if it is above or below the set range, send the value to the servo, rotating the base left or right 
  if(ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT) || bas_servopulse>1520 || bas_servopulse<1480){ //remember, bas_servopulse corresponds to the left analog stick
    //Fine Controll on D-Pad
    if(ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT)){
      if(ps2x.Button(PSB_PAD_RIGHT)) myssc.servoMove(BAS_SERVO, 1515);
      if(ps2x.Button(PSB_PAD_LEFT)) myssc.servoMove(BAS_SERVO, 1485);
    }
    //Left Stick X
    else{
      if(bas_servopulse>1520 || bas_servopulse<1480){
        //If gripper is up high, move normal speed 
        if(armdata.z_coord >= 200.00) myssc.servoMove(BAS_SERVO, bas_servopulse);
        //If gripper is down low, remap the base servopulse value to a smaller range and send the value to the base servo so the base rotates slower
        if(armdata.z_coord < 200.00) 
        {
          int slow_bas_servopulse = map(bas_servopulse, 1430, 1570, 1470, 1530);
          myssc.servoMove(BAS_SERVO, slow_bas_servopulse);
        }
      }
    }
  }
  else myssc.servoMove(BAS_SERVO, 1500); //If none of the values are true, set the rotation to neutral so it does not move

  //Not sure of difference between gripper angle and wrist angle
  float RX = (ps2x.Analog(PSS_RX));
  int RXm = map(RX, 0, 255, 5, -5);
  if((RXm >2) || (RXm<-2))
    armdata.gripper_angle -= ( RXm );

  //
  float RY = (ps2x.Analog(PSS_RY));  //RY and RYm are also used for Wrist Angle
  int RYm = map(RY, 0, 255, 10, -10);
  if(ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) || bas_servopulse>1510 || bas_servopulse<1490){
    //Fine Controll on D-Pad
    if(ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN))
    {
      if(ps2x.Button(PSB_PAD_UP)) armdata.y_coord += ( 6 );
      if(ps2x.Button(PSB_PAD_DOWN)) armdata.y_coord -= ( 6 );
    }
  }
  else{
    if((RYm >4) || (RYm<-4))
      if(ps2x.Button(PSB_L1))
        armdata.y_coord += ( RYm );
      else
        armdata.z_coord += ( RYm * 2 );
  }

  //
  float LY = (ps2x.Analog(PSS_LY));
  int LYm = map(LY, 0, 255, 10, -10);
  if((LYm >3) || (LYm<-3))
    armdata.y_coord += ( LYm );

  //Wrist Rotate Re-Center (when PSX button R3 is pressed)
  if(ps2x.Button(PSB_R3))
    armdata.wrist_rotate = 1500;

  //Gripper Open/Close (R2/L2)
  //Open gripper with R2
  if(ps2x.Button(PSB_R2)) 
    armdata.gripper_servo += ( 300 );
  //Close Gripper with L2
  if(ps2x.Button(PSB_L2)) 
    armdata.gripper_servo -= ( 300 );
  //Fully Open
  if(ps2x.Button(PSB_L3))
    armdata.gripper_servo = ( 890 );
  if(armdata.gripper_servo < 500 ) armdata.gripper_servo = 500;
  if(armdata.gripper_servo > 2600 ) armdata.gripper_servo = 2600;

  //Pre-Programed Positions, enabled by pressing R1 and the corresponding second button - these are set emperically through testing desired positions and copying from the Arduino serial log on the computer
  ps2x.read_gamepad();
  ps2x.read_gamepad(false, vibrate);
  if(ps2x.Button(PSB_R1))
  {
    if(ps2x.Button(PSB_RED)){ //Circle (Batteries)
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
      set_arm( armdata.x_coord = 0, armdata.y_coord = 170.00, armdata.z_coord = 190.00, armdata.gripper_angle = -41.00);
    }

    /*
    if(ps2x.Button(PSB_PINK)){ //Square (Nails)
     set_arm( armdata.x_coord = 0, armdata.y_coord = 200.00, armdata.z_coord = 136.00, armdata.gripper_angle = -53.00);
     myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
     }
     */

    if(ps2x.Button(PSB_BLUE)){ //X
      set_arm( armdata.x_coord = 0, armdata.y_coord = 241.00, armdata.z_coord = 136.00, armdata.gripper_angle = -52.00);
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    if(ps2x.Button(PSB_PINK)){ //Square
      if(armdata.z_coord < 308.00 && armdata.y_coord > 300){
        delay(100);
        set_arm( armdata.x_coord = 0, armdata.y_coord, armdata.z_coord = 348.00, armdata.gripper_angle);
        delay(100);
        set_arm( armdata.x_coord = 0, armdata.y_coord = 241.00, armdata.z_coord = 301.00, armdata.gripper_angle);
        delay(100);
      }
      set_arm( armdata.x_coord = 0, armdata.y_coord = 241.00, armdata.z_coord = 136.00, armdata.gripper_angle = -52.00);
      delay(300);
      set_arm( armdata.x_coord = 0, armdata.y_coord = 206.00, armdata.z_coord = 108.00, armdata.gripper_angle = -59.00);
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    if(ps2x.Button(PSB_GREEN)){ //Triangle
      if(armdata.z_coord < 308.00 && armdata.y_coord > 300){
        delay(100);
        set_arm( armdata.x_coord = 0, armdata.y_coord, armdata.z_coord = 348.00, armdata.gripper_angle);
        delay(100);
        set_arm( armdata.x_coord = 0, armdata.y_coord = 241.00, armdata.z_coord = 301.00, armdata.gripper_angle);
        delay(100);
      }
      set_arm( armdata.x_coord = 0, armdata.y_coord = 200.00, armdata.z_coord = 136.00, armdata.gripper_angle = -53.00);
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    if(ps2x.Button(PSB_PAD_DOWN)){
      set_arm( armdata.x_coord = 0, armdata.y_coord, armdata.z_coord = 234.00, armdata.gripper_angle = -42.00 );
      delay(100);
      myssc.servoMove( WRI_SERVO, 1679.95 );
      delay(100);
      armdata.y_coord = 354.00;
      armdata.z_coord = 411.00;
      armdata.gripper_angle = 1.50;
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    /*
    if(ps2x.Button(PSB_PAD_RIGHT)){
     armdata.y_coord = -200;
     armdata.z_coord = 355+10;
     armdata.gripper_angle = 0;
     myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
     }
     */

    if(ps2x.Button(PSB_PAD_LEFT)){
      set_arm( armdata.x_coord = 0, armdata.y_coord = 284.00, armdata.z_coord = 348.00, armdata.gripper_angle );
      delay(50);
      armdata.gripper_angle = 47.00;
      delay(100);
      set_arm( armdata.x_coord = 0, armdata.y_coord = 401.00, armdata.z_coord = 412.00, armdata.gripper_angle = 41.00 );
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    if(ps2x.Button(PSB_PAD_UP)){
      set_arm( armdata.x_coord = 0, armdata.y_coord, armdata.z_coord = 234.00, armdata.gripper_angle = -42.00 );
      delay(100);
      myssc.servoMove( WRI_SERVO, 1679.95 );
      delay(100);
      set_arm( armdata.x_coord = 0, armdata.y_coord = 262.00, armdata.z_coord = 239.00, armdata.gripper_angle = 35.50 );
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    if(ps2x.Button(PSB_START)){
      set_arm( armdata.x_coord = 0, armdata.y_coord = 200, armdata.z_coord = 450, armdata.gripper_angle );
      delay(200);
      myssc.servoMove( WRI_SERVO, 2457.03 );
      delay(300);
      armdata.y_coord = 385.00;
      armdata.z_coord = 540.00;
      armdata.gripper_angle = 92.00;
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }

    if(ps2x.Button(PSB_L3)){
      armdata.y_coord = 384;
      armdata.z_coord = 232;
      armdata.gripper_angle = -29;
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1200 );
    }

    /*
    if(ps2x.Button(PSB_R3)){
     armdata.y_coord = 175;
     armdata.z_coord = 438;
     armdata.gripper_angle = 36;
     myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1200 );
     }
     */

    if(ps2x.Button( PSB_L1 )){
      set_arm( armdata.x_coord = 0, armdata.y_coord = 205, armdata.z_coord = 423, armdata.gripper_angle = 62.00);
      delay(300);
      set_arm( armdata.x_coord = 0, armdata.y_coord = 270.00, armdata.z_coord = 307.00, armdata.gripper_angle );
      myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
    }
  }
  else
  {
    if(ps2x.Button(PSB_GREEN)) //X
      armdata.wrist_rotate -= ( 100 );
    if(ps2x.Button(PSB_BLUE)) //Circle
      armdata.wrist_rotate += ( 100 );
    //Check Limits
    if(armdata.wrist_rotate > 2500 ) armdata.wrist_rotate = 2500;
    if(armdata.wrist_rotate < 500 ) armdata.wrist_rotate = 500;
  }

  //Reset Position
  if(ps2x.Button(PSB_SELECT))
    arm_park();
}


/* Arm positioning routine utilizing inverse kinematics from circuits@home */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
//void set_arm( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
void set_arm( float x, float y, float z, float grip_angle_d )
{
  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
  /* Grip offsets calculated based on grip angle */
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
  /* Wrist position */
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;
  /* Shoulder to wrist distance ( AKA sw ) */
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );
  /* s_w angle to ground */
  //float a1 = atan2( wrist_y, wrist_z );
  float a1 = atan2( wrist_z, wrist_y );
  /* s_w angle to humerus */
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );
  /* elbow angle */
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );
  /* wrist angle */
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;

  /* Servo pulses and tests on limits*/
  float shl_servopulse = 1450.0 + (( shl_angle_d - 90.0 ) * 6.6 );
  float elb_servopulse = 1450.0 -  (( elb_angle_d - 90.0 ) * 6.6 );
  float wri_servopulse = 1550 + ( wri_angle_d  * 11.1 );

  /* Set servos and communicate their position*/
  myssc.servoMove( WRI_SERVO, ftl( wri_servopulse ));
  Serial.print("wri_servopulse");
  Serial.println();
  Serial.println( wri_servopulse );
  myssc.servoMove( SHL_SERVO, ftl( shl_servopulse ));
  myssc.servoMove( ELB_SERVO, ftl( elb_servopulse ));

}

/* moves the arm to parking position */
void arm_park()
{
  set_arm( armdata.x_coord = 0, armdata.y_coord = 200, armdata.z_coord = 350, armdata.gripper_angle = 0 );
  delay(500);
  myssc.servoMove( WRO_SERVO, armdata.wrist_rotate = 1500 );
  //myssc.servoMove( GRI_SERVO, armdata.gripper_servo = 890 );
}



















































