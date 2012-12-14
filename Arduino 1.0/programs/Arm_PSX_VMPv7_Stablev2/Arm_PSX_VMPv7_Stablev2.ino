// AL5D robotic arm manual control using PSX (PS2) controller. Servo controller by Lynxmotion SSC32 servo controller.

/////////
// Setup 
/////////

// Include the SSC32, SPI, and PS2X libraries
#include <SSC32.h>
#include <SPI.h>
#include <PS2X_lib.h>

// Enable Devices/Classes
SSC32 myssc = SSC32();  // SSC32 Servo Controller
PS2X ps2x;              // create PS2 Controller Class

// Set PSX Controller Variables
int error = 0;
byte type = 0;
byte vibrate = 0;

// Float to long conversion
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

// Servo names and numbers on SSC32
#define BAS_SERVO 16          // Base servo HS-485HB
#define SHL_SERVO 17          // Shoulder Servo HS-5745-MG
#define ELB_SERVO 18          // Elbow Servo HS-5745-MG
#define WRIST_SERVO 19        // Wrist servo HS-645MG
#define WRIST_ROTATE_SERVO 20 // Wrist rotate servo HS-485HB
#define GRIP_SERVO 21         // Gripper servo HS-422

// Arm position variables
float x;                   // X coordinate of the gripper tip (Side to side position)
float y;                   // Y coordinate of the gripper tip (Distance out from base)
float z;                   // Z coordinate of the gripper tip (Height)
float gripperAngle;        // Gripper angle
int gripperServoPulse;     // Gripper servo pulse 
int wristRotateServoPulse; // Wrist rotate servo pulse

// Arm dimensions (mm)
#define BASE_HGT 98.31 // Base hight: 2.65"
#define HUMERUS 265.50 // Shoulder-to-elbow "bone": 5.75"    (350.00 for long, 265.50 for medium, 146.50 for none)
#define ULNA 326.00    // Elbow-to-wrist "bone":             (Long: 326.39, Medium: 170.00)
#define GRIPPER 0.00   // Gripper + medium duty wrist: 3.94" (With Gripper: 11.00) - Currently set to 0 to simplify movement

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

void setup()
{
  // Begin communication with SSC32 and computer
  myssc.begin(9600);  // Communicate with servo controller - Note: Servo Controller Library modified to use serial1 (one)
  Serial.begin(9600); // Communicate to computer via USB Serial - Used for position feedback to computer to find positions

  // Configuring PS2 controller and checking for errors
  error = ps2x.config_gamepad( 8, 10, 12, 7, false, false);  // Setup PSX pins and settings and then check for error, following: GamePad(clock, command, attention, data, Pressures?, Rumble?)
  type = ps2x.readType();

  // Start Position of gripper using X (rotation speed), Y cordinates (horizontal out from base), and Z (height)
  // Use (y=-76, z=652) for long, (y=0, z=680) for medium, (y =-200, z=350) for no extension
  setArmTo( x = 0, y = -76.00, z = 652.00, gripperAngle = 0, wristRotateServoPulse = 1500, gripperServoPulse = 890 );

  // Controller Ready Loop - operator prepares controller by turning on analog mode and hitting select
  do
  {
    // Check PSX state
    ps2x.read_gamepad();
    ps2x.read_gamepad(false, vibrate);
  } while( !ps2x.Button(PSB_SELECT));  // Continue checking the status until select is pressed
  // Start program
}


/////////////
// Main Loop
/////////////

// Updates gripper cordinates according to new variable values and reports them to computer
void loop()
{
  // Quit program if no controller is found
  if(error == 1) return;

  // Send gripper values to SSC32
  // Update arm to X, Y, Z, Wrist, and Grip positions for Gripper Angle
  setArmTo( x, y, z, gripperAngle, wristRotateServoPulse, gripperServoPulse );

  // Send Computer Coordinates, formatted to copy and paste back into code
  Serial.print("\n\nsetArmTo( x = ");
  Serial.print( x );
  Serial.print(", y = ");
  Serial.print( y );
  Serial.print(", z = ");
  Serial.print( z );
  Serial.print(", gripperAngle = ");
  Serial.print( gripperAngle );
  Serial.print(", wristRotateServoPulse = ");
  Serial.print( wristRotateServoPulse );
  Serial.print(", gripperServoPulse = ");
  Serial.print( gripperServoPulse );
  Serial.print(" );");

  // Run PSX_poll to get controller readings
  PSX_poll();
}


/////////////
// Controlls
/////////////

// Poll PSX controller using Get Report and fill arm data structure
byte PSX_poll( void )
{
  // Check PSX state
  ps2x.read_gamepad();
  ps2x.read_gamepad(false, vibrate);

  // Direct Controll
  ////////////// // // 

  // Base Rotate
  float LX = (ps2x.Analog(PSS_LX));  // Create variable LX for left stick analog values
  int bas_servopulse = map(LX, 0, 255, 1430, 1570); // Map the left analog stick values to the range of values the servo accepts and then save it as the base servo pulse value
  
  // 
  if(ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_R1) )
  {
    if(ps2x.Button(PSB_PAD_RIGHT)) myssc.servoMove(BAS_SERVO, 1515);
    if(ps2x.Button(PSB_PAD_LEFT)) myssc.servoMove(BAS_SERVO, 1485);
  }
  // 
  else if( bas_servopulse>1520 || bas_servopulse<1480) // remember, bas_servopulse corresponds to the left analog stick
  {
    // Left Stick X
    if( z >= 200.00 ) // If gripper is up high, move normal speed 
    {
      myssc.servoMove(BAS_SERVO, bas_servopulse);
    }
    if(z < 200.00) // If gripper is down low, remap the base servopulse value to a smaller range and send the value to the base servo so the base rotate slower
    {
      int slow_bas_servopulse = map(bas_servopulse, 1430, 1570, 1470, 1530);
      myssc.servoMove(BAS_SERVO, slow_bas_servopulse);
    }
  }
  // 
  else myssc.servoMove(BAS_SERVO, 1500); // If none of the values are true, set the rotation to neutral so it does not move

  // Not sure of difference between gripper angle and wrist angle
  float RX = (ps2x.Analog(PSS_RX));
  int RXm = map(RX, 0, 255, 5, -5);
  if((RXm >2) || (RXm<-2)) gripperAngle -= ( RXm );

  // 
  float RY = (ps2x.Analog(PSS_RY));  // RY and RYm are also used for Wrist Angle
  int RYm = map(RY, 0, 255, 10, -10);
  if(ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) || bas_servopulse>1510 || bas_servopulse<1490){
    // Fine Controll on D-Pad
    if(ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN))
    {
      if(ps2x.Button(PSB_PAD_UP)) y += ( 6 );
      if(ps2x.Button(PSB_PAD_DOWN)) y -= ( 6 );
    }
  }
  else
  {
    if((RYm >4) || (RYm<-4))
      if(ps2x.Button(PSB_L1))
        y += ( RYm );
      else
        z += ( RYm * 2 );
  }

  // 
  float LY = (ps2x.Analog(PSS_LY));
  int LYm = map(LY, 0, 255, 10, -10);
  if((LYm >3) || (LYm<-3))
    y += ( LYm );

  // Wrist Rotate Re-Center (when PSX button R3 is pressed)
  if(ps2x.Button(PSB_R3))
    wristRotateServoPulse = 1500;

  // Gripper Open/Close (R2/L2)
  // Open gripper with R2
  if(ps2x.Button(PSB_R2)) 
    gripperServoPulse += ( 300 );
  // Close Gripper with L2
  if(ps2x.Button(PSB_L2)) 
    gripperServoPulse -= ( 300 );
  // Fully Open
  if(ps2x.Button(PSB_L3))
    gripperServoPulse = ( 890 );
  if(gripperServoPulse < 500 ) gripperServoPulse = 500;
  if(gripperServoPulse > 2600 ) gripperServoPulse = 2600;

  // Preset Positions
  ///////////////////

  // Enabled by pressing R1 and the corresponding second button - these are set emperically through testing desired positions and copying from the Arduino serial log on the computer
    ps2x.read_gamepad();
  ps2x.read_gamepad(false, vibrate);
  if(ps2x.Button(PSB_R1))
  {
    if(ps2x.Button(PSB_RED)){ // Circle (Batteries)
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
      setArmTo( x = 0, y = 170.00, z = 190.00, gripperAngle = -41.00, wristRotateServoPulse, gripperServoPulse);
    }

    if(ps2x.Button(PSB_BLUE)){ // X
      setArmTo( x = 0, y = 241.00, z = 136.00, gripperAngle = -52.00, wristRotateServoPulse, gripperServoPulse);
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PINK)){ // Square
      if(z < 308.00 && y > 300){
        delay(100);
        setArmTo( x = 0, y, z = 348.00, gripperAngle, wristRotateServoPulse, gripperServoPulse);
        delay(100);
        setArmTo( x = 0, y = 241.00, z = 301.00, gripperAngle, wristRotateServoPulse, gripperServoPulse);
        delay(100);
      }
      setArmTo( x = 0, y = 241.00, z = 136.00, gripperAngle = -52.00, wristRotateServoPulse, gripperServoPulse);
      delay(300);
      setArmTo( x = 0, y = 206.00, z = 108.00, gripperAngle = -59.00, wristRotateServoPulse, gripperServoPulse);
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_GREEN)){ // Triangle
      if(z < 308.00 && y > 300){
        delay(100);
        setArmTo( x = 0, y, z = 348.00, gripperAngle, wristRotateServoPulse, gripperServoPulse);
        delay(100);
        setArmTo( x = 0, y = 241.00, z = 301.00, gripperAngle, wristRotateServoPulse, gripperServoPulse);
        delay(100);
      }
      setArmTo( x = 0, y = 200.00, z = 136.00, gripperAngle = -53.00, wristRotateServoPulse, gripperServoPulse);
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PAD_DOWN)){
      setArmTo( x = 0, y, z = 234.00, gripperAngle = -42.00, wristRotateServoPulse, gripperServoPulse);
      delay(100);
      myssc.servoMove( WRIST_SERVO, 1679.95 );
      delay(100);
      y = 354.00;
      z = 411.00;
      gripperAngle = 1.50;
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PAD_LEFT)){
      setArmTo( x = 0, y = 284.00, z = 348.00, gripperAngle, wristRotateServoPulse, gripperServoPulse);
      delay(50);
      gripperAngle = 47.00;
      delay(100);
      setArmTo( x = 0, y = 401.00, z = 412.00, gripperAngle = 41.00, wristRotateServoPulse, gripperServoPulse );
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PAD_UP)){
      setArmTo( x = 0, y, z = 234.00, gripperAngle = -42.00, wristRotateServoPulse, gripperServoPulse );
      delay(100);
      myssc.servoMove( WRIST_SERVO, 1679.95 );
      delay(100);
      setArmTo( x = 0, y = 262.00, z = 239.00, gripperAngle = 35.50, wristRotateServoPulse, gripperServoPulse );
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_START)){
      setArmTo( x = 0, y = 200, z = 450, gripperAngle, wristRotateServoPulse, gripperServoPulse );
      delay(200);
      myssc.servoMove( WRIST_SERVO, 2457.03 );
      delay(300);
      y = 385.00;
      z = 540.00;
      gripperAngle = 92.00;
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_L3)){
      y = 384;
      z = 232;
      gripperAngle = -29;
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1200 );
    }

    if(ps2x.Button( PSB_L1 )){
      setArmTo( x = 0, y = 205, z = 423, gripperAngle = 62.00, wristRotateServoPulse, gripperServoPulse);
      delay(300);
      setArmTo( x = 0, y = 270.00, z = 307.00, gripperAngle, wristRotateServoPulse, gripperServoPulse );
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }
  }
  else
  {
    if(ps2x.Button(PSB_GREEN)) // X
      wristRotateServoPulse -= ( 100 );
    if(ps2x.Button(PSB_BLUE)) // Circle
      wristRotateServoPulse += ( 100 );
    // Check Limits
    if(wristRotateServoPulse > 2500 ) wristRotateServoPulse = 2500;
    if(wristRotateServoPulse < 500 ) wristRotateServoPulse = 500;
  }

  // Reset Position
  if(ps2x.Button(PSB_SELECT))
  {
  setArmTo( x = 0, y = 200, z = 350, gripperAngle = 0, wristRotateServoPulse = 1500 , gripperServoPulse );
  delay(500);
  }
}


/////////////
// Back Code
/////////////

/* Arm positioning routine utilizing inverse kinematics from circuits@home */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
// void setArmTo( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
void setArmTo( float x, float y, float z, float grip_angle_d, int wristRotateServoPulse, int gripperServoPulse )
{
  float grip_angle_r = radians( grip_angle_d );    // grip angle in radians for use in calculations
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
  // float a1 = atan2( wrist_y, wrist_z );
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
  myssc.servoMove( WRIST_SERVO, ftl( wri_servopulse ));
  myssc.servoMove( SHL_SERVO, ftl( shl_servopulse ));
  myssc.servoMove( ELB_SERVO, ftl( elb_servopulse ));

  /* Set other servos */
  myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse );
  myssc.servoMove( GRIP_SERVO, gripperServoPulse );
}



