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

// Servo names and port numbers on SSC32
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
float wristAngle;        // Gripper angle
int gripperOpeningServoPulse;     // Gripper servo pulse 
int wristRotateServoPulse; // Wrist rotate servo pulse

// Arm dimensions (mm)
#define BASE_HGT 98.31  // Base hight: 2.65"
#define HUMERUS 265.50  // Shoulder-to-elbow "bone": 5.75"    (350.0 for long, 265.50 for medium, 146.50 for none)
#define ULNA 326.0     // Elbow-to-wrist "bone":             (Long: 326.39, Medium: 170.0)
#define GRIPPER 0.0    // Gripper + medium duty wrist: 3.94" (With Gripper: 11.0) - Currently set to 0 to simplify movement

// PSX controller analog stick dead zones
#define RIGHT_ANALOG_X_DEADZONE 0.0
#define RIGHT_ANALOG_Y_DEADZONE 0.0
#define LEFT_ANALOG_X_DEADZONE  0.0
#define LEFT_ANALOG_Y_DEADZONE  0.0
#define ANALOG_STICK_STEPS 20  // This is the max value for the mapped (non-raw) analog stick variables
                               // The minimum is the negative of this, so the total number of steps is actually double this value

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

void setup()
{
  // Begin communication with SSC32 and computer
  myssc.begin(9600);  // Communicate with servo controller - Note: Servo Controller Library modified to use serial1 (serial one) instead of serial
  Serial.begin(9600); // Communicate to computer via USB Serial - Used for position feedback to computer to find positions

  // Configuring PS2 controller and checking for errors
  error = ps2x.config_gamepad( 8, 10, 12, 7, false, false);  // Setup PSX pins and settings and then check for error, following GamePad(clock, command, attention, data, Pressures?, Rumble?)
  type = ps2x.readType();

  // Start Position of gripper using X (rotation speed), Y cordinates (horizontal out from base), and Z (height)
  // Use (y=-76, z=652) for long, (y=0, z=680) for medium, (y =-200, z=350) for no extension
  setArmTo( x = 0, y = -76.0, z = 652.0, wristAngle = 0, wristRotateServoPulse = 1500, gripperOpeningServoPulse = 890 );

  // Controller Ready Loop - operator prepares controller by turning on analog mode and hitting select to start
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
  // Quit program if no controller is found or it is disconnected
  if(error == 1)  return;

  // Send gripper values to SSC32 to update arm to X, Y, Z, Gripper Angle, Wrist Rotate, and Gripper Opening
  setArmTo( x, y, z, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse );

  // Send Computer Coordinates, formatted to easily copy and paste back into code
  Serial.print("\n\nsetArmTo( x = ");
  Serial.print( x );
  Serial.print(", y = ");
  Serial.print( y );
  Serial.print(", z = ");
  Serial.print( z );
  Serial.print(", wristAngle = ");
  Serial.print( wristAngle );
  Serial.print(", wristRotateServoPulse = ");
  Serial.print( wristRotateServoPulse );
  Serial.print(", gripperOpeningServoPulse = ");
  Serial.print( gripperOpeningServoPulse );
  Serial.print(" );");

  // Run PSX_poll to get controller values
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

  // Base Rotate
  /////////////// 
  float leftAnalogXRaw = (ps2x.Analog(PSS_LX));  // Create variable leftAnalogXRaw for left stick analog values
  int baseServoPulse = map(leftAnalogXRaw, 0, 255, 1430, 1570); // Map the left analog stick values to the range of values the servo accepts and then save it as the base servo pulse value

  // If left or right is pressed (and R1 is not), then move slowly in that direction
  if(ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_R1) )
  {
    if(ps2x.Button(PSB_PAD_RIGHT))  myssc.servoMove(BAS_SERVO, 1515);
    if(ps2x.Button(PSB_PAD_LEFT))  myssc.servoMove(BAS_SERVO, 1485);
  }
  // Otherwise, if the baseServoPulse is outside of the deadzone, move acording to height
  else if( baseServoPulse>1520 || baseServoPulse<1480) // The deadzone is 1480-1520
  {
    // If gripper is up high, move normal speed 
    if( z >= 200.0 )  myssc.servoMove(BAS_SERVO, baseServoPulse);
    // If gripper is down low, remap the base servopulse value to a smaller range and send the value to the base servo so the base rotate slower
    if(z < 200.0)
    {
      int slowBaseServoPulse = map(baseServoPulse, 1430, 1570, 1470, 1530);
      myssc.servoMove(BAS_SERVO, slowBaseServoPulse);
    }
  }
  // 
  else  myssc.servoMove(BAS_SERVO, 1500); // If none of the values are true, set the rotation to neutral so it does not move


  // Wrist Angle Controlls
  ////////////////////////
  // If the Right Analog stick is moved outside of the deadzone in the x direction, increase or decrease the wrist angle
  float rightAnalogXRaw = ps2x.Analog(PSS_RX);
  int rightAnalogX = map(rightAnalogXRaw, 0, 255, 5, -5);  // Map the raw controller data to a more usable range
  if((rightAnalogX >2) || (rightAnalogX<-2))  { wristAngle -= (rightAnalogX); }


  // IDK if z or y, seems like both
  /////////////////////////////////
  float rightAnalogYRaw = (ps2x.Analog(PSS_RY));
  int rightAnalogY = map(rightAnalogYRaw, 0, 255, 10, -10);  // Map the raw controller data to a more usable range
  // If the Left Analog X is outside of the deadzone
  if(!baseServoPulse>1510 && !baseServoPulse<1490) // NOT SURE WHY, seems out of place
  {
    // Fine Controll on D-Pad
    if(ps2x.Button(PSB_PAD_UP))  y += ( 6 );
    if(ps2x.Button(PSB_PAD_DOWN))  y -= ( 6 );
  }
  else if((rightAnalogY>4) || (rightAnalogY<-4))
  {
    if(ps2x.Button(PSB_L1))  y += ( rightAnalogY ); // Half speed if L1 is held
    else  z += ( rightAnalogY * 2 );
  }


  // Distance from base (y) Controll
  //////////////////////////////////
  float leftAnalogYRaw = (ps2x.Analog(PSS_LY));
  int leftAnalogY = map(leftAnalogYRaw, 0, 255, 10, -10);
  if((leftAnalogY >3) || (leftAnalogY <-3))  y += ( leftAnalogY );

  // Others
  //////////
  // Wrist Rotate Re-Center (when PSX button R3 is pressed)
  if(ps2x.Button(PSB_R3))
    wristRotateServoPulse = 1500;

  // Gripper Open/Close (R2/L2)
  if(ps2x.Button(PSB_R2))  gripperOpeningServoPulse += ( 300 ); // Open gripper with R2
  if(ps2x.Button(PSB_L2))  gripperOpeningServoPulse -= ( 300 ); // Close Gripper with L2
  if(ps2x.Button(PSB_L3))  gripperOpeningServoPulse = ( 890 ); // Open the gripper all the way with left analog stick click
  // Gripper opening stop points
  if(gripperOpeningServoPulse < 500 )  gripperOpeningServoPulse = 500;
  if(gripperOpeningServoPulse > 2600 )  gripperOpeningServoPulse = 2600;


  // Preset Positions
  ///////////////////

  // Enabled by pressing R1 and then the second button at the same time
  ps2x.read_gamepad();
  ps2x.read_gamepad(false, vibrate);
  if(ps2x.Button(PSB_R1))
  {
    if(ps2x.Button(PSB_RED)){ // Circle (Batteries)
      setArmTo( x = 0, y = 170.0, z = 190.0, wristAngle = -41.0, wristRotateServoPulse = 1500, gripperOpeningServoPulse);
      setArmTo( x = 0, y = 170.0, z = 190.0, wristAngle = -41.0, wristRotateServoPulse, gripperOpeningServoPulse);
    }

    if(ps2x.Button(PSB_BLUE)){ // X
      setArmTo( x = 0, y = 241.0, z = 136.0, wristAngle = -52.0, wristRotateServoPulse, gripperOpeningServoPulse);
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PINK)){ // Square
      if(z < 308.0 && y > 300){
        delay(100);
        setArmTo( x = 0, y, z = 348.0, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse);
        delay(100);
        setArmTo( x = 0, y = 241.0, z = 301.0, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse);
        delay(100);
      }
      setArmTo( x = 0, y = 241.0, z = 136.0, wristAngle = -52.0, wristRotateServoPulse, gripperOpeningServoPulse);
      delay(300);
      setArmTo( x = 0, y = 206.0, z = 108.0, wristAngle = -59.0, wristRotateServoPulse, gripperOpeningServoPulse);
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_GREEN)){ // Triangle
      if(z < 308.0 && y > 300){
        delay(100);
        setArmTo( x = 0, y, z = 348.0, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse);
        delay(100);
        setArmTo( x = 0, y = 241.0, z = 301.0, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse);
        delay(100);
      }
      setArmTo( x = 0, y = 200.0, z = 136.0, wristAngle = -53.0, wristRotateServoPulse, gripperOpeningServoPulse);
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PAD_DOWN)){
      setArmTo( x = 0, y, z = 234.0, wristAngle = -42.0, wristRotateServoPulse, gripperOpeningServoPulse);
      delay(100);
      myssc.servoMove( WRIST_SERVO, 1679.95 );
      delay(100);
      y = 354.0;
      z = 411.0;
      wristAngle = 1.50;
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PAD_LEFT)){
      setArmTo( x = 0, y = 284.0, z = 348.0, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse);
      delay(50);
      wristAngle = 47.0;
      delay(100);
      setArmTo( x = 0, y = 401.0, z = 412.0, wristAngle = 41.0, wristRotateServoPulse, gripperOpeningServoPulse );
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_PAD_UP)){
      setArmTo( x = 0, y, z = 234.0, wristAngle = -42.0, wristRotateServoPulse, gripperOpeningServoPulse );
      delay(100);
      myssc.servoMove( WRIST_SERVO, 1679.95 );
      delay(100);
      setArmTo( x = 0, y = 262.0, z = 239.0, wristAngle = 35.50, wristRotateServoPulse, gripperOpeningServoPulse );
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_START)){
      setArmTo( x = 0, y = 200, z = 450, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse );
      delay(200);
      myssc.servoMove( WRIST_SERVO, 2457.03 );
      delay(300);
      y = 385.0;
      z = 540.0;
      wristAngle = 92.0;
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1500 );
    }

    if(ps2x.Button(PSB_L3)){
      y = 384;
      z = 232;
      wristAngle = -29;
      myssc.servoMove( WRIST_ROTATE_SERVO, wristRotateServoPulse = 1200 );
    }

    if(ps2x.Button( PSB_L1 )){
      setArmTo( x = 0, y = 205, z = 423, wristAngle = 62.0, wristRotateServoPulse, gripperOpeningServoPulse);
      delay(300);
      setArmTo( x = 0, y = 270.0, z = 307.0, wristAngle, wristRotateServoPulse, gripperOpeningServoPulse );
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
  setArmTo( x = 0, y = 200, z = 350, wristAngle = 0, wristRotateServoPulse = 1500 , gripperOpeningServoPulse );
  delay(500);
  }
}


/////////////
// Back Code
/////////////

/* Arm positioning routine utilizing inverse kinematics from circuits@home */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
// void setArmTo( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
void setArmTo( float x, float y, float z, float grip_angle_d, int wristRotateServoPulse, int gripperOpeningServoPulse )
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
  myssc.servoMove( GRIP_SERVO, gripperOpeningServoPulse );
}



