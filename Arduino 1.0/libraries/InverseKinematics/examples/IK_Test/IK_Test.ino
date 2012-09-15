//Test for Inverse Kinematics for 3 jointed (shoulder, elbow, wrist) robotic arm
//Designed for Lynxmotion AL5D robot arm - a different dimention robot arm will it's arm dimension, and the movement may be too small or large, so some altering may be required

//Requires Lynxmotion SSC32 Servo Driver and SSC32 Driver
#include <SSC32.h>
SSC32 myssc = SSC32(); // Enable SSC32 Servo Controller
 
// Arm dimensions( mm )
#define BASE_HGT 67.31      //base hight 2.65"
#define HUMERUS 146.05      //shoulder-to-elbow "bone" 5.75"
#define ULNA 187.325        //elbow-to-wrist "bone" 7.375"
#define GRIPPER 100.00          //gripper (incl.heavy duty wrist rotate mechanism) length 3.94"

void setup()
{
  //Begin communication with SSC32 and Computer
  myssc.begin(9600);  //Communicate with servo controller - Note: Servo Controller Library modified to use serial1
  Serial.begin(9600); //Communicate to computer via USB Serial
  Serial.println("Start");
  //Set arm to default position
  arm_park();
  delay( 500 );
}
 
void loop()
{
  //zero_x();
  //line();
  circle();
 }
 
//Arm positioning routine utilizing inverse kinematics - calculates shl_servopulse, elb_servopulse, and wri_servopulse
//z is height, y is distance from base center out, x is side to side. y,z can only be positive
void set_arm_calc( float x, float y, float z, float grip_angle_d )
{
  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist is y coordinate for the arm */
  y = rdist;
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
 
  /* Servo pulses */
  float shl_servopulse = ftl(1450.0 + (( shl_angle_d - 90.0 ) * 6.6 ));
  float elb_servopulse = ftl(1450.0 -  (( elb_angle_d - 90.0 ) * 6.6 ));
  float wri_servopulse = ftl(1550 + ( wri_angle_d  * 11.1 ));
 
}
 
/* move servos to parking position */
void arm_park()
{
  servos.setposition( BAS_SERVO, 1715 );
  servos.setposition( SHL_SERVO, 2100 );
  servos.setposition( ELB_SERVO, 2100 );
  servos.setposition( WRI_SERVO, 1800 );
  servos.setposition( WRO_SERVO, 600 );
  servos.setposition( GRI_SERVO, 900 );
  return;
}
 
void zero_x()
{
  for( double yaxis = 150.0; yaxis < 356.0; yaxis += 1 ) {
    set_arm_calc( 0, yaxis, 127.0, 0 );
    delay( 10 );
  }
  for( double yaxis = 356.0; yaxis > 150.0; yaxis -= 1 ) {
    set_arm_calc( 0, yaxis, 127.0, 0 );
    delay( 10 );
  }
}
 
/* moves arm in a straight line */
void line()
{
    for( double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5 ) {
      set_arm_calc( xaxis, 250, 100, 0 );
      delay( 10 );
    }
    for( float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5 ) {
      set_arm_calc( xaxis, 250, 100, 0 );
      delay( 10 );
    }
}
 
void circle()
{
  #define RADIUS 80.0
  //float angle = 0;
  float zaxis,yaxis;
  for( float angle = 0.0; angle < 360.0; angle += 1.0 ) {
      yaxis = RADIUS * sin( radians( angle )) + 200;
      zaxis = RADIUS * cos( radians( angle )) + 200;
      set_arm_calc( 0, yaxis, zaxis, 0 );
      delay( 1 );
  }
}
