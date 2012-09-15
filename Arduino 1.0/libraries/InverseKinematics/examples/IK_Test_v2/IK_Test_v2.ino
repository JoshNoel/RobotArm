//Test for Inverse Kinematics for 3 jointed (shoulder, elbow, wrist) robotic arm
//Designed for Lynxmotion AL5D robot arm - a different dimention robot arm will it's arm dimension, and the movement may be too small or large, so some altering may be required

//SSC32 Servo Controller Startup
  #include <SSC32.h>  //Include SSC32 servo controller library
  SSC32 myssc = SSC32(); // Enable SSC32 Servo Controller


//Arm dimensions and servo channel configuration
  // Arm dimensions( mm )
    #define BASE_HGT 67.31      //base hight 2.65"
    #define HUMERUS 146.05      //shoulder-to-elbow "bone" 5.75"
    #define ULNA 187.325        //elbow-to-wrist "bone" 7.375"
    #define GRIPPER 100.00          //gripper (incl.heavy duty wrist rotate mechanism) length 3.94"
  
  //Servo names and acompanying channel on SSC32
    #define BAS_SERVO 0    // Base servo HS-485HB
    #define SHL_SERVO 1    // Shoulder Servo HS-5745-MG
    #define ELB_SERVO 2    // Elbow Servo HS-5745-MG
    #define WRI_SERVO 3    // Wrist servo HS-645MG
    #define WRO_SERVO 4    // Wrist rotate servo HS-485HB
    #define GRI_SERVO 5    // Gripper servo HS-422

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
 
//Arm positioning routine utilizing inverse kinematics
//z is height, y is distance from base center out, x is side to side. y,z can only be positive
void set_arm( float x, float y, float z, float grip_angle_d )
{
  //Calculate servo pulses (i.e. direction or angle) for coordinates
    set_arm_calc(x, y, z, grip_angle_d)
  //Send servo pulses to servos
    myssc.servoMove( WRI_SERVO, wri_servopulse);
    myssc.servoMove( SHL_SERVO, shl_servopulse);
    myssc.servoMove( ELB_SERVO, elb_servopulse);
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
