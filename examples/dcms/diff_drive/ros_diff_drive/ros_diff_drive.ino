#include "dcms.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

/*
If the system must remain online, one tuning method is to first set Ki Kd values to zero. 
Increase the Kp until the output of the loop oscillates, then the Kp should be set to approximately 
half of that value for a "quarter amplitude decay" type response. Then increase Ki until any offset 
is corrected in sufficient time for the process. However, too much Ki will cause instability. 
Finally, increase Kd, if required, until the loop is acceptably quick to reach its reference after a 
load disturbance. However, too much Kd will cause excessive response and overshoot. 
A fast PID loop tuning usually overshoots slightly to reach the setpoint more quickly; however, some systems 
cannot accept overshoot, in which case an overdamped closed-loop system is required, which will 
require a Kp setting significantly less than half that of the Kp setting that was causing oscillation.

https://robotics.stackexchange.com/questions/18048/inverse-kinematics-for-differential-robot-knowing-linear-and-angular-velocities
*/ 

volatile int posiL = 0;
long previousEncoderTimeL = micros();
float deltaEncoderTimeL = 0;
long currentEncoderTimeL = 0;
int encoderRPML = 0;

volatile int posiR = 0;
long previousEncoderTimeR = micros();
float deltaEncoderTimeR = 0;
long currentEncoderTimeR = 0;
int encoderRPMR = 0;

float WHEEL_BASE = 0.25;
float RPM_TO_RAD_PER_S = 0.1047;
float DIST_PER_RAD = 0.03; // this works out to be wheel radius - 2*pi*WHEEL_RADIUS, divided by the number of radians in a complete revolution, which is just 2*pi

float linear_velocity = 0.0;
float angular_velocity = 0.0;

int pwmL = 17; // needs changing to actual pin!!
int in1 = 9;// needs changing to actual pin!!
int in2 = 8;// needs changing to actual pin!!
int enca = 4; // needs changing to actual pin!! RIGHT
int encb = 5; // needs changing to actual pin!!

int pwmR = 16;
int in3 = 10;
int in4 = 11;
int encc = 2; // YELLOW LEFT 2
int encd = 3; // WHITE 3

int ticksPerRev = 80;

/*
int pwmR = 17;
int in3 = 9;
int in4 = 8;
int encc = 4; // YELLOW LEFT
int encd = 5; // WHITE

int pwmL = 16; // needs changing to actual pin!!
int in1 = 10;// needs changing to actual pin!!
int in2 = 11;// needs changing to actual pin!!
int enca = 2; // needs changing to actual pin!! RIGHT
int encb = 3; // needs changing to actual pin!!
*/

void readEncoderL(){
  //2225 ticks per rev in this mode

  currentEncoderTimeL = micros();
  deltaEncoderTimeL = ((float) (currentEncoderTimeL - previousEncoderTimeL))/( 1.0e6 );
  previousEncoderTimeL = currentEncoderTimeL;
  if(digitalRead(encb) > 0){
    posiL++;
    encoderRPML = (1/(deltaEncoderTimeL * ticksPerRev)*60);  
  }
  else{
    posiL--;
    encoderRPML = (-1/(deltaEncoderTimeL * ticksPerRev)*60); 
  }
}

void readEncoderR(){
  //2225 ticks per rev in this mode

  currentEncoderTimeR = micros();
  deltaEncoderTimeR = ((float) (currentEncoderTimeR - previousEncoderTimeR))/( 1.0e6 );
  previousEncoderTimeR = currentEncoderTimeR;
  if(digitalRead(encd) > 0){
    posiR++;
    encoderRPMR = (1/(deltaEncoderTimeR * ticksPerRev)*60);  
  }
  else{
    posiR--;
    encoderRPMR = (-1/(deltaEncoderTimeR * ticksPerRev)*60); 
  }
}

void cmdVelCallback(const geometry_msgs::Twist& cmdVel) {
  linear_velocity = cmdVel.linear.x;
  angular_velocity = cmdVel.angular.z;
}



dcms pidL(10.0, 0.05, 8.0, 255, 0.0, 0.0);
dcms pidR(10.0, 0.05, 8.0, 255, 0.0, 0.0);
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> subCmdVel("turtle1/cmd_vel", &cmdVelCallback );

void setup() {
  Serial.begin(115200);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  attachInterrupt(digitalPinToInterrupt(enca),readEncoderL,RISING); // only need one encoder on interrupt as the other is read in the ISR for direction
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  
  pinMode(pwmL,OUTPUT); // the duplicated declaration is required due to a bug in the arduino pico firmware, cba finding the link but it took ages to figure out.
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  pinMode(encc,INPUT);
  pinMode(encd,INPUT);
  attachInterrupt(digitalPinToInterrupt(encc),readEncoderR,RISING);
  pinMode(encc,INPUT);
  pinMode(encd,INPUT);
  
  pinMode(pwmR,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  pidL.setParams(10,0.05,8,255);//P D I maxOuput set p to 1 for posi control, D to 0.025 and i to 0
  pidR.setParams(10,0.05,8,255);//P D I maxOuput set p to 1 for posi control, D to 0.025 and i to 0
  
  
  
  nh.initNode();
  
  //nh.advertise(rightPub);
  //nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  //nh.logdebug("Debug Statement");
  nh.loginfo("Osprey Systems Engineering UGV");
  nh.loginfo("Version 0.0.1");
  nh.loginfo("Dr. Benjamin Bird");
  nh.loginfo("Isis - Beginning setup");
  nh.loginfo("Osiris - Interrupts configured");
  nh.loginfo("Set - Setup complete");
  //nh.logwarn("Warnings.");
  //nh.logerror("Errors..");
  //nh.logfatal("Fatalities!");

}

void loop() { // put a time checker here, and then an if statment that looks for it and then publishes
  int target = -45;
  int pwrL, pwrR, dir;
  int loop_counter = 0;
  int loop_counter2 = 0;
  int debug = 0;
  float left_rpm, right_rpm;
  nh.loginfo("Ra - Sunrise - Entering main loop");

  while(1){
    nh.spinOnce();

    loop_counter++;
    loop_counter2++;

    left_rpm  = -(linear_velocity - 0.5f*angular_velocity*WHEEL_BASE)/(RPM_TO_RAD_PER_S * DIST_PER_RAD);
    right_rpm = (linear_velocity + 0.5f*angular_velocity*WHEEL_BASE)/(RPM_TO_RAD_PER_S * DIST_PER_RAD);

    pidL.evaluatePosition(encoderRPML,left_rpm,pwrL,dir);
    if(left_rpm == 0){
      pwrL = 0.0;
    }
    pidL.setMotor(dir,pwrL,pwmL,in1,in2);

    pidR.evaluatePosition(encoderRPMR,right_rpm,pwrR,dir);
    if(right_rpm == 0){
      pwrR = 0.0;
    }
    pidR.setMotor(dir,pwrR,pwmR,in3,in4);

  }
}
















