#include "dcms.h"

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


dcms pidL(10.0, 0.05, 8.0, 255, 0.0, 0.0);
dcms pidR(10.0, 0.05, 8.0, 255, 0.0, 0.0);

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
  
  Serial.println("target pos");
}

void loop() { // put a time checker here, and then an if statment that looks for it and then publishes
  int target = -45;
  int pwrL, pwrR, dir;
  int loop_counter = 0;
  int loop_counter2 = 0;
  int debug = 1;
  float linear_velocity = 0.3;
  float angular_velocity = 0.05;
  float left_rpm, right_rpm;
    
  while(1){
    loop_counter++;
    loop_counter2++;

    left_rpm  = -(linear_velocity - 0.5f*angular_velocity*WHEEL_BASE)/(RPM_TO_RAD_PER_S * DIST_PER_RAD);
    right_rpm = (linear_velocity + 0.5f*angular_velocity*WHEEL_BASE)/(RPM_TO_RAD_PER_S * DIST_PER_RAD);

    pidL.evaluatePosition(encoderRPML,left_rpm,pwrL,dir);
    pidL.setMotor(dir,pwrL,pwmL,in1,in2);

    pidR.evaluatePosition(encoderRPMR,right_rpm,pwrR,dir);
    pidR.setMotor(dir,pwrR,pwmR,in3,in4);



    
    if(loop_counter == 10000){
      //target = random(-40, 40);
      //target = 25;
      target++;
      if(target > 45){
        target = -45;
      }
      loop_counter = 0;
    }
    if(loop_counter2 == 1000 && debug == 1){
      Serial.print(target);
      Serial.print(" ");
      Serial.print(encoderRPML);
      Serial.print(" ");
      Serial.print(encoderRPMR);    
      Serial.println();
      loop_counter2 = 0;
    }



  }
}














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


/*


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
















#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup()
{
  nh.initNode();
  broadcaster.init(nh);
}

void loop()
{  
  // drive in a circle
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();
  
  delay(10);
}




*/

