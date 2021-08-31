#include "Arduino.h"
#include "dcms.h"


dcms::dcms(float kp, float kd, float ki, char umax, float eprev, float eintegral){
  kp = kp;
  ki = ki;
  kd = kd;
  prevT = 0;
  eprev = 0.0;
  eintegral = 0.0;
  umax = umax;
}


void dcms::setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
  kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  prevT = 0;
  eprev = 0.0;
  eintegral = 0.0;

}


void dcms::evaluatePosition(int value, int target, int &pwr, int &dir){
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = target - value;
  
  // derivative
  float dedt = (e-eprev)/(deltaT);
  
  // integral
  eintegral = eintegral + e*deltaT;
  
  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;
  
  // motor power
  pwr = (int) fabs(u);
  if( pwr > umax ){
    pwr = umax;
  }
  
  // motor direction
  dir = 1;
  if(u<0){
    dir = -1;
  }
  
  // store previous error
  eprev = e;
}

void dcms::setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
  
