#ifndef DCMS_h
#define DCMS_h

#include "Arduino.h"

class dcms{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage
    long prevT;

  public:
  // Constructor
  dcms(float kp, float kd, float ki, char umax, float eprev, float eintegral);
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);
  void evaluatePosition(int value, int target, int &pwr, int &dir);
  void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
}; 

 
#endif