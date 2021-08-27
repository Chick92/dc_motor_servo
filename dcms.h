#ifndef DCMS_h
#define DCMS_h

#include "Arduino.h"

class dcms
{
  public:
    dcms(char IN1, char IN2, char EN);
    void forwards(char pwm);
    void backwards(char pwm);
  private:
    char _IN1, _IN2, _EN;
};

#endif