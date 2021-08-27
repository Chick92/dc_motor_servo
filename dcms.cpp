#include "Arduino.h"
#include "L298N.h"

L298N::L298N(char IN1, char IN2, char EN)
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
  _IN1 = IN1;
  _IN2 = IN2;
  _EN = EN;
}

void L298N::forwards(char pwm)
{
  digitalWrite(_IN1, HIGH); 
  digitalWrite(_IN2, LOW); 
  analogWrite(_EN, pwm);
  return; 
}

void L298N::backwards(char pwm)
{
  digitalWrite(_IN1, LOW); 
  digitalWrite(_IN2, HIGH); 
  analogWrite(_EN, pwm);
  return;
}