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
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
*/ 

volatile int posi = 0;
long previousEncoderTime = micros();
float deltaEncoderTime = 0;
long currentEncoderTime = 0;
int encoderRPM = 0;


int enca = 4; // YELLOW
int encb = 5; // WHITE
int pwm = 10;
int in2 = 9;
int in1 = 8;


dcms pid(8.0, 0.025, 4.0, 255, 0.0, 0.0);

void setup() {
  Serial.begin(115200);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  attachInterrupt(digitalPinToInterrupt(enca),readEncoder,RISING);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  
  pinMode(pwm,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  pid.setParams(8,0.025,4,255);//P D I maxOuput set p to 1 for posi control, D to 0.025 and i to 0
  
  Serial.println("target pos");
}

void loop() {
  int target = 0;
  int pos;
  int pwr, dir;
  int loop_counter = 0;
  
  while(1){
    loop_counter++;
    if(loop_counter == 10000){
      target = random(0, 100);
      loop_counter = 0;
    }

    pos = encoderRPM;
    pid.evaluatePosition(pos,target,pwr,dir);
    pid.setMotor(dir,pwr,pwm,in1,in2);
    Serial.print(target);
    Serial.print(" ");
    Serial.print(pos);
    //Serial.print(" ");
    //Serial.print(encoderRPM);
    Serial.println();
  }
}


void readEncoder(){
  //2225 ticks per rev in this mode
  int b = digitalRead(encb);
  currentEncoderTime = micros();
  deltaEncoderTime = ((float) (currentEncoderTime - previousEncoderTime))/( 1.0e6 );
  previousEncoderTime = currentEncoderTime;
  if(b > 0){
    posi++;
    encoderRPM = (1/(deltaEncoderTime * 2225)*60);  
  }
  else{
    posi--;
    encoderRPM = (-1/(deltaEncoderTime * 2225)*60); 
  }
}