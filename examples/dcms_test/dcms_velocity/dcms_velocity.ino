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


// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage
    long prevT;

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    prevT = 0;

  }

  // A function to compute the control signal
  void evaluatePosition(int value, int target, int &pwr, int &dir){
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
  
};

volatile int posi = 0;
long previousEncoderTime = micros();
float deltaEncoderTime = 0;
long currentEncoderTime = 0;
int encoderRPM = 0;



int enca = 4; // YELLOW
int encb = 5; // WHITE
int pwm = 17;
int in2 = 8;
int in1 = 9;


// PID class instances
SimplePID pid;

void setup() {
  Serial.begin(9600);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  attachInterrupt(digitalPinToInterrupt(enca),readEncoder,RISING);
  pinMode(enca,INPUT);
  pinMode(encb,INPUT);
  
  pinMode(pwm,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);

  pid.setParams(8,0,0,255);//P D I maxOuput set p to 1 for posi control, D to 0.025 and i to 0
  
  Serial.println("target pos");
}

void loop() {
  // set target position
  int target;
  target = -50;

  int pos;
  pos = encoderRPM;
  
  int pwr, dir;
  // evaluate the control signal
  pid.evaluatePosition(pos,target,pwr,dir);
  // signal the motor
  setMotor(dir,pwr,pwm,in1,in2);

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(encoderRPM);
  
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
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

void readEncoder(){
  //515 ticks per rev in this mode
  int b = digitalRead(encb);
  currentEncoderTime = micros();
  deltaEncoderTime = ((float) (currentEncoderTime - previousEncoderTime))/( 1.0e6 );
  previousEncoderTime = currentEncoderTime;
  if(b > 0){
    posi++;
    encoderRPM = (1/(deltaEncoderTime * 515)*60);  
  }
  else{
    posi--;
    encoderRPM = (-1/(deltaEncoderTime * 515)*60); 
  }
}