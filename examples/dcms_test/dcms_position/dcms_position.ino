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
/*
 * ackermann
int enca = 4; // YELLOW
int encb = 5; // WHITE
int pwm = 10;
int in2 = 9;
int in1 = 8;
*/

// Dagu right
int enca = 2; // YELLOW
int encb = 3; // WHITE
int pwm = 16;
int in1 = 10;
int in2 = 11;

/*
//Dagu left
int enca = 5; // YELLOW
int encb = 4; // WHITE
int pwm = 17;
int in1 = 8;
int in2 = 9;
*/


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

  pid.setParams(2.5,0.025,0,255);//P D I maxOuput //2.5P for Dagu, 1 for ackermann
  
  Serial.println("target pos");
}

void loop() {
  // set target position
  int target;
  target = 100;

  int pos;
  pos = posi;
  
  int pwr, dir;
  // evaluate the control signal
  pid.evaluatePosition(pos,target,pwr,dir);
  // signal the motor
  setMotor(dir,pwr,pwm,in1,in2);

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  
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
  int b = digitalRead(encb);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}