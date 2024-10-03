/*----------------------- DO NOT CHANGE THIS ------------------------*/

/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/
#include <math.h>

/*********************************************************************/
/***************************   VARIABLES   ***************************/
/*********************************************************************/
//Holaaaaaaa 
// Number of motors
#define NMOTORS 4
#define pi 3.14159265359
// Pins of each motor
const int enc[] = {4, 5, 8, 13};  // Encoder pins
const int DIR[] = {7, 2, 9, 12};  // Direction pins
const int pwm[] = {6, 3, 10, 11}; // PWM pins

/*********************************************************************/
/*****************************   SETUP   *****************************/
/*********************************************************************/

void setup() {
  // Begin communication
  Serial.begin(9600);
  // Setup I/O pins
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enc[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(DIR[k], OUTPUT);
  }
  delay(2000);
}
// Globals
float vt[]   = {0.0, 0.0, 0.0, 0.0};
int dir[]    = {0, 0, 0, 0};
// Velocities signs
int sgn[]   = {1, 1, 1, 1};
// Target variables
double vx = 0, vy = 0, vw = 0;

/*-------------------------------------------------------------------*/

/*----------------------- YOU CAN MODIFY THIS -----------------------*/

// Sequences variables
int nseq     = 11;   // Number of sequences
int seq      = 0;    // Counter sequence variable
float T      = 1000; // Time of each sequence in ms
float waitT  = 500;  // Waiting time of each sequence in ms

// Velocity target sequences
float vxSeq[11] = {0,0.1,-0.1,0,0,0.1,0.1,-0.1,-0.1,0,0}; // X Axis linear velocity sequence
float vySeq[11] = {0,0,0,0.1,-0.1,0.1,-0.1,0.1,-0.1,0,0}; // Y Axis linear velocity sequence
float vwSeq[11] = {0,0,0,0,0,0,0,0,0,-0.62831853071,0.62831853071}; // Z Axis angular velocity sequence

// Dimensions
const double a_b = (0.21 + 0.195)/2; // a+b
const double R = 0.04;   // radius

// Maximun values
float maxPWM = 255; // Input value
float maxRPM = 90; // Input value

/*********************************************************************/
/*****************************   LOOP   ******************************/
/*********************************************************************/

void loop() {
  // Set target velocity
  vx = vxSeq[seq]; 
  vy = vySeq[seq]; 
  vw = vwSeq[seq];
 if(seq < nseq){
    // Calculate target angular velocities
    CalculateVelAng(vx,vy,vw);
    for(int k = 0; k < NMOTORS; k++){
      // Compute input motor signal
      float pwr = vt[k]*maxPWM/maxRPM;
      // Truncate value
      if(pwr > maxPWM){
        pwr = maxPWM;
      }
      if(pwr < 0){
        pwr = 0;
      }
      // Signal to the motor
      setMotor(dir[k], (int) pwr, pwm[k], DIR[k]);
    }
    delay(T);
    StopMotors();
    delay(waitT);
    seq++;
 }
  else{
  StopMotors();
 }
}

/*********************************************************************/
/***************************   FUNCTIONS   ***************************/
/*********************************************************************/

void CalculateVelAng(double vx, double vy, double vw) { 
  /* 
  Function that computes the velocity in rpm and the direction 
  of each wheel from the absolute velocity.

  Inputs:
    - vx: Linear velocity in X axis, in m/s.
    - vy: Linear velocity in Y axis, in m/s.
    - vw: Angular velocity in Z axis, in rad/s.
  */
  // Calculate velocity of each motor in rad/s in w[] (from inverse kinematics)
  
  double arreglo_velocidad[3][1] = {vx,vy,vw};
  double jacobiano_seudoinverso[4][3]={
                                        {1, -1, -(a_b)},
                                        {1,  1,  (a_b)},
                                        {1,  1, -(a_b)},
                                        {1, -1,  (a_b)}
  };

  double w[4]= {(vx-vy - vw*a_b)/R,(vx+vy + vw*a_b)/R,(vx+vy - vw*a_b)/R,(vx-vy + vw*a_b)/R};

  for (int i = 0; i < NMOTORS; i++) {
    sgn[i] = w[i] / fabs(w[i]); 
    // Update motor direction
    dir[i] = (1 + sgn[i]) / 2;
    // Calculate desired angular velocity in rpm
    vt[i] = fabs(w[i]*30/PI);
  }
}

/*-------------------------------------------------------------------*/

/*----------------------- DO NOT CHANGE THIS ------------------------*/

void setMotor(int dir, int pwmVal, int pwmch, int dirch) {
  /* 
  Function to setup pins to control motors.

  Inputs:
    - dir: Motor direction (1 or 0).
    - pwmVal: PWM control to pin.
    - pwmch: PWM pin channel.
    - dirch: Direction pin channel.
  */
  analogWrite(pwmch, pwmVal);
  if(dirch==2 || dirch==12
  ){
    if (dir == 1) {
      digitalWrite(dirch, LOW);
    } else if (dir == 0) {
      digitalWrite(dirch, HIGH);
    } else {
      digitalWrite(dirch, LOW);
    }
  }
  else{
    if (dir == 1) {
      digitalWrite(dirch, HIGH);
    } else if (dir == 0) {
      digitalWrite(dirch, LOW);
    } else {
      digitalWrite(dirch, HIGH);
    }
  }
}


void StopMotors(){
  /* 
  Function that stops each DC motor. 
  */
  for(int k = 0; k < NMOTORS; k++){
    setMotor(dir[k], 0.0, pwm[k], DIR[k]);
  }
}

/*-------------------------------------------------------------------*/
