/*----------------------- DO NOT CHANGE THIS ------------------------*/

/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/

#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>

/*********************************************************************/
/****************************   CLASSES   ****************************/
/*********************************************************************/

class SimplePID{
  /* 
  This class computes the control signal for velocity control of a 
  DC motor with a PID controller base on controller gains, a setpoint and 
  the actual velocity value.
  Params:
    - kp: Proportional gain.
    - kd: Derivative gain.
    - ki: Integral gain.
    - umax: Maximun control value.
    - eprec: Previous error in the control loop.
    - umax: Integral cumulative error.
    - vmin: Minimun velocity in rpm.
  */
  private:
    float kp, kd, ki, umax, vmin; // Parameters
    float eprev, eintegral; // Cumulative variables

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), vmin(15.0){}
  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn, float vminIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn; vmin = vminIn;
  }
  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr){
    // Error
    float e = (target - value)*((float) fabs(target) > vmin);
    // Derivative
    float dedt = (e - eprev)/(deltaT)*((float) fabs(target) > vmin);
    // Integral
    eintegral = (eintegral + e * deltaT)*((float) fabs(target) > vmin);
    if (umax/ki<eintegral){
      eintegral = umax/ki;
    }
    if (-umax/ki>eintegral){
      eintegral = -umax/ki;
    }
    // Control signal
    float u = kp * e + kd * dedt + ki * eintegral;
    pwr = (int) fabs(u);
    // Truncate signal
    if (pwr > umax) {
      pwr = umax;
    }
    if (pwr < 0) {
      pwr = 0;
    }
    eprev = e;
  }
};

/*********************************************************************/
/***************************   VARIABLES   ***************************/
/*********************************************************************/

// Number of motors
#define NMOTORS 4
// Pins
const int enc[] = {4, 5, 8, 13};
const int DIR[] = {2, 7, 9, 12};
const int pwm[] = {3, 6, 10, 11};

// Globals
float vel[]   = {0.0, 0.0, 0.0, 0.0};
float vt[]    = {0.0, 0.0, 0.0, 0.0};
int dir[]     = {0, 0, 0, 0};
long prevT    = 0;
// PPR of each motor
const double ppr[] = {1390, 1390, 1390, 1390}; 

// PID class instances
SimplePID pid[NMOTORS];

// Dynamic variables
int velEnc[]       = {0, 0, 0, 0};
int velEncSlack[]  = {0, 0, 0, 0};
float sampleT      = 0.1;

// Velocity limits
double vminLim = 15.0;
// Velocities signs
int sgn[] = {1, 1, 1, 1};
// Initial and elapsed time
double t0 = 0.0, t1 = 0.0;

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
  // PID gains for each motor
  pid[0].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, 255, vminLim);
  //pid[1].setParams(6.5, 0.2, 0.5, 255, vminLim);
  pid[1].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, 255, vminLim);
  pid[2].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, 255, vminLim);
  pid[3].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, 255, vminLim);
  // Activate interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
  delay(2000);
  prevT = micros();
  t0 = ((float) (prevT))/( 1.0e6 );
}

/*-------------------------------------------------------------------*/

/*----------------------- YOU CAN MODIFY THIS -----------------------*/

// Target variables for control
double vx = /* COMPLETE HERE */;
double vy = /* COMPLETE HERE */;
double vw = /* COMPLETE HERE */;
// Time duration
double time = /* COMPLETE HERE */;
// Robot dimentions
const double a_b = /* COMPLETE HERE */;   // a+b
const double R = /* COMPLETE HERE */;     // radius

/*********************************************************************/
/*****************************   LOOP   ******************************/
/*********************************************************************/

void loop() {
  
  // Time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  t1 = ((float) (currT))/( 1.0e6 );
  
  // Compute program each sample time
  if(sampleT <= deltaT){
    prevT = currT;
    // Disable interrupts temporarily while reading
    noInterrupts(); 
    for(int k = 0; k < NMOTORS; k++){
      // Reset counter
      velEncSlack[k] = velEnc[k];
      velEnc[k] = 0;
    }
    interrupts();

    // Get current velocity in rpm
    for(int k = 0; k < NMOTORS; k++){
      // Calculate velocity in rpm
      vel[k] = velEncSlack[k]/deltaT/ppr[k]*60.0;
    }

    // Move robot
    if(t1-t0 < time){
      CalculateVelAng(/* COMPLETE HERE */,/* COMPLETE HERE */,/* COMPLETE HERE */);
      for(int k = 0; k < NMOTORS; k++){
        int pwr;
        // Get control signal from PID algorithm 
        pid[k].evalu(/* COMPLETE HERE */, /* COMPLETE HERE */, deltaT, pwr);
        // signal the motor
        setMotor(dir[k], pwr, pwm[k], DIR[k]);
        Serial.println(vel[k])
        Serial.println(vt[k])
      }
    }
    else{
      StopMotors();
    }
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
  double w[] = {0, 0, 0, 0};
  // Angular velocity of each motor in rad/s (from the first exercise)

  /* COMPLETE HERE */

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
  if(dirch==12 || dirch==7){
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


template <int j>
void readEncoder() {
  /* 
  Function that counts each rising edge of a encoder
  */
  velEnc[j]++;
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