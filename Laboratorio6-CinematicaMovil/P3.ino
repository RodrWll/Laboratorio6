/*----------------------- DO NOT CHANGE THIS ------------------------*/

/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/

#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>
#include <EEPROM.h>

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
// Pins of each motor
const int enc[] = {4, 5, 8, 13};  // Encoder pins
const int DIR[] = {2, 7, 9, 12};  // Direction pins
const int pwm[] = {3, 6, 10, 11}; // PWM pins

// Globals
int posPrev[] = {0, 0, 0, 0};
float vel[]   = {0.0, 0.0, 0.0, 0.0}; // Measured velocity in rpm
float vt[]    = {0.0, 0.0, 0.0, 0.0}; // Target vel in rpm
int dir[]     = {0, 0, 0, 0};         // Direction of each motor
long prevT    = 0;                    // Previous measured time
// PPR of each motor
const double ppr[] = {1390, 1390, 1390, 1390}; // Resolution of each motor

// PID class instances
SimplePID pid[NMOTORS];

// Dynamic variables
float velAng[]     = {0, 0, 0, 0}; // Angular velocity of each motor in rad/s
float meanVelAng[] = {0, 0, 0, 0}; // Mean angular velocity of current and previous time to compute pose estimation
int velEnc[]       = {0, 0, 0, 0}; // Encoder count of each motor
int velEncSlack[]  = {0, 0, 0, 0}; // Auxiliary counter variable
float sampleT      = 0.1;

// Pose variables
double x = 0, y = 0, theta = 0; // Current pose variables 
double xPrev = 0, yPrev = 0, thetaPrev = 0; // Previous pose variables 

// Velocity limits
double vminLim = 15.0;
// Velocities signs
int sgn[]     = {1, 1, 1, 1}; // Current sign (to compute pose estimation)
int sgnPrev[] = {1, 1, 1, 1}; // Previous sign (to compute pose estimation)
// Initial and elapsed time of a sequence
double t0 = 0.0, t1 = 0.0;
// Target auxiliary control variables
double vx = 0, vy = 0, vw = 0;

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
  pid[0].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, vminLim);
  pid[1].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, vminLim);
  pid[2].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, vminLim);
  pid[3].setParams(/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */, vminLim);

  // Activate interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
  delay(2000);
  
  // Show previous sample
  float x_prev = EEPROM_readFloat(0);
  float y_prev = EEPROM_readFloat(49);
  float theta_prev = EEPROM_readFloat(99);
  Serial.print("X_prev = ");
  Serial.print(x_prev*1000);
  Serial.print(" (mm), ");
  Serial.print("Y_prev = ");
  Serial.print(y_prev*1000);
  Serial.print(" (mm), ");
  Serial.print("theta_prev = ");
  Serial.print(theta_prev*180/PI);
  Serial.println(" (deg)");
  Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");
  prevT = micros();
  t0 = ((float) (prevT))/( 1.0e6 );
}
/*-------------------------------------------------------------------*/

/*----------------------- YOU CAN MODIFY THIS -----------------------*/

// Sequences variables
int nseq      = 3; // Number of sequences
int seq       = 0; // Counter sequence variable
float T[]     = {/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */}; // Time of each sequence
// Velocity target sequences
float vxSeq[] = {/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */}; // Target relative linear velocity of the platform fixed frame in X axis
float vySeq[] = {/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */}; // Target relative linear velocity of the platform fixed frame in Y axis
float vwSeq[] = {/* COMPLETE HERE */, /* COMPLETE HERE */, /* COMPLETE HERE */}; // Target relative angular velocity of the platform fixed frame in Z axis
// Robot dimentions
const double a_b = /* COMPLETE HERE */;   // a+b
const double R = /* COMPLETE HERE */;     // radius
const double l_a_b = /* COMPLETE HERE */; // 1/(a+b)

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
    // Update time variable
    prevT = currT;
    // Disable interrupts temporarily while reading counter variables
    noInterrupts(); 
    for(int k = 0; k < NMOTORS; k++){
      // Reset counter
      velEncSlack[k] = velEnc[k];
      velEnc[k] = 0;
    }
    interrupts();

    // Process variables
    for(int k = 0; k < NMOTORS; k++){
      // Calculate velocity in rpm
      vel[k] = velEncSlack[k]/deltaT/ppr[k]*60.0;
      // Calculate velocity and differential velocity in rad/s (use sgn[] and sgnPrev[] variables)
      meanVelAng[k] = /* COMPLETE HERE */;
      velAng[k] = vel[k]*PI/30;
    }

    // Estimate Pose
    BilinealEstimation(deltaT);

    // Motion Sequences
    if(seq<nseq){
      vx = /* COMPLETE HERE */; 
      vy = /* COMPLETE HERE */; 
      vw = /* COMPLETE HERE */;
      // Compute target velocity for each motor
      CalculateVelAng(/* COMPLETE HERE */,/* COMPLETE HERE */,/* COMPLETE HERE */);
      for(int k = 0; k < NMOTORS; k++){
        int pwr;
        // Obtaine control signal from PID algorithm 
        pid[k].evalu(/* COMPLETE HERE */, /* COMPLETE HERE */, deltaT, pwr);
        // signal the motor
        setMotor(dir[k], pwr, pwm[k], DIR[k]);
      }
      if(T[seq] <= t1-t0){
        t0 = t1;
        seq++;
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

void BilinealEstimation(double deltaT){
  /* 
  Function that integrates numericaly online robot position.

  Inputs:
    - deltaT: Time step.
  */
  double ct, st;
  // Pre-compute cosine and sine
  ct = cos(theta); st = sin(theta);
  // Bilineal numerical estimation (use meanVelAng[] variable)

  /* COMPLETE HERE */

  // Update previous values
  xPrev = x;
  yPrev = y;
  thetaPrev = theta;
  // Print estimated pose
  Serial.print("X = ");
  Serial.print(x*1000);
  Serial.print(" (mm), ");
  Serial.print("Y = ");
  Serial.print(y*1000);
  Serial.print(" (mm), ");
  Serial.print("theta = ");
  Serial.print(theta*180/PI);
  Serial.println(" (deg)");
  Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");
  // Save estimated pose
  EEPROM_writeFloat(0, x);
  EEPROM_writeFloat(49, y);
  EEPROM_writeFloat(99, theta);
}

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
    sgnPrev[i] = sgn[i];
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

void EEPROM_writeFloat(int address, float value) {
  /* 
  Function to write final pose value. 
  */
  byte* floatPtr = (byte*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address + i, floatPtr[i]);
  }
}

float EEPROM_readFloat(int address) {
  /* 
  Function to read final pose value. 
  */
  float value = 0.0;
  byte* floatPtr = (byte*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    floatPtr[i] = EEPROM.read(address + i);
  }
  return value;
}

/*-------------------------------------------------------------------*/