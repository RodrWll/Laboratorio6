/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/

#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>

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
float vel   = 0.0;
long prevT  = 0;
// PPR of each motor
const double ppr[] = {1390, 1390, 1390, 1390}; 

// Dynamic variables
int velEnc[]     = {0, 0, 0, 0};
int velEncSlack  = 0;

// Variables
int   input  = 255;
int   dir[]  = {1, 1, 1, 1};
float t0     = 0.0;
long  initT  = 0;

/*----------------------- YOU CAN MODIFY THIS -----------------------*/
int   motor   = 1;   // From 1 to 4.
float waitT   = 2.0; // In seconds.
float totalT  = 4.0; // In seconds.
float sampleT = 0.1; // In seconds.
/*-------------------------------------------------------------------*/

/*********************************************************************/
/*****************************   SETUP   *****************************/
/*********************************************************************/

void setup() {
  int k = motor-1;
  // Begin communication
  Serial.begin(9600);
  // Setup I/O pins
  pinMode(enc[k], INPUT);
  pinMode(pwm[k], OUTPUT);
  pinMode(DIR[k], OUTPUT);
  // Activate interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
  delay(2000);
  prevT = micros();
  initT = prevT;
  t0 = ((float) (initT))/( 1.0e6 );
  Serial.print(0);
  Serial.print(",");
  Serial.println(0.0);
}

/*********************************************************************/
/*****************************   LOOP   ******************************/
/*********************************************************************/

void loop() {
  int k = motor-1;
  // Time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  float t1 = ((float) (currT))/( 1.0e6 );
  
  // Compute program each sample time
  if(sampleT <= deltaT){
    prevT = currT;
    // Disable interrupts temporarily while reading
    noInterrupts(); 
    // Reset counter
    velEncSlack = velEnc[k];
    velEnc[k] = 0;
    interrupts();
    // Calculate angular velocity in rpm
    vel = velEncSlack/deltaT/ppr[k]*60.0;

    // Wait time
    if((t1-t0) < waitT){
      StopMotors();
      Serial.print(currT- initT);
      Serial.print(",");
      Serial.println(vel);
    }  // Run time
    else if((t1-t0) < totalT){
      setMotor(dir[k], input, pwm[k], DIR[k]);
      Serial.print(currT - initT);
      Serial.print(",");
      Serial.println(vel);
    } // End
    else{
      StopMotors();
    }
  }
}

/*********************************************************************/
/***************************   FUNCTIONS   ***************************/
/*********************************************************************/

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