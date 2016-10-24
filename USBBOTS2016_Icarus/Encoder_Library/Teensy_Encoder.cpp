/*
   Rotary Encoders Speed and Steps calculation
   Teensy_Encoder.cpp Library file
*/
#include "Arduino.h"
#include "Teensy_Encoder.h"

// Rotary encoder variables, used by interrupt routines
 volatile int8_t old_enc1 = 0, old_enc2 = 0;
 volatile int8_t new_enc1 = 0, new_enc2 = 0;
 volatile int32_t rot1Steps = 0, rot2Steps = 0;
// Square matrix for direction calculation.
 volatile int8_t QEM [16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

// Float speeds
volatile float speed1 = 0.0;
volatile float speed2 = 0.0;
volatile int32_t old_rot1Steps = 0, old_rot2Steps = 0;

// Create an IntervalTimer object
IntervalTimer rotationTimer;

// Configure the interrupts pins and interrupts
void configureEncoderLib(void){
  // Configure pins
  pinMode(rotA1Pin, INPUT);
  pinMode(rotB1Pin, INPUT);
  pinMode(rotA2Pin, INPUT);
  pinMode(rotB2Pin, INPUT);

  // Configure Interrupts on the variables changes
  attachInterrupt(rotA1Pin,ISRrotA1Change, CHANGE);
  attachInterrupt(rotB1Pin,ISRrotB1Change, CHANGE);
  attachInterrupt(rotA2Pin,ISRrotA2Change, CHANGE);
  attachInterrupt(rotB2Pin,ISRrotB2Change, CHANGE);

  // Configure a timer to take measurements every certain amount of time
  rotationTimer.begin(calculateSpeed, timerRotationTime);
}

// Calculates both speeds from the encoders
void calculateSpeed(void){

  // If you are here, other interrupts are disable,
  // Which lets you access interrupt variables without
  // extra precaution
  speed1 = ((rot1Steps - old_rot1Steps)*stepstoDistance)/(timerRotationTime)*1000000;
  speed2 = ((rot2Steps - old_rot2Steps)*stepstoDistance)/(timerRotationTime)*1000000;


  old_rot1Steps = rot1Steps;
  old_rot2Steps = rot2Steps;
}

// Return the current value of the speed1
// counter in an interrupt safe way.
float getSpeed1(void) {
  float speed;

  cli();
  speed = speed1;
  sei();
  return speed;
}

// Return the current value of the speed2
// counter in an interrupt safe way.
float getSpeed2(void) {
  float speed;

  cli();
  speed = speed2;
  sei();
  return speed;
}

// Return the current value of the rotaryEncoder1
// counter in an interrupt safe way.
int getRot1Steps(void) {
  int rot;

  cli();
  rot = rot1Steps;
  sei();
  return rot;
}

// Return the current value of the rotaryEncoder2
// counter in an interrupt safe way.
int getRot2Steps(void) {
  int rot;

  cli();
  rot = rot2Steps/stepstoDistance;
  sei();
  return rot;
}

// Return the current value of the rotaryEncoder1
// counter in an interrupt safe way.
float getRot1Distance(void) {
  float dist;

  cli();
  dist = rot1Steps/stepstoDistance;
  sei();
  return dist;
}

// Return the current value of the rotaryEncoder2
// counter in an interrupt safe way.
float getRot2Distance(void) {
  float dist;

  cli();
  dist = rot2Steps/stepstoDistance;
  sei();
  return dist;
}


// Interrupt routines
void ISRrotA1Change(){
    new_enc1 = (digitalReadFast(rotB1Pin)<<1) | digitalReadFast(rotA1Pin);
    rot1Steps+= QEM[(old_enc1<<2) + new_enc1];
    old_enc1 = new_enc1;
}

void ISRrotB1Change() {
    new_enc1 = (digitalReadFast(rotB1Pin)<<1) | digitalReadFast(rotA1Pin);
    rot1Steps+= QEM[(old_enc1<<2) + new_enc1];
    old_enc1 = new_enc1;
}

// Interrupt routines
void ISRrotA2Change(void) {
    new_enc2 = (digitalReadFast(rotA2Pin)<<1) | digitalReadFast(rotB2Pin);
    rot2Steps+= QEM[(old_enc2<<2) + new_enc2];
    old_enc2 = new_enc2;
}

void ISRrotB2Change()
{
    new_enc2 = (digitalReadFast(rotA2Pin)<<1) | digitalReadFast(rotB2Pin);
    rot2Steps+= QEM[(old_enc2<<2) + new_enc2];
    old_enc2 = new_enc2;
}


// Reference for the step calculations: http://www.robotpark.com/DT/PRO/91112-tutorial-how-to-use-a-quadrature-encoder-rs011a.pdf
