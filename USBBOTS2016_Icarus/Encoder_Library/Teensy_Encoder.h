/*
   Rotary Encoders Speed and Steps calculation
   Teensy_Encoder.h Header file
*/

#ifndef Teensy_Encoder_h
#define Teensy_Encoder_h
#endif

// Rotary encoder 1 phase A pin
#define rotB1Pin 7
// Rotary encoder 1 phase B pin
#define rotA1Pin 8
// Rotary encoder 2 phase A pin
#define rotB2Pin 12
// Rotary encoder 2 phase B pin
#define rotA2Pin 11

// Set this to 1 to see some debug output from the Serial Port
// (If there is the serial port is not open, it will cause errors)
#define DEMO 0

// Constant used for the timer time
#define timerRotationTime 1000000 // This equals 1 Second

// Constant used for the conversion Steps to Distance in cm
#define stepstoDistance 13.5


//------------------Avaliable to user funcions----------------------------
// Configure the interrupts pins and interrupts
void configureEncoderLib(void);

// Return the current value of the speed1
// counter in an interrupt safe way.
float getSpeed1(void);

// Return the current value of the speed2
// counter in an interrupt safe way.
float getSpeed2(void);

// Return the current value of the rotaryEncoder1
// counter in an interrupt safe way.
int getRot1Steps(void);

// Return the current value of the rotaryEncoder2
// counter in an interrupt safe way.
int getRot2Steps(void);

// Return the current distance of the rotaryEncoder1
// counter in an interrupt safe way.
float getRot1Distance(void);

// Return the current distance of the rotaryEncoder2
// counter in an interrupt safe way.
float getRot2Distance(void);

//------------------Avaliable to user funcions----------------------------

//------------------Internal functions------------------------------------
// Interrupt routines
void ISRrotA1Change(void);
void ISRrotB1Change(void);
// Interrupt routines
void ISRrotA2Change(void);
void ISRrotB2Change(void);

// Calculates both speeds from the encoders
void calculateSpeed(void);
//------------------Internal functions------------------------------------
