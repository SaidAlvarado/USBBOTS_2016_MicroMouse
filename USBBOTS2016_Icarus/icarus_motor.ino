#include <Arduino.h>

#include "icarus_pinout.h"

/* =====================================================================================
                    MOTOR DRIVER IC (DRV8871)
====================================================================================== */

// This Functions lets you interface with the motor driver and modify the duty cycle
// to rotate the motors forward, backward, or brake them. Using the max resolutioin of the Teensy 3.2 PWM (16bits)


// Configures pins and register for using the motors
void motorSetup(){

  // Set Right motor pins as output
  pinMode(MOTOR1_PWM1, OUTPUT);
  pinMode(MOTOR1_PWM2, OUTPUT);

  // Set Left motor pins as output
  pinMode(MOTOR2_PWM1, OUTPUT);
  pinMode(MOTOR2_PWM2, OUTPUT);

  //Set resolution to maximum, 16 bits
  analogWriteResolution(16);

  // Initialize motors turned off
  analogWrite(MOTOR1_PWM1, 0);
  analogWrite(MOTOR1_PWM2, 0);
  analogWrite(MOTOR2_PWM1, 0);
  analogWrite(MOTOR2_PWM2, 0);
}


// Changes the PWM of the Left motor
// duty_cycle > 0    FORWARD
// duty_cycle < 0    BACKWARD
// duty_cycle == 0   BRAKE
void motorLeftWrite(int32_t duty_cycle){

  if (duty_cycle > 0){ //go forward
    analogWrite(MOTOR1_PWM2, 65535);
    analogWrite(MOTOR1_PWM1, 65535 - duty_cycle);
  }

  if (duty_cycle < 0){ //go backwards
    analogWrite(MOTOR1_PWM2, 65535 + duty_cycle);
    analogWrite(MOTOR1_PWM1, 65535);
  }

  if (duty_cycle == 0){ //Stop
    analogWrite(MOTOR1_PWM2, 65535);
    analogWrite(MOTOR1_PWM1, 65535);
  }
}

// Changes the PWM of the Right motor
// duty_cycle > 0    FORWARD
// duty_cycle < 0    BACKWARD
// duty_cycle == 0   BRAKE
void motorRightWrite(int32_t duty_cycle){

  if (duty_cycle > 0){ // Go forward
    analogWrite(MOTOR2_PWM2, 65535);
    analogWrite(MOTOR2_PWM1, 65535 - duty_cycle);       // this is a compensation so that a higher PWM value equals a higher speed.
  }

  if (duty_cycle < 0){ // Go backwards
    analogWrite(MOTOR2_PWM2, 65535 + duty_cycle);
    analogWrite(MOTOR2_PWM1, 65535);
  }

  if (duty_cycle == 0){ // Brake
    analogWrite(MOTOR2_PWM2, 65535);
    analogWrite(MOTOR2_PWM1, 65535);
  }
}
