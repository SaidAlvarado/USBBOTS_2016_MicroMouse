#include "icarus_pinout.h"
#include "Teensy_Encoder.h"

/*
   PD controller - and 1mS interrupt handler
*/

/* =====================================================================================
                                GLOBAL VARIABLES
====================================================================================== */

// Temporal interrupts
IntervalTimer controllerTimer;
#define interruptTiming 1000 //1000 -> 1ms

// Encoder variables.
#define counts_to_mm 1.35
int32_t  enc_count_left, enc_count_right;
int32_t  enc_count_left_old, enc_count_right_old;
float VL, VR, Vmean, Wenc; // mm/s
float distanceLeft;

// Gyroscope variables
float AngularSpeed;

// Controller variables
float setpointV, setpointW;
float kpX = 2, kdX = 4;
float kpW = 2, kdW = 4;


/* =====================================================================================
                                Functions
====================================================================================== */

// Configures the global variables for data transportation.
// and 1mS interrupt
void startController(void){
    // resetSpeedProfile();
    configureEncoderLib();
    controllerTimer.begin(temporalIntHandler, interruptTiming);

}

// Interrupt handler, this gets called every interruptTiming
// This reads the gyroscope
// Gets the encoder status
// Updates the speed curve
// Runs the PD controller
void temporalIntHandler(void){
    updateEncoderStatus();
    AngularSpeed = getAngularVelocity();
}


void updateEncoderStatus(void){

    // Read the encoder count
    enc_count_left  = getRot1Steps();
    enc_count_right = getRot2Steps();

    // Calculate wheel speeds in mm/s
    VR = (enc_count_right - enc_count_right_old) / counts_to_mm * 1000 * 1000/interruptTiming; // the *1000 is to convert from mm/ms to mm/s
    VL = (enc_count_left - enc_count_left_old)   / counts_to_mm * 1000 * 1000/interruptTiming;
    Vmean = (VR + VL)/2;        // Mean speed
    Wenc = VR - VL;             // Angular velocity measured by the encoders in degree/s

    // Update variables
    enc_count_right_old = enc_count_right;
    enc_count_left_old = enc_count_left;

    if (distanceLeft > 0) distanceLeft -= Vmean * (interruptTiming/1000) / 1000;  // (interruptTiming/1000) / 1000 this is deltaT in seconds
    else distanceLeft = 0;
}

//72mm wheel distance
void PD_controller(void) // encoder PD controller
{
	int gyroFeedback;
	int rotationalFeedback;
	int sensorFeedback;

    /* simple PD loop to generate base speed for both motors */
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

	gyroFeedback = aSpeed/gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture
	sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system

	// if(onlyUseGyroFeedback)
	// 	rotationalFeedback = gyroFeedback;
	// else if(onlyUseEncoderFeedback)
	// 	rotationalFeedback = encoderFeedbackW;
	// else
		rotationalFeedback = encoderFeedbackW + gyroFeedback;
	    //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
	    //make sure to check the sign of sensor error.

	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW += curSpeedW - rotationalFeedback;

	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	int16_t leftBaseSpeed = posPwmX - posPwmW;
	int16_t rightBaseSpeed = posPwmX + posPwmW;

	motorLeftWrite(leftBaseSpeed);
	motorRightWrite(rightBaseSpeed);
}



/* =====================================================================================
                                GETS & SETS
====================================================================================== */

int32_t getVR(void){

    return VR;
}

int32_t getVL(void){

    return VL;
}

int32_t getVmean(void){

    return Vmean;
}

float getDistanceLeft(void){

    return distanceLeft;
}

void setDistanceLeft(float dleft){

    distanceLeft = dleft;
}
