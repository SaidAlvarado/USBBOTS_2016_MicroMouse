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
#define interruptTiming 10000 //1000 -> 1ms

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
float kpX = 10, kdX = 6; //9,9
float kpW = 80, kdW = 30;
float posErrorX, posErrorW;
float oldPosErrorX, oldPosErrorW;
float posPwmX, posPwmW;
int16_t leftBaseSpeed;
int16_t rightBaseSpeed;


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
    PD_controller();
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
	float gyroFeedback;
	float rotationalFeedback;
    float encoderFeedbackX;
    float encoderFeedbackW;


    /* simple PD loop to generate base speed for both motors */
	encoderFeedbackX = Vmean;
	encoderFeedbackW = Wenc;
	gyroFeedback = AngularSpeed;
	// sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system

	rotationalFeedback = gyroFeedback;//encoderFeedbackW;


	posErrorX = setpointV - encoderFeedbackX;
	posErrorW = setpointW - rotationalFeedback;
	// posErrorX += curSpeedX - encoderFeedbackX;
	// posErrorW += curSpeedW - rotationalFeedback;

	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	leftBaseSpeed = (int16_t)(posPwmX + posPwmW);
	rightBaseSpeed = (int16_t)(posPwmX - posPwmW);

	motorLeftWrite(leftBaseSpeed);
	motorRightWrite(rightBaseSpeed);
}



/* =====================================================================================
                                GETS & SETS
====================================================================================== */

float getVR(void){

    return VR;
}

float getVL(void){

    return VL;
}

float getVmean(void){

    return Vmean;
}

float getDistanceLeft(void){

    return distanceLeft;
}

void setDistanceLeft(float dleft){

    distanceLeft = dleft;
}

//Controller setpoint
void setSetPointV(float spv){

    setpointV = spv;
}

float getSetPointV(void){

    return setpointV;
}

float getSetPointW(void){

    return setpointW;
}

void setSetPointW(float spw){

    setpointW = spw;
}
