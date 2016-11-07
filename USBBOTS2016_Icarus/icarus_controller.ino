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
#define counts_to_mm 2.10    //1.35
int32_t  enc_count_left, enc_count_right;
int32_t  enc_count_left_old, enc_count_right_old;
float VL, VR, Vmean, Wenc; // mm/s
float distanceLeft;

// Gyroscope variables
float AngularSpeed;
float angleLeft;

// Controller variables
float setpointV, setpointW;
float kpX = 20, kdX = 8; //9,9
float kpW = 80, kdW = 70;
float posErrorX, posErrorW;
float oldPosErrorX, oldPosErrorW;
float posPwmX, posPwmW;
int32_t leftBaseSpeed;
int32_t rightBaseSpeed;

// Acceleration profile variables
float targetSpeedV;
float targetSpeedW;
float currentSpeedV;
float currentSpeedW;

//Speed variables
#define moveSpeed 3000
#define inplace_turn_speed 500
#define AccelerationV 200       // mm/s^2
#define AccelerationW 50        // degrees/s^2

// IR sensor distances.
float ir_sensor_distances[4];
float sensorError;
#define sensor_error_trigger 5
#define a_scale 30

// Control variables (flags to shutdown parts of the controller)
uint8_t use_Sensor = 0; // 1 = center on straight with IR SENSORS, 0 = doenst
uint8_t use_Controller = 0; // 1 = Runs PD controller block, 0 = doenst



/* =====================================================================================
                                Functions
====================================================================================== */

// Configures the global variables for data transportation.
// and 1mS interrupt
void beginController(void){
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
    // updateCurrentSpeed();
    AngularSpeed = getAngularVelocity();
    getSensorEror();
    if (use_Controller == 1) PD_controller();
}


void updateCurrentSpeed(void)
{
	if(currentSpeedV < targetSpeedV)
	{
		currentSpeedV += (float)AccelerationV * (interruptTiming/1000) / 1000;
		if(currentSpeedV > targetSpeedV)
			currentSpeedV = targetSpeedV;
	}
	else if(currentSpeedV > targetSpeedV)
	{
		currentSpeedV -= (float)AccelerationV * (interruptTiming/1000) / 1000;
		if(currentSpeedV < targetSpeedV)
			currentSpeedV = targetSpeedV;
	}
	if(currentSpeedW < targetSpeedW)
	{
		currentSpeedW += AccelerationW * (interruptTiming/1000) / 1000;
		if(currentSpeedW > targetSpeedW)
			currentSpeedW = targetSpeedW;
	}
	else if(currentSpeedW > targetSpeedW)
	{
		currentSpeedW -= AccelerationV * (interruptTiming/1000) / 1000;
		if(currentSpeedW < targetSpeedW)
			currentSpeedW = targetSpeedW;
	}
}


void getSensorEror(void)//the very basic case
{
    getIRDistance(ir_sensor_distances);

    if (abs(ir_sensor_distances[1] - ir_sensor_distances[2]) > sensor_error_trigger)    sensorError = ir_sensor_distances[2] - ir_sensor_distances[1];
    else  sensorError = 0;

    if ((isWallRight() == 0) || (isWallLeft() == 0)) sensorError = 0;
}



float needToDecelerate(float dist, float curSpd, float endSpd)//speed are in encoder counts/ms, dist is in encoder counts
{
	if (curSpd<0) curSpd = -curSpd;
	if (endSpd<0) endSpd = -endSpd;
	if (dist<0) dist = 1;//-dist;
	if (dist == 0) dist = 1;  //prevent divide by 0

	return (abs((curSpd*curSpd - endSpd*endSpd)/dist/2)); //dist_counts_to_mm(dist)/2);
	//calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
	//use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
	//because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
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

//  4362.5  1976
//72mm wheel distance
void PD_controller(void) // encoder PD controller
{
	float gyroFeedback;
	float rotationalFeedback;
    float encoderFeedbackX;
    float encoderFeedbackW;
    float sensorFeedback;


    /* simple PD loop to generate base speed for both motors */
	encoderFeedbackX = Vmean;
	encoderFeedbackW = Wenc;
	gyroFeedback = AngularSpeed;

    // integrate the angle
    if (setpointW != 0) angleLeft -= gyroFeedback * (interruptTiming/1000) / 1000;  // (interruptTiming/1000) / 1000 this is deltaT in seconds
    else angleLeft = 0;
    //
	sensorFeedback = sensorError*a_scale;//have sensor error properly scale to fit the system

	rotationalFeedback = gyroFeedback + sensorFeedback * (use_Sensor); //encoderFeedbackW;


	posErrorX = setpointV - encoderFeedbackX;
	posErrorW = setpointW - rotationalFeedback;
	// posErrorX = currentSpeedV - encoderFeedbackX;
	// posErrorW = currentSpeedW - rotationalFeedback;

	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);// * (interruptTiming/1000) / 1000;
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);// * (interruptTiming/1000) / 1000;

    if (posPwmX >= 35000)  posPwmX = 35000;
    if (posPwmX <= -35000) posPwmX = -35000;

    // if (posPwmW >= 5000)  posPwmW = 5000;
    // if (posPwmW <= -5000) posPwmW = -5000;

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	leftBaseSpeed = (int32_t)(posPwmX - posPwmW);
	rightBaseSpeed = (int32_t)(posPwmX + posPwmW);

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

float getAngleLeft(void){

    return angleLeft;
}

void setAngleLeft(float aleft){

    angleLeft = aleft;
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

// Acceleration profile variables
void setTargetSpeedV(float tspv){

    targetSpeedV = tspv;
}

void setTargetSpeedW(float tspw){

    targetSpeedW = tspw;
}

void startController(void){

    use_Controller = 1;
}

void stopController(void){

    use_Controller = 0;
}

void startIRcentering(void){

    use_Sensor = 1;
}

void stopIRcentering(void){

    use_Sensor = 0;
}


// // Acceleration profile variables
// float getCurrentSpeedV(void){
//
//     targetSpeedV;
// }
//
// float getCurrentSpeedW(void){
//
//     targetSpeedW = tspw;
// }
