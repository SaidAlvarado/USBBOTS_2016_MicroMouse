// #define ENCODER_OPTIMIZE_INTERRUPTS
// #include "Encoder.h"
// //call speedProfile(void) in systick handle to make it execute every one milli second
// //this controller sample code doesn't include
//
// //here are the code you need to call periodically to make the mouse sample and excute and a constant rate
// //in order to make sample and pwm updating at a constant rate, you'd better call the code in the handler of a timer periodically
// //the period for intterupt handler is usually 1ms, which means you have to finish all code excuting within 1ms
// //I simply call the controller code in systick handler that intterupts every 1ms, notice systick timer is also used to
// //keep track the system time in millisecond
// /*
// //test the timing make sure everything finishes within 1ms and leave at least 30% extra time for other code to excute in main routing
// void systickHandler(void)
// {
// 	Millis++;  //keep track of the system time in millisecond
// 	if(bUseIRSensor)
// 		readIRSensor();
// 	if(bUseGyro)
// 		readGyro();
// 	if(bUseSpeedProfile)
// 		speedProfile
// }
// */
//
// //Temporal interrupts
// IntervalTimer controllerTimer;
// #define interruptTiming 1000 //1ms
//
//
// // Convertion Macros
// #define speed_to_counts(a)  (a/1.35) // in mm/s
// #define counts_to_speed(a)  (a*1.35) // in mm/s
//
// // Encoder variables
// Encoder motor_left(11, 12);
// Encoder motor_right(7, 8);
// int32_t leftEncoder = 0, rightEncoder = 0;
// int32_t leftEncoderOld = 0, rightEncoderOld = 0;
// int32_t encoderChange = 0, leftEncoderCount = 0, rightEncoderCount = 0, encoderCount = 0, distanceLeft = 0;
// int32_t leftEncoderChange = 0, rightEncoderChange = 0;
//
// // Sensor controller variables
// #define DLMiddleValue  10000    // value of the sensor to start correcting center position
// #define DRMiddleValue  10000
//
// // Infrared variables
// uint16_t IR[4];
//
// //Gyro variables
// float aSpeed =0;
//
// //IR variables
// int32_t sensorError = 0;
// int a_scale = 0;
//
// //other
// int oneCellDistance = 243;
//
// //ps. if you want to change speed, you simply change the targetSpeedX and targetSpeedW in main routing(int main) at any time.
//
//
// float curSpeedX = 0;
// float curSpeedW = 0;
// int targetSpeedX = 0;
// int targetSpeedW = 0;
// int encoderFeedbackX = 0;
// int encoderFeedbackW = 0;
// float pidInputX = 0;
// float pidInputW = 0;
// float posErrorX = 0;
// float posErrorW = 0;
// float oldPosErrorX = 0;
// float oldPosErrorW = 0;
// int posPwmX = 0;
// int posPwmW = 0;
// float kpX = 2, kdX = 4;
// float kpW = 1, kdW = 12;//used in straight
// float kpW1 = 1;//used for T1 and T3 in curve turn
// float kdW1 = 26;
// float kpW2 = 1;//used for T2 in curve turn
// float kdW2 = 36;
// float accX = 600;//6m/s/s
// float decX = 600;
// float accW = 1; //cm/s^2
// float decW = 1;
//
// //speed to count or count to speed are the macro or function to make the unit conversion
// // between encoder_counts/ms and mm/ms or any practical units you use.
// int moveSpeed = speed_to_counts(100*2);
// int turnSpeed = speed_to_counts(500*2);
// int returnSpeed = speed_to_counts(500*2);
// int stopSpeed = speed_to_counts(100*2);
// int maxSpeed = speed_to_counts(2000*2);
// // int moveSpeed = speed_to_counts(500*2);
// // int turnSpeed = speed_to_counts(500*2);
// // int returnSpeed = speed_to_counts(500*2);
// // int stopSpeed = speed_to_counts(100*2);
// // int maxSpeed = speed_to_counts(2000*2);
//
//
// int gyroFeedbackRatio = 589;//5900;  589.462
//
//
// void startController(void)
// {
//     resetSpeedProfile();
//     controllerTimer.begin(speedProfile, interruptTiming);
//
// }
//
//
// void speedProfile(void)
// {
//     // Read gyro
//     aSpeed =  getAngularVelocity();
//     // Read encoders
// 	getEncoderStatus();
//     // Calculate current profile
// 	updateCurrentSpeed();
//     // PD controller
// 	calculateMotorPwm();
// }
//
//
// void getEncoderStatus(void)
// {
// 	leftEncoder = motor_left.read();//read current encoder ticks from register of 32 bit general purpose timer 2
// 	rightEncoder = motor_right.read();//read current encoder ticks from register of 32 bit general purpose timer 5
//
// 	leftEncoderChange = leftEncoder - leftEncoderOld;
// 	rightEncoderChange = rightEncoder - rightEncoderOld;
// 	encoderChange = (leftEncoderChange + rightEncoderChange)/2;
//
// 	leftEncoderOld = leftEncoder;
// 	rightEncoderOld = rightEncoder;
//
// 	leftEncoderCount += leftEncoderChange;
// 	rightEncoderCount += rightEncoderChange;
// 	encoderCount =  (leftEncoderCount+rightEncoderCount)/2;
//
// 	distanceLeft -= encoderChange;// update distanceLeft
// }
//
//
//
// void updateCurrentSpeed(void)
// {
// 	if(curSpeedX < targetSpeedX)
// 	{
// 		curSpeedX += (float)(speed_to_counts(accX*2)/100);
// 		if(curSpeedX > targetSpeedX)
// 			curSpeedX = targetSpeedX;
// 	}
// 	else if(curSpeedX > targetSpeedX)
// 	{
// 		curSpeedX -= (float)speed_to_counts(decX*2)/100;
// 		if(curSpeedX < targetSpeedX)
// 			curSpeedX = targetSpeedX;
// 	}
// 	if(curSpeedW < targetSpeedW)
// 	{
// 		curSpeedW += accW;
// 		if(curSpeedW > targetSpeedW)
// 			curSpeedW = targetSpeedW;
// 	}
// 	else if(curSpeedW > targetSpeedW)
// 	{
// 		curSpeedW -= decW;
// 		if(curSpeedW < targetSpeedW)
// 			curSpeedW = targetSpeedW;
// 	}
// }
//
// //72mm wheel distance
// void calculateMotorPwm(void) // encoder PD controller
// {
// 	int gyroFeedback;
// 	int rotationalFeedback;
// 	int sensorFeedback;
//
//     /* simple PD loop to generate base speed for both motors */
// 	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
// 	encoderFeedbackW = rightEncoderChange - leftEncoderChange;
//
// 	gyroFeedback = aSpeed/gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture
// 	sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system
//
// 	// if(onlyUseGyroFeedback)
// 	// 	rotationalFeedback = gyroFeedback;
// 	// else if(onlyUseEncoderFeedback)
// 	// 	rotationalFeedback = encoderFeedbackW;
// 	// else
// 		rotationalFeedback = encoderFeedbackW + gyroFeedback;
// 	    //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
// 	    //make sure to check the sign of sensor error.
//
// 	posErrorX += curSpeedX - encoderFeedbackX;
// 	posErrorW += curSpeedW - rotationalFeedback;
//
// 	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
// 	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
//
// 	oldPosErrorX = posErrorX;
// 	oldPosErrorW = posErrorW;
//
// 	int16_t leftBaseSpeed = posPwmX - posPwmW;
// 	int16_t rightBaseSpeed = posPwmX + posPwmW;
//
// 	motorLeftWrite(leftBaseSpeed);
// 	motorRightWrite(rightBaseSpeed);
// }
//
//
// float needToDecelerate(float dist, float curSpd, float endSpd)//speed are in encoder counts/ms, dist is in encoder counts
// {
// 	if (curSpd<0) curSpd = -curSpd;
// 	if (endSpd<0) endSpd = -endSpd;
// 	if (dist<0) dist = 1;//-dist;
// 	if (dist == 0) dist = 1;  //prevent divide by 0
//
// 	return (abs((curSpd*curSpd - endSpd*endSpd)/dist/2)); //dist_counts_to_mm(dist)/2);
// 	//calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
// 	//use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
// 	//because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
// }
//
// void resetSpeedProfile(void)
// {
// 	//resetEverything;
//
//
// 	motorLeftWrite(0);
// 	motorRightWrite(0);
//
// 	pidInputX = 0;
// 	pidInputW = 0;
// 	curSpeedX = 0;
// 	curSpeedW = 0;
// 	targetSpeedX = 0;
// 	targetSpeedW = 0;
// 	posErrorX = 0;
// 	posErrorW = 0;
// 	oldPosErrorX = 0;
// 	oldPosErrorW = 0;
//     leftEncoderOld = 0;
// 	rightEncoderOld = 0;
// 	leftEncoder = 0;
// 	rightEncoder = 0;
// 	leftEncoderCount = 0;
// 	rightEncoderCount = 0;
// 	encoderCount = 0;
// 	// oldEncoderCount = 0;
// 	// leftBaseSpeed = 0;
// 	// rightBaseSpeed = 0;
//
// 	motor_left.write(0);//reset left encoder count
// 	motor_right.write(0);//reset right encoder count
// }
//
//
//
// void getSensorEror(void)//the very basic case
// {
// 	if(IR[1] > DLMiddleValue && IR[2] < DRMiddleValue)
// 		sensorError = DLMiddleValue - IR[1];
// 	else if(IR[2] > DRMiddleValue && IR[1] < DLMiddleValue)
// 		sensorError = IR[2] - DRMiddleValue;
// 	else
// 		sensorError = 0;
// }
//
//
// /*
// sample code for straight movement
// */
// // void moveOneCell(void)
// // {
// // 	// enable_sensor(),
// // 	// enable_gyro();
// // 	// enable_PID();
// //
// // 	targetSpeedW = 0;
// // 	targetSpeedX = moveSpeed;
// //
// // 	do
// // 	{
// // 		/*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)
// // 		here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
// // 		/*sample*/
// // 		if(needToDecelerate(distanceLeft, curSpeedX, moveSpeed) < decX)
// // 			targetSpeedX = maxSpeed;
// // 		else
// // 			targetSpeedX = moveSpeed;
// //
// // 		//there is something else you can add here. Such as detecting falling edge of post to correct longitudinal position of mouse when running in a straight path
// // 	}
// // 	while(  (encoderCount-oldEncoderCount) < oneCellDistance );
// // 	//LFvalues1 and RFvalues1 are the front wall sensor threshold when the center of mouse between the boundary of the cells.
// // 	//LFvalues2 and RFvalues2 are the front wall sensor threshold when the center of the mouse staying half cell farther than LFvalues1 and 2
// // 	//and LF/RFvalues2 are usually the threshold to determine if there is a front wall or not. You should probably move this 10mm closer to front wall when collecting
// // 	//these thresholds just in case the readings are too weak.
// //
// // 	oldEncoderCount = encoderCount;	//update here for next movement to minimized the counts loss between cells.
// // }
