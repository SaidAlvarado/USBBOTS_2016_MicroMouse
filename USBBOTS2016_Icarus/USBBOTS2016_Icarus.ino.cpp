# 1 "/tmp/tmpAE5n2x"
#include <Arduino.h>
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/USBBOTS2016_Icarus.ino"
#include "icarus_pinout.h"
#include "Teensy_Encoder.h"



float dist[4];
uint16_t irval[4];

extern float posErrorX, posErrorW;
extern float posPwmX, posPwmW;
extern int32_t leftBaseSpeed;
extern int32_t rightBaseSpeed;
void setup();
void loop();
uint8_t floodMaze(uint8_t goal_x, uint8_t goal_y);
void generatePath(uint8_t start_x, uint8_t start_y, char* out_path);
void floodAndFill(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y, char* out_path);
void saveMaze(void);
void restoreMaze(void);
void startController(void);
void temporalIntHandler(void);
void updateEncoderStatus(void);
void PD_controller(void);
float getVR(void);
float getVL(void);
float getVmean(void);
float getDistanceLeft(void);
void setDistanceLeft(float dleft);
void setSetPointV(float spv);
float getSetPointV(void);
float getSetPointW(void);
void setSetPointW(float spw);
void gyroSetup(void);
int16_t gyroReadCorrected(void);
void gyroCalibration(void);
void setAngle(float z_angle);
float getAngle(void);
float getAngularVelocity(void);
int16_t gyroRead(int16_t gyro_bias_error);
uint8_t i2c_read_byte(uint8_t address, uint8_t reg);
void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value);
int16_t i2c_read_word(uint8_t address, uint8_t reg);
void irSensorSetup(void);
void irRead(void);
void getIR(uint16_t *valores);
uint8_t isWallRight(void);
uint8_t isWallLeft(void);
uint8_t isWallFront(void);
void motorSetup();
void motorLeftWrite(int32_t duty_cycle);
void motorRightWrite(int32_t duty_cycle);
void wifiSetup();
#line 16 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/USBBOTS2016_Icarus.ino"
void setup() {

  pinMode(BTN_SPEED, INPUT);
  wifiSetup();
  irSensorSetup();
  motorSetup();
  gyroSetup();
  startController();

  motorLeftWrite(0);
  motorRightWrite(0);
  Serial.begin(115200);




}


uint8_t data;

void loop() {


    Serial2.print("distanceLeft = ");
    Serial2.print(getDistanceLeft());
    Serial2.print("\t");

    Serial2.print("posErrorX = ");
    Serial2.print(posErrorX);
    Serial2.print("\t");

    Serial2.print("posPwmX = ");
    Serial2.print(posPwmX);
    Serial2.print("\t");

    Serial2.print("posPwmW = ");
    Serial2.print(posPwmW);
    Serial2.print("\t");

    Serial2.print("PWM_L = ");
    Serial2.print(leftBaseSpeed);
    Serial2.print("\t");

    Serial2.print("PWM_R = ");
    Serial2.print(rightBaseSpeed);
    Serial2.print("\t");

    Serial2.print("Vmean = ");
    Serial2.print(getVmean());
    Serial2.print("mm/s    ");


    Serial2.print("   ");
    Serial2.print(getRot1Steps());
    Serial2.print(",");
    Serial2.print(getRot2Steps());
    Serial2.println("");

    if (getDistanceLeft() == 0) setSetPointV(0);


    if (Serial2.available()) {

        data = Serial2.read();

        if (data == '0') {
            setSetPointV(0);
            setDistanceLeft(0);
        }

        if (data == '1') {
            setSetPointV(3000);
            setDistanceLeft(2000);
        }

        if (data == 'g') {
            gyroCalibration();
        }

    }

    delay(20);

}
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/Flood_and_Fill.ino"
#include "Arduino.h"
#include <EEPROM.h>
# 36 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/Flood_and_Fill.ino"
#define NORTH 1
#define EAST 2
#define SOUTH 4
#define WEST 8
#define VISITED 16
#define ONROUTE 32

#define NOTNORTH (255-NORTH)
#define NOTEAST (255-EAST)
#define NOTSOUTH (255-SOUTH)
#define NOTWEST (255-WEST)
#define ALLWALLS (NORTH|EAST|SOUTH|WEST)


#define NUMCELLS 256

uint8_t maze[NUMCELLS] = { 12, 8, 8, 8, 8, 8, 8, 8, 8, 9, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            6, 2, 2, 2, 2, 2, 2, 2, 2, 3, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t maps[NUMCELLS];


#define MAX_PATH_LENGTH 150
char path[MAX_PATH_LENGTH];
# 104 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/Flood_and_Fill.ino"
uint8_t floodMaze(uint8_t goal_x, uint8_t goal_y)
{
    uint8_t i,j;
    uint8_t now,next;
    uint8_t passes;
    uint8_t cellwalls;
    uint8_t changed;


    i = 0;
    do{
        maps[i--] = 255;
    } while (i);


    maps[goal_x + goal_y*16]=0;
    passes = 0;
    now = 0;
    next = now+1;


    do
    {
        changed = 0;
        i = 0;
        do
        {
            if (maps[i]==now)
            {
                cellwalls=maze[i];
                if ((cellwalls & NORTH) == 0)
                {
                    j = i+1;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
                if ((cellwalls & EAST) == 0)
                {
                    j = i + 16;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
                if ((cellwalls & SOUTH) == 0)
                {
                    j = i + 255;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
                if ((cellwalls & WEST) == 0)
                {
                    j = i + 240;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
            }
            i--;
        } while(i);
        now = now+1;
        next = now+1;
        passes++;
    } while(changed);
    return passes;
}


void generatePath(uint8_t start_x, uint8_t start_y, char* out_path){


    uint8_t current_cell, index = 0;

    uint8_t go_north=0, go_east=0, go_west=0, go_south=0;
    char orientation = 'S';


    current_cell = start_x + 16*start_y;


    while(maps[current_cell] > 0){
# 193 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/Flood_and_Fill.ino"
        go_north = 0;
        go_east = 0;
        go_west = 0;
        go_south = 0;




        if (maps[current_cell + 1 ] == maps[current_cell] - 1 ) go_north = 1;


        if (maps[current_cell + 16 ] == maps[current_cell] - 1 ) go_east = 1;


        if (current_cell >= 16)
        if (maps[current_cell - 16 ] == maps[current_cell] - 1 ) go_west = 1;


        if (current_cell >= 0)
        if (maps[current_cell - 1 ] == maps[current_cell] - 1 ) go_south = 1;





        if (orientation == 'N'){


            if (go_north) {

                orientation = 'N';
                out_path[index++] = 'F';

                current_cell += 1;
            }

            else if (go_east) {

                orientation = 'E';
                out_path[index++] = 'R';

                current_cell += 16;
            }

            else if (go_west) {

                orientation = 'W';
                out_path[index++] = 'L';

                current_cell -= 16;
            }

            else if (go_south) {

                orientation = 'S';
                out_path[index++] = 'G';
                out_path[index++] = 'F';

                current_cell -= 1;
            }
        }


        if (orientation == 'E'){


            if (go_east) {

                orientation = 'E';
                out_path[index++] = 'F';

                current_cell += 16;
            }

            else if (go_south) {

                orientation = 'S';
                out_path[index++] = 'R';

                current_cell -= 1;
            }

            else if (go_north) {

                orientation = 'N';
                out_path[index++] = 'L';

                current_cell += 1;
            }

            else if (go_west) {

                orientation = 'W';
                out_path[index++] = 'G';
                out_path[index++] = 'F';

                current_cell -= 16;
            }
        }


        if (orientation == 'W'){


            if (go_west) {

                orientation = 'W';
                out_path[index++] = 'F';

                current_cell -= 16;
            }

            else if (go_north) {

                orientation = 'N';
                out_path[index++] = 'R';

                current_cell += 1;
            }

            else if (go_south) {

                orientation = 'S';
                out_path[index++] = 'L';

                current_cell -= 1;
            }

            else if (go_east) {

                orientation = 'E';
                out_path[index++] = 'G';
                out_path[index++] = 'F';

                current_cell += 16;
            }
        }


        if (orientation == 'S'){


            if (go_south) {

                orientation = 'S';
                out_path[index++] = 'F';

                current_cell -= 1;
            }

            else if (go_west) {

                orientation = 'W';
                out_path[index++] = 'R';

                current_cell -= 16;
            }

            else if (go_east) {

                orientation = 'E';
                out_path[index++] = 'L';

                current_cell += 16;
            }

            else if (go_north) {

                orientation = 'N';
                out_path[index++] = 'G';
                out_path[index++] = 'F';

                current_cell += 1;
            }
        }

    }


    out_path[index++] = 'S';
    out_path[index++] = '\0';
}


void floodAndFill(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y, char* out_path){

    int passes;


    for (size_t i = 0; i < MAX_PATH_LENGTH; i++) {
        out_path[i] = 0;
    }


    passes = floodMaze(goal_x,goal_y);
    generatePath(start_x,start_y,out_path);

}


void saveMaze(void){

    for (size_t i = 0; i < NUMCELLS; i++) {
        EEPROM.write(i, maze[i]);
    }
}


void restoreMaze(void){

    for (size_t i = 0; i < NUMCELLS; i++) {
        maze[i] = EEPROM.read(i);
    }
}
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/controller-sample.ino"
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_controller.ino"
#include "icarus_pinout.h"
#include "Teensy_Encoder.h"
# 13 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_controller.ino"
IntervalTimer controllerTimer;
#define interruptTiming 1000


#define counts_to_mm 2.10
int32_t enc_count_left, enc_count_right;
int32_t enc_count_left_old, enc_count_right_old;
float VL, VR, Vmean, Wenc;
float distanceLeft;


float AngularSpeed;


float setpointV, setpointW;
float kpX = 20, kdX = 6;
float kpW = 80, kdW = 50;
float posErrorX, posErrorW;
float oldPosErrorX, oldPosErrorW;
float posPwmX, posPwmW;
int32_t leftBaseSpeed;
int32_t rightBaseSpeed;
# 43 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_controller.ino"
void startController(void){

    configureEncoderLib();
    controllerTimer.begin(temporalIntHandler, interruptTiming);

}






void temporalIntHandler(void){
    updateEncoderStatus();
    AngularSpeed = getAngularVelocity();
    PD_controller();
}


void updateEncoderStatus(void){


    enc_count_left = getRot1Steps();
    enc_count_right = getRot2Steps();


    VR = (enc_count_right - enc_count_right_old) / counts_to_mm * 1000 * 1000/interruptTiming;
    VL = (enc_count_left - enc_count_left_old) / counts_to_mm * 1000 * 1000/interruptTiming;
    Vmean = (VR + VL)/2;
    Wenc = VR - VL;


    enc_count_right_old = enc_count_right;
    enc_count_left_old = enc_count_left;

    if (distanceLeft > 0) distanceLeft -= Vmean * (interruptTiming/1000) / 1000;
    else distanceLeft = 0;
}



void PD_controller(void)
{
 float gyroFeedback;
 float rotationalFeedback;
    float encoderFeedbackX;
    float encoderFeedbackW;



 encoderFeedbackX = Vmean;
 encoderFeedbackW = Wenc;
 gyroFeedback = AngularSpeed;


 rotationalFeedback = gyroFeedback;


 posErrorX = setpointV - encoderFeedbackX;
 posErrorW = setpointW - rotationalFeedback;



 posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
 posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);

    if (posPwmX >= 35000) posPwmX = 35000;
    if (posPwmX <= -35000) posPwmX = -35000;




 oldPosErrorX = posErrorX;
 oldPosErrorW = posErrorW;

 leftBaseSpeed = (int32_t)(posPwmX - posPwmW);
 rightBaseSpeed = (int32_t)(posPwmX + posPwmW);

 motorLeftWrite(leftBaseSpeed);
 motorRightWrite(rightBaseSpeed);
}







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
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_gyroscope.ino"

#include "Arduino.h"

#include <i2c_t3.h>
#include <math.h>
# 19 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_gyroscope.ino"
    #define GYRO_ADDR 0x20







    #define GYRO_REGISTER_CTRL_REG0 0x0D
    #define GYRO_REGISTER_CTRL_REG1 0x13
    #define GYRO_REGISTER_CTRL_REG2 0x14
    #define GYRO_REGISTER_CTRL_REG3 0x15
    #define GYRO_REGISTER_OUT_Z_MSB 0x05
    #define GYRO_REGISTER_STATUS 0x00







    #define LSBtoDEG 0.125

    int16_t gyro_bias_z = 0;
    float gyro_angle = 0;

    elapsedMicros gyro_timer;
# 56 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_gyroscope.ino"
void gyroSetup(void){

    int32_t bias_accumulator = 0;

    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_600);
    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG0, 0x02);
    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG1, 0x03);


    gyroCalibration();


    gyro_timer = 0;
}




int16_t gyroReadCorrected(void){

    int16_t Z_AngSpeed;


    Z_AngSpeed = i2c_read_word(GYRO_ADDR, GYRO_REGISTER_OUT_Z_MSB);

    Z_AngSpeed = Z_AngSpeed>>3;

    Z_AngSpeed -= gyro_bias_z;



    gyro_angle += Z_AngSpeed*LSBtoDEG*(gyro_timer/1000000.0);

    gyro_timer = 0;

    return Z_AngSpeed;
}


void gyroCalibration(void){

    int32_t bias_accumulator = 0;

    for (size_t i = 0; i < 16; i++) {
        bias_accumulator += gyroRead(0);
    }
    gyro_bias_z = (int16_t)(bias_accumulator >> 4);
}


void setAngle(float z_angle){

    gyro_angle = z_angle;
}


float getAngle(void){

    return gyro_angle;
}


float getAngularVelocity(void){

    int16_t angvel;

    angvel = gyroReadCorrected();
    return angvel*LSBtoDEG;
}






int16_t gyroRead(int16_t gyro_bias_error){

    int16_t Z_AngSpeed;


    Z_AngSpeed = i2c_read_word(GYRO_ADDR, GYRO_REGISTER_OUT_Z_MSB);

    Z_AngSpeed = Z_AngSpeed>>3;
    Z_AngSpeed -= gyro_bias_error;

    return Z_AngSpeed;
}






uint8_t i2c_read_byte(uint8_t address, uint8_t reg){


    uint8_t value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(address, 1, I2C_STOP);
    value = Wire.readByte();
    Wire.endTransmission();

    return value;
}


void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value){


    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}


int16_t i2c_read_word(uint8_t address, uint8_t reg){


    uint8_t value1, value2;
    int16_t value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(address, 2, I2C_STOP);
    while (Wire.available() < 2);
        value1 = Wire.readByte();
        value2 = Wire.readByte();
    Wire.endTransmission();


    value = ((value1 << 8) & 0xFF00) | (value2 & 0x00FF);
    return value;
}
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_ir_sensors.ino"





#include <ADC.h>
#include "icarus_pinout.h"
#include "math.h"



ADC *adc = new ADC();


uint16_t valorInfra[4];
uint16_t ambientlight[4];
uint16_t light[4];


float FL[3] = {6790, 105098, 0.28172};
float DL[3] = {3555, 58459, 0.34614};
float DR[3] = {5853, 7247, 0.67896};
float FR[3] = {12850, 133321, 0.26744};


float distancias[4];


#define right_wall_th 5000
#define left_wall_th 8000
#define front_wall_lf_th 3000
#define front_wall_lf_close_th 25000
#define front_wall_rg_th 5500
#define front_wall_rg_close_th 35000
# 45 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_ir_sensors.ino"
void irSensorSetup(void) {




  pinMode(IR_EMITTER, OUTPUT );
  digitalWriteFast(IR_EMITTER, LOW);

  pinMode( IR_DIAG_RIGHT, INPUT );
  pinMode( IR_DIAG_LEFT, INPUT );
  pinMode( IR_FRONT_RIGHT , INPUT );
  pinMode( IR_FRONT_LEFT , INPUT );

  adc->setAveraging(4, ADC_0);
  adc->setResolution(16, ADC_0);
  adc->setConversionSpeed(ADC_VERY_HIGH_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_VERY_HIGH_SPEED, ADC_0);

}



void irRead(void) {

  ambientlight[0] = adc->analogRead(IR_FRONT_LEFT);
  ambientlight[1] = adc->analogRead(IR_DIAG_LEFT);
  ambientlight[2] = adc->analogRead(IR_DIAG_RIGHT);
  ambientlight[3] = adc->analogRead(IR_FRONT_RIGHT);

  digitalWriteFast(IR_EMITTER, HIGH);

  delayMicroseconds(50);
  light[0] = adc->analogRead(IR_FRONT_LEFT);
  light[1] = adc->analogRead(IR_DIAG_LEFT);
  light[2] = adc->analogRead(IR_DIAG_RIGHT);
  light[3] = adc->analogRead(IR_FRONT_RIGHT);
  digitalWriteFast(IR_EMITTER, LOW);
  valorInfra[0] = light[0] - ambientlight[0];
  valorInfra[1] = light[1] - ambientlight[1];
  valorInfra[2] = light[2] - ambientlight[2];
  valorInfra[3] = light[3] - ambientlight[3];
}



void getIR(uint16_t *valores) {

    irRead();
    valores[0] = valorInfra[0];
    valores[1] = valorInfra[1];
    valores[2] = valorInfra[2];
    valores[3] = valorInfra[3];
}


uint8_t isWallRight(void){

    return right_wall_th < valorInfra[2];
}


uint8_t isWallLeft(void){

    return left_wall_th < valorInfra[1];
}


uint8_t isWallFront(void){


    if (front_wall_lf_close_th < valorInfra[0] && front_wall_rg_close_th < valorInfra[3] ) return 2;
    if (front_wall_lf_th < valorInfra[0] && front_wall_rg_th < valorInfra[3] ) return 1;
    return 0;


}
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_motor.ino"
#include <Arduino.h>

#include "icarus_pinout.h"
# 14 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_motor.ino"
void motorSetup(){


  pinMode(MOTOR1_PWM1, OUTPUT);
  pinMode(MOTOR1_PWM2, OUTPUT);


  pinMode(MOTOR2_PWM1, OUTPUT);
  pinMode(MOTOR2_PWM2, OUTPUT);


  analogWriteResolution(16);


  analogWrite(MOTOR1_PWM1, 0);
  analogWrite(MOTOR1_PWM2, 0);
  analogWrite(MOTOR2_PWM1, 0);
  analogWrite(MOTOR2_PWM2, 0);
}






void motorLeftWrite(int32_t duty_cycle){

  if (duty_cycle > 0){
    analogWrite(MOTOR2_PWM2, 65535 - duty_cycle);
    analogWrite(MOTOR2_PWM1, 65535);
  }

  if (duty_cycle < 0){
    analogWrite(MOTOR2_PWM2, 65535);
    analogWrite(MOTOR2_PWM1, 65535 + duty_cycle);
  }

  if (duty_cycle == 0){
    analogWrite(MOTOR2_PWM2, 65535);
    analogWrite(MOTOR2_PWM1, 65535);
  }
}





void motorRightWrite(int32_t duty_cycle){

  if (duty_cycle > 0){
    analogWrite(MOTOR1_PWM2, 65535 - duty_cycle);
    analogWrite(MOTOR1_PWM1, 65535);
  }

  if (duty_cycle < 0){
    analogWrite(MOTOR1_PWM2, 65535);
    analogWrite(MOTOR1_PWM1, 65535 + duty_cycle);
  }

  if (duty_cycle == 0){
    analogWrite(MOTOR1_PWM2, 65535);
    analogWrite(MOTOR1_PWM1, 65535);
  }
}
# 1 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_wifi.ino"
#include <Arduino.h>

#include "icarus_pinout.h"
# 20 "/home/said/Arduino/USBBOTS2016_Icarus/USBBOTS2016_Icarus/icarus_wifi.ino"
void wifiSetup() {


    Serial2.begin(115200);


    pinMode(WIFI_AUTODETECT, INPUT);
    pinMode(WIFI_RST, OUTPUT);
    pinMode(WIFI_BOOT, OUTPUT);


    digitalWriteFast(WIFI_BOOT, HIGH);
    digitalWriteFast(WIFI_RST, HIGH);

}