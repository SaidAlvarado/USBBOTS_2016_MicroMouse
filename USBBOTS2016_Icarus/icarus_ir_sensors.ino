/*
   Distance calculations
   icarus_ir_sensors.ino  file
*/

#include <ADC.h>
#include "icarus_pinout.h"
#include "math.h"


// Class Variable for ADC hardware access.
ADC *adc = new ADC();

// Variables for data storage
uint16_t valorInfra[4];
uint16_t ambientlight[4];
uint16_t light[4];

//variables de ajuste de los sensores
#define DIST_CALIB_LENGTH_FRONT 44
#define DIST_CALIB_LENGTH_DIAG 39
float dist_calib[] = {1,     1.5,   2,     2.5,   3,     3.5,   4,     4.5,   5,     5.5,   6,     6.5,   7,     7.5,   8,     8.5,   9,     9.5,   10,    11,    12,    13,    14,    15,    16,    17,    18,    19,    20,    21,    22,   23,   24,   25,   26,   27,   28,   29,   30,   31,   32,   33,   34,   35,   36  };
float FL[] =         {60700, 59000, 57820, 55780, 50970, 43900, 39100, 34400, 30800, 27570, 25220, 22800, 21000, 18760, 18405, 17010, 16352, 15590, 14270, 14830, 11550, 10520, 9640,  8810,  8630,  8000,  7540,  7000,  6450,  6000,  5540, 5170, 4760, 4410, 4010, 3650, 3310, 2980, 2690, 2350, 2220, 1900, 1580, 1340, 1080  };
float FR[] =         {62540, 62030, 61650, 61220, 60560, 59000, 57400, 55630, 50160, 44720, 40410, 36500, 33700, 31210, 30020, 27380, 26740, 25660, 24080, 21930, 20270, 18710, 17560, 16140, 15320, 14280, 13290, 12410, 11280, 10520, 9600, 9070, 8390, 7900, 7290, 6780, 6370, 5930, 5600, 5120, 5050, 4720, 4420, 4190, 3880  };
float DL[] =         {61800, 61700, 61300, 60300, 59200, 57600, 55600, 50000, 44200, 37700, 32400, 29100, 26100, 22700, 20400, 19000, 17100, 15200, 13900, 12000, 9800,  8700,  7500,  6700,  6000,  5420,  4980,  4520,  4080,  3800,  3485, 3200, 3000, 2820, 2630, 2450, 2330, 2220, 2120};
float DR[] =         {61800, 61700, 61300, 61200, 60100, 58200, 55600, 53300, 43200, 37400, 32200, 27800, 23700, 21000, 19200, 17200, 15500, 13400, 12200, 11000, 9500,  8000,  7000,  6400,  5680,  5220,  4850,  4460,  4150,  3880,  3650, 3450, 3280, 3140, 3000, 2930, 2800, 2700, 2650};

//Distancias medidas
float distancias[4];

//Wall Threshold
#define right_wall_th 20
#define left_wall_th  20
#define front_wall_lf_th 27
#define front_wall_lf_close_th 8
#define front_wall_rg_th 27
#define front_wall_rg_close_th 8





/* =====================================================================================
                                PUBLIC FUNCTIONS
====================================================================================== */

//Initializes the pins and hardware of the IR drivers and ADC
void irSensorSetup(void) {
  //Serial.begin(9600);  //Inicializar para el test en el lab

  // adc = new ADC();

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


// FUnction that access the hardware
void irRead(void) {

  ambientlight[0] = adc->analogRead(IR_FRONT_LEFT);
  ambientlight[1] = adc->analogRead(IR_DIAG_LEFT);
  ambientlight[2] = adc->analogRead(IR_DIAG_RIGHT);
  ambientlight[3] = adc->analogRead(IR_FRONT_RIGHT);
  // delayMicroseconds(10);
  digitalWriteFast(IR_EMITTER, HIGH);
  // delayMicroseconds(10);
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


// Send ADC values to the code
void getIR(uint16_t *valores) {

    irRead();
    valores[0] = valorInfra[0];
    valores[1] = valorInfra[1];
    valores[2] = valorInfra[2];
    valores[3] = valorInfra[3];
}


// Distance interpolation
void getIRDistance(float *valores) {

    irRead();
    int point;

    // Sensor Front Left
    point = 0;
    for (size_t i = 0; (i < DIST_CALIB_LENGTH_FRONT) && (valorInfra[0] < FL[i]) ; i++) point = i;
    if ((point > 0) && (point < DIST_CALIB_LENGTH_FRONT)) valores[0] = dist_calib[point] + ((valorInfra[0] - FL[point-1]) * (dist_calib[point] - dist_calib[point - 1]) / (FL[point] - FL[point - 1]));
    else if (valorInfra[0] < FL[DIST_CALIB_LENGTH_FRONT - 1]) valores[0] = 1.0;
    else if (valorInfra[0] > FL[0]) valores[0] = 36.0;

    // Sensor Diagonal Left
    point = 0;
    for (size_t i = 0; (i < DIST_CALIB_LENGTH_DIAG) && (valorInfra[1] < DL[i]) ; i++) point = i;
    if ((point > 0) && (point < DIST_CALIB_LENGTH_DIAG)) valores[1] = dist_calib[point] + ((valorInfra[1] - DL[point-1]) * (dist_calib[point] - dist_calib[point - 1]) / (DL[point] - DL[point - 1]));
    else if (valorInfra[1] < FL[DIST_CALIB_LENGTH_DIAG - 1]) valores[1] = 1.0;
    else if (valorInfra[1] > FL[0])valores[1] = 30.0;

    // Sensor Diagonal Right
    point = 0;
    for (size_t i = 0; (i < DIST_CALIB_LENGTH_DIAG) && (valorInfra[2] < DR[i]) ; i++) point = i;
    if ((point > 0) && (point < DIST_CALIB_LENGTH_DIAG)) valores[2] = dist_calib[point] + ((valorInfra[2] - DR[point-1]) * (dist_calib[point] - dist_calib[point - 1]) / (DR[point] - DR[point - 1]));
    else if (valorInfra[2] < FL[DIST_CALIB_LENGTH_DIAG - 1]) valores[2] = 1.0;
    else if (valorInfra[2] > FL[0]) valores[2] = 30.0;

    // Sensor Front Right
    point = 0;
    for (size_t i = 0; (i < DIST_CALIB_LENGTH_FRONT) && (valorInfra[3] < FR[i]) ; i++) point = i;
    if ((point > 0) && (point < DIST_CALIB_LENGTH_FRONT)) valores[3] = dist_calib[point] + ((valorInfra[3] - FR[point-1]) * (dist_calib[point] - dist_calib[point - 1]) / (FR[point] - FR[point - 1]));
    else if (valorInfra[3] < FL[DIST_CALIB_LENGTH_FRONT - 1]) valores[3] = 1.0;
    else if (valorInfra[3] > FL[0]) valores[3] = 36.0;

    // Actualize distancie variables
    distancias[0] = valores[0];
    distancias[1] = valores[1];
    distancias[2] = valores[2];
    distancias[3] = valores[3];
}

// Ge distance
uint8_t isWallRight(void){

    return right_wall_th > distancias[2];
}


uint8_t isWallLeft(void){

    return left_wall_th > distancias[1];
}


uint8_t isWallFront(void){


    if (front_wall_lf_close_th > distancias[0] || front_wall_rg_close_th > distancias[3] ) return 2;
    if (front_wall_lf_th > distancias[0] || front_wall_rg_th > distancias[3] ) return 1;
    return 0;


}
