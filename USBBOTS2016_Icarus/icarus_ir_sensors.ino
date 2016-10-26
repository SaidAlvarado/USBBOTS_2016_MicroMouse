/*
   Distance calculations
   icarus_ir_sensors.ino  file
*/

#include <ADC.h>
#include "icarus_pinout.h"


// Class Variable for ADC hardware access.
ADC *adc = new ADC();

// Variables for data storage
uint16_t valorInfra[4];
uint16_t ambientlight[4];
uint16_t light[4];


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

  ambientlight[0] = adc->analogRead(IR_DIAG_RIGHT);
  ambientlight[1] = adc->analogRead(IR_FRONT_RIGHT);
  ambientlight[2] = adc->analogRead(IR_FRONT_LEFT);
  ambientlight[3] = adc->analogRead(IR_DIAG_LEFT);
  // delayMicroseconds(10);
  digitalWriteFast(IR_EMITTER, HIGH);
  // delayMicroseconds(10);
  delayMicroseconds(50);
  light[0] = adc->analogRead(IR_DIAG_RIGHT);
  light[1] = adc->analogRead(IR_FRONT_RIGHT);
  light[2] = adc->analogRead(IR_FRONT_LEFT);
  light[3] = adc->analogRead(IR_DIAG_LEFT);
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
