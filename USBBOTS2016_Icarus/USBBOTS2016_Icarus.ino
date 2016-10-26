#include "icarus_pinout.h"

uint16_t IR[4];


//USBBOTS2016_Icarus.ino


void setup() {
  irSensorSetup();

  pinMode(13, INPUT);   //Safe mode for onboard led
  wifiSetup();

  Serial.begin(115200);

}  // put your setup code here, to run once:




void loop() {

  //
  // getIR(IR);
  // Serial.println(IR[0]);
  // Serial.println(IR[1]);
  // Serial.println(IR[2]);
  // Serial.println(IR[3]);
  // Serial.println(" ");
  //
  //
  // delay(100);


  if (Serial2.available()) {
    datos = Serial2.read();
    Serial.print(datos);
  }

  if (Serial.available()) {
    datos = Serial.read();
    Serial2.print(datos);
  }



}
