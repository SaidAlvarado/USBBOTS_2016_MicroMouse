#include "icarus_pinout.h"

uint16_t IR[4];


//USBBOTS2016_Icarus.ino


void setup() {
  irSensorSetup();

  pinMode(13, INPUT);   //Safe mode for onboard led
  wifiSetup();

  Serial.begin(115200);

}  // put your setup code here, to run once:


uint8_t datos;

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

  if (digitalReadFast(WIFI_AUTODETECT)) Serial.println("Wifi desconectado");
  else {
      Serial2.println("hola!");
      Serial.println("Wifi Listo!");
  }


  delay(1000);



}
