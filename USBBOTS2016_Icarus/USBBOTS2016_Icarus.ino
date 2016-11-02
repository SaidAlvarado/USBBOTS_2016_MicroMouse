#include "icarus_pinout.h"
#include "Teensy_Encoder.h"

uint16_t IR[4];


//USBBOTS2016_Icarus.ino

int32_t PWM;

void setup() {

  configureEncoderLib();
  pinMode(BTN_SPEED, INPUT);   //Safe mode for onboard led
  wifiSetup();
  motorSetup();

  saveMaze();

  Serial.begin(115200);

  Serial.println("PWM,step1,step2");

  for (PWM = 0; PWM < 65535; PWM = PWM + 1000) {

        motorLeftWrite(PWM);
        motorRightWrite(PWM);

        setRot1(0);
        setRot2(0);

        Serial.print(PWM);
        Serial.print(",");
        delay(1000);

        Serial.print(getRot1Steps());
        Serial.print(",");
        Serial.print(getRot2Steps());
        Serial.println();
  }

  for (PWM = 0; PWM > -65536; PWM = PWM - 1000) {

      motorLeftWrite(PWM);
      motorRightWrite(PWM);

      setRot1(0);
      setRot2(0);

      Serial2.print(PWM);
      Serial2.print(",");
      delay(1000);

      Serial2.print(getRot1Steps());
      Serial2.print(",");
      Serial2.print(getRot2Steps());
      Serial2.println();
  }


}  // put your setup code here, to run once:



void loop() {
  delay(100);
  motorLeftWrite(0);
  motorRightWrite(0);
}
