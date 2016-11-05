#include "icarus_pinout.h"
#include "Teensy_Encoder.h"

//USBBOTS2016_Icarus.ino

float dist[4];
uint16_t irval[4];

void setup() {

  pinMode(BTN_SPEED, INPUT);   //Safe mode for onboard led
  wifiSetup();
  irSensorSetup();
  motorSetup();
  gyroSetup();
  // startController();

  motorLeftWrite(0);
  motorRightWrite(0);
  Serial.begin(115200);

  setDistanceLeft(200);
}  // put your setup code here, to run once:

uint8_t data;

void loop() {


    getIR(irval);


    Serial2.print("FL = ");
    Serial2.print(irval[0]);
    Serial2.print("   ");

    Serial2.print("DL = ");
    Serial2.print(irval[1]);
    Serial2.print("   ");

    Serial2.print("DR = ");
    Serial2.print(irval[2]);
    Serial2.print("   ");

    Serial2.print("FR = ");
    Serial2.print(irval[3]);
    Serial2.print("   ");

    Serial2.print("Wall_left = ");
    Serial2.print(isWallLeft());
    Serial2.print("   ");

    Serial2.print("Wall_left = ");
    Serial2.print(isWallRight());
    Serial2.print("   ");

    Serial2.print("Front Wall = ");
    Serial2.print(isWallFront());
    Serial2.println("   ");


    delay(200);




    // Serial2.print("VL = ");
    // Serial2.print(getVL());
    // Serial2.print("mm/s      ");
    //
    // Serial2.print("VR = ");
    // Serial2.print(getVR());
    // Serial2.print("mm/s      ");
    //
    // Serial2.print("distanceLeft = ");
    // Serial2.print(getDistanceLeft());
    // Serial2.println("mm      ");
    //
    // delay(50);
    //
    // // Start in the safe range
    //
    // if (Serial2.available()) {
    //
    //     data = Serial2.read();
    //
    //     if (data == '0') {
    //         motorLeftWrite(0);
    //         motorRightWrite(0);
    //     }
    //
    //     if (data == '1') {
    //         motorLeftWrite(25000);
    //         // motorRightWrite(25000);
    //     }
    //
    //     if (data == '2') {
    //         motorLeftWrite(60000);
    //         // motorRightWrite(60000);
    //     }
    //
    //     if (data == '3') {
    //         // motorLeftWrite(-25000);
    //         motorRightWrite(25000);
    //     }
    //
    //     if (data == '4') {
    //         // motorLeftWrite(-50000);
    //         motorRightWrite(60000);
    //     }
    //
    //     if (data == 'a') {
    //         setDistanceLeft(200);
    //     }
    // }


}


//Code that writes the data for the motor balancer
//
// for (PWM = 0; PWM < 65535; PWM = PWM + 1000) {
//
//       motorLeftWrite(PWM);
//       motorRightWrite(PWM);
//
//       setRot1(0);
//       setRot2(0);
//
//       Serial2.print(PWM);
//       Serial2.print(",");
//       delay(1000);
//
//       Serial2.print(getRot1Steps());
//       Serial2.print(",");
//       Serial2.print(getRot2Steps());
//       Serial2.println();
// }
//
// for (PWM = 0; PWM > -65536; PWM = PWM - 1000) {
//
//     motorLeftWrite(PWM);
//     motorRightWrite(PWM);
//
//     setRot1(0);
//     setRot2(0);
//
//     Serial2.print(PWM);
//     Serial2.print(",");
//     delay(1000);
//
//     Serial2.print(getRot1Steps());
//     Serial2.print(",");
//     Serial2.print(getRot2Steps());
//     Serial2.println();
// }
