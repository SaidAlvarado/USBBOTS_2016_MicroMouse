#include <Arduino.h>
#include <icarus_pinout.h>

//USBBOTS2016_Icarus.ino

extern int16_t gyro_bias_z;
int led = 13;

void setup() {
  // put your setup code here, to run once:

  pinMode(0, OUTPUT);  //Safe mode for IR emitter
  digitalWriteFast(0, LOW);
  pinMode(13, INPUT);   //Safe mode for onboard led

  pinMode(BTN_SEARCH, INPUT);
  pinMode(BTN_SPEED, INPUT);

  Serial.begin(115200);

  gyroSetup();
  motorSetup();
}


float angle=0, dps;


void loop() {

    while(digitalReadFast(BTN_SPEED)){

        Serial.print(digitalReadFast(BTN_SEARCH));
        Serial.print(" ");
        Serial.println(digitalReadFast(BTN_SPEED));
        delay(20);

    };

    delay(3000);

    setAngle(0.0);

    while(angle < 90){
        angle = getAngle();
        dps = getAngularVelocity();
        Serial.print(angle);
        Serial.print(" ");
        Serial.println(dps);
        motorLeftWrite(-20000);
        motorRightWrite(20000);
        delay(20);
    }

    motorLeftWrite(0);
    motorRightWrite(0);

}
