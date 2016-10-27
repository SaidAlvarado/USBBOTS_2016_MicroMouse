#include <Arduino.h>

//**** Using a Teensy 3.1 to Connect an ESP8266 to PC USB Serial *******
//     Compiled with Arduino 1.60 and Teensyduino 1.21b6
//     ESP8266 Firmware: AT21SDK95-2015-01-24.bin

#define WIFI_TX         9
#define WIFI_RX         10
#define WIFI_RST        15
#define WIFI_AUTODETECT 16
#define WIFI_BOOT       17


void setup() {

      //icarus safety code

    pinMode(0, OUTPUT);  // YELLOW LED - SENDING
    digitalWrite(0,LOW);
    pinMode(13, INPUT);  // YELLOW LED - SENDING


    //Prepares GPIO
    pinMode(WIFI_AUTODETECT, INPUT);
    pinMode(WIFI_TX, INPUT);
    pinMode(WIFI_RX, OUTPUT);
    pinMode(WIFI_RST, OUTPUT);
    pinMode(WIFI_BOOT, OUTPUT);

    // GPIO inital values for flashing programming.
    digitalWriteFast(WIFI_BOOT, LOW);
    digitalWriteFast(WIFI_RST, HIGH);


    //Wait for ESP to initalize, and reset it
    delay(300);
    digitalWriteFast(WIFI_BOOT, LOW);
    digitalWriteFast(WIFI_RST, LOW);
    delay(400);
    digitalWriteFast(WIFI_RST, HIGH);


    // Setup computer to Teensy serial
    Serial.begin(115200);

    // Setup Teensy to ESP8266 serial
    // Use baud rate 115200 during firmware update
    Serial2.begin(115200);

}

void loop() {

    // Send bytes from ESP8266 -> Teensy to Computer
    if ( Serial2.available() ) {
        Serial.write( Serial2.read() );
    }

    // Send bytes from Computer -> Teensy back to ESP8266
    if ( Serial.available() ) {
        Serial2.write( Serial.read() );
    }


}
