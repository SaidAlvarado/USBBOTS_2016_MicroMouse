#include <Arduino.h>

#include "icarus_pinout.h"

/* =====================================================================================
                    WIFI Communication chip (ESP-12)
====================================================================================== */

// This Functions lets you interface with the wifi chip to create a serial console over wifi.
// To communicate only use the Serial2 of the teensy, everything else is handled by the ESP-12

//The ESP-12 will create a Telnet server on IP: 192.168.4.1 on port 23.

/*===================================================================
                Public functions
====================================================================*/

// Funtion initializes serial number 3, Sends initialization commands to the ESP-12,
// and prepares the GPIO for programming the ESP-12.
void wifiSetup() {

    // Initialize Hardware Serial
    Serial2.begin(115200);

    //Prepares GPIO
    pinMode(WIFI_AUTODETECT, INPUT);
    pinMode(WIFI_RST, OUTPUT);
    pinMode(WIFI_BOOT, OUTPUT);

    // GPIO inital values for AT command programming.
    digitalWriteFast(WIFI_BOOT, HIGH);
    digitalWriteFast(WIFI_RST, HIGH);

}
