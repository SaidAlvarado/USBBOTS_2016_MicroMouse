#include <Arduino.h>

#include "icarus_pinout.h"

/* =====================================================================================
                    WIFI Communication chip (ESP-12)
====================================================================================== */

// This Functions lets you interface with the wifi chip to create a serial console over wifi.



/*=========================================================================
    Variables
    -----------------------------------------------------------------------*/
    String ordenes[]=   {"AT+CWMODE=2",
                         "AT+CWJAP=Icarus,usbbots328"
                         "AT+CIPMUX=1",
                         "AT+CIPSERVER=1,90",
                         "END"                 // Para reconocer el fin de los comandos AT
    };
/*=========================================================================*/


/*===================================================================
                Public functions
====================================================================*/

// Funtion initializes serial number 3, Sends initialization commands to the ESP-12,
// and prepares the GPIO for programming the ESP-12.
void wifiSetup() {

    // Initialize Hardware Serial
    Serial2.begin(9600);

    //Prepares GPIO
    pinMode(WIFI_AUTODETECT, INPUT);
    pinMode(WIFI_TX, INPUT);
    pinMode(WIFI_RX, OUTPUT);
    pinMode(WIFI_RST, OUTPUT);
    pinMode(WIFI_BOOT, OUTPUT);

    // GPIO inital values for AT command programming.
    digitalWriteFast(WIFI_BOOT, HIGH);
    digitalWriteFast(WIFI_RST, HIGH);

    // Sends the commands
    for (size_t i = 0; i < 5; i++) {
        Serial2.println(ordenes[i]);
    }
}

uint8_t datos;
//
// void loop() {
//
//   if (Serial2.available()) {
//     datos = Serial2.read();
//     Serial.print(datos);
//   }
//
//   if (Serial.available()) {
//     datos = Serial.read();
//     Serial2.print(datos);
//   }
// }
