#include <ESP8266WiFi.h>

/* This Program Starts the ESP-12 as an Access point with a Telnet Server.
It provides a Transparent Telnet to Serial Bridge to the teensy on the Serial Port 2.
Up to <MAX_SRV_CLIENTS> clients can connect to the server simultaneously.

It also controls the TX and RX leds on the Micromouse to signal occuring transactions.
And initializes the WIFI_AUTODETECT pin so that the teensy can automatically register that
the module was connected to the board
*/

//////////////////////
// WiFi Definitions //
//////////////////////
const char AP_Name[] = "Icarus_Wifi";
const char WiFiAPPSK[] = "USBBOTS328";
# define SERVER_PORT 23

/////////////////////
// Pin Definitions //
/////////////////////
#define LED_RX          12
#define LED_TX          13
#define WIFI_AUTODETECT 5

// Definitions of the Telnet
#define MAX_SRV_CLIENTS 3
WiFiServer server(SERVER_PORT);
WiFiClient serverClients[MAX_SRV_CLIENTS];

// Definitions for RX TX leds timers
#define LED_ON_time  20 //ms
long LED_TX_TimeOn=0;
long LED_RX_TimeOn=0;

void setup()
{
    // Start the serial terminal
    Serial.begin(115200);

    // Configure the pins directions
    pinMode(LED_RX, OUTPUT);
    pinMode(LED_TX, OUTPUT);
    pinMode(WIFI_AUTODETECT, OUTPUT);

    // Configure start up values;
    digitalWrite(WIFI_AUTODETECT, LOW);     // Way for the teensy to know the ESP-12 is connected
    digitalWrite(LED_RX, LOW);              // Start the RX, TX leds off
    digitalWrite(LED_TX, LOW);

    // Configure Wifi options
    WiFi.mode(WIFI_AP);                 // Configure the ESP as an Access Point
    WiFi.softAP(AP_Name, WiFiAPPSK);    // Configure the name and password for the access point

    // Start the server on the desired port
    server.begin();
    server.setNoDelay(true);
}



void loop(){
    uint8_t i;
    //check if there are any new clients
    if (server.hasClient()){
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        //find free/disconnected spot
        if (!serverClients[i] || !serverClients[i].connected()){
          if(serverClients[i]) serverClients[i].stop();
          serverClients[i] = server.available();
        //   Serial1.print("New client: "); Serial1.print(i);
          continue;
        }
      }
      //no free/disconnected spot so reject
      WiFiClient serverClient = server.available();
      serverClient.stop();
    }
    //check clients for data
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        if(serverClients[i].available()){

          // Turn on the RX led
          digitalWrite(LED_RX, HIGH);
          LED_RX_TimeOn = millis();

          //get data from the telnet client and push it to the UART
          while(serverClients[i].available()) Serial.write(serverClients[i].read());
        }
      }
    }


    //check UART for data
    if(Serial.available()){

      // Turn on the TX led
      digitalWrite(LED_TX, HIGH);
      LED_TX_TimeOn = millis();

      // Recive UART data and send it through Wifi
      size_t len = Serial.available();
      uint8_t sbuf[len];
      Serial.readBytes(sbuf, len);
      //push UART data to all connected telnet clients
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        if (serverClients[i] && serverClients[i].connected()){
          serverClients[i].write(sbuf, len);
        }
      }
    }


    // Turn off leds after enough time lit on.
    if (millis() - LED_TX_TimeOn > LED_ON_time) {
        digitalWrite(LED_TX, LOW);   // set the LED off
    }
    if (millis() - LED_RX_TimeOn > LED_ON_time) {
        digitalWrite(LED_RX, LOW);   // set the LED off
    }

}
