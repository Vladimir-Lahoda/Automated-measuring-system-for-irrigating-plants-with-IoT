#include "heltec.h" 
#include "ThingSpeak.h"
#include <WiFi.h>

int8_t temperature;   
uint8_t soil_mois;
uint8_t aku_level;
uint8_t uv;
uint8_t watter_level;
int brightness;
int pressure;
uint8_t humidity;
int web_err_weather;
int web_err_service;
int wait;
bool pump_active;
int16_t rssi;
uint8_t recipient;
uint8_t sender;
uint8_t packet_size;
bool disp;

uint8_t localAddress = 0xBC;                                      // Adress of this device
uint8_t WSL_ADRESS = 0xBB;                                        // Adress of WSL sender 

char ssid[] = "TP-LINK_E46FG";                                    // SSID of WiFi  
char pass[] = "prohosty";                                         // Password for WiFi  
const char * WriteAPIKey_weather = "QYOXSTCGVWUVB9Z9";            // API Keys of THINGSPEAK channels
const char * WriteAPIKey_service = "FZJXEPFV4K0PBG42";
WiFiClient  client;                                               //  WiFi mode - client

void logo(){                                                      // Function for draw a logo (only name of project after turn ON)
  Heltec.display->clear();                                        // Function for clear OLED display  
  Heltec.display->setFont(ArialMT_Plain_16);                      // Set font ArialMT, size 16  
  Heltec.display->drawString(40 , 0 , "Systém");                  // Draw strings on OLED 
  Heltec.display->drawString(12 , 14 , "automatického");
  Heltec.display->drawString(22 , 28 , "zavlazování");
  Heltec.display->display();                                      // Show on display (render)
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(38 , 41 , "s IoT");
  Heltec.display->display();
  delay(4500);
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(15 , 0 , "LoRa receiver");
  Heltec.display->display();
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(20 , 18 , "Vladimír");
  Heltec.display->drawString(25 , 40 , "Lahoda");
  Heltec.display->display();
  delay(4500);
}

void kit_inic(){                                                  // Function for initialization module
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "LoRa Initial success!");
  Heltec.display->drawString(0, 10, "Receiver ON");
  if(WiFi.status() == WL_CONNECTED){                              // If WiFi is connect, show on display "Connect", or "ERROR" if there is problem with WiFi  
      Heltec.display->drawString(0 ,20 , "WiFi : Connect");
  }
  else {
      Heltec.display->drawString(0 , 20 , "WiFi : ERROR!"); 
  }
  Heltec.display->display();
}

void Display_values(){                                            // Function for draw values on OLED display
  Heltec.display->clear();
  Heltec.display->drawString(0 , 0 , "Temperature: " + String(temperature) + "°C");
  Heltec.display->drawString(0 , 10 , "Humidity: " + String(humidity) + " %");
  Heltec.display->drawString(0 , 20 , "Soil Moisture: " + String(soil_mois) + " %");
  Heltec.display->drawString(0 , 30 , "Tank: " + String(watter_level) + " %");
  Heltec.display->drawString(65 , 30 , "AKU: " + String(aku_level) + " %");
  Heltec.display->drawString(0 , 40 , "Bright: " + String(brightness) + " lux");
  Heltec.display->drawString(85 , 40 , "UV: " + String(uv));  
  Heltec.display->drawString(0 , 50 , "Pressure: " + String(pressure) + " hPa");
  Heltec.display->display();
}

int WebUpdate_weather(){                                          // Function for update Thingspeak channel for weather station
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, soil_mois);
  ThingSpeak.setField(3, aku_level);
  ThingSpeak.setField(4, watter_level);
  ThingSpeak.setField(5, brightness);
  ThingSpeak.setField(6, pressure);
  ThingSpeak.setField(7, uv);
  ThingSpeak.setField(8, humidity);
  int x = ThingSpeak.writeFields(849764, WriteAPIKey_weather);
  return x;
}

int WebUpdate_service(){                                          // Function for update Thingspeak service channel
  double day_on = millis()/3600000.00;
  ThingSpeak.setField(1, pump_active);
  ThingSpeak.setField(2, rssi);
  ThingSpeak.setField(3, packet_size);
  ThingSpeak.setField(4, sender);
  ThingSpeak.setField(5, String(day_on, DEC));
  int x = ThingSpeak.writeFields(901557, WriteAPIKey_service);
  return x;
}

void setup() { 
  Heltec.begin(true, true, false, false, 868E6);   
  WiFi.mode(WIFI_STA);  
  ThingSpeak.begin(client);                        // Initialization of Thingspeak library (client mode)
  WiFi.begin(ssid, pass);                          // WiFi connect
  LoRa.setSpreadingFactor(9);                      // Set spreading factor of LoRa (between 6-12) 
  LoRa.setSignalBandwidth(62.5E3);                 // Set Bandwidth for LoRa
  pinMode(2, INPUT_PULLUP);                        // Set pin for Button as input with internal Pull-UP resistor
  Heltec.display->init();                          // Initialization for OLED display
  //Heltec.display->flipScreenVertically();        // Rotating OLED (180°)
  logo();                                          // Draw LOGO on OLED after turn on for 9s
  kit_inic();                                      // Call function for intialization
  web_err_weather = 200;                           // Set variables to 200 (code 200 is OK for update web, must be set for protective condition)
  web_err_service = 200;
  disp = true;                                     // Default value for OLED - display after start is turned on     
  wait = millis();                                 // Variable wait is set to actual time after start (must be set for protective condition) 
  LoRa.receive();                                  // Set LoRa receive mode (LoRa modul SX1276 continuously receives data) 
  
}

void loop() {
  packet_size = LoRa.parsePacket();                // If a LoRa packet is available, the size of packet is written there
  if (packet_size) {                               // If packet size is higher than zero, is called this function 
    recipient = LoRa.read();         
    sender = LoRa.read();
  
    if (recipient == localAddress & sender == WSL_ADRESS){    // If any LoRa packet is available, the first 2 address bytes are checked here, if they are the same as set, the whole message is received
      wait = millis();
      rssi = LoRa.packetRssi();                               // RSSI is a level of receive signal (in dBm)
      uint8_t temp = LoRa.read();                             // LoRa.read() is function for read only 1B
      soil_mois = LoRa.read();
      aku_level = LoRa.read();
      watter_level = LoRa.read();
      brightness = LoRa.read();
      uint8_t pres = LoRa.read();
      uv = LoRa.read();
      humidity = LoRa.read();
      temperature = map(temp, 0, 255, -128, 127);             // Some parameters must be decode to right range  
      pressure = map(pres, 0, 255, 844, 1100);
      
      web_err_weather = WebUpdate_weather();                  // Calling functions for Thingspeak update
      web_err_service = WebUpdate_service();

      Display_values();                                       // Calling function for draw to OLED display 
    }
  }
  
  if ((millis() - wait) > 300000 || WiFi.status() != WL_CONNECTED || web_err_weather != 200 || web_err_service != 200){ // Protective condition - guard status WiFi, updates and time of last LoRa message
   ESP.restart();
  }

  if (digitalRead(2) == LOW)        // Condition for OLED button - function for turn on or turn off OLED display
  {
   if(disp == true)
   {
    Heltec.display->sleep();
    disp = false;
   }
   else
   {
    Heltec.display->wakeup();
    disp = true;
   }
  }
}
