#include "heltec.h" 
#include "ThingSpeak.h"
#include "secrets.h"
#include <WiFi.h>

float temperature, soil_mois, aku_level, pressure, humidity;
uint8_t uv, watter_level, bright_MSB, bright_LSB, recipient, sender, packet_size;
uint16_t brightness;
int web_err_weather, web_err_service, wait;
bool pump_active, disp;
int16_t rssi;

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
  Heltec.display->drawString(0 , 0 , "Temperature: " + String(temperature,1) + "°C");
  Heltec.display->drawString(0 , 10 , "Humidity: " + String(humidity,1) + " %");
  Heltec.display->drawString(0 , 20 , "Soil Moisture: " + String(soil_mois, 1) + " %");
  Heltec.display->drawString(0 , 30 , "Tank: " + String(watter_level) + " %");
  Heltec.display->drawString(65 , 30 , "AKU: " + String(aku_level, 1) + " %");
  Heltec.display->drawString(0 , 40 , "Bright: " + String(brightness) + " lux");
  Heltec.display->drawString(0 , 50 , "Pressure: " + String(pressure, 1) + " hPa");
  if(uv ==12)
  {
    Heltec.display->drawString(85 , 40 , "UV Error");
  }
  else
  {
    Heltec.display->drawString(85 , 40 , "UV: " + String(uv));  
  }
  Heltec.display->display();
}

void Display_parity_error(){                                            // Function for draw values on OLED display
  Heltec.display->clear();
  Heltec.display->drawString(8 , 25 , "LoRa Parity ERROR !!!");
  Heltec.display->display();
}

int WebUpdate_weather(){                                          // Function for update Thingspeak channel for weather station
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, soil_mois);
  ThingSpeak.setField(3, aku_level);
  ThingSpeak.setField(4, watter_level);
  ThingSpeak.setField(5, brightness);
  ThingSpeak.setField(6, pressure);
  ThingSpeak.setField(8, humidity);
  if (uv < 12)
  {
  ThingSpeak.setField(7, uv);
  }
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
  ThingSpeak.setField(6, 0);
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
  pinMode(0, INPUT_PULLUP);                        // Set pin for Button as input with internal Pull-UP resistor
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
  
    if (recipient == V2_ADDRESS & sender == WSL_ADDRESS){    // If any LoRa packet is available, the first 2 address bytes are checked here, if they are the same as set, the whole message is received
      wait = millis();
      rssi = LoRa.packetRssi();                               // RSSI is a level of receive signal (in dBm)
      uint8_t temp = LoRa.read();                             // LoRa.read() is function for read only 1B
      uint8_t soilMois = LoRa.read();
      uint8_t akuLevel = LoRa.read();
      watter_level = LoRa.read();
      uint8_t humid = LoRa.read();
      uint8_t pres = LoRa.read();
      uv = LoRa.read();
      bright_MSB = LoRa.read();
      bright_LSB = LoRa.read();
      uint8_t parity_control_WSL = LoRa.read();
      uint8_t parity_control_V2 = recipient xor sender xor temp xor soilMois xor akuLevel xor watter_level xor humid xor pres xor uv xor bright_MSB xor bright_LSB;
      brightness = bright_MSB *256 + bright_LSB;
      temperature = map(temp, 0, 250, -25.0, 135.0)/2.5;             // Some parameters must be decode to right range  
      pressure = map(pres, 0, 255, 2397.0, 2723.0)/2.55;
      
      soil_mois = soilMois / 2.5;
      aku_level = map(akuLevel, 0, 250, 3200, 4200);
      humidity = humid / 2.5;

      if(brightness < 28)
      {
        Heltec.display->sleep();
      }
      else
      {
        Heltec.display->wakeup();
      }
      if(parity_control_WSL == parity_control_V2)
      {
        web_err_weather = WebUpdate_weather();                  // Calling functions for Thingspeak update
        web_err_service = WebUpdate_service();
        Display_values();                                       // Calling function for draw to OLED display 
      }
      else
      {
        Display_parity_error();
      }
    }
  }
  
  if ((millis() - wait) > 300000 || WiFi.status() != WL_CONNECTED || web_err_weather != 200 || web_err_service != 200){ // Protective condition - guard status WiFi, updates and time of last LoRa message
   ESP.restart();
  }

  if (digitalRead(0) == LOW)        // Condition for OLED button - function for turn on or turn off OLED display
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
