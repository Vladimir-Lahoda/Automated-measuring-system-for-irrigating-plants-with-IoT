#include "heltec.h" 
#include "ThingSpeak.h"
#include "secrets.h"
#include <WiFi.h>

#define wait_WSL 10 // time in minutes

float temperature, soil_mois, aku_level, pressure, humidity;
uint8_t uv, watter_level, bright_MSB, bright_LSB, recipient, sender, packet_size;
uint16_t brightness;
int web_err_weather, web_err_service, wait;
bool pump_active, disp, pump_error, parity_error;
int16_t rssi;
int bat;

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
  delay(3500);
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(15 , 0 , "LoRa receiver");
  Heltec.display->display();
  Heltec.display->setFont(ArialMT_Plain_24);
  Heltec.display->drawString(20 , 18 , "Vladimír");
  Heltec.display->drawString(25 , 40 , "Lahoda");
  Heltec.display->display();
  delay(3500);
}

void kit_inic(){                                                  // Function for initialization module
  byte MAC_address[6]; 
  WiFi.macAddress(MAC_address);
  String MAC = String() + String(MAC_address[0], HEX) + ":" +  String(MAC_address[1], HEX) + ":" +  String(MAC_address[2], HEX) + ":" +  String(MAC_address[3], HEX) + ":" + String(MAC_address[4], HEX) + ":" + String(MAC_address[5], HEX); 
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "MAC: " + MAC);
  
  if(WiFi.status() == WL_CONNECTED){                              // If WiFi is connect, show on display "Connect", or "ERROR" if there is problem with WiFi  
      
      String IP = String() + WiFi.localIP()[0] + "." + WiFi.localIP()[1] + "." + WiFi.localIP()[2] + "." + WiFi.localIP()[3];
      String Mask = String() + WiFi.subnetMask()[0] + "." + WiFi.subnetMask()[1] + "." + WiFi.subnetMask()[2] + "." + WiFi.subnetMask()[3];
      Heltec.display->drawString(0 ,10 , "WiFi : " + String(ssid));
      Heltec.display->drawString(0 ,20 , "IP : " + IP);
      Heltec.display->drawString(0, 30, "MASK: " + Mask);
  }
  else {
      Heltec.display->drawString(0 , 15 , "WiFi : ERROR!"); 
  }
  
  Heltec.display->drawString(0, 45, "Wait for incomming packet");
  Heltec.display->display();
}

void Display_values(){                                            // Function for draw values on OLED display
  Heltec.display->clear();
  Heltec.display->drawString(0 , 0 , "Temperature: " + String(temperature,1) + "°C");
  Heltec.display->drawString(0 , 10 , "Humidity: " + String(humidity,1) + " %");
  Heltec.display->drawString(0 , 20 , "Soil Moisture: " + String(soil_mois, 1) + " %");
  Heltec.display->drawString(0 , 30 , "Tank: " + String(watter_level) + " %");
  Heltec.display->drawString(65 , 30 , "AKU: " + String(aku_level/1000, 3) + " V");
  Heltec.display->drawString(0 , 40 , "Bright: " + String(brightness) + " lux");
  Heltec.display->drawString(0 , 50 , "Pressure: " + String(pressure, 1) + " hPa");
  Heltec.display->drawString(85 , 40 , "UV: " + String(uv));  
  Heltec.display->display();
}

void Display_parity_error(){                                            // Function for draw values on OLED display
  Heltec.display->clear();
  Heltec.display->drawString(8 , 25 , "LoRa CRC ERROR !!!");
  Heltec.display->display();
}

int WebUpdate_weather(){                                          // Function for update Thingspeak channel for weather station
  ThingSpeak.setField(1, String(temperature, 2));
  ThingSpeak.setField(2, String(soil_mois, 2));
  ThingSpeak.setField(3, String(aku_level, 1));
  ThingSpeak.setField(4, watter_level);
  ThingSpeak.setField(5, brightness);
  ThingSpeak.setField(6, String(pressure, 2));
  ThingSpeak.setField(8, String(humidity, 1));
  ThingSpeak.setField(7, uv);
  int x = ThingSpeak.writeFields(849764, WriteAPIKey_weather);
  return x;
}

int WebUpdate_service(){                                          // Function for update Thingspeak service channel
  double day_on = millis()/3600000.00;
  ThingSpeak.setField(1, pump_active);
  ThingSpeak.setField(2, rssi);
  ThingSpeak.setField(3, packet_size);
  ThingSpeak.setField(4, pump_error);
  ThingSpeak.setField(5, String(day_on, DEC));
  ThingSpeak.setField(6, parity_error);
  int x = ThingSpeak.writeFields(901557, WriteAPIKey_service);
  return x;
}

void Protects()
{
   if ((millis() - wait) > (wait_WSL * 60000) && aku_level > 3250.0){ // Protective condition - guard status WiFi, updates and time of last LoRa message
    ESP.restart();
  }
  if (WiFi.status() != WL_CONNECTED || web_err_weather != 200 || web_err_service != 200){ // Protective condition - guard status WiFi, updates and time of last LoRa message
    esp_sleep_enable_timer_wakeup(300000);
    esp_deep_sleep_start();
  }
  if ((millis() - wait) > (wait_WSL * 60000) && aku_level <= 3250.0){ // Protective condition - guard time of last LoRa message
    esp_deep_sleep_start();
  }
}

void Display_control()
{
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

void setup() { 
  Heltec.begin(true, true, false, false, 868E6);   
  WiFi.mode(WIFI_STA);  
  ThingSpeak.begin(client);                        // Initialization of Thingspeak library (client mode)
  WiFi.begin(ssid, pass);                          // WiFi connect
  LoRa.setSpreadingFactor(9);                      // Set spreading factor of LoRa (between 6-12) 
  LoRa.setSignalBandwidth(125E3);                  // Set Bandwidth for LoRa
  pinMode(0, INPUT_PULLUP);                        // Set pin for Button as input with internal Pull-UP resistor
  pinMode(21, OUTPUT);
  Heltec.display->init();                          // Initialization for OLED display
  Heltec.display->flipScreenVertically();          // Rotating OLED (180°)
  
  logo();                                          // Draw LOGO on OLED after turn on for 9s
  kit_inic();                                      // Call function for intialization
  web_err_weather = 200;                           // Set variables to 200 (code 200 is OK for update web, must be set for protective condition)
  web_err_service = 200;
  aku_level = 3260.0;
  disp = true;                                     // Default value for OLED - display after start is turned on     
  pump_active = LOW;
  pump_error = LOW;
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
      
      brightness = bright_MSB * 256 + bright_LSB;
      temperature = (0.64 * temp - 25)/2.5;             // Some parameters must be decode to right range  
      pressure = (1.28 * pres + 2350)/2.5;
      soil_mois = soilMois / 2.5;
      aku_level = 5.12 * akuLevel + 3200;
      humidity = humid / 2.5;
      //watter_level *= 5;
      
      if(uv > 63 & uv < 127)
      {
        uv -= 64;
        pump_active = LOW;
        pump_error = HIGH;
      }
      else if(uv > 127 & uv < 140)
      {
        uv -= 128;
        pump_active = HIGH;
        pump_error = LOW;
      }
      else if(uv >= 140)
      {
        pump_active = HIGH;
        pump_error = HIGH;
        uv -= 192;
      }
      else if(uv <= 63)
      {
        pump_active = LOW;
        pump_error = LOW;
      }

     
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
        parity_error = LOW;
      }
      else
      {
        Display_parity_error();
        parity_error = HIGH;
      }
    }
  }
  
  Protects();
  Display_control();
}
