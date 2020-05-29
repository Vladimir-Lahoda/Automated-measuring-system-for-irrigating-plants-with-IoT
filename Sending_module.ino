#include "heltec.h"
#include "secrets.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_VEML6070.h>


//-------------------------------------------------------------PINS---------------------------------------------------------------------------
#define DS18B20 23                                                                                    // Thermometer PIN 
#define SOIL_M 36                                                                                     // Soil Mosture PIN 
#define PUMP_FEEDBACK 37
#define PUMP 12                                                                                       // Water pump switch PIN   
#define VCC 2                                                                                         // Sensors switch PIN 
#define POWER_LEVEL 39                                                                                // Power level PIN 
//#define UART-RX1 9                                                                                        // UART1 pin - RX (UART0 is used for PC connect)
//#define UART-TX1 10                                                                                         // UART1 pin - TX

//----------------------------------------------------------PARAMETERS------------------------------------------------------------------------

#define SLEEP    180                                                                                  // Deep sleep for X second
#define SOIL_MOS_MIN 40                                                                               // Minimal soil mosture (%)
#define WATER_LIMIT 15                                                                                // Minimal water level (%)
#define TIME_WATER 0.5                                                                                // Pump on time (in minutes)
#define distance_bottom 370                                                                           // Distance from sensor US-100 to water bottom (in mm)
#define distance_surface 50                                                                           // Distance from sensor US-100 to water surface (in mm)
#define RES 0.805664                                                                                  // Constant of ADC (resolution)
#define SOIL_MIN 2796.0
#define SOIL_MAX 1320.0
#define BRIG_MAX 6500
#define UV_MAX 5

#define BME280_ADDRESS (0x76)                                                                         // BME280 I2C adress
#define TSL2561_ADDRESS (0x39)


HardwareSerial hwSerial_1(2);                                                                         // Declaration for UART1  
Adafruit_BME280 BME280;                                                                               // Declaration for BME280
OneWire oneWireDS(DS18B20);                                                                           // Declaration oneWire bus (on pin DS18B20)
DallasTemperature DS_Temp(&oneWireDS);                                                                // Declaration for thermometer on oneWire bus
Adafruit_TSL2561_Unified TSL_2561 = Adafruit_TSL2561_Unified(TSL2561_ADDRESS, 12345);
Adafruit_VEML6070 UV = Adafruit_VEML6070();

void setup()
{
  int begin_m = millis();                                                                             // Variable for measuring time  
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);                                                                             // Switching on the sensors connected to the switch and delay for balance                                                                     
  delay(4500);
  Heltec.begin(false, true, false, false, 868E6);                                                      // Inicialization WSL:  Display OFF (false),  LoRa ON (true), Serial ON (true), PA BOOST OFF (false), LoRa frequency (868MHz for czech)
  hwSerial_1.begin(9600, SERIAL_8N1, 9, 10);                                                       // Inicializaton of UART1 (UART0 is used by PC programming and console, ESP32 has got 3 UARTS)
  pinMode(PUMP,OUTPUT);                                                                               // Set SWITCH Pin as Output for water pump switch
  pinMode(PUMP_FEEDBACK, INPUT);
  DS_Temp.begin();                                                                                    // Begin thermometer (DS18B20)
  Serial.begin(115200);
  UV.begin(VEML6070_1_T);
  BME280.begin(BME280_ADDRESS);                                                                       // Inicialization BME280 on I2C adress, next step is setting for parameters (selected setting for lowest power consumption)
  BME280.setSampling(Adafruit_BME280::MODE_FORCED,                                                    // Forced mode - single mode    
                    Adafruit_BME280::SAMPLING_X1,                                                     // Temperature sampling
                    Adafruit_BME280::SAMPLING_X1,                                                     // Pressure sampling
                    Adafruit_BME280::SAMPLING_X1,                                                     // Humidity sampling
                    Adafruit_BME280::FILTER_OFF   );                                                  // IIR filter     
  TSL_2561.begin();
  TSL_2561.setGain(TSL2561_GAIN_16X);
  TSL_2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
  Serial.println("OK1");
  analogSetClockDiv(255);
  
  
  float pres = (BME280.readPressure()/100.00) + 32;                                                   // Calculating pressure from BME280 including correction
  uint8_t pressure = constrain((1.9531 * pres - 1835.9),0,250);                               // Change adjust to value 0 - 255 (1 B)    
  float humid = BME280.readHumidity();                                                                // Function for read humidity from BME280
  uint8_t humidity = constrain((2.5 * humid), 0, 250); 

  float aku_volt = ((analogRead(POWER_LEVEL) * RES) / 0.30986)+400;
  uint8_t batteryLevel = constrain((0.1953*aku_volt - 625.0), 0, 250);                        // Data format adjustment (Battery level betwen 4,2V (100%) and 3,2V (0%))        
  Serial.println("OK2");
  sensors_event_t event;
  TSL_2561.getEvent(&event);
  uint16_t brightness = event.light;                                                                  // Value from luxmeter  
  uint8_t bright_MSB = brightness / 256;
  uint8_t bright_LSB = brightness % 256;
  DS_Temp.setResolution(10);                                                                           // Set resolution for thermometer,the lowest resolution is sufficient (9 bit)
  DS_Temp.requestTemperatures();                                                                      // Get data from thermometer(s)
  uint8_t temperature = constrain((3.9063 * DS_Temp.getTempCByIndex(0)+ 39.063), 0, 250);      // Data format adjustment (Temperature between -128 to 127 is adjust to 0 - 255 (1B)) 
  uint16_t UV_num = UV.readUV();                                                                      // Function for calculatin UV index
  uint8_t uvIndex = UV_Index_VEML(UV_num);
  delay(800);
  Serial.println("OK3");
  hwSerial_1.flush();
  Serial.println("OK4");
  hwSerial_1.write(0x55);
  while(!hwSerial_1.available());
  uint8_t high = hwSerial_1.read();
  uint8_t low = hwSerial_1.read();
  uint16_t distance = (high * 256 + low)-40;
  Serial.println("OK5");
 if(distance < 50 | distance > 450) ESP.restart();
  //int waterLevel = (-0,0625 * distance + 23,125);
  uint8_t waterLevel = constrain(map(distance, distance_surface, distance_bottom, 100, 0), 0, 100);   // Function for calculating water level from distance value
  delay(5);
  float soil = analogRead(SOIL_M) * RES;                                
  uint8_t soil_mos = constrain(map(soil,SOIL_MIN, SOIL_MAX, 0, 250), 0, 250);                         // Calculating value of Soil moisture from analog voltage                                                               
  UV.sleep(HIGH);
  //hwSerial_1.end();
  Serial.println("OK6");
  uint8_t pump = water_switch(soil_mos, waterLevel, brightness, uvIndex);                                                             // Function for irrigation management
  
  if(pump == 1)
  {
    uvIndex += 128;
  }
  else if (pump == 2)
  {
    uvIndex += 64;
  }
  
  
  uint8_t parity_control = V2_ADDRESS xor WSL_ADDRESS xor temperature xor soil_mos xor batteryLevel xor waterLevel xor humidity xor pressure xor uvIndex xor bright_MSB xor bright_LSB;

  
  LoRa.idle();                                                                                        // Set LoRa to Standby mode
  LoRa.setTxPower(14,PA_OUTPUT_RFO_PIN);                                                              // Set transmit power of LoRa and pin (for RFO is between 0 and 14, for PA BOOST is 0-17. In Czech republic is transmit power limit max 25mW (14dBm)) 
  LoRa.setSpreadingFactor(9);                                                                         // Set spreading factor of LoRa (between 6-12) 
  LoRa.setSignalBandwidth(125E3);                                                                     // Set frequency bandwith (62,5E3 - 250E3 Hz)
  int MS_time = millis() - begin_m;                                                                   // Calculate measuring time 
  int begin_p = millis();                                                                             // Variable for time calculating
  LoRa.beginPacket();                                                                                 // Begin new LoRa packet 
  LoRa.write(V2_ADDRESS);                                                                             // Write destination adress to LoRa packet (1B)
  LoRa.write(WSL_ADDRESS);                                                                            // Write local adress to LoRa packet (1B)
  LoRa.write(temperature);                                                                            // Write sensors values to LoRa packet (every value - 1B)
  LoRa.write(soil_mos);
  LoRa.write(batteryLevel);
  LoRa.write(waterLevel);
  LoRa.write(humidity);
  LoRa.write(pressure);
  LoRa.write(uvIndex);
  LoRa.write(bright_MSB);
  LoRa.write(bright_LSB);
  LoRa.write(parity_control);
  LoRa.endPacket();                                                                                   // End LoRa packet (10B)
  int TX_time = millis() - begin_p;                                                                   // Calculate sending packet time 
  LoRa.sleep();                                                                                       // Activate sleep mode of LoRa (minimalize energy consumption) 



 serialDiagnostic (temperature, soil_mos, batteryLevel, aku_volt, distance, waterLevel, brightness, pressure, uvIndex, UV_num, humidity, parity_control, MS_time, TX_time);       // Function for device diagnostic - print all important values from sensors to UART console 

  digitalWrite(VCC,LOW);
  UnderVoltProt(aku_volt);                                                                            // Function for under discharge protect 
  esp_sleep_enable_timer_wakeup(SLEEP * 1000000);                                                     // Set timer for Deep sleep mode (parameter SLEEP)
  esp_deep_sleep_start();                                                                             // Enable deep sleep mode - Current consumption WSL in deep sleep mode: 1.8mA
}

void loop()
{
}

uint8_t water_switch(float soil_mos, uint8_t waterLevel, uint16_t brightness, uint8_t uv)            // Function for irrigation management (if soil moisture is less than the set parameter and water level in tank is higher than the set parameter, is turn on pump to parameter time)
  {
  if((soil_mos/2.5) < SOIL_MOS_MIN & waterLevel > WATER_LIMIT & brightness < BRIG_MAX & uv < UV_MAX)
    {
      digitalWrite(PUMP, HIGH);
      delay(200);
      if (digitalRead(37))
      {    
        delay(TIME_WATER*60000);
        digitalWrite(PUMP, LOW);
        return 1;
      }
      else
      {
        digitalWrite(PUMP, LOW);
        return 2;
      }   
    }
    else
    {
      return 0;
    }
  }

uint8_t UV_Index_VEML(float index)                                                                    // Function for calculatin UV index from analog voltage (according to the linearized characteristic given by the datasheet UMV30A)
  {
  if(index <= 186) return 0;
  else if (index > 186 & index <= 373) return 1;
  else if (index > 373 & index <= 560) return 2;
  else if (index > 560 & index <= 747) return 3;
  else if (index > 747 & index <= 933) return 4;
  else if (index > 933 & index <= 1120) return 5;
  else if (index > 1120 & index <= 1307) return 6;
  else if (index > 1307 & index <= 1494) return 7;
  else if (index > 1494 & index <= 1681) return 8;
  else if (index > 1681 & index <= 1868) return 9;
  else if (index > 1868 & index <= 2054) return 10;
  else if (index > 2054 & index <= 65000) return 11;
  }
  
void UnderVoltProt(float aku_volt)                                                                  // Function for under discharge protect (if aku voltage is lower than the set parameter, is permanently enabled deep sleep mode)
  {
  if(aku_volt < 3250)
    {
    esp_deep_sleep_start();
    }
  }

/*uint16_t Water_level()                                                                                   // Function for getting distance value from ultrasonic sensor (US-100) via UART1
{
    uint8_t high;
    uint8_t low;
    hwSerial_1.write(0x55);
    //delay(500); 
    while(hwSerial_1.available());
    uint8_t high = hwSerial_1.read();
    uint8_t low = hwSerial_1.read();
      
    return ((high * 256 + low)-40);                                                                      // Return calculated value of distance
}
*/
void serialDiagnostic (int8_t temp, uint8_t soil_mos, uint8_t batteryLevel, float aku_volt, uint16_t distance, uint8_t waterLevel, int bright, uint16_t pressure, uint8_t uvIndex, uint16_t UV_num, uint8_t humidity, uint8_t parity_control, int MS_time, int TX_time)
{
  float temperature = map(temp, 0, 250, -25.0, 135.0)/2.5;
  Serial.begin(115200);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Soil moisture: ");
  Serial.print(soil_mos/2.5);
  Serial.println(" %");
  Serial.print("AKU charge status: ");
  Serial.print(aku_volt);
  Serial.print("mV  |  Comprimated: ");
  Serial.print(map(batteryLevel, 0, 250, 3200, 4480));
  Serial.println(" mV");
  Serial.print("Vzdálenost hladiny: ");
  Serial.print(distance);
  Serial.println(" mm");
  Serial.print("Water level in tank: ");
  Serial.print(waterLevel);
  Serial.println(" %");
  Serial.print("Brightness: ");
  Serial.print(bright);
  Serial.println(" lux");
  Serial.print("Atmospheric pressure: ");
  Serial.print(map(pressure, 0, 255, 2350, 2670)/2.5);
  Serial.println(" hPa");
  Serial.print("UV index: ");
  Serial.println(uvIndex);
  Serial.print("UV number: ");
  Serial.println(UV_num);
  Serial.print("Humidity: ");
  Serial.print(map(humidity, 0, 250, 0.0, 100.0));
  Serial.println(" %");
  Serial.print("Parity control: ");
  Serial.println(parity_control);
  Serial.print("Measuring time: ");
  Serial.print(MS_time);
  Serial.println(" ms");
  Serial.print("LoRa sending time: ");
  Serial.print(TX_time);
  Serial.println(" ms");
}
