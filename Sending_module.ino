#include "heltec.h"
#include "secrets.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>

//-------------------------------------------------------------PINS---------------------------------------------------------------------------
#define DS18B20 23                                                                                    // Thermometer PIN 
#define SOIL_M 2                                                                                      // Soil Mosture PIN 
#define SWITCH 2                                                                                      // Water pump switch PIN   
#define VCC 15                                                                                        // Sensors switch PIN 
#define POWER_LEVEL 1                                                                                 // Power level PIN 
#define UVM 0                                                                                         // UV sensor PIN on external ADC
#define RX1 10                                                                                        // UART1 pin - RX (UART0 is used for PC connect)
#define TX1 9                                                                                         // UART1 pin - TX

//----------------------------------------------------------PARAMETERS------------------------------------------------------------------------

#define SLEEP    180                                                                                  // Deep sleep for X second
#define SOIL_MOS_MIN 40                                                                               // Minimal soil mosture (%)
#define WATER_LIMIT 15                                                                                // Minimal water level (%)
#define TIME_WATER 0.5                                                                                // Pump on time (in minutes)
#define distance_bottom 370                                                                           // Distance from sensor US-100 to water bottom (in mm)
#define distance_surface 50                                                                           // Distance from sensor US-100 to water surface (in mm)
#define RES 0.1875                                                                                    // Constant of ADC (resolution)
#define RES1 0.805664

#define BME280_ADDRESS (0x76)                                                                         // BME280 I2C adress
#define ADS_1115_ADDRESS (0x48)                                                                       // ADC I2C adress 
#define TSL2561_ADDRESS (0x39)


HardwareSerial hwSerial_1(1);                                                                         // Declaration for UART1  
Adafruit_BME280 BME280;                                                                               // Declaration for BME280
OneWire oneWireDS(DS18B20);                                                                           // Declaration oneWire bus (on pin DS18B20)
DallasTemperature DS_Temp(&oneWireDS);                                                                // Declaration for thermometer on oneWire bus
Adafruit_TSL2561_Unified TSL_2561 = Adafruit_TSL2561_Unified(TSL2561_ADDRESS, 12345);

void setup()
{
  
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);                                                                             // Switching on the sensors connected to the switch and delay for balance                                                                     
  delay(300);
  Heltec.begin(false, true, true, false, 868E6);                                                      // Inicialization WSL:  Display OFF (false),  LoRa ON (true), Serial ON (true), PA BOOST OFF (false), LoRa frequency (868MHz for czech)
  int begin_m = millis();                                                                             // Variable for measuring time  
  pinMode(SWITCH,OUTPUT);                                                                             // Set SWITCH Pin as Output for water pump switch
  DS_Temp.begin();                                                                                    // Begin thermometer (DS18B20)
  BME280.begin(BME280_ADDRESS);                                                                       // Inicialization BME280 on I2C adress, next step is setting for parameters (selected setting for lowest power consumption)
  BME280.setSampling(Adafruit_BME280::MODE_FORCED,                                                    // Forced mode - single mode    
                    Adafruit_BME280::SAMPLING_X1,                                                     // Temperature sampling
                    Adafruit_BME280::SAMPLING_X1,                                                     // Pressure sampling
                    Adafruit_BME280::SAMPLING_X1,                                                     // Humidity sampling
                    Adafruit_BME280::FILTER_OFF   );                                                  // IIR filter     
  hwSerial_1.begin(9600, SERIAL_8N1, RX1, TX1);                                                       // Inicializaton of UART1 (UART0 is used by PC programming and console, ESP32 has got 3 UARTS)
  TSL_2561.enableAutoRange(true);
  TSL_2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  
  float pres = (BME280.readPressure()/100.00) + 32;                                                   // Calculating pressure from BME280 including correction
  uint8_t pressure = constrain(map(pres, 940.0, 1068.0, 0, 250),0,250);                               // Change adjust to value 0 - 255 (1 B)    
  float humid = BME280.readHumidity();                                                                // Function for read humidity from BME280
  uint8_t humidity = constrain(map(humid, 0.0, 100.0, 0, 250), 0, 250); 
  TSL_2561.begin();
  sensors_event_t event;
  TSL_2561.getEvent(&event);
  uint16_t brightness = event.light;                                                                  // Value from luxmeter  
  uint8_t bright_MSB = brightness / 256;
  uint8_t bright_LSB = brightness % 256;
  DS_Temp.setResolution(9);                                                                           // Set resolution for thermometer,the lowest resolution is sufficient (9 bit)
  DS_Temp.requestTemperatures();                                                                      // Get data from thermometer(s)
  uint8_t temperature = constrain(map(DS_Temp.getTempCByIndex(0), -10.0, 54.0, 0, 250), 0, 250);      // Data format adjustment (Temperature between -128 to 127 is adjust to 0 - 255 (1B)) 
 
  //adcAttachPin(13);
  analogSetClockDiv(255);
  double aku_volt = ((analogRead(39) * RES1 * 32) / 10) + 306;
  delay(50);
  float UV_volt = constrain(analogRead(37) * RES1, 0.0, 1170.0);
  delay(10);
  uint16_t distance = Water_level();
  int soil = analogRead(36) * RES1;                                                                
  
  uint8_t batteryLevel = constrain(map(aku_volt, 3200, 4200, 0, 250), 0, 250);                        // Data format adjustment (Battery level betwen 4,2V (100%) and 3,2V (0%))                                            
  uint8_t soil_mos = constrain(map(soil,2909,1327, 0, 250), 0, 250);                                  // Calculating value of Soil moisture from analog voltage
  uint8_t uvIndex = UV_Index(UV_volt);                                                                // Function for calculatin UV index
  uint8_t waterLevel = constrain(map(distance, distance_surface, distance_bottom, 100, 0), 0, 100);   // Function for calculating water level from distance value
  
  
  
  uint8_t parity_control = V2_ADDRESS xor WSL_ADDRESS xor temperature xor soil_mos xor batteryLevel xor waterLevel xor humidity xor pressure xor uvIndex xor bright_MSB xor bright_LSB;


  
  LoRa.idle();                                                                                        // Set LoRa to Standby mode
  LoRa.setTxPower(14,PA_OUTPUT_RFO_PIN);                                                       // Set transmit power of LoRa and pin (for RFO is between 0 and 14, for PA BOOST is 0-17. In Czech republic is transmit power limit max 25mW (14dBm)) 
  LoRa.setSpreadingFactor(9);                                                                         // Set spreading factor of LoRa (between 6-12) 
  LoRa.setSignalBandwidth(62.5E3);                                                                    // Set frequency bandwith (62,5E3 - 250E3 Hz)
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



  serialDiagnostic (temperature, soil_mos, batteryLevel, aku_volt, distance, waterLevel, brightness, pressure, uvIndex, UV_volt, humidity, parity_control, MS_time, TX_time);       // Function for device diagnostic - print all important values from sensors to UART console 
 // water_switch(soil_mos, waterLevel);                                                               // Function for irrigation management
  digitalWrite(21,LOW);
  UnderVoltProt(aku_volt);                                                                            // Function for under discharge protect 
  esp_sleep_enable_timer_wakeup(SLEEP * 1000000);                                                     // Set timer for Deep sleep mode (parameter SLEEP)
  esp_deep_sleep_start();                                                                             // Enable deep sleep mode - Current consumption WSL in deep sleep mode: 1.8mA
}

void loop()
{
}

void water_switch(float soil_mos, int waterLevel)                                                     // Function for irrigation management (if soil moisture is less than the set parameter and water level in tank is higher than the set parameter, is turn on pump to parameter time)
  {
  if(soil_mos < SOIL_MOS_MIN & waterLevel > WATER_LIMIT)
    {
    digitalWrite(SWITCH, HIGH);
    delay(TIME_WATER*60000);
    digitalWrite(SWITCH, LOW);    
    }
  else
    {
    }
  }

float UV_Index(float voltage)                                                                         // Function for calculatin UV index from analog voltage (according to the linearized characteristic given by the datasheet UMV30A)
  {
  int volt = constrain(voltage, 0, 1171);
  float UV_index;
  if(volt <= 230)
    {
    UV_index = volt / 230;
    }
  else if (volt > 230 & volt < 1170)
    {
    UV_index = (volt - 136) / 94;
    }
  else
  {
    UV_index = 12;
  }
return UV_index;
  }

void UnderVoltProt(float aku_volt)                                                                  // Function for under discharge protect (if aku voltage is lower than the set parameter, is permanently enabled deep sleep mode)
  {
  if(aku_volt < 3200)
    {
    esp_deep_sleep_start();
    }
  }

int Water_level()                                                                                   // Function for getting distance value from ultrasonic sensor (US-100) via UART1
{
    uint8_t high;
    uint8_t low;
    hwSerial_1.write(0x55);
    delay(350); 
    if (hwSerial_1.available()) 
    {
      high = hwSerial_1.read();
      low = hwSerial_1.read();
    }
    return (high * 256 + low);                                                                      // Return calculated value of distance
}

void serialDiagnostic (int8_t temp, uint8_t soil_mos, uint8_t batteryLevel, float aku_volt, int distance, uint8_t waterLevel, int bright, uint16_t pressure, uint8_t uvIndex, int UV_volt, uint8_t humidity, uint8_t parity_control, int MS_time, int TX_time)
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
  Serial.print(map(batteryLevel, 0, 250, 3200, 4200));
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
  Serial.print("UV voltage: ");
  Serial.print(UV_volt);
  Serial.println(" mV");
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
double ReadVoltage(byte pin){
  double reading = analogRead(pin);
  return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
}
