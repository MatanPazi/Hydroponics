/*
 * DHT part credited to http://www.esp32learning.com/code/esp32-and-am2301-sensor-example.php
 * Connecting to ESP32:
 * https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
 * Analog inputs:
 * https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
 * I2C LCD Display:
 * https://www.circuitschools.com/interfacing-16x2-lcd-module-with-esp32-with-and-without-i2c/#Method_2_Interfacing_16X2_LCD_module_with_ESP_32_using_I2C_adapter
 */

#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
//
#define DHTPIN 12                     //Modify to the pin we're connected to (IO 12). 
#define DHTTYPE DHT21                 //AM2301 
#define ONE_WIRE_BUS 12               //IO12
//
const uint16_t pHPin = 34;            //pH analog input is connected to GPIO 34 (Analog ADC1_CH6)
const uint16_t TDSPin = 35;           //TDS analog input is connected to GPIO 35 (Analog ADC1_CH7)
uint16_t pHVal = 0
uint16_t TDSVal = 0

DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C LCD(0x27, 16,2);  //Will need to scan for I2C addresses if 0x27 isn't correct
 
void setup() 
{
  //ADC default attenuation is 11db , so no need to set it (~2600 mV FS).
  LCD.init();
  LCD.backlight();
  Serial.begin(9600); 
  sensors.begin();
  delay(200);
  if (sensors.getDS18Count() != 0) sensors.setResolution(12););
  dht.begin();
}
 
void loop() 
{
  LCD.setCursor(0, 1);
  uint8_t Water_Temp = 0;
  pHVal = analogRead(pHPin);
  TDSVal = analogRead(TDSPin);
  if (sensors.getDS18Count() != 0) 
  {
    sensors.requestTemperatures();
    Water_Temp = sensors.getTempCByIndex(0); 
  } 
  float Humidity = dht.readHumidity();
  float Amb_Temp = dht.readTemperature();
  if (isnan(Amb_Temp) || isnan(Humidity) || Water_Temp == 0)   //Check if returns are valid, if they are NaN (not a number) then something went wrong!
  {
    Serial.println("Failed to read from AM2301 or DS1820");
  } 
  else
  {
    Serial.print("Humidity: "); 
    Serial.print(Humidity);
    Serial.print(" %\t");
    Serial.print("Ambient Temperature: "); 
    Serial.print(Amb_Temp);
    Serial.print(" C");
    Serial.print("\t");
    Serial.print("Water Temperature");
    Serial.print(Water_Temp);
    Serial.println(" C");
    // Scroll through the data and present on the LCD
    delay(5000);
 }
}
