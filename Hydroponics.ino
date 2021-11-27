// Example testing sketch for various DHT humidity/temperature sensors written by ladyada
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

//Ambient temp and humidity sensor (AM2301)
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT21   // DHT 21 (AM2301)

//Water temp sensor (ds18b20)
#define ONE_WIRE_BUS 11               //IO11

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

const uint16_t pHPin = 34;            //pH analog input is connected to GPIO 34 (Analog ADC1_CH6)
const uint16_t TDSPin = 35;           //TDS analog input is connected to GPIO 35 (Analog ADC1_CH7)
uint16_t pHVal = 0;
uint16_t TDSVal = 0;
//y = ax + b, where x are the pH sensor readings and y is the actual pH value
//measured using water (7 pH) and lemon extract (2 pH)
const float a = -9.75;
const float b = 22.11;
// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
LiquidCrystal_I2C LCD(0x3F, 16,2);  //

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  delay(100);
  LCD.init();
  LCD.backlight();
  delay(100);  
  ds18b20.begin();
  delay(100);
  if (ds18b20.getDS18Count() != 0) ds18b20.setResolution(12);
  delay(100);
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);
  LCD.setCursor(0, 1);
  uint8_t Water_Temp = 0;
  pHVal = a*analogRead(pHPin) + b;
  TDSVal = analogRead(TDSPin);
  if (ds18b20.getDS18Count() != 0) 
  {
    ds18b20.requestTemperatures();
    Water_Temp = ds18b20.getTempCByIndex(0); 
  } 
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || (Water_Temp == 0)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    //return;
  }

  Serial.print(F("Humidity: "));
  Serial.print(h);

}
