/*
  MS5611 Barometric Pressure & Temperature Sensor. Simple Example
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/czujnik-cisnienia-i-temperatury-ms5611.html
  GIT: https://github.com/jarzebski/Arduino-MS5611
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski

  Ricky's notes:
  Teensy uses SCL0 and SDA0 pins by default. The wiring diagram based on the wires I have in my barometer is:
  GND (black) to GND
  Vin (red) to 3.3V
  SCL (blue) to pin 19 (SCL0)
  SDA (white) to pin 18 (SDA0)
*/

#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

double referencePressure;

void setup() 
{
  Serial.begin(9600);

  // Initialize MS5611 sensor
  Serial.println("Initialize MS5611 Sensor");

  while(!ms5611.begin())
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // Get reference pressure for relative altitude
  referencePressure = ms5611.readPressure();

  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}

void loop()
{
  // Read raw values
  int t = micros();
  uint32_t rawTemp = ms5611.readRawTemperature();
  uint32_t rawPressure = ms5611.readRawPressure();

  // Read true temperature & Pressure
  double realTemperature = ms5611.readTemperature();
  long realPressure = ms5611.readPressure();

  // Calculate altitude
  float absoluteAltitude = ms5611.getAltitude(realPressure);
//    Serial.print("dtabs = ");
//   Serial.println(micros()-t);
//  float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
//    Serial.print("dtrel = ");
//   Serial.println(micros()-t);
//
//  Serial.println("--");
//
//  Serial.print(" rawTemp = ");
//  Serial.print(rawTemp);
//  Serial.print(", realTemp = ");
//  Serial.print(realTemperature);
//  Serial.println(" *C");
//
//  Serial.print(" rawPressure = ");
//  Serial.println(rawPressure);
//  Serial.print(", realPressure = ");
//  Serial.print(realPressure);
//  Serial.println(" Pa");
//
//  Serial.print(" absoluteAltitude = ");
  Serial.println(absoluteAltitude);
//  Serial.print(" m, relativeAltitude = ");
//  Serial.print(relativeAltitude);    
//  Serial.println(" m");
 
 

  //delay(1000);
}
