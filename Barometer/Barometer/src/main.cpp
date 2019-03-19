#include <Wire.h>
#include <MS5611.h>
 
MS5611 ms5611;
 
 // raw values
  uint32_t rawTemp ;
  uint32_t rawPressure ;
 
  // true temperature & Pressure
  double realTemperature ;
  long realPressure ;
 
  //  altitude
  float absoluteAltitude ;
  float relativeAltitude ;
 

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
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}
 

 
void loop()
{
  // Read raw values
 rawTemp = ms5611.readRawTemperature();
 rawPressure = ms5611.readRawPressure();
 
  // Read true temperature & Pressure
 realTemperature = ms5611.readTemperature();
 realPressure = ms5611.readPressure();
 
  // Calculate altitude
  int s = micros();
 absoluteAltitude = ms5611.getAltitude(realPressure);
  int s1 = micros();
 relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
 
  Serial.println("--");
 /*
  Serial.print(" rawTemp = ");
  Serial.print(rawTemp);
  Serial.print(", realTemp = ");
  Serial.print(realTemperature);
  Serial.println(" *C");
 
  Serial.print(" rawPressure = ");
  Serial.print(rawPressure);
  Serial.print(", realPressure = ");
  Serial.print(realPressure);
  Serial.println(" Pa");
 */

  Serial.print(" absoluteAltitude = ");
  Serial.print(absoluteAltitude);
  Serial.print(" m, relativeAltitude = ");
  Serial.print(relativeAltitude);    
  Serial.println(" m");
 Serial.println("Time: ");
 Serial.println(s1-s);
  delay(1000);
}
 