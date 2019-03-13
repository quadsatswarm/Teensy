
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <Madgwick.h>
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002


/* 
 * QuadSat Hardware Test
 * Teensy 3.5
 * Adafruit NXP IMU
 * 
 * IMU angles and rates
 * Reciever inputs
 * ESC outputs
 * ESC calibration if started with throttle in max position
 *
 * 3/12/2019
 */


Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
elapsedMicros elapsedTime;


/////////////////////////////////////////
// IMU Calibration
/////////////////////////////////////////

// Offsets applied to raw x/y/z mag values
 float mag_offsets[3] = { 0.93F, -7.47F, -35.23F };

// Soft iron error compensation matrix
 float mag_softiron_matrix[3][3] = { {  0.943,  0.011,  0.020 },
                                    {  0.022,  0.918, -0.008 },
                                    {  0.020, -0.008,  1.156 } };

 float mag_field_strength = 50.23F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
 float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

////////////////////////////////////////
// Configurations
////////////////////////////////////////

// PWM Settings
 int pwmFreq = 250;
 int pwmRes = 8;
 int escPulseTime = 4000;

// Filter sample rate
 int updateFreq = 250;
 int updateTime = 4000;

// Filter type
 //Mahony filter;
 Madgwick filter;

// How often to print variables
 int printTimer = 5000;

// Wait for Serial ?
 bool waitSerial = true;

///////////////////////////////////////
// Pins
//////////////////////////////////////

// ESC Pins
 int escOut1 = 5;
 int escOut2 = 6;
 int escOut3 = 10;
 int escOut4 = 20;

 int led = 13;

//Rx Pins
 int ch1 = 7;
 int ch2 = 8;
 int ch3 = 14;
 int ch4 = 35;
 int ch5 = 33;
 int ch6 = 34;

 int IMUint = 33;

///////////////////////////////////
// Variables
//////////////////////////////////

// ESC Control Pulses
 int escPulse1, escPulse2, escPulse3, escPulse4;

// Reciever Pulses
 int rxPulse1;
 int rxPulse2;
 int rxPulse3;
 int rxPulse4;

// Timing Variables for Pulse Width
 unsigned long prev1 = 0;
 volatile unsigned long delta1 = 0;
 unsigned long prev2 = 0;
 volatile unsigned long delta2 = 0;
 unsigned long prev3 = 0;
 volatile unsigned long delta3 = 0;
 unsigned long prev4 = 0;
 volatile unsigned long delta4 = 0;
 unsigned long prev5 = 0;
 volatile unsigned long delta5= 1500;
 unsigned long prev6= 0;
 volatile unsigned long delta6= 1500;
 long lastUpdate = 0;
 long lastPrint = 0;

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

// IMU 
 float pitch, roll, yaw;
 float pitch_rate, roll_rate, yaw_rate;
///////////////////////////////////////////////////////////////////////
// IMU
//////////////////////////////////////////////////////////////////////

void getIMU()
{
   sensors_event_t gyro_event;
   sensors_event_t accel_event;
   sensors_event_t mag_event;
   gyro.getEvent(&gyro_event);
   accelmag.getEvent(&accel_event, &mag_event);

   // Apply mag offset compensation (base values in uTesla)
   float x = mag_event.magnetic.x - mag_offsets[0];
   float y = mag_event.magnetic.y - mag_offsets[1];
   float z = mag_event.magnetic.z - mag_offsets[2];

   // Apply mag soft iron error compensation
   float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
   float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
   float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

   // Apply gyro zero-rate error compensation
   float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
   float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
   float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

   // The filter library expects gyro data in degrees/s, but adafruit sensor
   // uses rad/s so we need to convert them first (or adapt the filter lib
   // where they are being converted)
   gx *= 57.2958F;
   gy *= 57.2958F;
   gz *= 57.2958F;

    filter.update(gx, gy, gz,
                 accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                 mx, my, mz);

   pitch = filter.getRoll();
   roll = filter.getPitch();
   yaw = filter.getYaw();
   yaw_rate = gyro_event.gyro.z;
   pitch_rate = gyro_event.gyro.x;
   roll_rate = gyro_event.gyro.y;
}

///////////////////////////////////////////////////////////////////////
// Interrupts
///////////////////////////////////////////////////////////////////////
 // Get pulse timing for each of the rx channels

void ch5Int()
{
  if (digitalReadFast(ch5)){
    prev5 = micros();
  }
  else{
    delta5 = micros() - prev5;
    
  }
}

void ch6Int()
{
  if (digitalReadFast(ch6)){
    prev6 = micros();
  }
  else{
    delta6 = micros() - prev6;
   
  }
}

void ch1Int()
{
  if (digitalReadFast(ch1)){
    prev1 = micros();
  }
  else{
    delta1 = micros() - prev1;
  }
}

void ch2Int()
{
  if (digitalReadFast(ch2)){
    prev2 = micros();
  }
  else{
    delta2 = micros() - prev2;
  }
}
 
void ch3Int()
{
   if (digitalReadFast(ch3)){
    prev3 = micros();
   }
   else{
    delta3 = micros() - prev3;
   }
}
 
  void ch4Int()
{
   if (digitalReadFast(ch4)){
    prev4 = micros();
   }
   else{
    delta4 = micros() - prev4;
   }
}

////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
  pinMode(led,OUTPUT);
  Serial.begin(115200);
 
 if (waitSerial == true){
 while (!Serial) {
     // wait for serial port to connect.
  }}

  Serial.println("Setup ... ");
  delay(1000);
  Serial.print("Filter Update Frequency  ");
  Serial.println(updateFreq);
  Serial.print("Filter Update Timer  ");
  Serial.println(updateTime);
  delay(1000);

  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

  // Setup ESC Pins
  pinMode(escOut1, OUTPUT);
  pinMode(escOut2, OUTPUT);
  pinMode(escOut3, OUTPUT);
  pinMode(escOut4, OUTPUT);

  //Setup rx pin interrupts
  attachInterrupt(ch1,ch1Int,CHANGE);
  attachInterrupt(ch2,ch2Int,CHANGE);
  attachInterrupt(ch3,ch3Int,CHANGE);
  attachInterrupt(ch4,ch4Int,CHANGE);
  
  // All on Timer FTM0 -> pwmFreq
  analogWriteFrequency(escOut1, pwmFreq);

  // Set PWM resolution
  analogWriteResolution(pwmRes);

  delay(3000);

  if(delta3 > 1800) {
    Serial.println("Calibrate ESCs");
    Serial.println("Keep throttle in maximum position");
    Serial.println("Wait For beeping to stop");
    Serial.println("Then place throttle in minimum position");
    delay(5000);
  }
  else{
    Serial.println("Initializing ESCs");
     analogWrite(escOut1, escInit);
     analogWrite(escOut2, escInit);
     analogWrite(escOut3, escInit);
     analogWrite(escOut4, escInit);
     delay(5000);
  }
  digitalWrite(led,HIGH);
  filter.begin(updateFreq);
  Serial.print("Setup Finished!");
  delay(1000);

}

////////////////////////////////////////////////
// Main
/////////////////////////////////////////////
void loop() {

  // Calculate ESC pulse for PWM resolution using throttle
  escPulse1 = delta3*(pwmMax)/escPulseTime;
  escPulse2 = delta3*(pwmMax)/escPulseTime;
  escPulse3 = delta3*(pwmMax)/escPulseTime;
  escPulse4 = delta3*(pwmMax)/escPulseTime;
  
  // Create PWM for ESCs
  analogWrite(5,escPulse1);
  analogWrite(6,escPulse2);
  analogWrite(10,escPulse3);
  analogWrite(20,escPulse4);

if ((elapsedTime - lastUpdate) > updateTime)
{
   lastUpdate = elapsedTime;
   getIMU();
}


if ((elapsedTime - lastPrint) >= printTimer)
{ 
  Serial.print("Time Elapsed ");
  Serial.println(elapsedTime);
  Serial.print("Pitch: ");
  Serial.println(pitch);
  Serial.print("Roll: ");
  Serial.println(roll);
  Serial.print("Yaw: ");
  Serial.println(yaw);

  Serial.print("Pitch Rate: ");
  Serial.println(pitch_rate);
  Serial.print("Roll Rate: ");
  Serial.println(roll_rate);
  Serial.print("Yaw Rate: ");
  Serial.println(yaw_rate);

  Serial.print("CH1 :");
  Serial.println(delta1);
  Serial.print("CH2 :");
  Serial.println(delta2);
  Serial.print("CH3 :");
  Serial.println(delta3);
  Serial.print("CH4 :");
  Serial.println(delta4);
  Serial.print("CH5 :");
  Serial.println(delta5);
  Serial.print("CH6 :");
  Serial.println(delta6);

  lastPrint = elapsedTime;
}
}

