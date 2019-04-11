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
 * Setupt Using PCB pin configuration
 * Adafruit NXP IMU
 * Custom Motor Speed Sensors
 * 
 * Optional Settings Config
 * IMU angles and rates
 * Reciever inputs
 * ESC outputs
 * ESC calibration if started with throttle in max position
 * Motor speed sensor ADC values
 * Detect if transmitter is connected to reciever
 *
 * 4/9/19
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

// Motor speed settings
// Set mRPM to Max RPM measurement at max ADC value
// set mADC to Max ADC reading, according to resolution
int mRPM = 10000;
int mADC = 1024;
int RPMconv = mRPM/mADC;

///////////////////////////////////////
// Pins
//////////////////////////////////////
// ESC Pins
 int escOut1 = 6;
 int escOut2 = 5;
 int escOut3 = 10;
 int escOut4 = 20;

 int led = 13;

//Rx Pins
 int ch1 = 24;
 int ch2 = 25;
 int ch3 = 26;
 int ch4 = 27;
 int ch5 = 28;
 int ch6 = 29;

 //Motor Speed Sensor Pins
 int IRmtr1 = 39;
 int IRmtr2 = 38;
 int IRmtr3 = 37;
 int IRmtr4 = 36;

 //Optional Settings Pins
 int opt1 = 17;
 int opt2 = 16;
 int opt3 = 15;

 //Reciever LED
 int txOn = 30;
///////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////

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
 unsigned long prev6 = 0;
 volatile unsigned long delta6= 1500;
 long lastUpdate = 0;
 long lastPrint = 0;
 long txTime = 0;
 bool transmitterOn = false;
 bool remoteCheck = true;
 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;

 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

 // IMU 
 float pitch, roll, yaw;
 float pitch_rate, roll_rate, yaw_rate;

 // Motor speeds
 int mtr1, mtr2, mtr3, mtr4;

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

//  
void transmitterTimer(){
txTime = elapsedTime;
}


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

///////////////////////////////////////////////////////////////////////
// Motor Speed
///////////////////////////////////////////////////////////////////////

void readMtrSpeed(){
 // Read outputs of motor speed sensors
  mtr1 = analogRead(IRmtr1);
  mtr2 = analogRead(IRmtr2);
  mtr3 = analogRead(IRmtr3);
  mtr4 = analogRead(IRmtr4);
}

void convertMtrSpeed(){
  // Set motor speeds in terms of RPM
  mtr1 = mtr1 * (RPMconv);
  mtr2 = mtr2 * (RPMconv);
  mtr3 = mtr3 * (RPMconv);
  mtr4 = mtr4 * (RPMconv);
}

///////////////////////////////////////////////////////////////////////
// Reciever/Transmitter Check
///////////////////////////////////////////////////////////////////////

void transmitterCheck()
{
  // Check if the transmitter output is constant
  if ( elapsedTime - txTime > 1000000)
 {
    transmitterOn = true;
 }
  else
  {
    transmitterOn = false;
  }

  // If the output value is constant HIGH, then connection is broken
  if (digitalReadFast(txOn) == HIGH)
  {
    transmitterOn = false;
  }
}


///////////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(led,OUTPUT);
  Serial.begin(115200);

  //Setup Option pins
  pinMode(opt1,INPUT_PULLUP);
  pinMode(opt2,INPUT_PULLUP);
  pinMode(opt3,INPUT_PULLUP);
  pinMode(txOn,INPUT_PULLUP);

  // Option 1 allows skipping the wait for serial
  if(digitalReadFast(opt1) == LOW)
  {
    waitSerial = false;
  }

  // Option 2 sets ESC PWM resolution to 10 bit vs 8
  if(digitalReadFast(opt2) == LOW)
  {
    pwmRes = 10;
    pwmMax = 1024;
    escInit = pwmMax/4;
  }

  // Option 3 turns off remote check
  if(digitalReadFast(opt3) == LOW)
  {
    remoteCheck = false;
  }
 
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
  Serial.print("PWM Resolution  ");
  Serial.println(pwmMax);
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
  attachInterrupt(ch5,ch5Int,CHANGE);
  attachInterrupt(ch6,ch6Int,CHANGE);
  attachInterrupt(txOn,transmitterTimer,CHANGE);
  
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
     Serial.print("ESC Init Pulse : ");
     Serial.println(escInit);
     analogWrite(escOut1, escInit);
     analogWrite(escOut2, escInit);
     analogWrite(escOut3, escInit);
     analogWrite(escOut4, escInit);
     delay(5000);
  }
  digitalWrite(led,HIGH);
  filter.begin(updateFreq);
  Serial.print("Setup Finished!");
  delta3=1000;
  delay(1000);

}



////////////////////////////////////////////////
// Main
/////////////////////////////////////////////
void loop() {

 if (remoteCheck == true)
 {
   transmitterCheck();
  while(transmitterOn == false){
     transmitterCheck();
     analogWrite(escOut1, escInit);
     analogWrite(escOut2, escInit);
     analogWrite(escOut3, escInit);
     analogWrite(escOut4, escInit);
  }
 }

  // Calculate ESC pulse for PWM resolution using throttle
   escPulse1 = delta3*(pwmMax)/escPulseTime;
   escPulse2 = delta3*(pwmMax)/escPulseTime;
   escPulse3 = delta3*(pwmMax)/escPulseTime;
   escPulse4 = delta3*(pwmMax)/escPulseTime;
  
  
  // Create PWM for ESCs
   analogWrite(escOut1,escPulse1);
   analogWrite(escOut2,escPulse2);
   analogWrite(escOut3,escPulse3);
   analogWrite(escOut4,escPulse4);

  readMtrSpeed();

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

  Serial.print("ESC 1 :");
  Serial.println(escPulse1);
  Serial.print("ESC 2 :");
  Serial.println(escPulse2);
  Serial.print("ESC 3 :");
  Serial.println(escPulse3);
  Serial.print("ESC 4 :");
  Serial.println(escPulse4);

  Serial.print("Mtr 1 ADC :");
  Serial.println(mtr1);
  Serial.print("Mtr 2 ADC :");
  Serial.println(mtr2);
  Serial.print("Mtr 3 ADC :");
  Serial.println(mtr3);
  Serial.print("Mtr 4 ADC :");
  Serial.println(mtr4);

  Serial.print("Opt 1 :");
  Serial.println(digitalRead(opt1));
  Serial.print("Opt 2 :");
  Serial.println(digitalRead(opt2));
  Serial.print("Opt 3 :");
  Serial.println(digitalRead(opt3));

  Serial.print("Transmitter On : ");
  Serial.println(transmitterOn);
  Serial.println(digitalReadFast(txOn));

  lastPrint = elapsedTime;
}
}

