#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>
#include <Arduino.h>
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <PID_v1.h>

/* 
 * QuadSat PID Flight Controller
 * Teensy 3.5
 * Adafruit NXP IMU
 *
 * 3/12/2019
 */


Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
elapsedMicros elapsedTime;

///////////////////////////////////////////////////////////////////////////
// IMU Calibration
///////////////////////////////////////////////////////////////////////////

// Offsets applied to raw x/y/z mag values
 float mag_offsets[3] = { 0.93F, -7.47F, -35.23F };

// Soft iron error compensation matrix
 float mag_softiron_matrix[3][3] = { {  0.943,  0.011,  0.020 },
                                    {  0.022,  0.918, -0.008 },
                                    {  0.020, -0.008,  1.156 } };

 float mag_field_strength = 50.23F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
 float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

//////////////////////////////////////////////////////////////////////////
// Configurations
//////////////////////////////////////////////////////////////////////////

// PWM Settings
 int pwmFreq = 250;
 int pwmRes = 8;
 bool debug = true;

// Total pulse length for esc
 int escPulseTime = 4000;

//Auto level on (true) or off (false)
 bool auto_level = true;

// Filter sample rate
 int updateFreq = 250;
 int updateTime = 4000;

// Filter type
 //Mahony filter;
 Madgwick filter;

 // Wait for Serial ?
 bool waitSerial = false;

 int yaw_gain = 6;

//////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
//////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 1.3;                 //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18;                 //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4;                  //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                  //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

int pidDiv = 3;                            // Adjust scaling for rad/s response (500-8)/pidDiv

//////////////////////////////////////////////////////////////////////////
// Pins
//////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////
// Variables
//////////////////////////////////////////////////////////////////////////

// ESC Control Pulses
 int escPulse1, escPulse2, escPulse3, escPulse4;

// Reciever Pulses
 int rxPulse1;
 int rxPulse2;
 int rxPulse3;
 int rxPulse4;

// Timing Variables for Pulse Width
 unsigned long prev1 = 0;
 volatile unsigned long delta1 = 1500;
 unsigned long prev2 = 0;
 volatile unsigned long delta2 = 1500;
 unsigned long prev3 = 0;
 volatile unsigned long delta3 = 1500;
 unsigned long prev4 = 0;
 volatile unsigned long delta4 = 1500;
 unsigned long prev5 = 0;
 volatile unsigned long delta5= 1500;
 unsigned long prev6= 0;
 volatile unsigned long delta6= 1500;

 int escPulse1PWM;
 int escPulse2PWM;
 int escPulse3PWM;
 int escPulse4PWM;

 unsigned long escLoopTime = 0;
 unsigned long timer_channel_1 = 0;

//////////////////////////////////////////////////////////////////////////
 //byte eeprom_data[36];
 int throttle, battery_voltage;
 int start = 0;
 float roll_level_adjust, pitch_level_adjust;
 long pid_error_temp;
 long pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
 long pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
 long pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
 bool gyro_angles_set;

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

// IMU 
 float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
 bool first_angle;
 long loop_timer = 0;
 long prevPrint = 0;
 float pitch, roll, yaw;
 float pitch_rate, roll_rate, yaw_rate;
//////////////////////////////////////////////////////////////////////////
// IMU Functions
//////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////
// Interrupts
//////////////////////////////////////////////////////////////////////////

 // Get pulse timing for each of the rx channels
 
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

//////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////
void setup() 
{
  pinMode(led,OUTPUT);
  Serial.begin(115200);


 if (waitSerial == true){
 while (!Serial) {
     // wait for serial port to connect.
  }}

  Serial.println("Setup Start");

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
  attachInterrupt(ch5,ch5Int,CHANGE);
  attachInterrupt(ch6,ch6Int,CHANGE);
  
  // All on Timer FTM0 -> pwmFreq
  analogWriteFrequency(escOut1, pwmFreq);

  // Set PWM resolution
  analogWriteResolution(pwmRes);

  // Initialize ESCs
  
  analogWrite(escOut1, escInit);
  analogWrite(escOut2, escInit);
  analogWrite(escOut3, escInit);
  analogWrite(escOut4, escInit);
  delay(5000);

  // Setup filter to get IMU data
  filter.begin(updateFreq);

  // Turn on LED exit setup 
  digitalWrite(led,HIGH);

  Serial.println("Setup Finished");

}

//////////////////////////////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////////////////////////////

void calcPulse()
{


 throttle = delta3;                                              //We need the throttle signal as a base signal.

  if (start == 2){                                                //The motors are started.
                  
    if (throttle > 1800) throttle = 1800;                                       //We need some room to keep full control at full throttle.
    
    escPulse1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    escPulse2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    escPulse3 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    escPulse4 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    // if (battery_voltage < 1240 && battery_voltage > 800){                            //Is the battery connected?
    //   escPulse1 += escPulse1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
    //   escPulse2 += escPulse2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
    //   escPulse3 += escPulse3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
    //   escPulse4 += escPulse4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    // } 

    if (escPulse1 < 1100) escPulse1 = 1100;                                         //Keep the motors running.
    if (escPulse2 < 1100) escPulse2 = 1100;                                         //Keep the motors running.
    if (escPulse3 < 1100) escPulse3 = 1100;                                         //Keep the motors running.
    if (escPulse4 < 1100) escPulse4 = 1100;                                         //Keep the motors running.

    if(escPulse1 > 2000)escPulse1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(escPulse2 > 2000)escPulse2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(escPulse3 > 2000)escPulse3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(escPulse4 > 2000)escPulse4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    escPulse1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    escPulse2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    escPulse3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    escPulse4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  // Calculate ESC pulse for PWM resolution
   escPulse1PWM = escPulse1*(pwmMax)/escPulseTime;
   escPulse2PWM = escPulse2*(pwmMax)/escPulseTime;
   escPulse3PWM = escPulse3*(pwmMax)/escPulseTime;
   escPulse4PWM = escPulse4*(pwmMax)/escPulseTime;

}

void calculate_pid()
{

  //Roll calculations
  pid_error_temp = roll - pid_roll_setpoint;

  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitch - pid_pitch_setpoint;

  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  if(pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = yaw_rate * yaw_gain - pid_yaw_setpoint;

  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;


}

void inputs()
{
   //For starting the motors: throttle low and yaw left (step 1).
  if(delta3 < 1050 && delta4 < 1050)start = 1;

  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && delta3 < 1050 && delta4 > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && delta3 < 1050 && delta4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;

  // SETUP ROLL SETPOINT
  //We need a little dead band of 16us for better results.
  if(delta1 > 1508)pid_roll_setpoint = delta1 - 1508;
  else if(delta1 < 1492)pid_roll_setpoint =  -(1492-delta1) ;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= pidDiv;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;

  //SETUP PITCH SETPOINT
  //We need a little dead band of 16us for better results.
  if(delta2 > 1508)pid_pitch_setpoint = delta2 - 1508;
  else if(delta2 < 1492)pid_pitch_setpoint = delta2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= pidDiv;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  if(delta3 > 1050){ //Do not yaw when turning off the motors.
    if(delta4 > 1508)pid_yaw_setpoint = (delta4 - 1508);
    else if(delta4 < 1492)pid_yaw_setpoint = (delta4 - 1492);
  }
  pid_yaw_setpoint /= pidDiv;

}

void escOutputs()
{

   //Create PWM for ESCs
    analogWrite(escOut1,escPulse1PWM);
    analogWrite(escOut2,escPulse2PWM);
    analogWrite(escOut3,escPulse3PWM);
    analogWrite(escOut4,escPulse4PWM);
}

void displayVals()
{

 if (debug == true){
  if ((elapsedTime - prevPrint) >= 50000)
  { 
  Serial.print("Start ");
  Serial.println(start);
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

  Serial.print("Pitch Setpoint: ");
  Serial.println(pid_pitch_setpoint);
  Serial.print("Roll Setpoint: ");
  Serial.println(pid_roll_setpoint);
  Serial.print("Yaw Setpoint: ");
  Serial.println(pid_yaw_setpoint);

  Serial.print("Throttle: ");
  Serial.println(throttle);
  Serial.print("PID Pitch: ");
  Serial.println(pid_output_pitch); 
  Serial.print("PID Roll: ");
  Serial.println(pid_output_roll);
  Serial.print("PID Yaw: ");
  Serial.println(pid_output_yaw);
  Serial.print("PID Pitch error: ");
  Serial.println(pid_last_pitch_d_error);
  Serial.print("PID Roll error: ");
  Serial.println(pid_last_roll_d_error);
  Serial.print("PID D Gain: ");
  Serial.println(pid_d_gain_pitch);
  Serial.print("PID P Gain: ");
  Serial.println(pid_p_gain_pitch);
  Serial.print("PID I Gain: ");
  Serial.println(pid_i_gain_pitch);
  

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
  Serial.println("--ESC output--");
  Serial.print("ESC 1 FR ");
  Serial.println(escPulse1);
  Serial.println(escPulse1PWM);
  Serial.print("ESC 2 RR ");
  Serial.println(escPulse2);
  Serial.println(escPulse2PWM);
  Serial.print("ESC 3 RL ");
  Serial.println(escPulse3);
  Serial.println(escPulse3PWM);
  Serial.print("ESC 4 FL ");
  Serial.println(escPulse4);
  Serial.println(escPulse4PWM);
  Serial.print("ESC Time 1 ");
  Serial.println(timer_channel_1);

    prevPrint = elapsedTime;
  } 
  }
}

//////////////////////////////////////////////////////////////////////////
// Main Loop
//////////////////////////////////////////////////////////////////////////
void loop() 
{

 if ((elapsedTime - loop_timer) >= updateTime)
 {

  getIMU();

  inputs();

  calculate_pid();

  calcPulse();
 
  escOutputs();
  
  loop_timer = elapsedTime ;
 }

displayVals();

}

