#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>



/* 

 Revised flight code using Teensy 3.5
 2/17/19

 */


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
boolean auto_level = true;                 

//////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
//////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.02;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 5  ;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 2;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.005;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


int pidDiv = 5;                           // Adjust scaling for rad/s response (500-8)/pidDiv
int dtimer = 3500;
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

 elapsedMicros elapsedTime;
 unsigned long escLoopTime = 0;
 unsigned long timer_channel_1 = 0;

//////////////////////////////////////////////////////////////////////////
// byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
 byte eeprom_data[36];
// byte highByte, lowByte;
// volatile int delta1, delta2, delta3, delta4;
// int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
// int escPulse1, escPulse2, escPulse3, escPulse4;

 int throttle, battery_voltage;
 int start = 0;
 float roll_level_adjust, pitch_level_adjust;
 long pid_error_temp;
 long pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
 long pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
 long pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
 boolean gyro_angles_set;

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

// IMU 
 const int MPU_addr=0x68;  // I2C address of the MPU-6050
 int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 int16_t AcX_Cal,AcY_Cal,AcZ_Cal,Tmp_Cal,GyX_Cal,GyY_Cal,GyZ_Cal;
 int gyro_address=0x68, acc_axis[4], gyro_axis[4], cal_int;
 long gyro_pitch, gyro_roll, gyro_yaw;
 float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
 double gyro_axis_cal[4];
 int temperature, loop_counter;
 long acc_x, acc_y, acc_z, acc_total_vector[20];
 boolean first_angle;
 unsigned long loop_timer = 1563;
 unsigned long timer1;
 float timerMult;
 float timerMult2;
 long printTimer;
 unsigned long prevLoop =0;
 int offset = 0;
 long lastESC = 0;

 int escPulse1PWM;
 int escPulse2PWM;
 int escPulse3PWM;
 int escPulse4PWM;




//////////////////////////////////////////////////////////////////////////
// IMU Functions
//////////////////////////////////////////////////////////////////////////

void set_gyro_registers(){
  //Setup the MPU-6050
    Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
    Wire.write(0x6B); //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00); //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission(); //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
    Wire.write(0x1B); //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08); //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission(); //End the transmission with the gyro

    Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
    Wire.write(0x1C); //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10); //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(); //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address); //Start communication with the address found during search
    Wire.write(0x1B); //Start reading @ register 0x1B
    Wire.endTransmission(); //End the transmission
    Wire.requestFrom(gyro_address, 1); //Request 1 bytes from the gyro
    while(Wire.available() < 1); //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){ //Check if the value is 0x08
      digitalWrite(13,HIGH); //Turn on the warning led
      while(1)delay(10); //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address); //Start communication with the address found during search
    Wire.write(0x1A); //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03); //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
   Wire.endTransmission(); //End the transmission with the gyro
}

void gyro_signalen(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gyro_roll = GyX-GyX_Cal;
  gyro_pitch = GyY-GyY_Cal;
  gyro_yaw = GyZ-GyZ_Cal;

  acc_x = AcX;
  acc_y = AcY;
  acc_z = AcZ;

  // gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  // if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  // gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  // if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  // gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  // if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  // acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  // if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  // acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  // if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  // acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  // if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}
 


void calibrate_gyro(){

    GyX_Cal = 0;
    GyY_Cal = 0;
    GyZ_Cal = 0;

  Serial.println("Calibrating Gyro");
  for(cal_int =0; cal_int<2000; cal_int++ )
  {
    //delay(1);
    gyro_signalen();
    GyX_Cal = GyX+GyX_Cal;
    GyY_Cal = GyY+GyY_Cal;
    GyZ_Cal = GyZ+GyZ_Cal;
  }

   GyX_Cal = GyX_Cal/2000;
   GyY_Cal = GyY_Cal/2000;
   GyZ_Cal = GyZ_Cal/2000;

  Serial.println("Gyro Calibration ");
  Serial.println(GyX_Cal);
  Serial.println(GyY_Cal);
  Serial.println(GyZ_Cal);
  delay(100);
}

//////////////////////////////////////////////////////////////////////////
// Interrupts
//////////////////////////////////////////////////////////////////////////

 // Get pulse timing for each of the rx channels
 
 void ch1Int(){
  if (digitalReadFast(ch1)){
    prev1 = micros();
  }
  else{
    delta1 = micros() - prev1;
  }
 }

 void ch2Int(){
  if (digitalReadFast(ch2)){
    prev2 = micros();
  }
  else{
    delta2 = micros() - prev2;
  }
 }
 
  void ch3Int(){
   if (digitalReadFast(ch3)){
    prev3 = micros();
   }
   else{
    delta3 = micros() - prev3;
   }  
  }
 
  void ch4Int(){
   if (digitalReadFast(ch4)){
    prev4 = micros();
   }
   else{
    delta4 = micros() - prev4;
   }
 }


//////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////
void setup() {

  //for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  //start = 0; //Set start back to zero.

  pinMode(led,OUTPUT);
  Serial.begin(115200);

  //Setup IMU
  //I2C
  Wire.begin();
  Wire.setClock(4000000);

  // Registers and Cal
  set_gyro_registers();
  calibrate_gyro();

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

  // Initialize ESCs
  analogWrite(escOut1, escInit);
  analogWrite(escOut2, escInit);
  analogWrite(escOut3, escInit);
  analogWrite(escOut4, escInit);
  delay(5000);

  // Turn on LED exit setup 
  digitalWrite(led,HIGH);

}

//////////////////////////////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////////////////////////////
void calcAngles(){
  
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  // Gyro_angle calculations
  // Loop time used to integrate between samples
  timerMult = loop_timer * 0.000001/ 65.5;
  angle_pitch += gyro_pitch * timerMult;       
  angle_roll += gyro_roll * timerMult;         

  timerMult2 = timerMult * 3.142 / 180;
  angle_pitch -= angle_roll * sin(gyro_yaw * timerMult2);                         
  angle_roll += angle_pitch * sin(gyro_yaw * timerMult2); 

  //Accelerometer angle calculations
  acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector[0])* 57.296;                //Calculate the pitch angle.
  angle_roll_acc = asin((float)acc_x/acc_total_vector[0])* -57.296;                //Calculate the roll angle.
 
  if(!first_angle){
    angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
    angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
    first_angle = true;
 }
 else{
   angle_pitch = angle_pitch * 0.998 + angle_pitch_acc * 0.002;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
   angle_roll = angle_roll * 0.998 + angle_roll_acc * 0.002;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
 }
     if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
     pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
     roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
   }
}
void calcPulse(){


 throttle = delta3;                                              //We need the throttle signal as a base signal.

  if (start == 2){                                                //The motors are started.
                                                          
    if (throttle > 1800) throttle = 1800;                                       //We need some room to keep full control at full throttle.
    escPulse1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    escPulse2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    escPulse3 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    escPulse4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

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

void calculate_pid(){

  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;

  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;

  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  if(pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;

  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;


}

void inputs(){
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

void escOutputs(){

   //Create PWM for ESCs
    analogWrite(escOut1,escPulse1PWM);
    analogWrite(escOut2,escPulse2PWM);
    analogWrite(escOut3,escPulse3PWM);
    analogWrite(escOut4,escPulse4PWM);

}

void escPulseOutput() {
  unsigned long zero_timer = micros();
  digitalWriteFast(5, HIGH);
  digitalWriteFast(6, HIGH);
  digitalWriteFast(10, HIGH);
  digitalWriteFast(20, HIGH);

  // PORTD |= B11110000;                                          //Set port 4, 5, 6 and 7 high at once
   timer_channel_1 = escPulse1 + zero_timer;                          //Calculate the time when digital port 4 is set low.
  unsigned long timer_channel_2 = escPulse2 + zero_timer;                          //Calculate the time when digital port 5 is set low.
  unsigned long timer_channel_3 = escPulse3 + zero_timer;                          //Calculate the time when digital port 6 is set low.
  unsigned long timer_channel_4 = escPulse4 + zero_timer;                          //Calculate the time when digital port 7 is set low.

  while( (digitalReadFast(5) == HIGH) || (digitalReadFast(6) == HIGH) || (digitalReadFast(10) == HIGH) || (digitalReadFast(20) == HIGH)){                                                      //Execute the loop until digital port 4 to 7 is low.
    unsigned long esc_loop_timer = micros();                                             //Check the current time.
    if(timer_channel_1 <= esc_loop_timer)digitalWriteFast(5,LOW);     //When the delay time is expired, digital port 4 is set low.
    if(timer_channel_2 <= esc_loop_timer)digitalWriteFast(6,LOW);     //When the delay time is expired, digital port 5 is set low.
    if(timer_channel_3 <= esc_loop_timer)digitalWriteFast(10,LOW);     //When the delay time is expired, digital port 6 is set low.
    if(timer_channel_4 <= esc_loop_timer)digitalWriteFast(20,LOW);     //When the delay time is expired, digital port 7 is set low.
  }
}


void displayVals(){

if (debug == true){
  if ((timer1 - prevLoop) >= 50000)
  { 
  printTimer = timerMult * 10000000;
  Serial.print("Start ");
  Serial.println(start);
  Serial.print("Time Elapsed ");
  Serial.println(elapsedTime);
  Serial.print("Loop Time ");
  Serial.println(loop_timer);
  Serial.print("Time Multi E-7 * ");
  Serial.println(printTimer);
  Serial.print("Pitch: ");
  Serial.println(angle_pitch);
  Serial.print("Roll: ");
  Serial.println(angle_roll);
  Serial.print("Yaw: ");
  Serial.println(gyro_yaw / 65.5 );

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


  Serial.print("CH1 :");
  Serial.println(delta1);
  Serial.print("CH2 :");
  Serial.println(delta2);
  Serial.print("CH3 :");
  Serial.println(delta3);
  Serial.print("CH4 :");
  Serial.println(delta4);
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


    prevLoop = elapsedTime;
  } }
}

//////////////////////////////////////////////////////////////////////////
// Main Loop
//////////////////////////////////////////////////////////////////////////
void loop() {
  //Setup timer for IMU loop calcs and get current measurement
  timer1 = elapsedTime;

  gyro_signalen();

  calcAngles();

  inputs();

  calculate_pid();

  calcPulse();
 
  escOutputs();
  //escPulseOutput();

  displayVals();

  //delayMicroseconds(dtimer);

   loop_timer = elapsedTime  - timer1;
}

