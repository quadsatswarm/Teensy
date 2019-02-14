#include <Arduino.h>
#include <Wire.h>

/* 
 * Using Teensy 3.5 -
 * Recieves inputs from RF controller, and uses inputs to control motor speeds.
 * Each Rx channel controls one motor.
 * 2/12/19
 */


////////////////////////
// Configurations
///////////////////////

// PWM Settings
 int pwmFreq = 2500;
 int pwmRes = 8;

// Total pulse length for esc
 int escPulseTime = 4000;

///////////////////////////////
// Pins
//////////////////////////////

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

///////////////////////
// Variables
//////////////////////

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

 elapsedMicros elapsedTime;

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

// IMU 
 const int MPU_addr=0x68;  // I2C address of the MPU-6050
 int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 int16_t AcX_Cal,AcY_Cal,AcZ_Cal,Tmp_Cal,GyX_Cal,GyY_Cal,GyZ_Cal;
 int gyro_address=0x68, acc_axis[4], gyro_axis[4], cal_int;
 double gyro_pitch, gyro_roll, gyro_yaw;
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
 int offset = 500;



///////////////////////////////////////////////////////////////////////
// IMU
//////////////////////////////////////////////////////////////////////

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
 
}

void calibrate_gyro(){
  Serial.println("Calibrating Gyro");
  for(cal_int =0; cal_int<2000; cal_int++ )
  {
    if(cal_int % 125 == 0 )
    {
      digitalWrite(13, !digitalRead(13));
    }
    gyro_signalen();
    GyX_Cal = GyX;
    GyY_Cal = GyY;
    GyZ_Cal = GyZ;
  }
  GyX_Cal /= 2000;
  GyY_Cal /= 2000;
  GyZ_Cal /= 2000;
}



///////////////////////////////////////////////////////////////////////
// Interrupts
///////////////////////////////////////////////////////////////////////
 // Get pulse timing for each of the rx channels
 
 void ch1Int(){
  if (digitalReadFast(ch1)){
    prev1 = micros();
  }
  else{
    delta1 = micros() - prev1 - offset;
  }
 }

 void ch2Int(){
  if (digitalReadFast(ch2)){
    prev2 = micros();
  }
  else{
    delta2 = micros() - prev2 - offset;
  }
 }
 
  void ch3Int(){
   if (digitalReadFast(ch3)){
    prev3 = micros();
   }
   else{
    delta3 = micros() - prev3 - offset;
   }
  }
 
  void ch4Int(){
   if (digitalReadFast(ch4)){
    prev4 = micros();
   }
   else{
    delta4 = micros() - prev4 - offset;
   }
 }

////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////
void setup() {
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
    delay(7000);
  // Turn on LED exit setup 
  digitalWrite(led,HIGH);

}

////////////////////////////////////////////////
// Main
/////////////////////////////////////////////
void loop() {

  //Setup timer for IMU loop calcs and get current measurement
  timer1 = elapsedTime;
  gyro_signalen();

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
   angle_pitch = angle_pitch * 0.9975 + angle_pitch_acc * 0.0025;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
   angle_roll = angle_roll * 0.9975 + angle_roll_acc * 0.0025;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
 }


  // Calculate ESC pulse for PWM resolution
  escPulse1 = delta1*(pwmMax)/escPulseTime;
  escPulse2 = delta2*(pwmMax)/escPulseTime;
  escPulse3 = delta3*(pwmMax)/escPulseTime;
  escPulse4 = delta4*(pwmMax)/escPulseTime;
  
   // Create PWM for ESCs
   analogWrite(5,escPulse1);
   analogWrite(6,escPulse2);
   analogWrite(10,escPulse3);
   analogWrite(20,escPulse4);
 

if ((timer1 - prevLoop) >= 50000)
{ 
  printTimer = timerMult * 10000000;
   Serial.print("Time Elapsed ");
  Serial.println(elapsedTime);
  Serial.print("Loop Time ");
  Serial.println(loop_timer);
  Serial.print("Time Multi E-7 ");
  Serial.println(printTimer);
  Serial.print("Pitch: ");
  Serial.println(angle_pitch);
  Serial.print("Roll: ");
  Serial.println(angle_roll);
  Serial.print("Yaw: ");
  Serial.println(gyro_yaw / 65.5 );

  Serial.print("CH1 :");
  Serial.println(delta1+offset);
   Serial.print("CH2 :");
  Serial.println(delta2+offset);
   Serial.print("CH3 :");
  Serial.println(delta3+offset);
   Serial.print("CH4 :");
  Serial.println(delta4+offset);
  Serial.print("ESC output offset by -");
  Serial.println(offset);


    prevLoop = elapsedTime;
}
   loop_timer = elapsedTime  - timer1;
   
}

