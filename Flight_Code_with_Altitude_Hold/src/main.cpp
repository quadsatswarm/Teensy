    
/* ////////////////////////////////////// 
 * //FLIGHT CONTROLLER
 * //////////////////////////////////////
 * 
 * ////////////////////////////////////// 
 * // HARDWARE
 * ////////////////////////////////////// 
 * 
 * MICROCONTROLLER: TEENSY 3.5 
 * 
 * HAND HELD CONTROLLER: https://www.amazon.com/FlySky-FS-i6-M2-2-4GHz-6-Channel-Transmitter/dp/B00PF160IK
 * 
 * MAKE SURE TO SET THE FAIL SAFE MODE ON THE CONTROLLER 
 * REFER TO THIS VIDEO: https://www.youtube.com/watch?v=LXTEXqR_ghI
 * (add settings here ) 
 * 
 * 
 * IMU: NXP 9DOF ADIAFRUIT 
 * https://www.adafruit.com/product/3463
 * 
 * MOTOR: Turnigy Multistar 2213-980Kv 14 Pole Multi-Rotor Outrunner V2
 * https://hobbyking.com/en_us/turnigy-multistar-2213-980kv-14-pole-multi-rotor-outrunner-v2.html
 * 
 * PROPELLERS: 10x4.5 Inch 
 * https://www.amazon.com/uxcell-Propellers-Fixed-Wing-Airplane-Adapter/dp/B07PXKKV3G/ref=sr_1_6?crid=3S7H28DITMXYT&keywords=1045+propeller&qid=1555036892&s=gateway&sprefix=1045+p%2Caps%2C129&sr=8-6
 * 
 * FRAME: 4-Axis Multi Rotor Airframe 450mm
 * https://www.amazon.com/ShareGoo-Airframe-FrameWheel-Quadcopter-Aircraft/dp/B07H3WDSX3/ref=sr_1_fkmrnull_1_sspa?keywords=4-Axis+Multi+Rotor+Airframe+450mm&qid=1555038824&s=gateway&sr=8-1-fkmrnull-spons&psc=1
 * 
 * CREATED BY:
 * JOSHUA WALLACE, KIETH NASON
 * 
 * 4-12-19 
 * 
 * ////////////////////////////////////// 
 * // WARNING!! 
 * //////////////////////////////////////
 * Don't be an idiot. -JOSH
 * 
 * //////////////////////////////////////
 * UPDATED: July 2019 - Jaiden
 * 
 * ###################################################################################################################################################################################
 * 7/20/2019
 * ANDREW:
 * I have labeled all the altitude hold additions with the phrase "AltHold Add-In" so you can locate all the pieces more easily (each piece is bounded by *'s).
 * You will likely have to change variable names to match what you have in your version of the code.  The altitude hold is turned on/off with the other control nob on the controller
 * (again, you may have to switch around a few values if your nob assignments are flipped).  The majority of the barometer code is well commented by Joop (I have a few lines in there that 
 * I added which I put my name after in the pertinent comment if you want to see exactly what I added).  If you have questions shoot me a text, I'll reply as soon as I can.
 * 
 * Good Luck!
 * ~Jaiden
 * ###################################################################################################################################################################################
 *  
*/

/////////////////////////////////////////////////////////
//LIBARIES 
/////////////////////////////////////////////////////////
#include <Arduino.h> 							//Used for Visual Studio's code 
#include <Wire.h>
#include <Adafruit_Sensor.h> 						// IMU libaries 
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>							// Filters for IMU
#include <Madgwick.h>
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002	

/////////////////////////////////////////////////////////
//RANDOM
/////////////////////////////////////////////////////////
elapsedMicros elapsedTime;

/////////////////////////////////////////////////////////
//SWITCHS
/////////////////////////////////////////////////////////
bool debug = false; 
int t=0;
bool autoLevel = true;
//*******************************************************
//AltHold Add-In
bool altitudeHold = true;
//*******************************************************

// Variables for debugging
int printTimer = 5000;
int lastPrint = 0;

////////////////////////////////////////////////////////
//PIN DEFINITIONS
////////////////////////////////////////////////////////

// Reciever  
 int ch1 = 24;			//Right Stick - Horizontal (Roll)
 int ch2 = 25;			//Right Stick - Vertical (Pitch)
 int ch3 = 26;  		//Left Stick - Vertical (Throttle)
 int ch4 = 27;			//Left Stick - Horizontal (Yaw)
 int ch5 = 28;  		//Left Nob (Start)
 int ch6 = 29; 			//Right Nob (Altitude Hold)

// esc
 int escOut1 = 6;
 int escOut2 = 10;
 int escOut3 = 5;
 int escOut4 = 20;

// led 
int led = 13; 

/////////////////////////////////////////////////////////
// PULSE LENGTH AND PWM 
/////////////////////////////////////////////////////////

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 float escInit = pwmMax/4;

 int pwmFreq = 250;
 int pwmRes = 8;
 int escPulseTime = 4000;

 // Pulse length 
 float escPulse1;
 float escPulse2;
 float escPulse3;
 float escPulse4;

 // Value for analog write function 
 float escPulse1PWM;
 float escPulse2PWM;
 float escPulse3PWM;
 float escPulse4PWM;

//************************************************************************************************************************************
//AltHold Add-In

 /////////////////////////////////////////////////////////
 //Global Variables for Barometer
 /////////////////////////////////////////////////////////
uint8_t MS5611_address = 0x00000077;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
uint8_t start;
uint8_t error;

int16_t count_var;

uint32_t loop_timer;

float dummy_float;
int16_t manual_throttle;
uint8_t manual_altitude_change;
float pid_error_temp;

//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
double altitude, referenceAltitude;
double seaLevelPressure = 101325;                     //Needed for altitude calculations, taken from google (same value as used in JoopBarometerTest)
//Altitude PID variables
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;

float pid_p_gain_altitude = 5.0;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.1;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.95;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

//*************************************************************************************************************************************

//////////////////////////////////////////////////////////
// IMU 
/////////////////////////////////////////////////////////
// The IMU is used to get rates and angles

// Variables 
 float pitch;
 float roll;
 float yaw;

 float pitch_prev;
 float roll_prev;
 float yaw_prev;

 float pitch_rate;
 float roll_rate;
 float yaw_rate;


 // Offsets
 float offsetPitch_rate;
 float offsetRoll_rate;
 float offsetYaw_rate;

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Calibration of magnetometer
// Offsets applied to raw x/y/z mag values
 float mag_offsets[3] = { 20.74F, -34.59F, 42.05F };

// Soft iron error compensation matrix
 float mag_softiron_matrix[3][3] = {  { 0.978, -0.035,  0.020 },
								   	  { -0.035,  0.987, -0.042 },
								   	  { 0.020, -0.042,  1.039 } };

 float mag_field_strength = 37.27F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
 float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Filter type
// Mahnony filter is "ligher" than Madwich
 Mahony filter;
 //Madgwick filter;

 // Filter sample rate
 int updateFreq = 250;
 int updateTime = 4000;
 int lastUpdate = 0;

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

	 // Degrees (LOOKS LIKE WE WON'T NEED OFFSET FOR ANGLES)
	 pitch = filter.getRoll();
	 roll = filter.getPitch();
	 yaw = -1*filter.getYaw(); 										// negative sign added to correct sign convention

	 // Degrees per second 
	 pitch_rate = gyro_event.gyro.x*(180/3.14) - offsetPitch_rate;
	 roll_rate = gyro_event.gyro.y*(180/3.14) - offsetRoll_rate;
	 yaw_rate = -1*gyro_event.gyro.z*(180/3.14) - offsetYaw_rate;    // negative sign added to correct sign convention

}

//////////////////////////////////////////////////////////
// IMU CALIBRATION
/////////////////////////////////////////////////////////
// Zero the angular rates

// Variables

float sumPitch_rate;
float sumRoll_rate;
float sumYaw_rate;

void calIMU()
{
	for (int loop = 0; loop < 2000; loop++)
	{
		if((elapsedTime - lastUpdate) > updateTime)
		{
			// Grab angles
			getIMU();

			// Sum rates
			sumPitch_rate += pitch_rate;
			sumRoll_rate += roll_rate;
			sumYaw_rate += yaw_rate;

		}
	}

	// Calculate Angular Rate offsets
	offsetPitch_rate = sumPitch_rate/2000;
	offsetRoll_rate = sumRoll_rate/2000;
	offsetYaw_rate = sumYaw_rate/2000;
	
}

 /////////////////////////////////////////////////////////
 // INTERUPTS
 /////////////////////////////////////////////////////////
 // The interupts are used to obtain the pulse lengths from the hand held controller

 // Timing Variables for Pulse Width
 unsigned long prev1 = 0;
 volatile unsigned int roll_ratePulse = 1500;
 unsigned long prev2 = 0;
 volatile unsigned int pitch_ratePulse = 1500;
 unsigned long prev3 = 0;
 volatile unsigned int throttle_Pulse = 1500;
 //**************************************************
 // AltHold Add-In
 volatile unsigned int throttle, base_throttle;
 //**************************************************
 unsigned long prev4 = 0;
 volatile unsigned int yaw_ratePulse = 1500;
 unsigned long prev5 = 0;
 volatile unsigned int activateMotor = 1500;
 unsigned long prev6 = 0;
 volatile unsigned int activateHold = 1500;

 // Get pulse length 

 // Roll 
 void ch1Int()
{
  if (digitalReadFast(ch1)){
	prev1 = elapsedTime;
  }
  else{
	roll_ratePulse = elapsedTime - prev1;
  }
}

// Pitch 
void ch2Int()
{
  if (digitalReadFast(ch2)){
	prev2 = elapsedTime;
  }
  else{
	pitch_ratePulse = elapsedTime - prev2;
  }
}

// Throttle 
void ch3Int()
{
   if (digitalReadFast(ch3)){
	prev3 = elapsedTime;
   }
   else{
	throttle_Pulse = elapsedTime - prev3;
   }  
}

// Yaw 
void ch4Int()
{
   if (digitalReadFast(ch4)){
	prev4 = micros();
   }
   else{
	yaw_ratePulse = micros() - prev4;
   }
}

// Switch 
void ch5Int()
{
   if (digitalReadFast(ch5)){
	prev5 = elapsedTime;
   }
   else{
	activateMotor = elapsedTime - prev5;
   }
}
//*************************************************************
// AltHold Add-In

// Altitude Hold 
void ch6Int()
{
   if (digitalReadFast(ch6)){
	prev6 = elapsedTime;
   }
   else{
	activateHold = elapsedTime - prev6;
   }
}

//************************************************************


/////////////////////////////////////////////////////////
// INPUT
/////////////////////////////////////////////////////////
// Calculate the input signal needed to be sent the the PID function

// Variables

// Pitch
int autoPitch; 
int inputPitch;

// Roll 
int autoRoll;
int inputRoll;

// Yaw
int inputYaw;

void getInput()
{
	// Pitch bandwith of 16
	if(pitch_ratePulse > 1508)
	{
		inputPitch = 1508 - pitch_ratePulse;
	}

	else if(pitch_ratePulse < 1492)
	{
		inputPitch = 1492 - pitch_ratePulse;
	} 
	
	else
	{
		inputPitch = 0;
	}

	// Roll bandwith of 16
	if(roll_ratePulse > 1508)
	{
		inputRoll = roll_ratePulse - 1508;
	}

	else if(roll_ratePulse < 1492)
	{
		inputRoll = roll_ratePulse - 1492;
	} 
	
	else
	{
		inputRoll = 0;
	}

	// Yaw bandwidth of 16
	if(yaw_ratePulse > 1508)
	{
		inputYaw = yaw_ratePulse - 1508;
	}

	else if(yaw_ratePulse < 1492)
	{
		inputYaw = yaw_ratePulse - 1492 ;
	} 
	
	else
	{
		inputYaw = 0;
	}

	inputYaw /= 3;

	// AutoLevel
	autoPitch = 15*pitch;
	autoRoll = 15*roll;

	if (autoLevel == false)
	{
		autoPitch = 0;
		autoRoll = 0;
	}

	inputPitch -= autoPitch;
	inputPitch /= 3; 

	inputRoll -= autoRoll;
	inputRoll /= 3;
}


//******************************************************************************************************************************************************
// AltHold Add-In

//////////////////////////////////////////////////////////
// Barometer
//////////////////////////////////////////////////////////

void read_barometer(void) {
  barometer_counter ++;
  
  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);                                   //Open a connection with the MS5611
      Wire.write((u_int8_t)0x00);                                                         //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                   //End the transmission with the MS5611.

      Wire.requestFrom(MS5611_address, 3);                                       //Poll 3 data bytes from the MS5611.
      delayMicroseconds(100);
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      
      raw_temperature_rotating_memory[average_temperature_mem_location] = (Wire.read() << 16 | Wire.read() << 8 | Wire.read());

      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write((u_int8_t)0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
 
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      delayMicroseconds(100);
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.

    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write((u_int8_t)0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write((u_int8_t)0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
    altitude = (44330.0f * (1.0f - pow((double)actual_pressure / (double)seaLevelPressure, 0.1902949f)));
	Serial.println(altitude);
	
	Serial.println(micros()-t);
	t=micros();
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (altitudeHold && (activateHold > 1000)) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0){
		  pid_altitude_setpoint = actual_pressure;                                  //If not yet set, set the PID altitude setpoint.
		  base_throttle = throttle_Pulse;											//When altitude hold is first enabled the current throttle value is stored for calculating throttle -Jaiden
	  }
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.

      if (throttle_Pulse > 1600) {                                                   //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (throttle_Pulse - 1600) / 3;                               //To prevent very fast changes in hight limit the function of the throttle.
	  }

      if (throttle_Pulse < 1400) {                                                   //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = ((int)throttle_Pulse - 1400) / 5;                          //To prevent very fast changes in hight limit the function of the throttle. | Need to cast to actual integer to prevent loss of control -Jaiden
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
	  }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude) pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude) pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;

	  throttle = base_throttle + pid_output_altitude + manual_throttle;					//Caclulate throttle output that is passed to the ESCs -Jaiden
    }
 
    
    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (activateHold<=1000 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }

	else throttle = throttle_Pulse;													//Let throttle function normally when altitude hold is disabled -Jaiden
  }
  
}

//********************************************************************************************************************************************************************

/////////////////////////////////////////////////////////
// PID 
/////////////////////////////////////////////////////////
// Controller corrects for angular rates to converge to desired hand held contoller rates

// Variables

// Pitch 
float errorPitch; 
float pitchPulse;
float last_errorPitch;
float pid_max_pitch = 300.0;
float Ipitch;

float pPitch = 1.8;
float dPitch = 13.5;
float iPitch = 0.01;

// Roll
float errorRoll; 
float rollPulse;
float last_errorRoll;
float pid_max_roll = 300.0;
float Iroll;

float pRoll = pPitch;
float dRoll = dPitch;
float iRoll = iPitch;

// Yaw 
float errorYaw; 
float yawPulse;
float last_errorYaw;
float pid_max_yaw = 300.0;
float Iyaw;

float pYaw = 4.0;
float dYaw = 0.0;
float iYaw = 0.02;

void getPID()
{

	// Pitch
	errorPitch = inputPitch - pitch_rate;
	if (throttle_Pulse > 1100)	Ipitch += iPitch*errorPitch;
	if (activateMotor < 1100) Ipitch=0; 
	pitchPulse = pPitch*errorPitch + dPitch*(errorPitch - last_errorPitch) + Ipitch;
	last_errorPitch = errorPitch; 
	// Bound PID Pitch output
	if( pitchPulse > pid_max_pitch)
	{
		pitchPulse = pid_max_pitch;
	}

	if( pitchPulse < -pid_max_pitch )
	{
		pitchPulse = -pid_max_pitch;
	}


	// Roll 
	errorRoll = inputRoll - roll_rate;
	if (throttle_Pulse > 1100)	Iroll += iRoll*errorRoll; 
	if (activateMotor < 1100) Iroll=0;
	rollPulse = pRoll*errorRoll + dRoll*(errorRoll - last_errorRoll) + Iroll;
	last_errorRoll = errorRoll;

	// Bound PID Roll output 
	if( rollPulse > pid_max_roll)
	{
		rollPulse = pid_max_roll;
	}

	if( rollPulse < -pid_max_roll )
	{
		rollPulse = -pid_max_roll;
	}


	// Yaw
	errorYaw = inputYaw - yaw_rate;
	if (throttle_Pulse > 1100)	Iyaw += iYaw*errorYaw;
	if (activateMotor < 1100) Iyaw=0; 
	yawPulse = pYaw*errorYaw +dYaw*(errorYaw - last_errorYaw) + Iyaw;
	last_errorYaw = errorYaw;

	// Bound PID YAW output  
	if( yawPulse > pid_max_yaw)
	{
		yawPulse = pid_max_yaw;
	}

	if( yawPulse < -pid_max_yaw )
	{
		yawPulse = -pid_max_yaw;
	}


//********************************************************************************************************************************************************************
// AltHold Add-In
// Replace "throttle_Pulse" or direct input from the RC controller in the following calculations with the "throttle" variable that is defined in the barometer code
//********************************************************************************************************************************************************************

	// Calculate pulses to motors
	escPulse1 = throttle - rollPulse + pitchPulse + yawPulse;
	escPulse2 = throttle - rollPulse - pitchPulse - yawPulse;
	escPulse3 = throttle + rollPulse - pitchPulse + yawPulse; 
	escPulse4 = throttle + rollPulse + pitchPulse - yawPulse;



}

/////////////////////////////////////////////////////////
// BOUND PULSE
/////////////////////////////////////////////////////////
// This is a safey feature in the code so the pulses sent to motors won't exced the
// the max or min of the motors. In this case the min of the motors is 1000 while the
// max is 2000. Also will activate the motors.

// Variables 

// idle motors 
int minPulse = 1100;

// max motors
int maxPulse = 2000;

void boundPulse()
{
	// Upper Bound 
	if (escPulse1 > maxPulse)
	{
		escPulse1 = maxPulse;
	}

	if (escPulse2 > maxPulse)
	{
		escPulse2 = maxPulse;
	}

	if (escPulse3 > maxPulse)
	{
		escPulse3 = maxPulse;
	}

	if (escPulse4 > maxPulse)
	{
		escPulse4 = maxPulse;
	}

	// LowerBound 
	if (escPulse1 < minPulse)
	{
		escPulse1 = minPulse;
	}

	if (escPulse2 < minPulse)
	{
		escPulse2 = minPulse;
	}

	if (escPulse3 < minPulse)
	{
		escPulse3 = minPulse;
	}

	if (escPulse4 < minPulse)
	{
		escPulse4 = minPulse;
	}

	// Start/Kill MOTORS
	if(activateMotor < 1100)
	{
		escPulse1 = 1000;
		escPulse2 = 1000;
		escPulse3 = 1000;
		escPulse4 = 1000;
	}
}


/////////////////////////////////////////////////////////
// CONVERT PULSE TO PWM 
/////////////////////////////////////////////////////////
// This function converts the calculate pulse to the PWM signal.
// Then the PWM signal is written to the motors

void pulsetoPWM()
{
	// Convert Mircosecond time to PWM pulse for motors(CREATE NEW FUNCTION WILL NEXT LINES)
	escPulse1PWM = escPulse1*pwmMax/escPulseTime;
	escPulse2PWM = escPulse2*pwmMax/escPulseTime;
	escPulse3PWM = escPulse3*pwmMax/escPulseTime;
	escPulse4PWM = escPulse4*pwmMax/escPulseTime;

	// Send PWM pulse to motors
	analogWrite(escOut1, escPulse1PWM);
  	analogWrite(escOut2, escPulse2PWM);
  	analogWrite(escOut3, escPulse3PWM);
  	analogWrite(escOut4, escPulse4PWM);
}

/////////////////////////////////////////////////////////
// BINK
/////////////////////////////////////////////////////////
// Blinks the built in led

void blinkLed()
{
	digitalWrite(led,HIGH);
	delay(250);
	digitalWrite(led,LOW);	
	delay(250);
}

/////////////////////////////////////////////////////////
// CONTROLLER CHECK 
/////////////////////////////////////////////////////////
// Make sure controller is in the right position
void controllerCheck()
{
	while(activateMotor > 1100)
	{
		Serial.println("Turn left controller nobe to 1000");
		blinkLed();
	}

	while(throttle_Pulse > 1100)
	{
		Serial.println("Lower throttle pulse to 1000");
		blinkLed();
	}
}

/////////////////////////////////////////////////////////
// IMU INTILIZATION
/////////////////////////////////////////////////////////
// Check to see if IMU is active and calibrate 

void imuIntilization()
{

	// Gyro
	while(!gyro.begin(GYRO_RANGE_500DPS))
	{
		Serial.println("Ooops, no gyro detected ... Check your wiring!");
		blinkLed();
	}

	// Accellerometer
	while(!accelmag.begin(ACCEL_RANGE_4G))
	{
		Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
		blinkLed();
	}

	// Start IMU filter for desired frequency
	filter.begin(updateFreq);

	// Calibrate IMU to find angular rate offsets
	blinkLed();
	calIMU();
	blinkLed();
}


/////////////////////////////////////////////////////////
// ESC INTILIZATION
/////////////////////////////////////////////////////////
// Set motors so they are ready to run

void escInitialize()
{
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
	Serial.println("ESC Initialization Completed");	
}

//************************************************************************************************************************************
// AltHold Add-In

/////////////////////////////////////////////////////////
//Barometer Initialization
/////////////////////////////////////////////////////////

void barometerInitialize(){
  //Check if the MS5611 barometer is responding.                        
  Wire.begin();
  Wire.beginTransmission(MS5611_address);                      //Start communication with the MS5611.
  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MS5611 did not responde.
    Wire.beginTransmission(MS5611_address);
    error = Wire.endTransmission();
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0) {                                 //Every 125 loops (500ms).
      //digitalWrite(PB4, !digitalRead(PB4));                     //Change the led status.
    }
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.

  
  //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
    C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
    read_barometer();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }

  actual_pressure = 0;                                          //Reset the pressure calculations.
}

//********************************************************************************************************************************************


/////////////////////////////////////////////////////////
// SETUP
/////////////////////////////////////////////////////////
// Get ready to fly

void setup() 
{
	// Do you want to debug?
	if(debug == true)  
	{
		Serial.begin(115200);
		while(!Serial);
		Serial.println("DEBUGING");
	}

	// Set pin direction(INPUT/OUTPUT)

	// Reciever 
	pinMode(ch1,INPUT);
	pinMode(ch2,INPUT);
	pinMode(ch3,INPUT);
	pinMode(ch4,INPUT);

	// Escs
	pinMode(escOut1,OUTPUT);
	pinMode(escOut2,OUTPUT);
	pinMode(escOut3,OUTPUT);
	pinMode(escOut4,OUTPUT);

	//Led 
	pinMode(led,OUTPUT);

	// Setup rx pin interrupts
  	attachInterrupt(ch1,ch1Int,CHANGE);
  	attachInterrupt(ch2,ch2Int,CHANGE);
  	attachInterrupt(ch3,ch3Int,CHANGE);
  	attachInterrupt(ch4,ch4Int,CHANGE);
	  attachInterrupt(ch5,ch5Int,CHANGE);
	  attachInterrupt(ch6,ch6Int,CHANGE);

	// Make sure controller is in the right starting position
	controllerCheck();

	// Intilizate the IMU 
	imuIntilization();

	// Initialize esc
	escInitialize();


//****************************************
// AltHold Add-In

	//Initialize barometer
	barometerInitialize();

//****************************************

	// Setup completed 
	digitalWrite(led,HIGH);
	Serial.println("Finished Setup");
	delay(1000);
}

/////////////////////////////////////////////////////////
// MAIN CODE LOOP 
/////////////////////////////////////////////////////////
void loop() 
{
	// Update pulse to motors every 250hz
	if ((elapsedTime - lastUpdate) > updateTime)
	{

		lastUpdate = elapsedTime;

		// Get the rates and angles
		getIMU();

//*****************************************************
// AltHold Add-In

		//Call Barometer Function
		read_barometer();

//*****************************************************

		// Scale the pulse from 1000-2000 (CURRENTLY DOES NOT WORK)
		//scalePulse(); 

		// Get the input pulse for the PID 
		getInput();

		// Get the PID corrections and calculate the motor pulses 
		getPID();

		// Make sure pulse is in correct range and kill or activate motors  
		boundPulse();

		// Convert the Pulse to PWM in order to spin the motors 
		pulsetoPWM();

	}	

	// Check output variables
	if (debug == true)
	{
		if ((elapsedTime - lastPrint) >= printTimer)
		{
			lastPrint = elapsedTime;
			//Serial.println(yaw_ratePulse);
		}
	}
}