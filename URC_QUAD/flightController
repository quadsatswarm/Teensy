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
*/

// TODO'S 
/* 4-12-19
 * 1. CHECK TO MAKE SURE SETUP WORKS WITH NEW FUNCTIONS
*/ 

/////////////////////////////////////////////////////////
//LIBARIES 
/////////////////////////////////////////////////////////
#include <Arduino.h> 							//Used for Visual Studio's code 
#include <Wire.h>    							//I2C communication 
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
bool autoLevel = true;

// Variables for debugging
int printTimer = 5000;
int lastPrint = 0;

////////////////////////////////////////////////////////
//PIN DEFINITIONS
////////////////////////////////////////////////////////

// Reciever  
 int ch1 = 24;
 int ch2 = 25;
 int ch3 = 26;
 int ch4 = 27;
 int ch5 = 28;
 int ch6 = 29;

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
 int escInit = pwmMax/4;

 int pwmFreq = 250;
 int pwmRes = 8;
 int escPulseTime = 4000;

 // Pulse length 
 int escPulse1;
 int escPulse2;
 int escPulse3;
 int escPulse4;

 // Value for analog write function 
 int escPulse1PWM;
 int escPulse2PWM;
 int escPulse3PWM;
 int escPulse4PWM;

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
 unsigned long prev4 = 0;
 volatile unsigned int yaw_ratePulse = 1500;
 unsigned long prev5 = 0;
 volatile unsigned int activateMotor = 1500;

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

/////////////////////////////////////////////////////////
// PID 
/////////////////////////////////////////////////////////
// Controller corrects for angular rates to converge to desired hand held contoller rates

// Variables

// Pitch 
int errorPitch; 
int pitchPulse;
int last_errorPitch;
int pid_max_pitch = 300;
float Ipitch;

int pPitch = 2;
int dPitch = 18;
int iPitch = .02;

// Roll
int errorRoll; 
int rollPulse;
int last_errorRoll;
int pid_max_roll = 300;
int Iroll;

int pRoll = pPitch;
int dRoll = dPitch;
int iRoll = iPitch;

// Yaw 
int errorYaw; 
int yawPulse;
int last_errorYaw;
int pid_max_yaw = 300;
int Iyaw;

int pYaw = 4;
int dYaw = 0;
int iYaw = 0.02;

void getPID()
{

	// Pitch
	errorPitch = inputPitch - pitch_rate;
	Ipitch += iPitch*errorPitch; 
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
	Iroll += iRoll*errorRoll;
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
	Iyaw += iYaw*errorYaw;
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

	// Calculate pulses to motors
	escPulse1 = throttle_Pulse - rollPulse + pitchPulse + yawPulse;
	escPulse2 = throttle_Pulse - rollPulse - pitchPulse - yawPulse;
	escPulse3 = throttle_Pulse + rollPulse - pitchPulse + yawPulse; 
	escPulse4 = throttle_Pulse + rollPulse + pitchPulse - yawPulse;

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

	// Make sure controller is in the right starting position
	controllerCheck();

	// Intilizate the IMU 
	imuIntilization();

	// Initialize esc
	escInitialize();

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
			Serial.println(yaw_ratePulse);
		}
	}
}
