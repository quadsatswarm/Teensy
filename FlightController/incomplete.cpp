#include <Wire.h>							// Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>							// Include the EEPROM.h library so we can use information on the EEPROM


/***************************************/
/***** PID Gain and Limit Settings *****/
/***************************************/

float pid_p_gain_roll = 1.3;				// Gain for the roll P-control
float pid_i_gain_roll = 0.04;				// Gain for the roll I-control
float pid_d_gain_roll = 18.0;				// Gain for the roll D-control
int pid_max_roll = 400;						// Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;	// Gain for the pitch P-control
float pid_i_gain_pitch = pid_i_gain_roll;	// Gain for the pitch I-control
float pid_d_gain_pitch = pid_d_gain_roll;	// Gain for the pitch D-control
int pid_max_pitch = pid_max_roll;			// Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;					// Gain for the pitch P-control
float pid_i_gain_yaw = 0.02;				// Gain for the pitch I-control
float pid_d_gain_yaw = 0.0;					// Gain for the pitch D-control
int pid_max_yaw = 400;						// Maximum output of the PID-controller (+/-)

boolean auto_level = true;					// Auto level on (true) or off (false)



/************************************/
/***** Declare Global Variables *****/
/************************************/

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

byte LED = 12;



/*************************/
/***** Setup Routine *****/
/*************************/

void setup()
{
	// Copy the EEPROM data for fast access
	for( start = 0; start <= 35; start++ ) 
	{
		eeprom_data[ start ] = EEPROM.read( start );
	}

	start = 0;													// Set start back to zero
	gyro_address = eeprom_data[ 32 ];							// Retrieve the Gyro address from the EEPROM

	Wire.begin();												// Start the I2C as master

	TWBR = 12;													// Set the I2C clock speed to 400kHz

	// Configure digital ports 4, 5, 6, 7, 12, and 13 as outputs
	pinMode( 4, OUTPUT )
	pinMode( 5, OUTPUT )
	pinMode( 6, OUTPUT )
	pinMode( 7, OUTPUT )
	pinMode( 12, OUTPUT )
	pinMode( 13, OUTPUT )

	// Use the LED on the Arduino for startup indication; turn it on
	digitalWrite( LED, HIGH );

	// Check the EEPROM signature to make sure that the setup program has executed
	while( eeprom_data[ 33 ] != 'J' || eeprom_data[ 34 ] != 'M' || eeprom_data[ 35 ] != 'B') 
	{
		delay( 10 );
	}

	// The flight controller needs the MPU-6050 with gyro and accelerometer
	// If setup is completed without MPU-6050 stop the flight controller program  
	if( eeprom_data[ 31 ] == 2 || eeprom_data[ 31 ] == 3 )
	{
		delay( 10 );
	}

	// Set the specific gyro registers
	set_gyro_registers();


	// Wait 5 seconds before continuing 
	for( cal_int = 0; cal_int < 1250; cal_int++ )
	{                           
		// Send a pulse to the ESCs to keep 'em happy
		// Set digital ports 4, 5, 6, and 7 to high
		digitalWrite( 4, HIGH )
		digitalWrite( 5, HIGH )
		digitalWrite( 6, HIGH )
		digitalWrite( 7, HIGH )

		// Wait for 1000 microseconds
		delayMicroseconds( 1000 );                                                

		// Set digital ports 4, 5, 6, and 7 to low
		digitalWrite( 4, LOW )
		digitalWrite( 5, LOW )
		digitalWrite( 6, LOW )
		digitalWrite( 7, LOW )

		// Wait for 3000 microseconds
		delayMicroseconds( 3000 );
	}

	// Take 2000 gyro data samples to determine the average gyro offset (calibration)
	for( cal_int = 0; cal_int < 2000; cal_int++ )
	{                           
		if( cal_int % 15 == 0 )
		{
			digitalWrite( LED, !digitalRead( LED ) );				// Change the LED status to indicate calibration
		}

		gyro_signalen();											// Read the gyro output
		gyro_axis_cal[ 1 ] += gyro_axis[ 1 ];						// Add roll value to gyro_roll_cal
		gyro_axis_cal[ 2 ] += gyro_axis[ 2 ];						// Add pitch value to gyro_pitch_cal
		gyro_axis_cal[ 3 ] += gyro_axis[ 3 ];						// Add yaw value to gyro_yaw_cal

		// Send a pulse to the ESCs to keep 'em happy
		// Set digital ports 4, 5, 6, and 7 to high
		digitalWrite( 4, HIGH )
		digitalWrite( 5, HIGH )
		digitalWrite( 6, HIGH )
		digitalWrite( 7, HIGH )

		// Wait for 1000 microseconds
		delayMicroseconds( 1000 );                                                

		// Set digital ports 4, 5, 6, and 7 to low
		digitalWrite( 4, LOW )
		digitalWrite( 5, LOW )
		digitalWrite( 6, LOW )
		digitalWrite( 7, LOW )

		// Wait for 3000 microseconds
		delayMicroseconds( 3000 );
	}

	// Divide each total by 2000 to get the average gyro offset.
	gyro_axis_cal[ 1 ] /= 2000;										// Divide the roll total by 2000.
	gyro_axis_cal[ 2 ] /= 2000;										// Divide the pitch total by 2000.
	gyro_axis_cal[ 3 ] /= 2000;										// Divide the yaw total by 2000.

	// Set digital inputs 8, 9, 10, and 11 to trigger an interrupt on state changes
	attachInterrupt( digitalPinToInterrupt( 8 ), receiver_isr, CHANGE );		
	attachInterrupt( digitalPinToInterrupt( 9 ), receiver_isr, CHANGE );
	attachInterrupt( digitalPinToInterrupt( 10 ), receiver_isr, CHANGE );
	attachInterrupt( digitalPinToInterrupt( 11 ), receiver_isr, CHANGE );

	// Wait until the receiver is active and the throttle is set to the lower position.
	while( receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400 )
	{
		receiver_input_channel_3 = convert_receiver_channel( 3 );	// Convert the actual receiver signals for throttle to the standard 1000 - 2000us
		receiver_input_channel_4 = convert_receiver_channel( 4 );	// Convert the actual receiver signals for yaw to the standard 1000 - 2000us
		start++;													// Increment start with every loop

		// Send a pulse to the ESCs to keep 'em happy
		// Set digital ports 4, 5, 6, and 7 to high
		digitalWrite( 4, HIGH )
		digitalWrite( 5, HIGH )
		digitalWrite( 6, HIGH )
		digitalWrite( 7, HIGH )

		// Wait for 1000 microseconds
		delayMicroseconds( 1000 );                                                

		// Set digital ports 4, 5, 6, and 7 to low
		digitalWrite( 4, LOW )
		digitalWrite( 5, LOW )
		digitalWrite( 6, LOW )
		digitalWrite( 7, LOW )

		// Wait for 3000 microseconds
		delayMicroseconds( 3000 );


		// Every 125 loops (500ms)
		if( start == 125 )	
		{                                                       
			// Change the LED status to indicate calibration
			digitalWrite( LED, !digitalRead( LED ) );				

			// Set start back to 0
			start = 0;                                                            
		}
	}

	// Set start back to 0
	start = 0;

	// Set the timer for the first loop
	loop_timer = micros();

	// When everything is done, turn off the LED
	digitalWrite( LED, LOW );
}




/*****************************/
/***** Main Program Loop *****/
/*****************************/

void loop()
{
	// 65.5 = 1 deg/sec 
	gyro_roll_input = ( gyro_roll_input * 0.7 ) + ( ( gyro_roll / 65.5 ) * 0.3 );		// Gyro PID input is deg/sec
	gyro_pitch_input = ( gyro_pitch_input * 0.7 ) + ( ( gyro_pitch / 65.5 ) * 0.3 );	// Gyro PID input is deg/sec
	gyro_yaw_input = ( gyro_yaw_input * 0.7 ) + ( ( gyro_yaw / 65.5 ) * 0.3 );			// Gyro PID input is deg/sec


	// Gyro angle calculations
	// 0.0000611 = 1 / ( 250Hz / 65.5 )
	angle_pitch += gyro_pitch * 0.0000611;												// Calculate the traveled pitch angle and add this to the angle_pitch variable
	angle_roll += gyro_roll * 0.0000611;												// Calculate the traveled roll angle and add this to the angle_roll variable

	// 0.000001066 = 0.0000611 * ( PI / 180deg ) The Arduino sin function is in radians
	angle_pitch -= angle_roll * sin( gyro_yaw * 0.000001066 );							// If the IMU has yawed transfer the roll angle to the pitch angle
	angle_roll += angle_pitch * sin( gyro_yaw * 0.000001066 );							// If the IMU has yawed transfer the pitch angle to the roll angle


	// Accelerometer calculations
	// Calculate the total accelerometer vector
	acc_total_vector = sqrt( ( acc_x * acc_x ) + ( acc_y * acc_y ) + ( acc_z * acc_z ) );	
	
	// Prevent the asin function from producing a NaN
	if( abs( acc_y ) < acc_total_vector )
	{
		// Calculate the pitch angle
		angle_pitch_acc = asin( (float)acc_y / acc_total_vector ) * 57.296;          		
	}

	// Prevent the asin function from producing a NaN
	if( abs( acc_x ) < acc_total_vector )
	{                                        
		// Calculate the roll angle
		angle_roll_acc = asin( (float)acc_x / acc_total_vector ) * ( -57.296 );
	}

	angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;						// Correct the drift of the gyro pitch angle with the accelerometer pitch angle
	angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;							// Correct the drift of the gyro roll angle with the accelerometer roll angle

	pitch_level_adjust = angle_pitch * 15;												// Calculate the pitch angle correction
	roll_level_adjust = angle_roll * 15;												// Calculate the roll angle correction
	
	// If the quadcopter is not in auto-level mode
	if( !auto_level )
	{                                                          
		pitch_level_adjust = 0;															// Set the pitch angle correction to zero
		roll_level_adjust = 0;															// Set the roll angle correcion to zero
	}


	// For starting the motors: throttle low and yaw left (Step 1)
	if( receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050 )
	{
		start = 1;
	}

	// When yaw stick is back in the center position, start the motors (Step 2)
	if( start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450 )
	{
		start = 2;

		angle_pitch = angle_pitch_acc; 	// Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started
		angle_roll = angle_roll_acc;  	// Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started
		gyro_angles_set = true;			// Set the IMU started flag

		// Reset the PID controllers for a bumpless start
		pid_i_mem_roll = 0;
		pid_last_roll_d_error = 0;
		pid_i_mem_pitch = 0;
		pid_last_pitch_d_error = 0;
		pid_i_mem_yaw = 0;
		pid_last_yaw_d_error = 0;
	}

	// Stopping the motors: throttle low and yaw right
	if( start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950 )
	{
		start = 0;
	}

	// The PID set point in degrees per second is determined by the roll receiver input
	// In the case of dividing by 3 the max roll rate is approx. 164 deg/sec ( ( 500 - 8 ) / 3 = 164 deg/sec )
	pid_roll_setpoint = 0;

	// Implement dead band of 16us for better results
	if( receiver_input_channel_1 > 1508 )
	{
		pid_roll_setpoint = receiver_input_channel_1 - 1508;
	}

	else if( receiver_input_channel_1 < 1492 )
	{
		pid_roll_setpoint = receiver_input_channel_1 - 1492;
	}

	pid_roll_setpoint -= roll_level_adjust;		// Subtract the angle correction from the standardized receiver roll input value
	pid_roll_setpoint /= 3.0;					// Divide the setpoint for the PID roll controller by 3 to get angles in degrees


	// The PID set point in degrees per second is determined by the pitch receiver input
	// In the case of dividing by 3 the max pitch rate is approx. 164 deg/sec ( ( 500 - 8 ) / 3 = 164 deg/sec )
	pid_pitch_setpoint = 0;

	// Implement dead band of 16us for better results
	if( receiver_input_channel_2 > 1508 )
	{
		pid_pitch_setpoint = receiver_input_channel_2 - 1508;
	}

	else if( receiver_input_channel_2 < 1492 )
	{
		pid_pitch_setpoint = receiver_input_channel_2 - 1492;
	}

	pid_pitch_setpoint -= pitch_level_adjust;	// Subtract the angle correction from the standardized receiver pitch input value
	pid_pitch_setpoint /= 3.0;					// Divide the setpoint for the PID pitch controller by 3 to get angles in degrees

	// The PID set point in deg/sec is determined by the yaw receiver input
	// In the case of dividing by 3 the max yaw rate is approx. 164 deg/sec ( ( 500 - 8 ) / 3 = 164 deg/sec )
	pid_yaw_setpoint = 0;

	// Do not yaw when turning off the motors
	if( receiver_input_channel_3 > 1050 )
	{ 
		// Implement dead band of 16us for better results
		if( receiver_input_channel_4 > 1508 )
		{
			pid_yaw_setpoint = ( receiver_input_channel_4 - 1508 ) / 3.0;
		}

		else if( receiver_input_channel_4 < 1492 )
		{
			pid_yaw_setpoint = ( receiver_input_channel_4 - 1492 ) / 3.0;
		}
	}

	// PID inputs are known, calculate the PID output
	calculate_pid();

	// Get the throttle signal as a base signal
	throttle = receiver_input_channel_3;

	// If the motors have started
	if( start == 2 )
	{
		if( throttle > 1800 ) 
		{
			// Cap the value for full throttle
			throttle = 1800;
		}

		esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;		// Calculate the pulse for esc 1 (front-right - CCW)
		esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;		// Calculate the pulse for esc 2 (rear-right - CW)
		esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;		// Calculate the pulse for esc 3 (rear-left - CCW)
		esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;		// Calculate the pulse for esc 4 (front-left - CW)


		// Keep the motors running
		if( esc_1 < 1100 )
		{
			esc_1 = 1100;
		}

		if( esc_2 < 1100 ) 
		{
			esc_2 = 1100;
		}

		if( esc_3 < 1100 ) 
		{
			esc_3 = 1100;
		}

		if( esc_4 < 1100 ) 
		{
			esc_4 = 1100;
		}


		// Cap the pulse widths at 2000 microseconds
		if( esc_1 > 2000 ) 
		{
			esc_1 = 2000;
		}

		if( esc_2 > 2000 )
		{
			esc_2 = 2000;
		}

		if( esc_3 > 2000 )
		{
			esc_3 = 2000;
		}

		if( esc_4 > 2000 ) 
		{
			esc_4 = 2000;
		}
	}

	// If the quadcopter is not started, then set all the ESC pulses to a width of 1000 microseconds
	else
	{
		esc_1 = 1000;
		esc_2 = 1000;
		esc_3 = 1000;
		esc_4 = 1000;
	}

	// Turn on the LED if the loop time exceeds 4050us.
	if( micros() - loop_timer > 4050 )
	{
		digitalWrite( LED, HIGH );
	}

	// Now all the information for controlling the motors is available
	// The refresh rate is 250Hz. That means the ESCs need their pulse every 4ms.
	while( micros() - loop_timer < 4000 );	// Wait until 4000us have passed
	loop_timer = micros();					// Set the timer for the next loop

	// Set digital ports 4, 5, 6, and 7 to high
	digitalWrite( 4, HIGH )
	digitalWrite( 5, HIGH )
	digitalWrite( 6, HIGH )
	digitalWrite( 7, HIGH )

	// Calculate the time of the falling edge for each ESC pulse
	timer_channel_1 = esc_1 + loop_timer;
	timer_channel_2 = esc_2 + loop_timer;
	timer_channel_3 = esc_3 + loop_timer;
	timer_channel_4 = esc_4 + loop_timer;

	// There is always at least 1000 microseconds (25%) of spare time, so do something useful that is time consuming
	// Get the current gyro and receiver data and scale it to deg/sec for the PID calculations
	gyro_signalen();

	// Stay in this loop until digital ports 4, 5, 6, and 7 are low
	while( digitalRead( 4 ) == HIGH || digitalRead( 5 ) == HIGH || digitalRead( 6 ) == HIGH || digitalRead( 7 ) == HIGH )
	{
		// Read the current time
		esc_loop_timer = micros();

		if( timer_channel_1 <= esc_loop_timer )
		{
			// Set digital output 4 to low if the time is expired
			digitalWrite( 4, LOW )
		}

		if( timer_channel_2 <= esc_loop_timer )
		{
			// Set digital output 5 to low if the time is expired
			digitalWrite( 5, LOW )
		}

		if( timer_channel_3 <= esc_loop_timer )
		{
			// Set digital output 6 to low if the time is expired
			digitalWrite( 6, LOW )
		}

		if( timer_channel_4 <= esc_loop_timer )
		{
			// Set digital output 7 to low if the time is expired
			digitalWrite( 7, LOW )
		}
	}
}



/**********************************************/
/***** Receiver Interrupt Service Routine *****/
/**********************************************/

void receiver_isr()
{
	current_time = micros();


	// Channel 1
	// Is input 8 high?
	if( digitalRead( 8 ) == HIGH )
	{
		// Input 8 changed from 0 to 1
		if( last_channel_1 == 0 )
		{
			// Remember current input state
			last_channel_1 = 1;

			timer_1 = current_time;
		}
	}

	// Input 8 is not high and changed from 1 to 0
	else if( last_channel_1 == 1 )
	{
		// Remember current input state
		last_channel_1 = 0;

		receiver_input[ 1 ] = current_time - timer_1;
	}


	// Channel 2
	// Is input 9 high?
	if( digitalRead( 9 ) == HIGH )
	{
		// Input 9 changed from 0 to 1
		if( last_channel_2 == 0 )
		{
			// Remember current input state
			last_channel_2 = 1;

			timer_2 = current_time;
		}
	}

	// Input 9 is not high and changed from 1 to 0
	else if( last_channel_2 == 1 )
	{
		// Remember current input state
		last_channel_2 = 0;

		receiver_input[ 2 ] = current_time - timer_2;
	}

	
	// Channel 3
	// Is input 10 high?
	if( digitalRead( 10 ) == HIGH )
	{
		// Input 10 changed from 0 to 1
		if( last_channel_3 == 0 )
		{
			// Remember current input state
			last_channel_3 = 1;

			timer_3 = current_time;
		}
	}

	// Input 10 is not high and changed from 1 to 0
	else if( last_channel_3 == 1 )
	{
		// Remember current input state
		last_channel_3 = 0;

		receiver_input[ 3 ] = current_time - timer_3;
	}


	// Channel 4
	// Is input 11 high?
	if( digitalRead( 11 ) == HIGH )
	{
		// Input 11 changed from 0 to 1
		if( last_channel_4 == 0 )
		{
			// Remember current input state
			last_channel_4 = 1;

			timer_4 = current_time;
		}
	}

	// Input 11 is not high and changed from 1 to 0
	else if( last_channel_4 == 1 )
	{
		// Remember current input state
		last_channel_4 = 0;

		receiver_input[ 4 ] = current_time - timer_4;
	}
}



/*******************************************/
/***** Subroutine for Reading the Gyro *****/
/*******************************************/

void gyro_signalen()
{
	Wire.beginTransmission( gyro_address );							// Start communication with the gyro
	Wire.write( 0x3B );												// Start reading @ register 43h and auto increment with every read
	Wire.endTransmission();											// End the transmission
	Wire.requestFrom( gyro_address, 14 );							// Request 14 bytes from the gyro.

	receiver_input_channel_1 = convert_receiver_channel( 1 );		// Convert the actual receiver signals for pitch to the standard 1000 - 2000us
	receiver_input_channel_2 = convert_receiver_channel( 2 );		// Convert the actual receiver signals for roll to the standard 1000 - 2000us
	receiver_input_channel_3 = convert_receiver_channel( 3 );		// Convert the actual receiver signals for throttle to the standard 1000 - 2000us
	receiver_input_channel_4 = convert_receiver_channel( 4 );		// Convert the actual receiver signals for yaw to the standard 1000 - 2000us

	// Wait until the 14 bytes are received
	while( Wire.available() < 14 );

	// Add the low and high byte to each acc variable (x, y, z)
	acc_axis[ 1 ] = Wire.read() << 8 | Wire.read();
	acc_axis[ 2 ] = Wire.read() << 8 | Wire.read();
	acc_axis[ 3 ] = Wire.read() << 8 | Wire.read();

	// Add the low and high byte to the temperature variable
	temperature = Wire.read() << 8 | Wire.read();

	// Read high and low part of the angular data for each axis
	gyro_axis[ 1 ] = Wire.read() << 8 | Wire.read();
	gyro_axis[ 2 ] = Wire.read() << 8 | Wire.read();
	gyro_axis[ 3 ] = Wire.read() << 8 | Wire.read();


	// Only compensate after the calibration
	if( cal_int == 2000 )
	{
		gyro_axis[ 1 ] -= gyro_axis_cal[ 1 ];
		gyro_axis[ 2 ] -= gyro_axis_cal[ 2 ];
		gyro_axis[ 3 ] -= gyro_axis_cal[ 3 ];
	}


	// Set gyro_roll to the correct axis that was stored in the EEPROM.
	gyro_roll = gyro_axis[ eeprom_data[ 28 ] & 0b00000011 ];

	// Invert gyro_roll if the MSB of EEPROM bit 28 is set
	if( eeprom_data[ 28 ] & 0b10000000 )
	{
		gyro_roll *= -1;
	}

	// Set gyro_pitch to the correct axis that was stored in the EEPROM
	gyro_pitch = gyro_axis[ eeprom_data[ 29 ] & 0b00000011 ];

	// Invert gyro_pitch if the MSB of EEPROM bit 29 is set
	if( eeprom_data[ 29 ] & 0b10000000 )
	{
		gyro_pitch *= -1;
	}

	// Set gyro_yaw to the correct axis that was stored in the EEPROM
	gyro_yaw = gyro_axis[ eeprom_data[ 30 ] & 0b00000011 ];

	// Invert gyro_yaw if the MSB of EEPROM bit 30 is set
	if( eeprom_data[ 30 ] & 0b10000000 )
	{
		gyro_yaw *= -1;
	}


	// Set acc_x to the correct axis that was stored in the EEPROM
	acc_x = gyro_axis[ eeprom_data[ 29 ] & 0b00000011 ];

	// Invert acc_x if the MSB of EEPROM bit 29 is set
	if( eeprom_data[ 29 ] & 0b10000000 )
	{
		acc_x *= -1;
	}
	
	// Set acc_y to the correct axis that was stored in the EEPROM.
	acc_y = gyro_axis[ eeprom_data[ 28 ] & 0b00000011 ];

	// Invert acc_y if the MSB of EEPROM bit 28 is set
	if( eeprom_data[ 28 ] & 0b10000000 )
	{
		acc_y *= -1;
	}

	// Set acc_z to the correct axis that was stored in the EEPROM
	acc_z = gyro_axis[ eeprom_data[ 30 ] & 0b00000011 ];

	// Invert acc_z if the MSB of EEPROM bit 30 is set
	if( eeprom_data[ 30 ] & 0b10000000 )
	{
		acc_z *= -1;
	}
}



/**************************************************/
/***** Subroutine for Calculating PID Outputs *****/
/**************************************************/

void calculate_pid()
{
	// Roll Calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

	// Cap the Intergral part, cuz that can tend to accumulate
	if( pid_i_mem_roll > pid_max_roll )
	{
		pid_i_mem_roll = pid_max_roll;
	}

	else if( pid_i_mem_roll < pid_max_roll * -1 )
	{
		pid_i_mem_roll = pid_max_roll * -1;
	}

	pid_output_roll = ( pid_p_gain_roll * pid_error_temp ) + pid_i_mem_roll + ( pid_d_gain_roll * ( pid_error_temp - pid_last_roll_d_error ) );
	
	// Cap the total PID output at the max allowable output
	if( pid_output_roll > pid_max_roll )
	{
		pid_output_roll = pid_max_roll;
	}

	else if( pid_output_roll < pid_max_roll * -1 )
	{
		pid_output_roll = pid_max_roll * -1;
	}

	pid_last_roll_d_error = pid_error_temp;


	// Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	
	if( pid_i_mem_pitch > pid_max_pitch )
	{
		pid_i_mem_pitch = pid_max_pitch;
	}

	else if( pid_i_mem_pitch < pid_max_pitch * -1 )
	{
		pid_i_mem_pitch = pid_max_pitch * -1;
	}

	pid_output_pitch = ( pid_p_gain_pitch * pid_error_temp ) + pid_i_mem_pitch + ( pid_d_gain_pitch * ( pid_error_temp - pid_last_pitch_d_error ) );

	if( pid_output_pitch > pid_max_pitch )
	{
		pid_output_pitch = pid_max_pitch;
	}

	else if( pid_output_pitch < pid_max_pitch * -1 )
	{
		pid_output_pitch = pid_max_pitch * -1;
	}

	pid_last_pitch_d_error = pid_error_temp;


	// Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;

	if( pid_i_mem_yaw > pid_max_yaw )
	{
		pid_i_mem_yaw = pid_max_yaw;
	}

	else if( pid_i_mem_yaw < pid_max_yaw * -1 )
	{
		pid_i_mem_yaw = pid_max_yaw * -1;
	}

	pid_output_yaw = ( pid_p_gain_yaw * pid_error_temp ) + pid_i_mem_yaw + ( pid_d_gain_yaw * ( pid_error_temp - pid_last_yaw_d_error ) );

	if( pid_output_yaw > pid_max_yaw )
	{
		pid_output_yaw = pid_max_yaw;
	}

	else if( pid_output_yaw < pid_max_yaw * -1 )
	{
		pid_output_yaw = pid_max_yaw * -1;
	}

	pid_last_yaw_d_error = pid_error_temp;
}
