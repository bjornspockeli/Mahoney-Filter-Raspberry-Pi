#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
//#include <gsl/gsl_math.h>
#include <math.h>


//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	50.f 		// Sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

// Raspberry Pi Pins
#define INT_PIN		7


// MPU-6050 Register Addresses
#define MPU_ADDR	0x68
#define PWR_MGMT_1	0x6B
#define ACCEL_XOUT_H	0x3B
#define GYRO_XOUT_H	0x43
#define CONFIG		0x1A
#define SMPRT_DIV 	0x19
#define INT_ENABLE	0x38

// Software defines
#define ARRAY_SIZE	1000

// Senosr Offsets
#define X_ACC_OFFSET	238
#define Y_ACC_OFFSET	168
#define Z_ACC_OFFSET	309

#define X_GYRO_OFFSET	-75
#define Y_GYRO_OFFSET	215
#define Z_GYRO_OFFSET	79


//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;						// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;						// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;			// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

static int i2c_handle;

typedef struct {
	int16_t x_acc;
	int16_t y_acc;
	int16_t z_acc;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
} MPU6050_Data_t;

int array_counter = 0;

volatile int measurements_done = 0;

// New measurements flag
volatile int new_measurements = 0;

volatile MPU6050_Data_t mpu6050_data;

MPU6050_Data_t mpu6050_data_array[ARRAY_SIZE];

const MPU6050_Data_t * p_mpu6050_data = mpu6050_data_array;


// ##################################################################################################
// 					Functions
// ##################################################################################################

//---------------------------------------------------------------------------------------------------
// Function to initializing the I2C Bus

void i2c_init( int * i2c_handle)
{

	char *filename =(char*)"/dev/i2c-1";
	// Create file descriptor for the I2C Bus
	if((*i2c_handle = open(filename, O_RDWR)) == -1)
	{
		printf("Could not open I2C BUS \n");
	}
	// Tell the I2C peripheral that a 7-Bit address is used
	if(ioctl(*i2c_handle, I2C_TENBIT, 0) ==-1)
	{
		printf("Failed to set address type \n");
	}


	// Set the I2C address to that of the MPU-6050
	int mpu6050_addr = 0x68;
	if(ioctl(*i2c_handle, I2C_SLAVE, mpu6050_addr) == -1)
	{
		printf("Could not access I2C bus \n");
	}

}

//---------------------------------------------------------------------------------------------------
// Function for writting to data registers of the MPU-6050 over the I2C bus

void read_register(int i2c_filehandle, uint8_t * reg_addr, uint8_t * rx_buffer, int bytes)
{
	write(i2c_filehandle, reg_addr, 1);
	read(i2c_filehandle, rx_buffer, bytes);
}

//---------------------------------------------------------------------------------------------------
// Function for writting to data registers of the MPU-6050 over the I2C bus

void write_register( int i2c_filehandle, uint8_t * tx_buffer, int bytes)
{
	write(i2c_filehandle, tx_buffer, bytes);
}

//---------------------------------------------------------------------------------------------------
// Function for initializing the the MPU-6050

void mpu_6050_init()
{
	int result;
	uint8_t tx_buffer[2] = {0};

	// Wake up the MPU-6050 by writting to Power Management register 1
	tx_buffer[0] = PWR_MGMT_1;
	tx_buffer[1] = 0x00;
	result = write(i2c_handle, tx_buffer,2);
	printf("Result: %d \n", result);
	if(result < 0)
	{
		printf("Could not write to PWM_MGMT_1 \n");
	}

}

//---------------------------------------------------------------------------------------------------
// Function for reading the accelerometer data registers of the MPU-6050

void mpu_6050_acc_read(volatile int16_t * p_x_acc, volatile int16_t * p_y_acc, volatile int16_t * p_z_acc)
{

	uint8_t reg_addr = ACCEL_XOUT_H;
	uint8_t rx_buffer[6] = {0};

	read_register(i2c_handle, &reg_addr, rx_buffer, 6);

	*p_x_acc = (rx_buffer[0] << 8) | rx_buffer[1];
	*p_y_acc = (rx_buffer[2] << 8) | rx_buffer[3];
	*p_z_acc = (rx_buffer[4] << 8) | rx_buffer[5];

}

//---------------------------------------------------------------------------------------------------
// Function for reading the Gyro data registers of the MPU-6050

void mpu_6050_gyro_read( volatile int16_t * p_x_gyro, volatile int16_t * p_y_gyro, volatile int16_t * p_z_gyro)
{
	uint8_t reg_addr = GYRO_XOUT_H;
	uint8_t rx_buffer[6] = {0};

	read_register(i2c_handle, &reg_addr, rx_buffer, 6);

	*p_x_gyro = (rx_buffer[0] << 8) | rx_buffer[1];
	*p_y_gyro = (rx_buffer[2] << 8) | rx_buffer[3];
	*p_z_gyro = (rx_buffer[4] << 8) | rx_buffer[5];

}

//---------------------------------------------------------------------------------------------------
// Function for enabling the Digital Low-Pass Filter of the MPU-6050

void mpu_6050_dlpf_cfg( uint8_t  reg_val)
{
	uint8_t tx_buffer[2] = {CONFIG, reg_val};
	write_register(i2c_handle, tx_buffer, sizeof(tx_buffer));
}

//---------------------------------------------------------------------------------------------------
// Function for setting the samplerate of the MPU-6050 by writting to the SMPRT_DIV register

void mpu_6050_samplerate_set(uint8_t reg_val)
{
	uint8_t tx_buffer[2] = {SMPRT_DIV, reg_val};
	write_register(i2c_handle, tx_buffer, 2);
}

//---------------------------------------------------------------------------------------------------
// Function for setting the Interrupt Enable register

void mpu6050_int_enable(uint8_t reg_val)
{
	uint8_t tx_buffer[2] = {INT_ENABLE, reg_val};
	write_register(i2c_handle, tx_buffer,2);
}

//---------------------------------------------------------------------------------------------------
// MPU-6050 Calibration function - Calculates the accelerometer and gyro offsets

void mpu6050_calibrate()
{
	static int32_t x_acc_offs,y_acc_offs,z_acc_offs = 0;
	static int32_t x_gyro_offs, y_gyro_offs, z_gyro_offs = 0;


	mpu_6050_gyro_read( &mpu6050_data.x_gyro, &mpu6050_data.y_gyro, &mpu6050_data.z_gyro);
	mpu_6050_acc_read(&mpu6050_data.x_acc, &mpu6050_data.y_acc, &mpu6050_data.z_acc);
	*(mpu6050_data_array + array_counter) = mpu6050_data;
	printf("ACC DATA:  x: %f m/s^2  y: %f m/s^2  z: %f m/s^2 \n", ((mpu6050_data.x_acc - X_ACC_OFFSET)/16384.0)*9.818, ((mpu6050_data.y_acc - Y_ACC_OFFSET)/16384.0)*9.818, ((mpu6050_data.z_acc - Z_ACC_OFFSET)/16384.0)*9.8181);
	printf("GYRO DATA:  x: %f deg/s  y: %f deg/s  z: %f deg/s \n", (mpu6050_data.x_gyro- X_GYRO_OFFSET)/131.0, (mpu6050_data.y_gyro - Y_GYRO_OFFSET)/131.0, (mpu6050_data.z_gyro - Z_GYRO_OFFSET)/131.0);
	//array_counter ++;

	x_acc_offs += (*(mpu6050_data_array + array_counter)).x_acc - X_ACC_OFFSET;
	y_acc_offs += (*(mpu6050_data_array + array_counter)).y_acc - Y_ACC_OFFSET;
	z_acc_offs += (*(mpu6050_data_array + array_counter)).z_acc - Z_ACC_OFFSET;
	x_gyro_offs += (*(mpu6050_data_array + array_counter)).x_gyro;
	y_gyro_offs += (*(mpu6050_data_array + array_counter)).y_gyro;
	z_gyro_offs += (*(mpu6050_data_array + array_counter)).z_gyro;

/*
	x_acc_offs += mpu6050_data.x_acc;
	y_acc_offs += mpu6050_data.y_acc;
	z_acc_offs += mpu6050_data.z_acc;
	x_gyro_offs += mpu6050_data.x_gyro;
	y_gyro_offs += mpu6050_data.y_gyro;
	z_gyro_offs += mpu6050_data.z_gyro;
*/
	printf("ACC OFFSET :   x_acc: %d  y_acc: %d  z_acc: %d \n", x_acc_offs, y_acc_offs, z_acc_offs);
	printf("GYRO OFFSET:   x_gyro: %d  y_gyro: %d  z_gyro: %d \n \n", x_gyro_offs, y_gyro_offs, z_gyro_offs);

	array_counter ++;

	if(array_counter == ARRAY_SIZE)
	{
		x_acc_offs = x_acc_offs/ARRAY_SIZE;
		y_acc_offs = y_acc_offs/ARRAY_SIZE;
		z_acc_offs = z_acc_offs/ARRAY_SIZE;
		x_gyro_offs = x_gyro_offs/ARRAY_SIZE;
		y_gyro_offs = y_gyro_offs/ARRAY_SIZE;
		z_gyro_offs = z_gyro_offs/ARRAY_SIZE;

		printf("ACC OFFSET DATA:  x_acc: %d  y_acc: %d  z_acc: %d \n", x_acc_offs, y_acc_offs, z_acc_offs);
		printf("GYRO OFFSET DATA:  x_gyro: %d  y_gyro: %d  z_gyro: %d \n", x_gyro_offs, y_gyro_offs, z_gyro_offs);

		measurements_done = 1;
	}

}

//---------------------------------------------------------------------------------------------------
// MPU-6050 "Interrupt Service Routine"

void mpu6050_ISR()
{
	// Read Accelerometer and Gyro data and store it in mpu6050_data struct
	mpu_6050_gyro_read( &mpu6050_data.x_gyro, &mpu6050_data.y_gyro, &mpu6050_data.z_gyro);
	mpu_6050_acc_read(&mpu6050_data.x_acc, &mpu6050_data.y_acc, &mpu6050_data.z_acc);

	// Set new measurements flag
	new_measurements = 1;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//---------------------------------------------------------------------------------------------------
// Madgwicks Implementation of the "Mahoney filter"

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Function to Convert from Quaternions to Euler Angles

void quaternion2euler(volatile float * phi,volatile float * theta, volatile float * psi, float q0, float q1, float q2, float q3)
{
	*phi 	= atan2( (2*(q2*q3 + q1*q0)), ( 1-2*(q1*q1 + q2*q2) ) );
	*theta 	= asin( 2*(q1*q3 - q2*q0));
	*psi	= atan2( 2*(q1*q2 + q3*q0), ( 1-2*(q2*q2 + q3*q3) ) );
}

// ##################################################################################################
// 					Main
// ##################################################################################################

int main(int argc, char *argv[])
{
	int result;

	//MPU6050_Data_t mpu6050_data;
	memset(mpu6050_data_array,0,sizeof(MPU6050_Data_t));

	//Initialize the I2C Bus
	i2c_init(&i2c_handle);

	//Initialize the MPU-6050
	mpu_6050_init();

	//Configure the Digital Low Pass filter of the MPU-6050
	mpu_6050_dlpf_cfg(0x02);

	//Set the sample rate to 50Hz ( 1000Hz/20)
	mpu_6050_samplerate_set(0x14); //100Hz: 0x0A//0x64

	// Enable the Data Ready interrupt from the MPU-6050
	mpu6050_int_enable(0x01);

	// Setup the wiringPi library
	wiringPiSetup() ;

	//Configure the MPU6050 ISR
	result = wiringPiISR(INT_PIN, INT_EDGE_RISING, mpu6050_ISR );
	if(result == -1)
	{
		printf("Failed to setup interrupt\n");
	}

	float ax, ay, az = 0;
	float gx, gy, gz = 0;
	volatile float phi, theta, psi = 0;

	for(;;)
	{
		if(new_measurements)
		{
			// Convert to m/s^2 and rad/s
			ax = ((mpu6050_data.x_acc - X_ACC_OFFSET)/16384.0)*9.818;
			ay = ((mpu6050_data.y_acc - Y_ACC_OFFSET)/16384.0)*9.818;
			az = ((mpu6050_data.z_acc - Z_ACC_OFFSET)/16384.0)*9.8181;

			gx = ((mpu6050_data.x_gyro- X_GYRO_OFFSET)/131.0)*(M_PI/180);
			gy = ((mpu6050_data.y_gyro - Y_GYRO_OFFSET)/131.0)*(M_PI/180);
			gz = ((mpu6050_data.z_gyro - Z_GYRO_OFFSET)/131.0)*(M_PI/180);

			// Update attitude estiamate
			MahonyAHRSupdateIMU( gx, gy, gz, ax, ay, az);
			//printf("Quaterions: q0: %f q1: %f q2: %f q4: %f \n",q0,q1,q2,q3);

			//Convert from Quaternions to Euler angles
			quaternion2euler(&phi, &theta, &psi, q0, q1, q2 ,q3);
			printf("Euler Angles: phi: %f theta: %f psi: %f \n", phi*(180/M_PI), theta*(180/M_PI), psi*(180/M_PI));
		}
	}
	return 0;
}

