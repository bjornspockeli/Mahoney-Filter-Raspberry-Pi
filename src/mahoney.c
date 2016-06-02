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
#include <termios.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	50.f 		// Sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	//0.5 2 * proportional gain
#define twoKiDef	(2.0f * 0.01f)	//0.1 2 * integral gain

// Raspberry Pi Pins
#define MPU6050_INT_PIN		7
#define HMC5883_INT_PIN		0

// I2C Addresses
#define MPU6050_ADDR	0x68
#define HMC5883_ADDR	0x1E


// HMC5883 Register Addresses
#define CONF_REG_A	0x00
#define CONF_REG_B	0x01
#define MODE_REG	0x02
#define DATA_OUT_X_H	0x03
#define DATA_OUT_X_L	0x04
#define DATA_OUT_Y_H	0x05
#define DATA_OUT_Y_L	0x06
#define DATA_OUT_Z_H	0x07
#define DATA_OUT_Z_L	0x08


// MPU-6050 Register Addresses
#define PWR_MGMT_1	0x6B
#define ACCEL_XOUT_H	0x3B
#define GYRO_XOUT_H	0x43
#define CONFIG		0x1A
#define SMPRT_DIV 	0x19
#define INT_ENABLE	0x38

// Other defines
#define ARRAY_SIZE	1000

// Sensor Offsets
#define X_ACC_OFFSET	238
#define Y_ACC_OFFSET	168
#define Z_ACC_OFFSET	309

#define X_GYRO_OFFSET	-75
#define Y_GYRO_OFFSET	215
#define Z_GYRO_OFFSET	79

#define X_MAG_OFFSET	0
#define Y_MAG_OFFSET	0
#define Z_MAG_OFFSET	0

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;						// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;						// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;			// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

// I2C BUS Filehandles
static int i2c_handle;
static int i2c_handle_hmc5883;

// UART Filehandle
static int uart0_filestream = -1;

// UART Buffer
char uart_tx_buffer[29];


// Sensor Data Structures Typedefs
typedef struct {
	int16_t x_acc;
	int16_t y_acc;
	int16_t z_acc;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
} MPU6050_Data_t;

typedef struct {
	float x_acc;
	float y_acc;
	float z_acc;
	float x_gyro;
	float y_gyro;
	float z_gyro;
} MPU6050_Scaled_Data_t;

typedef struct {
	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;
} HMC5883_Data_t;

typedef struct {
	float x_mag;
	float y_mag;
	float z_mag;
} HMC5883_Corr_Data_t;

typedef struct {
	//Quaternions
	float q0;
	float q1;
	float q2;
	float q3;
	//Euler Angles
	float phi;
	float theta;
	float psi;
} Attitude_t;

int array_counter = 0;

volatile int measurements_done = 0;

// New measurements flag
volatile int new_measurements		= 0;
volatile int new_mag_measurement 	= 0;


// Sensor Data Structures
volatile MPU6050_Data_t mpu6050_data;
volatile HMC5883_Data_t hmc5883_data;
volatile HMC5883_Corr_Data_t hmc5883_corr_data;
volatile MPU6050_Scaled_Data_t mpu6050_scaled_data;

// Estimate Data Structures
volatile Attitude_t attitude;


MPU6050_Data_t mpu6050_data_array[ARRAY_SIZE];

const MPU6050_Data_t * p_mpu6050_data = mpu6050_data_array;

//Function Declarations

void send_to_magmaster(volatile int16_t * x_mag_raw, volatile int16_t * y_mag_raw, volatile int16_t * z_mag_raw);



// ##################################################################################################
// 					Functions
// ##################################################################################################

//---------------------------------------------------------------------------------------------------
// Function to initializing the I2C Bus

void i2c_init( int * i2c_handle, uint8_t slave_addr)
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
	//int mpu6050_addr = 0x68;
	if(ioctl(*i2c_handle, I2C_SLAVE, slave_addr) == -1)
	{
		printf("Could not access I2C bus \n");
	}

}

//---------------------------------------------------------------------------------------------------
// Function for writting to data registers of the MPU-6050 over the I2C bus

void read_register(int i2c_filehandle, uint8_t * reg_addr, uint8_t * rx_buffer, int bytes)
{
	int result;
	result = write(i2c_filehandle, reg_addr, 1);
	if(result < 0)
	{
		printf("Could not write to I2C Bus \n");
	}
	result = read(i2c_filehandle, rx_buffer, bytes);
	if(result < 0)
	{
		printf("Could not read from I2C Bus \n");
	}

}

//---------------------------------------------------------------------------------------------------
// Function for writting to data registers of the MPU-6050 over the I2C bus

void write_register( int i2c_filehandle, uint8_t * tx_buffer, int bytes)
{
	int result;
	result = write(i2c_filehandle, tx_buffer, bytes);
	if(result < 0)
	{
		printf("Could not write to I2C Bus \n");
	}
}

//---------------------------------------------------------------------------------------------------
// Function for initializing the HMC5883

void hmc5883_init()
{
	// Set the HMC5883 in Single-Measurement Mode
	uint8_t tx_buffer[2] = {MODE_REG,0x01};
	write_register(i2c_handle_hmc5883, tx_buffer, 2);

	// Set measurement rate to 7.5Hz
	tx_buffer[0] = CONF_REG_A;
	tx_buffer[1] = 0x01 << 2;
	write_register(i2c_handle_hmc5883, tx_buffer, 2);
	/*
	uint8_t rx_buffer = -1;
	uint8_t reg_addr = CONF_REG_A;
	read_register(i2c_handle_hmc5883, &reg_addr, &rx_buffer, 1);
	printf("CONF_REG_A: %d \n", rx_buffer);
	*/
}

//---------------------------------------------------------------------------------------------------
// Function for reading the HMC5883 Magnetometer Data registers

void hmc5883_mag_read(volatile int16_t * p_x_mag, volatile int16_t * p_y_mag, volatile int16_t * p_z_mag)
{
	uint8_t reg_addr = DATA_OUT_X_H;
	uint8_t rx_buffer[6] = {0};

	read_register(i2c_handle_hmc5883, &reg_addr, rx_buffer, 6);

	*p_x_mag = (rx_buffer[0] << 8) | rx_buffer[1];
	*p_z_mag = (rx_buffer[2] << 8) | rx_buffer[3];
	*p_y_mag = (rx_buffer[4] << 8) | rx_buffer[5];

	//Put HMC5883 back in single measurement mode
	uint8_t tx_buffer[2] = {MODE_REG,0x01};
	write_register(i2c_handle_hmc5883, tx_buffer, 2);

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
// Function for initializing the MPU-6050

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

	//Configure the Digital Low Pass filter of the MPU-6050
	mpu_6050_dlpf_cfg(0x02);

	//Set the sample rate to 50Hz ( 1000Hz/20)-1 = 0x13
	mpu_6050_samplerate_set(0x13); //100Hz: 0x0A//0x64

	// Enable the Data Ready interrupt from the MPU-6050
	mpu6050_int_enable(0x01);

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

	// Read Magnetometer Data and store it in hmc5883_data struct
	hmc5883_mag_read(&hmc5883_data.x_mag, &hmc5883_data.y_mag, &hmc5883_data.z_mag);

	// Send measurements to magmaster
	//send_to_magmaster(&hmc5883_data.x_mag, &hmc5883_data.y_mag, &hmc5883_data.z_mag);
	// Set new measurements flag
	new_measurements = 1;
}

//---------------------------------------------------------------------------------------------------
// HMC5883 "Interrupt Service Routine"

void hmc5883_ISR()
{
	//printf("HMC5883 Interrupt Detected! \n");

	// Read Magnetometer Data and store it in hmc5883_data struct
	//hmc5883_mag_read(&hmc5883_data.x_mag, &hmc5883_data.y_mag, &hmc5883_data.z_mag);

	//Set new mag measurement flag
	//new_mag_measurement = 1;

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
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

	        // Auxiliary variables to avoid repeated arithmetic
        	q0q0 = q0 * q0;
     	   	q0q1 = q0 * q1;
       		q0q2 = q0 * q2;
        	q0q3 = q0 * q3;
        	q1q1 = q1 * q1;
        	q1q2 = q1 * q2;
        	q1q3 = q1 * q3;
        	q2q2 = q2 * q2;
        	q2q3 = q2 * q3;
        	q3q3 = q3 * q3;

       		// Reference direction of Earth's magnetic field
        	hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        	hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        	bx = sqrt(hx * hx + hy * hy);
        	bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        	halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        	halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);

			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
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

//---------------------------------------------------------------------------------------------------
// Function to correct Hard and Soft Iron Distortions in Magnetometer Measurements

void mag_correction(volatile HMC5883_Corr_Data_t * hmc5883_corr_data, float raw_mx, float raw_my, float raw_mz)
{
	//float M[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	//float bx = 0,by = 0, bz = 0;
/*
	float M[3][3] =
	{
		{1.031, 0.009, 0.034},
		{0.009, 0.967, 0.006},
		{0.061, 0.023, 1.062}
	};
*/
	float M[3][3] = {{1.062,0.036,0.192},{0.003,0.907,-0.047},{0.041,0.004,1.124}};
	float bx = 15.76, by = -145.902, bz = -18.159;
	//float bx = 74.713,by =  -165.187, bz = 3.536;

	hmc5883_corr_data->x_mag = (M[0][0]*(raw_mx-bx) + M[0][1]*(raw_my-by) + M[0][2]*(raw_mz-bz) );
	hmc5883_corr_data->y_mag = (M[1][0]*(raw_mx-bx) + M[1][1]*(raw_my-by) + M[1][2]*(raw_mz-bz) );
	hmc5883_corr_data->z_mag = (M[2][0]*(raw_mx-bx) + M[2][1]*(raw_my-by) + M[2][2]*(raw_mz-bz) );

}

//---------------------------------------------------------------------------------------------------
// Function to setup the UART
void uart_init()
{
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(uart0_filestream == -1)
	{
		printf("Could not open UART. \n");
	}

	struct termios options;
	tcgetattr(uart0_filestream, &options);

	options.c_cflag = B921600 | CS8 | CLOCAL | CREAD; //B921600 ,B115200
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
}

//---------------------------------------------------------------------------------------------------
// Function to send bytes over the UART
void uart_send_bytes(int * uart_filestream, char * tx_buffer, uint32_t bytes)
{
	if(*uart_filestream != -1)
	{
		int bytes_sent = write(*uart_filestream, tx_buffer, bytes);
		if(bytes_sent < 0)
		{
			printf("UART TX Error\n");
		}
	}
}
//---------------------------------------------------------------------------------------------------
// Function to send magnetometer measurements to MagMaster
void send_to_magmaster(volatile int16_t * x_mag_raw, volatile int16_t * y_mag_raw, volatile int16_t * z_mag_raw)
{
	char tx_buffer[32];// = {'1',',','2',',','3','\n'};
	sprintf(tx_buffer, "%f,%f,%f\n", (float)*x_mag_raw, (float)*y_mag_raw, (float)*z_mag_raw);
	//printf("STRLEN: %d\n", sizeof(tx_buffer));
	uart_send_bytes(&uart0_filestream,tx_buffer,strlen(tx_buffer));

}


//---------------------------------------------------------------------------------------------------
// Function to send attitude estimates to Processing Sketch
void send_to_processing(volatile float * q0, volatile float * q1, volatile float * q2, volatile float * q3, volatile float * phi, volatile float * theta, volatile float * psi)
{
	long * ptr = 0;
	// Send estimates over UART
	ptr = (long *) q0;
	uart_tx_buffer[0] = *ptr;
	uart_tx_buffer[1] = *ptr>>8;
	uart_tx_buffer[2] = *ptr>>16;
	uart_tx_buffer[3] = *ptr>>24;
	ptr = (long *) q1;
	uart_tx_buffer[4] = *ptr;
	uart_tx_buffer[5] = *ptr>>8;
	uart_tx_buffer[6] = *ptr>>16;
	uart_tx_buffer[7] = *ptr>>24;
	ptr = (long *) q2;
	uart_tx_buffer[8] = *ptr;
	uart_tx_buffer[9] = *ptr>>8;
	uart_tx_buffer[10] = *ptr>>16;
	uart_tx_buffer[11] = *ptr>>24;
	ptr = (long *) q3;
	uart_tx_buffer[12] = *ptr;
	uart_tx_buffer[13] = *ptr>>8;
	uart_tx_buffer[14] = *ptr>>16;
	uart_tx_buffer[15] = *ptr>>24;
	ptr = (long *) phi;
	uart_tx_buffer[16] = *ptr;
	uart_tx_buffer[17] = *ptr>>8;
	uart_tx_buffer[18] = *ptr>>16;
	uart_tx_buffer[19] = *ptr>>24;
	ptr = (long *) theta;
	uart_tx_buffer[20] = *ptr;
	uart_tx_buffer[21] = *ptr>>8;
	uart_tx_buffer[22] = *ptr>>16;
	uart_tx_buffer[23] = *ptr>>24;
	ptr = (long *) psi;
	uart_tx_buffer[24] = *ptr;
	uart_tx_buffer[25] = *ptr>>8;
	uart_tx_buffer[26] = *ptr>>16;
	uart_tx_buffer[27] = *ptr>>24;
	uart_tx_buffer[28] = '\n';
	uart_send_bytes(&uart0_filestream,uart_tx_buffer, sizeof(uart_tx_buffer));
}

//---------------------------------------------------------------------------------------------------
// Function to calculate heading from magnetometer measurments
void get_heading(float * heading, float y_mag, float x_mag)
{
	*heading = atan2( y_mag, x_mag);

	if((*heading)<0)
	{
		*heading += 2*M_PI;
	}
	if((*heading)>2*M_PI)
	{
		(*heading) -= 2*M_PI;
	}
}



// ##################################################################################################
// 					Main
// ##################################################################################################

int main(int argc, char *argv[])
{
	int result;

	//Initialize UART
	uart_init();

	//Initialize the I2C Bus
	i2c_init(&i2c_handle, MPU6050_ADDR);
	i2c_init(&i2c_handle_hmc5883,HMC5883_ADDR);

	//Initialize the HMC5883
	hmc5883_init();

	//Initialize the MPU-6050
	mpu_6050_init();

	// Setup the wiringPi library
	wiringPiSetup();

	//Configure the MPU6050 ISR
	result = wiringPiISR(MPU6050_INT_PIN, INT_EDGE_RISING, mpu6050_ISR );
	if(result == -1)
	{
		printf("Failed to setup interrupt\n");
	}

	//Configure the HMC5883 ISR
	result = wiringPiISR(HMC5883_INT_PIN, INT_EDGE_RISING, hmc5883_ISR );
	if(result == -1)
	{
		printf("Failed to setup interrupt\n");
	}

	float heading = 0;
	for(;;)
	{
		if(new_measurements)
		{
			// Reset meas_flag
			new_measurements = 0;

			// Convert to m/s^2 and rad/s
			mpu6050_scaled_data.x_acc = ((mpu6050_data.x_acc - X_ACC_OFFSET)/16384.0)*9.818; //ax
			mpu6050_scaled_data.y_acc = ((mpu6050_data.y_acc - Y_ACC_OFFSET)/16384.0)*9.818; //ay
			mpu6050_scaled_data.z_acc = ((mpu6050_data.z_acc - Z_ACC_OFFSET)/16384.0)*9.818; //az

			mpu6050_scaled_data.x_gyro = ((mpu6050_data.x_gyro - X_GYRO_OFFSET)/131.0)*(M_PI/180); //gx
			mpu6050_scaled_data.y_gyro = ((mpu6050_data.y_gyro - Y_GYRO_OFFSET)/131.0)*(M_PI/180); //gy
			mpu6050_scaled_data.z_gyro = ((mpu6050_data.z_gyro - Z_GYRO_OFFSET)/131.0)*(M_PI/180); //gz

			//Correct Magnetometer measurements for Hard and Soft Iron Distortions
			mag_correction(&hmc5883_corr_data,
				(float)hmc5883_data.x_mag,
				(float)hmc5883_data.y_mag,
				(float)hmc5883_data.z_mag);

			// Update attitude estiamates
			MahonyAHRSupdate(mpu6050_scaled_data.x_gyro,
					mpu6050_scaled_data.y_gyro,
					mpu6050_scaled_data.z_gyro,
					mpu6050_scaled_data.x_acc,
 					mpu6050_scaled_data.y_acc,
					mpu6050_scaled_data.z_acc,
					hmc5883_corr_data.x_mag*0.92,
					hmc5883_corr_data.y_mag*0.92,
					hmc5883_corr_data.z_mag*0.92);

			get_heading(&heading, (float)hmc5883_data.y_mag*0.92, (float)hmc5883_data.x_mag*0.92 );

			printf("Heading: %f (Radians) \n", heading);
			printf("Heading: %f (Degrees) \n", heading*(180/M_PI));


			//Convert from Quaternions to Euler angles
			quaternion2euler(&attitude.phi, &attitude.theta, &attitude.psi, q0, q1, q2 ,q3);
			//printf("Euler Angles: phi: %f theta: %f psi: %f \n", phi*(180/M_PI), theta*(180/M_PI), psi*(180/M_PI));

			//sprintf(uart_tx_buffer,"Euler Angles: phi: %f theta: %f psi: %f \n", phi*(180/M_PI), theta*(180/M_PI), psi*(180/M_PI));
			//uart_send_bytes(&uart0_filestream,uart_tx_buffer, strlen(uart_tx_buffer));

			// Send estimates to Processing Sketch over UART
			send_to_processing(&q0, &q1, &q2, &q3, &attitude.phi, &attitude.theta, &attitude.psi);

		}
		if(new_mag_measurement)
		{
			// Reset new_mag_meas flag
			new_mag_measurement = 0;

			// Convert  to Gauss
		/*	mx =(hmc5883_data.x_mag - X_MAG_OFFSET)/1090.0;
			my =(hmc5883_data.y_mag - Y_MAG_OFFSET)/1090.0;
			mz =(hmc5883_data.z_mag - Z_MAG_OFFSET)/1090.0;
		*/
			//Correct Magnetometer measurements for Hard and Soft Iron Distortions
			mag_correction(&hmc5883_corr_data,
				(float)hmc5883_data.x_mag,
				(float)hmc5883_data.y_mag,
				(float)hmc5883_data.z_mag);

			MahonyAHRSupdate(mpu6050_scaled_data.x_gyro,
					mpu6050_scaled_data.y_gyro,
					mpu6050_scaled_data.z_gyro,
					mpu6050_scaled_data.x_acc,
 					mpu6050_scaled_data.y_acc,
					mpu6050_scaled_data.z_acc,
					hmc5883_corr_data.x_mag,
					hmc5883_corr_data.y_mag,
					hmc5883_corr_data.z_mag);

			//Convert from Quaternions to Euler angles

			quaternion2euler(&attitude.phi, &attitude.theta, &attitude.psi, q0, q1, q2 ,q3);
			printf("Euler Angles: phi: %f theta: %f psi: %f \n", attitude.phi*(180/M_PI), attitude.theta*(180/M_PI), attitude.psi*(180/M_PI));
		/*	printf("Scaled Magnetometer: X: %f Y: %f Z: %f \n", mx, my, mz);
			printf("Corrected Magnetometer: X: %f Y: %f Z: %f \n",
				hmc5883_corr_data.x_mag/1090.0,
				hmc5883_corr_data.y_mag/1090.0,
				hmc5883_corr_data.z_mag/1090.0);
		*/
			//sprintf(uart_tx_buffer, "Euler Angles: phi: %f theta: %f psi: %f \n", phi*(180/M_PI), theta*(180/M_PI), psi*(180/M_PI));

			// Send estimates to Processing Sketch over UART
			send_to_processing(&q0, &q1, &q2, &q3, &attitude.phi, &attitude.theta, &attitude.psi);
		}


	}
	return 0;
}


