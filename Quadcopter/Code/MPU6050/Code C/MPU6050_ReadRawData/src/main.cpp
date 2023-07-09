#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

#define sampleFreq 5.0f								   // sample frequency in Hz
#define betaDef 0.1f

typedef void (*function)();
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;					   // 2 * proportional gain
int16_t dt = (1/sampleFreq)*1000;
volatile float beta = betaDef;							   // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
double yaw = 0, pitch = 0, roll = 0;
unsigned long time_now = 0;	

void init_MPU6050();
void read_MPU6050();
void print_value();
void calibration();
void Simpletime(function func, int Simple_time);
void blinkled();
float invSqrt(float x);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void GetEuler();
void setup()
{
	Wire.begin();

	init_MPU6050();
	pinMode(13, OUTPUT);
	Serial.begin(9600);
	//calibration();
}

void loop()
{

	Simpletime(&read_MPU6050, dt);
	
}

void init_MPU6050()
{
	// MPU6050 Initializing & Reset
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B); // PWR_MGMT_1 register
	Wire.write(0);	  // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);

	// MPU6050 Clock Type
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B); // PWR_MGMT_1 register
	Wire.write(0x03); // Selection Clock 'PLL with Z axis gyroscope reference'
	Wire.endTransmission(true);

	// MPU6050 Gyroscope Configuration Setting
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1B); // Gyroscope Configuration register
	// Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
	// Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
	// Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
	Wire.write(0x18); // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
	Wire.endTransmission(true);

	// MPU6050 Accelerometer Configuration Setting
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1C); // Accelerometer Configuration register
	// Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
	// Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
	Wire.write(0x10); // AFS_SEL=2, Full Scale Range = +/- 8 [g]
	// Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
	Wire.endTransmission(true);

	// MPU6050 DLPF(Digital Low Pass Filter)
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1A); // DLPF_CFG register
	Wire.write(0x00); // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
	// Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz
	// Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz
	// Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz
	// Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz
	// Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz
	// Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz
	Wire.endTransmission(true);
}
void read_MPU6050()
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
	AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	print_value();
}
void print_value()
{
	// Print Data
	Serial.print("AcX = ");
	Serial.print(AcX);
	Serial.print(" | AcY = ");
	Serial.print(AcY);
	Serial.print(" | AcZ = ");
	Serial.print(AcZ);
	// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
	Serial.print(" | GyX = ");
	Serial.print(GyX);
	Serial.print(" | GyY = ");
	Serial.print(GyY);
	Serial.print(" | GyZ = ");
	Serial.println(GyZ);
}
void Simpletime(function func, int16_t Simple_time)
{
	if (millis() > time_now + Simple_time - 1)
	{
		(*func)();
		time_now += Simple_time;
	}
}
void blinkled()
{
	static bool state = 0;
	digitalWrite(13, state);
	if (state == 0)
		state = 1;
	else
		state = 0;
}
void calibration()
{
	long Cal_AcX, Cal_AcY, Cal_AcZ, Cal_GyX, Cal_GyY, Cal_GyZ;
	for (int i = 0; i < 2000; i++)
	{

		if (i % 200 == 0)
			Serial.println("Calculating .....");

		Wire.beginTransmission(MPU_ADDR);
		Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
		Wire.endTransmission(false);
		Wire.requestFrom(MPU_ADDR, 14, true);		   // request a total of 14 registers
		AcX = Wire.read() << 8 | Wire.read();		   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
		AcY = Wire.read() << 8 | Wire.read();		   // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		AcZ = (Wire.read() << 8 | Wire.read()) - 4096; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		Tmp = Wire.read() << 8 | Wire.read();		   // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		GyX = Wire.read() << 8 | Wire.read();		   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		GyY = Wire.read() << 8 | Wire.read();		   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		GyZ = Wire.read() << 8 | Wire.read();		   // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
		delay(10);

		// Sum data
		Cal_AcX += AcX;
		Cal_AcY += AcY;
		Cal_AcZ += AcZ;
		Cal_GyX += GyX;
		Cal_GyY += GyY;
		Cal_GyZ += GyZ;
	}

	// Average Data
	Cal_AcX /= 2000;
	Cal_AcY /= 2000;
	Cal_AcZ /= 2000;
	Cal_GyX /= 2000;
	Cal_GyY /= 2000;
	Cal_GyZ /= 2000;

	// Print Data
	Serial.println("End of Calculation");
	Serial.print("AcX = ");
	Serial.print(Cal_AcX);
	Serial.print(" | AcY = ");
	Serial.print(Cal_AcY);
	Serial.print(" | AcZ = ");
	Serial.print(Cal_AcZ);
	// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
	Serial.print(" | GyX = ");
	Serial.print(Cal_GyX);
	Serial.print(" | GyY = ");
	Serial.print(Cal_GyY);
	Serial.print(" | GyZ = ");
	Serial.println(Cal_GyZ);
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

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
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
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
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void GetEuler()
{
	yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1);	// psi
	pitch = -asin(2 * q1 * q3 + 2 * q0 * q2);								// theta
	roll = atan2(2 * q2 * q3 - 2 * q0 * q1, 2 * q0 * q0 + 2 * q3 * q3 - 1); // phi
	Serial.print("Yaw = ");
	Serial.print(yaw);
	Serial.print(" | Pitch = ");
	Serial.print(pitch);
	Serial.print(" | Roll = ");
	Serial.print(roll);
}