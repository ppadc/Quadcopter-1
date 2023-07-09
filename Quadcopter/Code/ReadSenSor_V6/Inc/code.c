#include "code.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, TIMEOUT);
	if (check == 104) // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up, gyroscope based clock
		Data = MPU6050_WAKEUP << 6 | MPU6050_CLKSEL_PLLX;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &Data, 1, TIMEOUT);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = MPU6050_SMPLRT_DIV_1;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &Data, 1, TIMEOUT);

		// This register configures the external Frame Synchronization (FSYNC) pin sampling
		// the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers
		Data = MPU6050_EXT_SYNC_SET << 3 | MPU6050_DLPF_CFG;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_CONFIG, 1, &Data, 1, TIMEOUT);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 degree/s
		Data = MPU6050_FS_SEL_2000 << 3 | 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &Data, 1, TIMEOUT);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
		Data = MPU6050_AFS_SEL_2 << 3 | 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &Data, 1, TIMEOUT);

		// sets the I2C master clock speed
		Data = MPU6050_I2C_MST_CLK_400;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_MST_CTRL, 1, &Data, 1, TIMEOUT);
		return 0;
	}
	return 1;
}
void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;
	// disable i2c master mode
	Data = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_USER_CTRL, 1, &Data, 1, TIMEOUT);
	// enable i2c master bypass mode
	Data = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_INT_PIN_CFG, 1, &Data, 1, TIMEOUT);
}
void MPU6050_Master(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;
	// disable i2c master bypass mode
	Data = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_INT_PIN_CFG, 1, &Data, 1, TIMEOUT);
	// enable i2c master mode
	Data = 0x20;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_USER_CTRL, 1, &Data, 1, TIMEOUT);
}
// configure the MPU6050 to automatically read the magnetometer
void MPU6050_Addslave(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;
	// slave 0 i2c address, read mode
	Data = HMC5883L_ADDRESS | 0x80;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_SLV0_ADDR, 1, &Data, 1, TIMEOUT);
	// slave 0 register = 0x03 (x axis)
	Data = HMC5883L_DATAX_H;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_SLV0_REG, 1, &Data, 1, TIMEOUT);
	// slave 0 transfer size = 6, enabled
	Data = 0x06 | 0x80;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_SLV0_CTRL, 1, &Data, 1, TIMEOUT);
	// enable slave 0 delay
	Data = MPU6050_I2C_SLV0_DLY_EN;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_MST_DELAY_CTRL, 1, &Data, 1, TIMEOUT);
}
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[20];

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 20, TIMEOUT);

	DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	//DataStruct->Temp_RAW = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

	DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	DataStruct->Magn_X_RAW = (int16_t)(Rec_Data[14] << 8 | Rec_Data[15]);
	DataStruct->Magn_Z_RAW = (int16_t)(Rec_Data[16] << 8 | Rec_Data[17]);
	DataStruct->Magn_Y_RAW = (int16_t)(Rec_Data[18] << 8 | Rec_Data[19]);

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_ACCE_SENS_2;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_ACCE_SENS_2;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_ACCE_SENS_2;

	//DataStruct->Temperature = DataStruct->Temp_RAW / 340.0 + 36.53;

	DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_GYRO_SENS_2000 * DEG_TO_RAD;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_GYRO_SENS_2000 * DEG_TO_RAD;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_GYRO_SENS_2000 * DEG_TO_RAD;

	DataStruct->Mx = DataStruct->Magn_X_RAW / HMC5883L_MAGN_SENS_1P3;
	DataStruct->My = DataStruct->Magn_Y_RAW / HMC5883L_MAGN_SENS_1P3;
	DataStruct->Mz = DataStruct->Magn_Z_RAW / HMC5883L_MAGN_SENS_1P3;
}
void HMC5883L_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;
	// write CONFIG_A register
	Data = HMC5883L_AVERAGING_1 << 5 | HMC5883L_RATE_75 << 2 | HMC5883L_BIAS_NORMAL;
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_CONFIG_A, 1, &Data, 1, TIMEOUT);
	// write CONFIG_B register
	Data = HMC5883L_SEL_1P3 << 5 | 0x00;
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_CONFIG_B, 1, &Data, 1, TIMEOUT);
	// write MODE register
	Data = HMC5883L_MODE_CONTINUOUS;
	HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_MODE, 1, &Data, 1, TIMEOUT);
}
void MS5611_Rest(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;

	Data = 0;
	HAL_I2C_Mem_Write(I2Cx, MS5611_ADDR, MS5611_CMD_REST, 1, &Data, 1, TIMEOUT);
	HAL_Delay(4);
}
uint8_t MS5611_PROM_read(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	uint8_t i;
	uint8_t data[2];
	uint8_t PROM[8] = {MS5611_PROM_READ_0,
										 MS5611_PROM_READ_1,
										 MS5611_PROM_READ_2,
										 MS5611_PROM_READ_3,
										 MS5611_PROM_READ_4,
										 MS5611_PROM_READ_5,
										 MS5611_PROM_READ_6,
										 MS5611_PROM_READ_7};
	//Address 0 contains factory data and the setup
	HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, PROM[0], 1, data, 2, TIMEOUT);
	datastruct->reserve = (uint16_t)(data[0] << 8 | data[1]);
	// Addresses 1-6 calibration coefficients
	for (i = 1; i < 7; i++)
	{
		HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, PROM[i], 1, data, 2, TIMEOUT);
		datastruct->C[i - 1] = (uint16_t)(data[0] << 8 | data[1]);
	}
	//Address 7 contains the serial code and CRC
	HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, PROM[7], 1, data, 2, TIMEOUT);
	datastruct->crc = (uint16_t)(data[0] << 8 | data[1]);

	return MS5611_OK;
}
uint8_t MS5611_init(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	MS5611_Rest(I2Cx);
	MS5611_PROM_read(I2Cx, datastruct);
	return MS5611_OK;
}
uint8_t MS5611_read_temp(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	uint8_t data[3];
	uint8_t Data;

	HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, MS6511_ADC_READ, 1, data, 3, TIMEOUT);
	datastruct->D[1] = (data[0] << 16 | data[1] << 8 | data[2]);
	Data = 0;
	HAL_I2C_Mem_Write(I2Cx, MS5611_ADDR, MS5611_CMD_CONVERT_D1_2048, 1, &Data, 1, TIMEOUT);// delay 4.13ms read press
	return MS5611_OK;
}
uint8_t MS5611_read_press(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	uint8_t data[3];
	uint8_t Data;
	
	HAL_I2C_Mem_Read(I2Cx, MS5611_ADDR, MS6511_ADC_READ, 1, data, 3, TIMEOUT);
	datastruct->D[0] = (data[0] << 16 | data[1] << 8 | data[2]);
	Data = 0;
	HAL_I2C_Mem_Write(I2Cx, MS5611_ADDR, MS5611_CMD_CONVERT_D2_2048, 1, &Data, 1, TIMEOUT);// delay 4.13ms read temp
	return MS5611_OK;
}
uint8_t MS5611_calculate(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	static int time = 0;
	static int8_t mode = 0;
	int64_t dT = 0, TEMP = 0, T2 = 0, OFF = 0, OFF2 = 0, SENS2 = 0, SENS = 0, PRES = 0;
	time++;
	if(time>2)
	{
	time = 0;
	if (mode)
	{
		MS5611_read_temp(I2Cx,datastruct);
		mode = 0;
	}
	else 
	{
		MS5611_read_press(I2Cx,datastruct);
		mode = 1;
	}
	if(mode == 0)
	{
	dT = datastruct->D[1] - ((int32_t)(datastruct->C[4]) << 8);
	TEMP = 2000 + ((int32_t)(dT * (datastruct->C[5])) >> 23);
	OFF = (((int64_t)(datastruct->C[1])) << 16) + (((datastruct->C[3]) * dT) >> 7);
	SENS = (((int64_t)(datastruct->C[0])) << 15) + (((datastruct->C[2]) * dT) >> 8);

	if (TEMP < 2000)
	{ // temperature < 20°C
		T2 = (dT * dT) >> 31;
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;

		if (TEMP < -1500)
		{ // temperature < -15°C
			OFF2 = OFF2 + (7 * (TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + (11 * (TEMP + 1500) * (TEMP + 1500) / 2);
		}
	}
	else
	{ // temperature > 20°C
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	datastruct->dT = dT;
	datastruct->OFF = OFF - OFF2;
	datastruct->TEMP = TEMP - T2;
	datastruct->SENS = SENS - SENS2;

	PRES = ((((int32_t)(datastruct->D[0]) * (datastruct->SENS)) >> 21) - (datastruct->OFF)) >> 15;
	datastruct->P = PRES;
	}
	}
	return MS5611_OK;
}
double getAltitude(double pressure, double referencePressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)referencePressure, 0.1902949f)));
}
// Multiply two quaternions and return a copy of the result, prod = L * R
Quaternion_t quat_mult(Quaternion_t L, Quaternion_t R)
{
	Quaternion_t product;
	product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
	product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
	product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
	product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);

	return product;
}
// Multiply a reference of a quaternion by a scalar, q = s*q
void quat_scalar(Quaternion_t *q, float scalar)
{
	q->q1 *= scalar;
	q->q2 *= scalar;
	q->q3 *= scalar;
	q->q4 *= scalar;
}
// Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
void quat_add(Quaternion_t *Sum, Quaternion_t L, Quaternion_t R)
{
	Sum->q1 = L.q1 + R.q1;
	Sum->q2 = L.q2 + R.q2;
	Sum->q3 = L.q3 + R.q3;
	Sum->q4 = L.q4 + R.q4;
}
// Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
void quat_sub(Quaternion_t *Sum, Quaternion_t L, Quaternion_t R)
{
	Sum->q1 = L.q1 - R.q1;
	Sum->q2 = L.q2 - R.q2;
	Sum->q3 = L.q3 - R.q3;
	Sum->q4 = L.q4 - R.q4;
}
// the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
Quaternion_t quat_conjugate(Quaternion_t q)
{
	q.q2 = -q.q2;
	q.q3 = -q.q3;
	q.q4 = -q.q4;
	return q;
}
// norm of a quaternion is the same as a complex number
float quat_Norm(Quaternion_t q)
{
	return sqrt(q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4);
}
// Normalizes pointer q by calling quat_Norm(q),
void quat_Normalization(Quaternion_t *q)
{
	float norm = quat_Norm(*q);
	q->q1 /= norm;
	q->q2 /= norm;
	q->q3 /= norm;
	q->q4 /= norm;
}
void Madgwick_update(MPU6050_t *DataStruct,Quaternion_t *q_est)
{
	float ax = DataStruct->Ax;
	float ay = DataStruct->Ay;
	float az = DataStruct->Az;
	float gx = DataStruct->Gx;
	float gy = DataStruct->Gy;
	float gz = DataStruct->Gz;
	float mx = DataStruct->Mx;
	float my = DataStruct->My;
	float mz = DataStruct->Mz;
	// Variables and constants
	float F_g[3] = {0};	   // eq(15/21/25) objective function for gravity
	float J_g[3][4] = {0}; // jacobian matrix for gravity
	float F_b[3] = {0};	   // eq(15/21/29) objective function for magnetic
	float J_b[3][4] = {0}; // jacobian matrix for magnetic
	static Quaternion_t q_w_gradient_integral = {0};
	Quaternion_t q_est_prev = *q_est;
	Quaternion_t q_est_dot = {0}; // eq 42 and 43
	Quaternion_t gradient = {0};

	Quaternion_t q_a = {0, ax, ay, az}; // eq (24)
	if (quat_Norm(q_a) == 0)
		return;
	quat_Normalization(&q_a); // normalize the acceleration quaternion to be a unit quaternion
	// const Quaternion_t q_g_ref = {0, 0, 0, 1};// eq (23) not needed because I used eq 25 instead of eq 21

	Quaternion_t q_m = {0, mx, my, mz}; // eq (28)
	if (quat_Norm(q_m) == 0)
	{
		Madgwick_imu(DataStruct,q_est);
		return;
	}
	quat_Normalization(&q_m); // normalize the magnetic quaternion to be a unit quaternion
	// const Quaternion_t q_b_ref = {0, 0.99, 0, -0.13};
	//  Magnetic distortion compensation
	Quaternion_t h = quat_mult(q_est_prev, quat_mult(q_m, quat_conjugate(q_est_prev))); // eq(45) (Group 1)
	Quaternion_t b = {0, sqrt(h.q2 * h.q2 + h.q3 * h.q3), 0, h.q4};						// {0, 0.99, 0, -0.13} // eq(46)

	Quaternion_t q_w = {0, gx, gy, gz}; // eq (10), places gyroscope readings in a quaternion

	// Compute the objective function for gravity, simplified to equation (25) due to the 0's in the acceleration reference quaternion
	F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
	F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) - q_a.q3;
	F_g[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;

	// Compute the Jacobian matrix, equation (26), for gravity
	J_g[0][0] = -2 * q_est_prev.q3;
	J_g[0][1] = 2 * q_est_prev.q4;
	J_g[0][2] = -2 * q_est_prev.q1;
	J_g[0][3] = 2 * q_est_prev.q2;

	J_g[1][0] = 2 * q_est_prev.q2;
	J_g[1][1] = 2 * q_est_prev.q1;
	J_g[1][2] = 2 * q_est_prev.q4;
	J_g[1][3] = 2 * q_est_prev.q3;

	J_g[2][0] = 0;
	J_g[2][1] = -4 * q_est_prev.q2;
	J_g[2][2] = -4 * q_est_prev.q3;
	J_g[2][3] = 0;

	// Compute the objective function for magnetic, simplified to equation (29) due to the 0's in the magnetic reference quaternion
	F_b[0] = 2 * b.q2 * (0.5 - q_est_prev.q3 * q_est_prev.q3 - q_est_prev.q4 * q_est_prev.q4) + 2 * b.q4 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_m.q2;
	F_b[1] = 2 * b.q2 * (q_est_prev.q2 * q_est_prev.q3 - q_est_prev.q1 * q_est_prev.q4) + 2 * b.q4 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) - q_m.q3;
	F_b[2] = 2 * b.q2 * (q_est_prev.q1 * q_est_prev.q3 + q_est_prev.q2 * q_est_prev.q4) + 2 * b.q4 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_m.q4;

	// Compute the Jacobian matrix, equation (26), for magnetic

	J_b[0][0] = -2 * b.q4 * q_est_prev.q3;
	J_b[0][1] = 2 * b.q4 * q_est_prev.q4;
	J_b[0][2] = -4 * b.q2 * q_est_prev.q3 - 2 * b.q4 * q_est_prev.q1;
	J_b[0][3] = -4 * b.q2 * q_est_prev.q4 + 2 * b.q4 * q_est_prev.q2;

	J_b[1][0] = -2 * b.q2 * q_est_prev.q4 + 2 * b.q4 * q_est_prev.q2;
	J_b[1][1] = 2 * b.q2 * q_est_prev.q3 + 2 * b.q4 * q_est_prev.q1;
	J_b[1][2] = 2 * b.q2 * q_est_prev.q2 + 2 * b.q4 * q_est_prev.q4;
	J_b[1][3] = -2 * b.q2 * q_est_prev.q1 + 2 * b.q4 * q_est_prev.q3;

	J_b[2][0] = 2 * b.q2 * q_est_prev.q3;
	J_b[2][1] = 2 * b.q2 * q_est_prev.q4 - 4 * b.q4 * q_est_prev.q2;
	J_b[2][2] = 2 * b.q2 * q_est_prev.q1 - 4 * b.q4 * q_est_prev.q3;
	J_b[2][3] = 2 * b.q2 * q_est_prev.q2;

	// now computer the gradient, equation (20), gradient = J_g'*F_g + J_b'*F_b
	gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2] + J_b[0][0] * F_b[0] + J_b[1][0] * F_b[1] + J_b[2][0] * F_b[2];
	gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2] + J_b[0][1] * F_b[0] + J_b[1][1] * F_b[1] + J_b[2][1] * F_b[2];
	gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2] + J_b[0][2] * F_b[0] + J_b[1][2] * F_b[1] + J_b[2][2] * F_b[2];
	gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2] + J_b[0][3] * F_b[0] + J_b[1][3] * F_b[1] + J_b[2][3] * F_b[2];

	// Normalize the gradient, equation (44)
	quat_Normalization(&gradient);

	// Gyroscope bias drift compensation (Group 2)
	Quaternion_t q_w_gradient = quat_mult(quat_conjugate(q_est_prev), gradient);
	quat_scalar(&q_w_gradient, 2 * DELTA_T);
	quat_add(&q_w_gradient_integral, q_w_gradient_integral, q_w_gradient);
	quat_scalar(&q_w_gradient_integral, ZETA);
	quat_sub(&q_w, q_w, q_w_gradient_integral);
	// Orientation from angular rate
	quat_scalar(&q_w, 0.5);			  // equation (12) dq/dt = (1/2)q*w
	q_w = quat_mult(q_est_prev, q_w); // equation (12)
	// quat_scalar(&q_w, deltaT);             // eq (13) integrates the angles velocity to position
	// quat_add(&q_w, q_w, q_est_prev);       // addition part of equation (13)

	// Combining
	quat_scalar(&gradient, BETA);		 // multiply normalized gradient by beta
	quat_sub(&q_est_dot, q_w, gradient); // subtract above from q_w, the integrated gyro quaternion
	quat_scalar(&q_est_dot, DELTA_T);
	quat_add(q_est, q_est_prev, q_est_dot); // Integrate orientation rate to find position
	quat_Normalization(q_est);				 // normalize the orientation of the estimate
											 //(shown in diagram, plus always use unit quaternions for orientation)
}
void Madgwick_imu(MPU6050_t *DataStruct,Quaternion_t *q_est)
{
	float ax = DataStruct->Ax;
	float ay = DataStruct->Ay;
	float az = DataStruct->Az;
	float gx = DataStruct->Gx;
	float gy = DataStruct->Gy;
	float gz = DataStruct->Gz;
	// Variables and constants
	float F_g[3] = {0};	   // eq(15/21/25) objective function for gravity
	float J_g[3][4] = {0}; // jacobian matrix for gravity
	Quaternion_t q_est_prev = *q_est;
	Quaternion_t q_est_dot = {0}; // eq 42 and 43
	Quaternion_t gradient = {0};
	// const Quaternion_t q_g_ref = {0, 0, 0, 1};// eq (23) not needed because I used eq 25 instead of eq 21

	Quaternion_t q_a = {0, ax, ay, az}; // eq (24) raw acceleration values, needs to be normalized
	if (quat_Norm(q_a) == 0)
		return;
	quat_Normalization(&q_a); // normalize the acceleration quaternion to be a unit quaternion

	Quaternion_t q_w = {0, gx, gy, gz}; // equation (10), places gyroscope readings in a quaternion

	quat_scalar(&q_w, 0.5);			  // equation (12) dq/dt = (1/2)q*w
	q_w = quat_mult(q_est_prev, q_w); // equation (12)

	// quat_scalar(&q_w, deltaT);             // eq (13) integrates the angles velocity to position
	// quat_add(&q_w, q_w, q_est_prev);       // addition part of equation (13)

	// Compute the objective function for gravity, equation(15), simplified to equation (25) due to the 0's in the acceleration reference quaternion
	F_g[0] = 2 * (q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
	F_g[1] = 2 * (q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3 * q_est_prev.q4) - q_a.q3;
	F_g[2] = 2 * (0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;

	// Compute the Jacobian matrix, equation (26), for gravity
	J_g[0][0] = -2 * q_est_prev.q3;
	J_g[0][1] = 2 * q_est_prev.q4;
	J_g[0][2] = -2 * q_est_prev.q1;
	J_g[0][3] = 2 * q_est_prev.q2;

	J_g[1][0] = 2 * q_est_prev.q2;
	J_g[1][1] = 2 * q_est_prev.q1;
	J_g[1][2] = 2 * q_est_prev.q4;
	J_g[1][3] = 2 * q_est_prev.q3;

	J_g[2][0] = 0;
	J_g[2][1] = -4 * q_est_prev.q2;
	J_g[2][2] = -4 * q_est_prev.q3;
	J_g[2][3] = 0;

	// now computer the gradient, equation (20), gradient = J_g'*F_g
	gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
	gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
	gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
	gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

	// Normalize the gradient, equation (44)
	quat_Normalization(&gradient);
	// Combining
	quat_scalar(&gradient, BETA);		 // multiply normalized gradient by beta
	quat_sub(&q_est_dot, q_w, gradient); // subtract above from q_w, the integrated gyro quaternion
	quat_scalar(&q_est_dot, DELTA_T);
	quat_add(q_est, q_est_prev, q_est_dot); // Integrate orientation rate to find position
	quat_Normalization(q_est);				 // normalize the orientation of the estimate
											 //(shown in diagram, plus always use unit quaternions for orientation)
}
void eulerAngles(Quaternion_t q, Euler_t *Angle)
{
	Angle->yaw = atan2((2 * q.q2 * q.q3 - 2 * q.q1 * q.q4), (2 * q.q1 * q.q1 + 2 * q.q2 * q.q2 - 1)); // equation (7)
	Angle->pitch = -asin(2 * q.q2 * q.q4 + 2 * q.q1 * q.q3);										   // equatino (8)
	Angle->roll = atan2((2 * q.q3 * q.q4 - 2 * q.q1 * q.q2), (2 * q.q1 * q.q1 + 2 * q.q4 * q.q4 - 1));

	Angle->yaw 		*= -(180.0f / PI);
	Angle->pitch 	*= -(180.0f / PI);
	Angle->roll 	*= -(180.0f / PI);
}
void Quat2Angle(Quaternion_t q, Euler_t *Angle)
{
	Angle->roll		= atan2(2*(q.q1*q.q2 + q.q3*q.q4),q.q1*q.q1 - q.q2*q.q2 - q.q3*q.q3 + q.q4*q.q4);
	Angle->pitch	= asin(2*(q.q1*q.q3 - q.q2*q.q4));
	Angle->yaw		= atan2(2*(q.q1*q.q4 + q.q2*q.q3),q.q1*q.q1 + q.q2*q.q2 - q.q3*q.q3 - q.q4*q.q4);
	if(Angle->pitch == PI/2)
	{
		Angle->roll = 0;
		Angle->yaw	= -2*atan2(q.q2,q.q1);
	}
	if(Angle->pitch == -PI/2)
	{
		Angle->roll = 0;
		Angle->yaw	= 2*atan2(q.q2,q.q1);
	}
	Angle->roll 	*= RAD_TO_DEG;
	Angle->pitch 	*= RAD_TO_DEG;
	Angle->yaw 		*= RAD_TO_DEG;
}
void OffsetAngle(Euler_t *Angle, Euler_t Offset)
{
	Angle->roll		+= Offset.roll;
	Angle->pitch	+= Offset.pitch;
	Angle->yaw		+= Offset.yaw;
}
void Lowpass4MPU(MPU6050_t input,MPU6050_t *output)
{

	float ePow_Accel = 1-exp(-DELTA_T * 2 * PI * 2);
	float ePow_Gyro 	=	1-exp(-DELTA_T * 2 * PI * 1.5);
	float ePow_Magn	= 1-exp(-DELTA_T * 2 * PI * 1);
	
	output->Ax += (input.Ax - output->Ax) * ePow_Accel;
	output->Ay += (input.Ay - output->Ay) * ePow_Accel;
	output->Az += (input.Az - output->Az) * ePow_Accel;
	
//	output->Gx += (input.Gx - output->Gx) * ePow_Gyro;
//	output->Gy += (input.Gy - output->Gy) * ePow_Gyro;
//	output->Gz += (input.Gz - output->Gz) * ePow_Gyro;
//	
//	output->Mx += (input.Mx - output->Mx) * ePow_Magn;
//	output->My += (input.My - output->My) * ePow_Magn;
//	output->Mz += (input.Mz - output->Mz) * ePow_Magn;
	
	output->Gx = input.Gx;
	output->Gy = input.Gy;
	output->Gz = input.Gz;
	
	output->Mx = input.Mx;
	output->My = input.My;
	output->Mz = input.Mz;
}
void MPU2Angle(MPU6050_t DataStruct, Euler_t *Angle)
{
	Angle->roll 	= RAD_TO_DEG * atan2(DataStruct.Ay, DataStruct.Az);
  Angle->pitch 	= RAD_TO_DEG * atan2(-DataStruct.Ax, sqrt(DataStruct.Ay*DataStruct.Ay + DataStruct.Az*DataStruct.Az));
	Angle->yaw		= 0;
}
void setupESC(TIM_HandleTypeDef *HTIMx)
{
	HAL_TIM_Base_Start_IT(HTIMx);
	HAL_TIM_PWM_Start(HTIMx, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_1, 20000 * 0.04);
	HAL_TIM_PWM_Start(HTIMx, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_2, 20000 * 0.04);
	HAL_TIM_PWM_Start(HTIMx, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_3, 20000 * 0.04);
	HAL_TIM_PWM_Start(HTIMx, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_4, 20000 * 0.04);
	HAL_Delay(2000);
}
void ESC_Control(TIM_HandleTypeDef *HTIMx, uint16_t Duty1,uint16_t Duty2,uint16_t Duty3,uint16_t Duty4)
{
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_1, Duty1); // 1000 to 2000
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_2, Duty2); // 1000 to 2000
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_3, Duty3); // 1000 to 2000
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_4, Duty4); // 1000 to 2000
}
float PID_Control(float Sp, float Fb, PID_t *pid)
{
	//
	float out;
	// Caculator Error
	pid->e = Sp - Fb;
	// Caculator Delta Error
	pid->v_e = pid->e - pid->e_pre;
	// Caculator Integral Error (not include Delta t)
		// error will be integrated when error is in the range
	if (fabs(pid->e) < (pid->range*Sp))
	{	
		// Integral
		pid->e_i += pid->e;
		// Limit Integral 
		if (pid->e_i > pid->Upper_limit) pid->e_i = pid->Upper_limit;
		if (pid->e_i < pid->Lower_limit) pid->e_i = pid->Lower_limit;
	}
	else pid->e_i = 0;
	
		// converge
	if(pid->e*pid->v_e < 0)
	{
		//do something
	}
		// divergence
	else 
	{
		// do something
	}
	
	// save value
	pid->e_pre = pid->e;
	// output PID
	return out = pid->Kp*pid->e + pid->Ki*pid->e_i*DELTA_T + pid->Kd*pid->v_e/DELTA_T;
}
void Combining_Angle_height(Motor_t *m1, Motor_t *m2, Motor_t *m3, Motor_t *m4, float roll, float pitch, float yaw, float height)
{
	m1->Duty = m1->Duty_min +  pitch + yaw + height;
	m2->Duty = m1->Duty_min +  roll  - yaw + height;
	m3->Duty = m1->Duty_min + -pitch + yaw + height;
	m4->Duty = m1->Duty_min + -roll  - yaw + height;
	// limit duty
	if(m1->Duty > m1->Duty_max) m1->Duty = m1->Duty_max;
	if(m2->Duty > m2->Duty_max) m2->Duty = m2->Duty_max;
	if(m3->Duty > m3->Duty_max) m3->Duty = m3->Duty_max;
	if(m4->Duty > m4->Duty_max) m4->Duty = m4->Duty_max;
	if(m1->Duty < m1->Duty_min) m1->Duty = m1->Duty_min;
	if(m2->Duty < m2->Duty_min) m2->Duty = m2->Duty_min;
	if(m3->Duty < m3->Duty_min) m3->Duty = m3->Duty_min;
	if(m4->Duty < m4->Duty_min) m4->Duty = m4->Duty_min;
}
void Lowpass(float in,Lowpass_t *filter, float iCutOffFrequency)
{	
	filter->ePow = (1-exp(-DELTA_T * 2 * PI * iCutOffFrequency));
	filter->out += (in - filter->out) * filter->ePow;
}
