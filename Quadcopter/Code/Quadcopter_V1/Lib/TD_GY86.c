/**
 * Initializes the GY86 sensor with the given I2C handle.
 *  * * @author Tan Dat
 * @date 2023-6
 */
#include "TD_GY86.h"

/**
 * Initializes the MPU6050 sensor.
 *
 * @param I2Cx The I2C bus used to communicate with the sensor.
 *
 * @returns 0 if initialization is successful, 1 otherwise.
 */
//1. Init MPU6050 sensor function
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
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 2000 degree/s
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
/**
 * Bypasses the Digital Motion Processor (DMP) of the MPU6050 sensor.
 *
 * @param I2Cx Pointer to the I2C_HandleTypeDef struct that contains the configuration information for the specified I2C bus.
 *
 * @returns None
 */
//2.Enable Bypass MPU6050 function
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
/**
 * Configures the MPU6050 sensor as an I2C master.
 *
 * @param I2Cx Pointer to the I2C handle.
 *
 * @returns None
 */
//3. Enable Master MPU6050 function
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
/**
 * Adds a slave device to the MPU6050 I2C bus.
 *
 * @param I2Cx Pointer to the I2C_HandleTypeDef struct for the I2C bus.
 *
 * @returns None
 */
//4. Configure the MPU6050 to automatically read the magnetometer
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
/**
 * Reads the raw data from the MPU6050 sensor and computes the corresponding acceleration, gyroscope, and magnetometer values.
 *
 * @param I2Cx The I2C handle for the MPU6050 sensor.
 * @param DataStruct A pointer to the MPU6050 data structure where the computed values will be stored.
 *
 * @returns None
 */
//5. Read data from MPU6050
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
/**
 * Initializes the HMC5883L magnetometer sensor.
 *
 * @param I2Cx The I2C bus used to communicate with the sensor.
 *
 * @returns None
 */
//6. Init HMC5883L sensor
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
/**
 * Sends a reset command to the MS5611 pressure sensor.
 *
 * @param I2Cx Pointer to the I2C bus used to communicate with the sensor.
 *
 * @returns None
 */
//7. Reset MS5611 sensor
void MS5611_Rest(I2C_HandleTypeDef *I2Cx)
{
	uint8_t Data;

	Data = 0;
	HAL_I2C_Mem_Write(I2Cx, MS5611_ADDR, MS5611_CMD_REST, 1, &Data, 1, TIMEOUT);
	HAL_Delay(4);
}
/**
 * Reads the PROM data from the MS5611 pressure sensor.
 *
 * @param I2Cx The I2C bus used to communicate with the sensor.
 * @param datastruct A pointer to the MS5611_t struct where the PROM data will be stored.
 *
 * @returns MS5611_OK if the operation was successful.
 */
//8. Read PROM from MS5611 sensor
uint8_t MS5611_PROM_read(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	uint8_t i;
	uint8_t data[2];
	uint8_t PROM[8] = {	MS5611_PROM_READ_0,
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
/**
 * Initializes the MS5611 sensor by resetting it and reading the PROM data.
 *
 * @param I2Cx Pointer to the I2C_HandleTypeDef struct for the I2C bus used to communicate with the sensor.
 * @param datastruct Pointer to the MS5611_t struct that holds the PROM data.
 *
 * @returns MS5611_OK if initialization is successful.
 */
//9. Init MS5611 sensor
uint8_t MS5611_init(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct)
{
	MS5611_Rest(I2Cx);
	MS5611_PROM_read(I2Cx, datastruct);
	return MS5611_OK;
}
/**
 * Reads the temperature from the MS5611 sensor.
 *
 * @param I2Cx The I2C bus used to communicate with the sensor.
 * @param datastruct A pointer to the MS5611 data structure.
 *
 * @returns MS5611_OK if the temperature was successfully read.
 */
//10. Read temp from MS5611 sensor
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
/**
 * Reads the pressure value from the MS5611 sensor.
 *
 * @param I2Cx The I2C bus used to communicate with the sensor.
 * @param datastruct A pointer to the MS5611 data structure.
 *
 * @returns MS5611_OK if the operation was successful.
 */
//11. Read pressure from MS5611 sensor
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
/**
 * Calculates the pressure and temperature using the MS5611 sensor.
 *
 * @param I2Cx The I2C bus used to communicate with the sensor.
 * @param datastruct A pointer to the MS5611 data structure.
 *
 * @returns MS5611_OK if the operation was successful.
 */
//12. Caculator pressure
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
	dT = datastruct->D[1] - ((int32_t)(datastruct->C[4])*256);
	TEMP = 2000 + ((int32_t)(dT * (datastruct->C[5])) >> 23);
	OFF = (((int64_t)(datastruct->C[1])) << 16) + (((datastruct->C[3]) * dT) >> 7);
	SENS = (((int64_t)(datastruct->C[0])) << 15) + (((datastruct->C[2]) * dT) >> 8);

	if (TEMP < 2000)
	{ // temperature < 20�C
		T2 = (dT * dT) >> 31;
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;

		if (TEMP < -1500)
		{ // temperature < -15�C
			OFF2 = OFF2 + (7 * (TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + (11 * (TEMP + 1500) * (TEMP + 1500) / 2);
		}
	}
	else
	{ // temperature > 20�C
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
/**
 * Computes the altitude based on the given pressure and reference pressure.
 *
 * @param pressure The pressure at the current altitude.
 * @param referencePressure The reference pressure at sea level.
 *
 * @returns The altitude in meters.
 */
//13. Calculator height
double getAltitude(double pressure, double referencePressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)referencePressure, 0.1902949f)));
}
/**
 * Computes the Euler angles from the raw accelerometer data of an MPU6050 sensor.
 *
 * @param DataStruct The MPU6050 data structure containing the raw accelerometer data.
 * @param Angle A pointer to the Euler angle structure to store the computed angles.
 *
 * @returns None
 */
//14. Calculator Euler angle
void MPU2Angle(MPU6050_t DataStruct, Euler_t *Angle)
{
	Angle->roll 	= atan2(DataStruct.Ay, DataStruct.Az);
	Angle->pitch 	= atan2(-DataStruct.Ax, sqrt(DataStruct.Ay*DataStruct.Ay + DataStruct.Az*DataStruct.Az));
	Angle->yaw		= 0;
}
/**
 * Applies an offset to an Euler angle.
 *
 * @param Angle A pointer to the Euler angle to be modified.
 * @param Offset The Euler angle offset to be applied.
 *
 * @returns None
 */
//15. Offset euler angle
void OffsetAngle(Euler_t *Angle, Euler_t Offset)
{
    Angle->roll += Offset.roll;
    Angle->pitch += Offset.pitch;
    Angle->yaw += Offset.yaw;
}
