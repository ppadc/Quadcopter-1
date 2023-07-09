/**
 * @file td_gy86.h
 * @brief Header file for the TD_GY86 sensor module.
 *
 * This header file contains the declarations of functions and constants used to interface with the TD_GY86 sensor module.
 * The module contains an MPU6050 accelerometer and gyroscope, an HMC5883L magnetometer, and an MS5611 barometer.
 */
#ifndef TD_GY86_H // Kiểm tra xem FILE_NAME_H đã được định nghĩa chưa
#define TD_GY86_H // Nếu chưa thì định nghĩa nó
/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include "i2c.h"
#include <math.h>
/* Private define ------------------------------------------------------------*/
#define PI 3.14159265358979
#define RAD_TO_DEG 57.2957795130
#define DEG_TO_RAD 0.01745329252
#define DELTA_T 0.002
#define TIMEOUT 100

/* Default I2C address */
#define MPU6050_ADDRESS 0x68
#define MPU6050_ADDR 0xD0

/* Who I am register value */
#define MPU6050_I_AM 0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO 0x01
#define MPU6050_SELF_TEST_X 0X0D
#define MPU6050_SELF_TEST_Y 0X0E
#define MPU6050_SELF_TEST_Z 0X0F
#define MPU6050_SELF_TEST_A 0X10
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_MOTION_THRESH 0x1F
#define MPU6050_FIFO_EN 0X23
#define MPU6050_I2C_MST_CTRL 0X24
#define MPU6050_I2C_SLV0_ADDR 0X25
#define MPU6050_I2C_SLV0_REG 0X26
#define MPU6050_I2C_SLV0_CTRL 0X27
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_MOT_DETECT_STATUS 0x61
#define MPU6050_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_SIGNAL_PATH_RESET 0x68
#define MPU6050_MOT_DETECT_CTRL 0x69
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W 0x74
#define MPU6050_WHO_AM_I 0x75

#define MPU6050_SMPLRT_DIV_1 0x00
#define MPU6050_EXT_SYNC_SET 0x00 // Input disabled
#define MPU6050_DLPF_CFG 0x01     // acc {184Hz-2.0ms} gyro {188Hz-1.9ms} Fs 1kHz

#define MPU6050_XG_ST 0x01
#define MPU6050_YG_ST 0x01
#define MPU6050_ZG_ST 0x01
#define MPU6050_FS_SEL_250 0x00
#define MPU6050_FS_SEL_500 0x01
#define MPU6050_FS_SEL_1000 0x02
#define MPU6050_FS_SEL_2000 0x03

#define MPU6050_XA_ST 0x01
#define MPU6050_YA_ST 0x01
#define MPU6050_ZA_ST 0x01
#define MPU6050_AFS_SEL_2 0x00
#define MPU6050_AFS_SEL_4 0x01
#define MPU6050_AFS_SEL_8 0x02
#define MPU6050_AFS_SEL_16 0x03

#define MPU6050_I2C_MST_CLK_400 0x0D // 400 kHz
#define MPU6050_I2C_SLV0_DLY_EN 0x01

#define MPU6050_WAKEUP 0x00
#define MPU6050_CLKSEL_PLLX 0x01

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250 ((float)131)
#define MPU6050_GYRO_SENS_500 ((float)65.5)
#define MPU6050_GYRO_SENS_1000 ((float)32.8)
#define MPU6050_GYRO_SENS_2000 ((float)16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2 ((float)16384)
#define MPU6050_ACCE_SENS_4 ((float)8192)
#define MPU6050_ACCE_SENS_8 ((float)4096)
#define MPU6050_ACCE_SENS_16 ((float)2048)

/* Default I2C address */
#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_ADDR 0x3C
/* HMC5883L registers */
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_DATAX_H 0x03
#define HMC5883L_DATAX_L 0x04
#define HMC5883L_DATAZ_H 0x05
#define HMC5883L_DATAZ_L 0x06
#define HMC5883L_DATAY_H 0x07
#define HMC5883L_DATAY_L 0x08
#define HMC5883L_STATUS 0x09
#define HMC5883L_ID_A 0x0A
#define HMC5883L_ID_B 0x0B
#define HMC5883L_ID_C 0x0C

#define HMC5883L_AVERAGING_1 0x00
#define HMC5883L_AVERAGING_2 0x01
#define HMC5883L_AVERAGING_4 0x02
#define HMC5883L_AVERAGING_8 0x03

#define HMC5883L_RATE_0P75 0x00
#define HMC5883L_RATE_1P5 0x01
#define HMC5883L_RATE_3 0x02
#define HMC5883L_RATE_7P5 0x03
#define HMC5883L_RATE_15 0x04
#define HMC5883L_RATE_30 0x05
#define HMC5883L_RATE_75 0x06

#define HMC5883L_BIAS_NORMAL 0x00
#define HMC5883L_BIAS_POSITIVE 0x01
#define HMC5883L_BIAS_NEGATIVE 0x02

#define HMC5883L_SEL_0P88 0x00
#define HMC5883L_SEL_1P3 0x01
#define HMC5883L_SEL_1P9 0x02
#define HMC5883L_SEL_2P5 0x03
#define HMC5883L_SEL_4P0 0x04
#define HMC5883L_SEL_4P7 0x05
#define HMC5883L_SEL_5P6 0x06
#define HMC5883L_SEL_8P1 0x07

#define HMC5883L_MODE_CONTINUOUS 0x00
#define HMC5883L_MODE_SINGLE 0x01
#define HMC5883L_MODE_IDLE 0x02

#define HMC5883L_STATUS_LOCK_BIT 1
#define HMC5883L_STATUS_READY_BIT 0

#define HMC5883L_MAGN_SENS_0P88 ((float)1370)
#define HMC5883L_MAGN_SENS_1P3 ((float)1090)
#define HMC5883L_MAGN_SENS_1P9 ((float)820)
#define HMC5883L_MAGN_SENS_2P5 ((float)660)
#define HMC5883L_MAGN_SENS_4P0 ((float)440)
#define HMC5883L_MAGN_SENS_4P7 ((float)390)
#define HMC5883L_MAGN_SENS_5P6 ((float)330)
#define HMC5883L_MAGN_SENS_8P1 ((float)230)
/* Default I2C address */
#define MS5611_ADDRESS 0x77
#define MS5611_ADDR 0xEE

#define MS5611_OK 0x01
#define MS5611_ERROR 0x00

// COMMAND
#define MS5611_CMD_REST 0X1E
#define MS5611_CMD_CONVERT_D1_256 0X40
#define MS5611_CMD_CONVERT_D1_512 0X42
#define MS5611_CMD_CONVERT_D1_1024 0X44
#define MS5611_CMD_CONVERT_D1_2048 0X46
#define MS5611_CMD_CONVERT_D1_4096 0X48
#define MS5611_CMD_CONVERT_D2_256 0X50
#define MS5611_CMD_CONVERT_D2_512 0X52
#define MS5611_CMD_CONVERT_D2_1024 0X54
#define MS5611_CMD_CONVERT_D2_2048 0X56
#define MS5611_CMD_CONVERT_D2_4096 0X58

#define MS6511_ADC_READ 0X00

#define MS5611_PROM_READ_0 0XA0
#define MS5611_PROM_READ_1 0XA2
#define MS5611_PROM_READ_2 0XA4
#define MS5611_PROM_READ_3 0XA6
#define MS5611_PROM_READ_4 0XA8
#define MS5611_PROM_READ_5 0XAA
#define MS5611_PROM_READ_6 0XAC
#define MS5611_PROM_READ_7 0XAE

/* Private typedef -----------------------------------------------------------*/
// MPU6050 structure
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    int16_t Magn_X_RAW;
    int16_t Magn_Y_RAW;
    int16_t Magn_Z_RAW;
    float Mx;
    float My;
    float Mz;

    int16_t Temp_RAW;
    float Temperature;

} MPU6050_t;
// MS5611 structure
typedef struct
{
    uint16_t C[6]; // PROM
    uint16_t reserve;
    uint16_t crc;
    uint32_t D[2]; // D1 temperature data & D2 pressure data
    int32_t dT;    // Difference between actual and reference temperature
    int64_t OFF;   // Offset at actual temperature
    int64_t SENS;  // Sensitivity at actual temperature
    int32_t TEMP;  // Actual temperature
    int32_t P;     // Actual pressure
} MS5611_t;
// Euler angle
typedef struct
{
	float roll;
	float pitch;
	float yaw;
} Euler_t;

/* Private function prototypes -----------------------------------------------*/
// MPU Function
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx);
void MPU6050_Master(I2C_HandleTypeDef *I2Cx);
void MPU6050_Addslave(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
// HMC Function
void HMC5883L_Init(I2C_HandleTypeDef *I2Cx);
// MS Function
void MS5611_Rest(I2C_HandleTypeDef *I2Cx);
uint8_t MS5611_PROM_read(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct);
uint8_t MS5611_init(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct);
uint8_t MS5611_read_temp(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct);
uint8_t MS5611_read_press(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct);
uint8_t MS5611_calculate(I2C_HandleTypeDef *I2Cx, MS5611_t *datastruct);
double getAltitude(double pressure, double referencePressure);
void MPU2Angle(MPU6050_t DataStruct, Euler_t *Angle);
void OffsetAngle(Euler_t *Angle, Euler_t Offset);

#endif // Kết thúc phần định nghĩa
