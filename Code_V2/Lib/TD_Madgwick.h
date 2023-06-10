/**
 * Header file for TD_LQR library.
 *
 * This library provides functions for computing the quaternion multiplication, scalar multiplication, addition, subtraction, conjugate, normalization, and conversion to Euler angles. It also includes functions for implementing the Madgwick algorithm for IMU sensor fusion.
 *
 * @file TD_Madgwick.h
 */
#ifndef TD_Madgwick_H // Kiểm tra xem FILE_NAME_H đã được định nghĩa chưa
#define TD_Madgwick_H // Nếu chưa thì định nghĩa nó

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include "TD_GY86.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define GYRO_MEAN_ERROR PI * (5.0f / 180.0f)	 // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#define BETA 0.041//sqrt(3.0f / 4.0f) * GYRO_MEAN_ERROR //*from paper* 0.041
#define ZETA 0.015
/* Private typedef -----------------------------------------------------------*/
// Quaternion structure
typedef struct
{
	float q1;
	float q2;
	float q3;
	float q4;
} Quaternion_t;

/* Private function prototypes -----------------------------------------------*/
// Quaternions
Quaternion_t quat_mult(Quaternion_t L, Quaternion_t R);
void quat_scalar(Quaternion_t *q, float scalar);
void quat_add(Quaternion_t *Sum, Quaternion_t L, Quaternion_t R);
void quat_sub(Quaternion_t *Sum, Quaternion_t L, Quaternion_t R);
Quaternion_t quat_conjugate(Quaternion_t q);
float quat_Norm(Quaternion_t q);
void quat_Normalization(Quaternion_t *q);
// Madwick Function
void Madgwick_imu(MPU6050_t *DataStruct, Quaternion_t *q_est);
void Madgwick_update(MPU6050_t *DataStruct, Quaternion_t *q_est);
void Quat2Angle(Quaternion_t q, Euler_t *Angle);
#endif // Kết thúc phần định nghĩa
