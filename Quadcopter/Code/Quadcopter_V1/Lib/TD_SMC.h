/**
 * @file td_smc.h
 * @brief Header file for lowpass filter functions.
 *  * * @author Tan Dat
 * @date 2023-6
 */
#ifndef TD_SMC_H // Kiểm tra xem FILE_NAME_H đã được định nghĩa chưa
#define TD_SMC_H // Nếu chưa thì định nghĩa nó

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include "math.h"
/* Private define ------------------------------------------------------------*/
// khai báo hằng số

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	double x;
	double y;
	double z;
}Vector_t;
/* Private function prototypes -----------------------------------------------*/
//1. SMC
void sliding_control_z(double roll, double pitch,
						double z_2dot_SP, double z_dot_SP, double z_SP,
						double a0, double K, double z, double z_dot, double *Uz);
//3.tính góc quay mong muốn theo Ux Uy yaw
void calculate_desired_angle(double Ux, double Uy, double yaw, double *roll_SP, double *pitch_SP);
#endif // Kết thúc phần định nghĩa
