/**
 * @file td_lowpass.h
 * @brief Header file for lowpass filter functions.
 */
#ifndef TD_LOWPASS_H // Kiểm tra xem FILE_NAME_H đã được định nghĩa chưa
#define TD_LOWPASS_H // Nếu chưa thì định nghĩa nó

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include <math.h>
#include "TD_GY86.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
// Lowpass filter
typedef struct
{
	float out;
	float ePow;
} Lowpass_t;
/* Private function prototypes -----------------------------------------------*/
// Lowpass Filter Function
void Lowpass(float in,Lowpass_t *filter, float iCutOffFrequency);
void Lowpass4MPU(MPU6050_t input,MPU6050_t *output);

#endif // Kết thúc phần định nghĩa
