/**
 * @file td_lowpass.h
 * @brief Header file for lowpass filter functions.
 * @author Tan Dat
 * @date 2023-6
 */
#ifndef TD_LOWPASS_H
#define TD_LOWPASS_H

/* Includes ------------------------------------------------------------------*/
#include "define.h"
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
double Average_filter(double value);
double Sum_array(double arr[], uint8_t n);
double Average_array(double arr[], uint8_t n);
#endif
