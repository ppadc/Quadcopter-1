/**
 * * @file TD_DEFINE_H.
 * @brief Header file for the STM32F407VGT6.
 * Defines constants used in the project, such as conversion factors and time intervals.
 * 
* @author Tan Dat
 * @date 2023-6
 */
#ifndef TD_DEFINE_H 
#define TD_DEFINE_H 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
/* Private define ------------------------------------------------------------*/
#define SIMPLE_TIME 0.01           //SIMPLE_TIME: The time interval for the control loop.
#define DELTA_T 0.01
#define RAD_TO_DEG 57.2957795130    // RAD_TO_DEG: The conversion factor from radians to degrees.
#define DEG_TO_RAD 0.01745329252    // DEG_TO_RAD: The conversion factor from degrees to radians.
#endif 
