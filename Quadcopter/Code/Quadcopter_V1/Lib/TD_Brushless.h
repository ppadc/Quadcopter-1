/**
 * @file td_brushless.h
 * @brief Header file for controlling brushless motors using TIM.
 * @author Tan Dat
 * @date 2023-6
 */
#ifndef TD_BRUSHLESS_H 
#define TD_BRUSHLESS_H 

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include "tim.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
// Brushless
typedef struct
{
    float Duty;
    float Duty_base;
    float Duty_max;
    float Duty_min;
} Motor_t;
/* Private function prototypes -----------------------------------------------*/
// ESC
void setupESC(TIM_HandleTypeDef *HTIMx);
void ESC_Control(TIM_HandleTypeDef *HTIMx, uint16_t Duty1, uint16_t Duty2, uint16_t Duty3, uint16_t Duty4);

#endif 
