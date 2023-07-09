/**
 * Initializes the Electronic Speed Controller (ESC) for brushless motors.
 *
 * @param HTIMx The timer handle for the ESC.
 *
 * @returns None
 */
#include <TD_Brushless.h>

/**
 * Sets up the ESC (Electronic Speed Control) using the given timer handle.
 *
 * @param HTIMx The timer handle to use for setting up the ESC.
 *
 * @returns None
 */
//1. Setup ESC function
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
/**
 * Sets the duty cycle of the ESCs connected to the specified timer.
 *
 * @param HTIMx Pointer to the timer handle.
 * @param Duty1 The duty cycle of ESC 1.
 * @param Duty2 The duty cycle of ESC 2.
 * @param Duty3 The duty cycle of ESC 3.
 * @param Duty4 The duty cycle of ESC 4.
 *
 * @returns None
 */
//2. ESC control function
void ESC_Control(TIM_HandleTypeDef *HTIMx, uint16_t Duty1,uint16_t Duty2,uint16_t Duty3,uint16_t Duty4)
{
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_1, Duty1); // 1000 to 2000
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_2, Duty2); // 1000 to 2000
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_3, Duty3); // 1000 to 2000
	__HAL_TIM_SET_COMPARE(HTIMx, TIM_CHANNEL_4, Duty4); // 1000 to 2000
}
