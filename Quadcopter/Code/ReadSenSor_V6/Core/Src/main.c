/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "code.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int enable = 0;
//Sensor
MPU6050_t MPU6050;
MPU6050_t MPU6050_LP;
MS5611_t MS5611;
//Lowpass
Lowpass_t LP_height = {0};
//Quaternions
Quaternion_t Q_est = {1,0,0,0};
//Brushless
Motor_t M1 = {
	.Duty = 0,
	.Duty_base = 15,
	.Duty_min = 955,
	.Duty_max = 1400,
};
Motor_t M2 = {
	.Duty = 0,
	.Duty_base = 15,
	.Duty_min = 955,
	.Duty_max = 1400,
};
Motor_t M3 = {
	.Duty = 0,
	.Duty_base = 15,
	.Duty_min = 955,
	.Duty_max = 1400,
};
Motor_t M4 = {
	.Duty = 0,
	.Duty_base = 15,
	.Duty_min = 955,
	.Duty_max = 1400,
};
//PID
PID_t PID_roll = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.e = 0,
	.e_pre = 0,
	.v_e = 0,
	.e_i = 0,
	.range = 0,
	.Upper_limit = 100,
	.Lower_limit = -100,
};
PID_t PID_pitch = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.e = 0,
	.e_pre = 0,
	.v_e = 0,
	.e_i = 0,
	.range = 1,
	.Upper_limit = 100,
	.Lower_limit = -100,
};
PID_t PID_yaw = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.e = 0,
	.e_pre = 0,
	.v_e = 0,
	.e_i = 0,
	.range = 0,
	.Upper_limit = 100,
	.Lower_limit = -100,
};
PID_t PID_height = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.e = 0,
	.e_pre = 0,
	.v_e = 0,
	.e_i = 0,
	.range = 0,
	.Upper_limit = 100,
	.Lower_limit = -100,
};
//Angle
Euler_t angle = {
	.roll 	= 0,
	.pitch 	= 0,
	.yaw 		= 0,
};
Euler_t Madgwick = {
	.roll 	= 0,
	.pitch 	= 0,
	.yaw 		= 0,
};
Euler_t Setpoint = {
	.roll 	= 0,
	.pitch	= 0,
	.yaw 		= 0,
};
Euler_t Offfset = {
	.roll 	= 0,
	.pitch 	= 0,
	.yaw	 	= 0,
};
//Height
double height;
double SetPoint_height = 0;
double Offset_height = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	while (MPU6050_Init(&hi2c1) == 1);
	MPU6050_Bypass(&hi2c1);
	HMC5883L_Init(&hi2c1);
	MPU6050_Master(&hi2c1);
	MPU6050_Addslave(&hi2c1);
	MS5611_init(&hi2c1,&MS5611);
	setupESC(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance)
	{
		// Read Data
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		MPU6050_Read_All(&hi2c1, &MPU6050);
		MS5611_calculate(&hi2c1, &MS5611);
		//Filter
		Lowpass4MPU(MPU6050,&MPU6050_LP);
		Lowpass(MS5611.P,&LP_height,0.3);
		MPU2Angle(MPU6050_LP,&angle);
		Madgwick_update(&MPU6050_LP, &Q_est);
		//Caculator
		Quat2Angle(Q_est, &Madgwick);
		height = getAltitude(LP_height.out,101325); 
//		//Control
//		float roll  =	PID_Control(Setpoint.roll,	Madgwick.roll,	&PID_roll); 	// 0 to 1000
//		float pitch = PID_Control(Setpoint.pitch,	Madgwick.pitch,	&PID_pitch);	// 0 to 1000
//		float yaw   = PID_Control(Setpoint.yaw,		Madgwick.yaw,		&PID_yaw);		// 0 to 1000
//		float Z     = PID_Control(SetPoint_height,			height,					&PID_height);	// 0 to 1000
//		
//		Combining_Angle_height(&M1, &M2, &M3, &M4, roll, pitch, yaw, Z);//Duty = Duty_min + Duty_PID (Duty_min to Duty_max)
//		// Enable output PID
		if(enable)
		{
			ESC_Control(&htim1, M1.Duty+M1.Duty_base, M2.Duty_min, M3.Duty+M3.Duty_base, M4.Duty_min); // Duty + Duty_base
		}
		else
		{
			ESC_Control(&htim1, M1.Duty_min, M2.Duty_min, M3.Duty_min, M4.Duty_min); // Duty_min
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
