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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// MPU6050 structure
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
		
		int16_t Magn_X_RAW;
    int16_t Magn_Y_RAW;
    int16_t Magn_Z_RAW;
    double Mx;
    double My;
    double Mz;
    float Temperature;
		float heading;
} MPU6050_t;
// Quaternion structure
typedef struct {
	  double q1;
    double q2;
    double q3;
    double q4;
} Quaternion_t;
// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
	
} Kalman_t;
// Euler angle
typedef struct {
		double roll;
		double pitch;
		double yaw;
} Euler_t;	

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI													3.14159265358979
#define RAD_TO_DEG 									57.295779513082320876798154814105
#define DEG_TO_RAD									0.01745329252
#define SIMPLE_TIME									0.002 // 2ms
#define DELTA_T											SIMPLE_TIME // 2ms
#define GYRO_MEAN_ERROR 						PI * (5.0f / 180.0f) // 5 deg/s gyroscope measurement error (in rad/s)  *from paper*
#define BETA 												sqrt(3.0f/4.0f) * GYRO_MEAN_ERROR    //*from paper* 0.041 
#define ZETA												0.015
/* Default I2C address */
#define MPU6050_ADDRESS							0x68
#define MPU6050_ADDR								0xD0

/* Who I am register value */
#define MPU6050_I_AM								0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO						0x01
#define	MPU6050_SELF_TEST_X 				0X0D
#define	MPU6050_SELF_TEST_Y 				0X0E
#define	MPU6050_SELF_TEST_Z 				0X0F
#define	MPU6050_SELF_TEST_A 				0X10
#define MPU6050_SMPLRT_DIV					0x19
#define MPU6050_CONFIG							0x1A
#define MPU6050_GYRO_CONFIG					0x1B
#define MPU6050_ACCEL_CONFIG				0x1C
#define MPU6050_MOTION_THRESH				0x1F
#define	MPU6050_FIFO_EN			 				0X23
#define	MPU6050_I2C_MST_CTRL				0X24
#define	MPU6050_I2C_SLV0_ADDR				0X25
#define	MPU6050_I2C_SLV0_REG				0X26
#define	MPU6050_I2C_SLV0_CTRL				0X27
#define MPU6050_INT_PIN_CFG					0x37
#define MPU6050_INT_ENABLE					0x38
#define MPU6050_INT_STATUS					0x3A
#define MPU6050_ACCEL_XOUT_H				0x3B
#define MPU6050_ACCEL_XOUT_L				0x3C
#define MPU6050_ACCEL_YOUT_H				0x3D
#define MPU6050_ACCEL_YOUT_L				0x3E
#define MPU6050_ACCEL_ZOUT_H				0x3F
#define MPU6050_ACCEL_ZOUT_L				0x40
#define MPU6050_TEMP_OUT_H					0x41
#define MPU6050_TEMP_OUT_L					0x42
#define MPU6050_GYRO_XOUT_H					0x43
#define MPU6050_GYRO_XOUT_L					0x44
#define MPU6050_GYRO_YOUT_H					0x45
#define MPU6050_GYRO_YOUT_L					0x46
#define MPU6050_GYRO_ZOUT_H					0x47
#define MPU6050_GYRO_ZOUT_L					0x48
#define MPU6050_MOT_DETECT_STATUS		0x61
#define MPU6050_I2C_MST_DELAY_CTRL	0x67
#define MPU6050_SIGNAL_PATH_RESET		0x68
#define MPU6050_MOT_DETECT_CTRL			0x69
#define MPU6050_USER_CTRL						0x6A
#define MPU6050_PWR_MGMT_1					0x6B
#define MPU6050_PWR_MGMT_2					0x6C
#define MPU6050_FIFO_COUNTH					0x72
#define MPU6050_FIFO_COUNTL					0x73
#define MPU6050_FIFO_R_W						0x74
#define MPU6050_WHO_AM_I						0x75

#define MPU6050_SMPLRT_DIV_1 				0x00
#define MPU6050_EXT_SYNC_SET				0x00 //Input disabled
#define MPU6050_DLPF_CFG						0x01 // acc {184Hz-2.0ms} gyro {188Hz-1.9ms} Fs 1kHz

#define MPU6050_XG_ST								0x01
#define MPU6050_YG_ST								0x01
#define MPU6050_ZG_ST								0x01
#define MPU6050_FS_SEL_250					0x00
#define MPU6050_FS_SEL_500					0x01
#define MPU6050_FS_SEL_1000					0x02
#define MPU6050_FS_SEL_2000					0x03

#define MPU6050_XA_ST								0x01
#define MPU6050_YA_ST								0x01
#define MPU6050_ZA_ST								0x01
#define MPU6050_AFS_SEL_2						0x00
#define MPU6050_AFS_SEL_4						0x01
#define MPU6050_AFS_SEL_8						0x02
#define MPU6050_AFS_SEL_16					0x03

#define MPU6050_I2C_MST_CLK_400			0x0D  //400 kHz
#define MPU6050_I2C_SLV0_DLY_EN			0x01

#define MPU6050_WAKEUP							0x00
#define MPU6050_CLKSEL_PLLX					0x01

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250				((float) 131)
#define MPU6050_GYRO_SENS_500				((float) 65.5)
#define MPU6050_GYRO_SENS_1000			((float) 32.8)
#define MPU6050_GYRO_SENS_2000			((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2					((float) 16384)
#define MPU6050_ACCE_SENS_4					((float) 8192)
#define MPU6050_ACCE_SENS_8					((float) 4096)
#define MPU6050_ACCE_SENS_16				((float) 2048)
	
/* Default I2C address */
#define HMC5883L_ADDRESS 						0x1E
#define HMC5883L_ADDR 							0x3C
/* HMC5883L registers */
#define HMC5883L_CONFIG_A       		0x00
#define HMC5883L_CONFIG_B        		0x01
#define HMC5883L_MODE            		0x02
#define HMC5883L_DATAX_H         		0x03
#define HMC5883L_DATAX_L        		0x04
#define HMC5883L_DATAZ_H        		0x05
#define HMC5883L_DATAZ_L         		0x06
#define HMC5883L_DATAY_H         		0x07
#define HMC5883L_DATAY_L         		0x08
#define HMC5883L_STATUS         		0x09
#define HMC5883L_ID_A            		0x0A
#define HMC5883L_ID_B            		0x0B
#define HMC5883L_ID_C            		0x0C

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_SEL_0P88          	0x00
#define HMC5883L_SEL_1P3          	0x01
#define HMC5883L_SEL_1P9          	0x02
#define HMC5883L_SEL_2P5           	0x03
#define HMC5883L_SEL_4P0           	0x04
#define HMC5883L_SEL_4P7           	0x05
#define HMC5883L_SEL_5P6           	0x06
#define HMC5883L_SEL_8P1           	0x07

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

#define HMC5883L_MAGN_SENS_0P88     ((float) 1370)
#define HMC5883L_MAGN_SENS_1P3      ((float) 1090)
#define HMC5883L_MAGN_SENS_1P9      ((float) 820)
#define HMC5883L_MAGN_SENS_2P5      ((float) 660)
#define HMC5883L_MAGN_SENS_4P0      ((float) 440)
#define HMC5883L_MAGN_SENS_4P7      ((float) 390)
#define HMC5883L_MAGN_SENS_5P6      ((float) 330)
#define HMC5883L_MAGN_SENS_8P1      ((float) 230)
/* Default I2C address */
#define MS5611_ADDRESS  						0x77
#define MS5611_ADDR  								0xEE


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
uint32_t timer;
MPU6050_t MPU6050;
Kalman_t Kalman = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,	
};
Euler_t Madgwick = {0};
double b2 ;
double b4 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void scanner(void);
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx);
void MPU6050_Master(I2C_HandleTypeDef *I2Cx);
void MPU6050_Addslave(I2C_HandleTypeDef *I2Cx);
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
void HMC5883L_Init(I2C_HandleTypeDef *I2Cx);
Quaternion_t quat_mult (Quaternion_t L, Quaternion_t R);
void quat_scalar(Quaternion_t * q, float scalar);
void quat_add(Quaternion_t * Sum, Quaternion_t L, Quaternion_t R);
void quat_sub(Quaternion_t * Sum, Quaternion_t L, Quaternion_t R);
Quaternion_t quat_conjugate(Quaternion_t q);
double quat_Norm (Quaternion_t q);
void quat_Normalization(Quaternion_t * q);
void Madgwick_imu(MPU6050_t *DataStruct);
void eulerAngles(Quaternion_t q, Euler_t *Angle);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	scanner();
	while (MPU6050_Init(&hi2c1) == 1);
	MPU6050_Bypass(&hi2c1);
	scanner();
	HMC5883L_Init(&hi2c1);
	MPU6050_Master(&hi2c1);
	MPU6050_Addslave(&hi2c1);
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 72000000*SIMPLE_TIME;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void scanner(void)
{
uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";
uint8_t i = 0, ret;

/*-[ I2C Bus Scanning ]-*/
		HAL_UART_Transmit(&huart6, StartMSG, sizeof(StartMSG), 100);
		for(i=1; i<128; i++)
		{
				ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
				if (ret != HAL_OK) /* No ACK Received At That Address */
				{
						HAL_UART_Transmit(&huart6, Space, sizeof(Space), 100);
				}
				else if(ret == HAL_OK)
				{
						sprintf(Buffer, "0x%X", i);
						HAL_UART_Transmit(&huart6, Buffer, sizeof(Buffer), 100);
				}
		}
		HAL_UART_Transmit(&huart6, EndMSG, sizeof(EndMSG), 100);
		/*--[ Scanning Done ]--*/
}
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, i2c_timeout);
    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up, gyroscope based clock
        Data = MPU6050_WAKEUP << 6 | MPU6050_CLKSEL_PLLX;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = MPU6050_SMPLRT_DIV_1;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &Data, 1, i2c_timeout);      
			
				//This register configures the external Frame Synchronization (FSYNC) pin sampling
				//the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers
				Data = MPU6050_EXT_SYNC_SET << 3 | MPU6050_DLPF_CFG;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_CONFIG, 1, &Data, 1, i2c_timeout);
				
				// Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 degree/s
        Data = MPU6050_FS_SEL_250 << 3 | 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &Data, 1, i2c_timeout);
				
				// Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
        Data = MPU6050_AFS_SEL_2 << 3 | 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &Data, 1, i2c_timeout);
				
				//sets the I2C master clock speed  
				Data = MPU6050_I2C_MST_CLK_400;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_MST_CTRL, 1, &Data, 1, i2c_timeout);
				return 0;
    }
    return 1;
}
void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx)
{
		uint8_t Data;
		// disable i2c master mode
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_USER_CTRL, 1, &Data, 1, i2c_timeout);
		// enable i2c master bypass mode
		Data = 0x02;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_INT_PIN_CFG, 1, &Data, 1, i2c_timeout);
}
void MPU6050_Master(I2C_HandleTypeDef *I2Cx)
{
		uint8_t Data;
		// disable i2c master bypass mode
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_INT_PIN_CFG, 1, &Data, 1, i2c_timeout);
		// enable i2c master mode
		Data = 0x20;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_USER_CTRL, 1, &Data, 1, i2c_timeout);
}
// configure the MPU6050 to automatically read the magnetometer
void MPU6050_Addslave(I2C_HandleTypeDef *I2Cx) 
{
		uint8_t Data;
		// slave 0 i2c address, read mode
		Data = HMC5883L_ADDRESS | 0x80;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_SLV0_ADDR, 1, &Data, 1, i2c_timeout);
		// slave 0 register = 0x03 (x axis)
		Data = HMC5883L_DATAX_H ;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_SLV0_REG, 1, &Data, 1, i2c_timeout);
		// slave 0 transfer size = 6, enabled
		Data =  0x06 | 0x80;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_SLV0_CTRL, 1, &Data, 1, i2c_timeout);
		// enable slave 0 delay
		Data =  MPU6050_I2C_SLV0_DLY_EN;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_I2C_MST_DELAY_CTRL, 1, &Data, 1, i2c_timeout);
}
void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[20];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 20, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
	
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
	
    DataStruct->Magn_X_RAW = (int16_t)(Rec_Data[14] << 8 | Rec_Data[15]);
    DataStruct->Magn_Z_RAW = (int16_t)(Rec_Data[16] << 8 | Rec_Data[17]);
    DataStruct->Magn_Y_RAW = (int16_t)(Rec_Data[18] << 8 | Rec_Data[19]);
	
    DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_ACCE_SENS_2;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_ACCE_SENS_2;
    DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_ACCE_SENS_2;
		
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
		
    DataStruct->Gx = DataStruct->Gyro_X_RAW / MPU6050_GYRO_SENS_250 * DEG_TO_RAD;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / MPU6050_GYRO_SENS_250 * DEG_TO_RAD;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / MPU6050_GYRO_SENS_250 * DEG_TO_RAD;
		
		DataStruct->Mx = DataStruct->Magn_X_RAW / HMC5883L_MAGN_SENS_1P3;
    DataStruct->My = DataStruct->Magn_Y_RAW / HMC5883L_MAGN_SENS_1P3;
    DataStruct->Mz = DataStruct->Magn_Z_RAW / HMC5883L_MAGN_SENS_1P3;
		
//		// To calculate heading in degrees. 0 degree indicates North
//		DataStruct->heading = atan2(DataStruct->My,DataStruct->Mx) * RAD_TO_DEG;
//		if(DataStruct->heading < 0)
//    DataStruct->heading += 360;
}
void HMC5883L_Init(I2C_HandleTypeDef *I2Cx)
{
		uint8_t Data;
		// write CONFIG_A register
		Data = HMC5883L_AVERAGING_1 << 5 | HMC5883L_RATE_75 << 2 | HMC5883L_BIAS_NORMAL;
		HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_CONFIG_A, 1, &Data, 1, i2c_timeout);
		// write CONFIG_B register
		Data = HMC5883L_SEL_1P3 << 5 | 0x00 ;
		HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_CONFIG_B, 1, &Data, 1, i2c_timeout);
		// write MODE register
		Data = HMC5883L_MODE_CONTINUOUS ;
		HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDR, HMC5883L_MODE, 1, &Data, 1, i2c_timeout);
}

// Multiply two quaternions and return a copy of the result, prod = L * R
Quaternion_t quat_mult (Quaternion_t L, Quaternion_t R)
{
		Quaternion_t product;
		product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);
    
    return product;
}

// Multiply a reference of a quaternion by a scalar, q = s*q
void quat_scalar(Quaternion_t * q, float scalar)
{
    q->q1 *= scalar;
    q->q2 *= scalar;
    q->q3 *= scalar;
    q->q4 *= scalar;
}

// Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
void quat_add(Quaternion_t * Sum, Quaternion_t L, Quaternion_t R)
{
    Sum -> q1 = L.q1 + R.q1;
    Sum -> q2 = L.q2 + R.q2;
    Sum -> q3 = L.q3 + R.q3;
    Sum -> q4 = L.q4 + R.q4;
}
// Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
void quat_sub(Quaternion_t * Sum, Quaternion_t L, Quaternion_t R)
{
    Sum -> q1 = L.q1 - R.q1;
    Sum -> q2 = L.q2 - R.q2;
    Sum -> q3 = L.q3 - R.q3;
    Sum -> q4 = L.q4 - R.q4;
}
// the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
Quaternion_t quat_conjugate(Quaternion_t q)
{
    q.q2 = -q.q2;
    q.q3 = -q.q3;
    q.q4 = -q.q4;
    return q;
}

// norm of a quaternion is the same as a complex number
double quat_Norm (Quaternion_t q)
{
    return sqrt(q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3 +q.q4*q.q4);
}

// Normalizes pointer q by calling quat_Norm(q),
void quat_Normalization(Quaternion_t * q)
{
    float norm = quat_Norm(*q);
    q -> q1 /= norm;
    q -> q2 /= norm;
    q -> q3 /= norm;
    q -> q4 /= norm;
}
void Madgwick_update(MPU6050_t *DataStruct)
{		
		static Quaternion_t q_est 	= {1,0,0,0};
		double ax = DataStruct->Ax;
    double ay = DataStruct->Ay;
		double az = DataStruct->Az;
		double gx = DataStruct->Gx;
		double gy = DataStruct->Gy;
		double gz = DataStruct->Gz;
		double mx = DataStruct->Mx;
		double my = DataStruct->My;
		double mz = DataStruct->Mz;		
    //Variables and constants
		double F_g [3] = {0};                      // eq(15/21/25) objective function for gravity
    double J_g [3][4] = {0};                   // jacobian matrix for gravity   
		double F_b [3] = {0};                      // eq(15/21/29) objective function for gravity
    double J_b [3][4] = {0};                   // jacobian matrix for magnetic   
		static Quaternion_t q_w_gradient_integral = {0};
    Quaternion_t q_est_prev = q_est;
    Quaternion_t q_est_dot = {0};            	// eq 42 and 43
    Quaternion_t gradient = {0};
		
    Quaternion_t q_a = {0, ax, ay, az};    		// eq (24) raw acceleration values, needs to be normalized
    if (quat_Norm(q_a) == 0) return;
		quat_Normalization(&q_a);              		// normalize the acceleration quaternion to be a unit quaternion
		//const Quaternion_t q_g_ref = {0, 0, 0, 1};// eq (23) not needed because I used eq 25 instead of eq 21
		
		Quaternion_t q_m = {0, mx, my, mz};    		// eq (28) raw magnetic values, needs to be normalized
    if (quat_Norm(q_m) == 0)
		{
			Madgwick_imu(&MPU6050);
			return;
		}
		quat_Normalization(&q_m);    							// normalize the magnetic quaternion to be a unit quaternion
		//const Quaternion_t q_b_ref = {0, 0.99, 0, -0.13};
		// Magnetic distortion compensation
		Quaternion_t h = quat_mult(q_est_prev,quat_mult(q_m,quat_conjugate(q_est_prev))); // eq(45) (Group 1)
		Quaternion_t b = {0, sqrt(h.q2*h.q2 + h.q3*h.q3), 0, h.q4};  // {0, 0.99, 0, -0.13} // eq(46)
	
    Quaternion_t q_w = {0, gx, gy, gz};  			// eq (10), places gyroscope readings in a quaternion                         
			  
    //Compute the objective function for gravity, simplified to equation (25) due to the 0's in the acceleration reference quaternion
    F_g[0] = 2*(q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
    F_g[1] = 2*(q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3* q_est_prev.q4) - q_a.q3;
    F_g[2] = 2*(0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;
    
    //Compute the Jacobian matrix, equation (26), for gravity
    J_g[0][0] = -2 * q_est_prev.q3;
    J_g[0][1] =  2 * q_est_prev.q4;
    J_g[0][2] = -2 * q_est_prev.q1;
    J_g[0][3] =  2 * q_est_prev.q2;
    
    J_g[1][0] = 2 * q_est_prev.q2;
    J_g[1][1] = 2 * q_est_prev.q1;
    J_g[1][2] = 2 * q_est_prev.q4;
    J_g[1][3] = 2 * q_est_prev.q3;
    
    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q2;
    J_g[2][2] = -4 * q_est_prev.q3;
    J_g[2][3] = 0;
    
		// Compute the objective function for magnetic, simplified to equation (29) due to the 0's in the magnetic reference quaternion
		F_b[0] = 2*b.q2*(0.5 - q_est_prev.q3*q_est_prev.q3 - q_est_prev.q4*q_est_prev.q4) + 2*b.q4*(q_est_prev.q2*q_est_prev.q4 - q_est_prev.q1*q_est_prev.q3) - q_m.q2;
    F_b[1] = 2*b.q2*(q_est_prev.q2*q_est_prev.q3 - q_est_prev.q1*q_est_prev.q4) + 2*b.q4*(q_est_prev.q1*q_est_prev.q2 + q_est_prev.q3*q_est_prev.q4) - q_m.q3;
    F_b[2] = 2*b.q2*(q_est_prev.q1*q_est_prev.q3 + q_est_prev.q2*q_est_prev.q4) + 2*b.q4*(0.5 - q_est_prev.q2*q_est_prev.q2 - q_est_prev.q3*q_est_prev.q3) - q_m.q4;
		
		//Compute the Jacobian matrix, equation (26), for magnetic
		
		J_b[0][0] = -2 * b.q4 * q_est_prev.q3;
    J_b[0][1] =  2 * b.q4 * q_est_prev.q4;
    J_b[0][2] = -4 * b.q2 * q_est_prev.q3 - 2 * b.q4 * q_est_prev.q1;
    J_b[0][3] = -4 * b.q2 * q_est_prev.q4 + 2 * b.q4 * q_est_prev.q2;
    
    J_b[1][0] = -2 * b.q2 * q_est_prev.q4 + 2 * b.q4 * q_est_prev.q2;
    J_b[1][1] =  2 * b.q2 * q_est_prev.q3 + 2 * b.q4 * q_est_prev.q1;
    J_b[1][2] =  2 * b.q2 * q_est_prev.q2 + 2 * b.q4 * q_est_prev.q4;
    J_b[1][3] = -2 * b.q2 * q_est_prev.q1 + 2 * b.q4 * q_est_prev.q3;
    
    J_b[2][0] =  2 * b.q2 * q_est_prev.q3;
    J_b[2][1] =  2 * b.q2 * q_est_prev.q4 - 4 * b.q4 * q_est_prev.q2;
    J_b[2][2] =  2 * b.q2 * q_est_prev.q1 - 4 * b.q4 * q_est_prev.q3;
    J_b[2][3] =  2 * b.q2 * q_est_prev.q2;

    // now computer the gradient, equation (20), gradient = J_g'*F_g + J_b'*F_b
    gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2] + J_b[0][0] * F_b[0] + J_b[1][0] * F_b[1] + J_b[2][0] * F_b[2] ;
    gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2] + J_b[0][1] * F_b[0] + J_b[1][1] * F_b[1] + J_b[2][1] * F_b[2];
    gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2] + J_b[0][2] * F_b[0] + J_b[1][2] * F_b[1] + J_b[2][2] * F_b[2];
    gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2] + J_b[0][3] * F_b[0] + J_b[1][3] * F_b[1] + J_b[2][3] * F_b[2];
    
    // Normalize the gradient, equation (44)
    quat_Normalization(&gradient);
		
		// Gyroscope bias drift compensation (Group 2)
		Quaternion_t q_w_gradient = quat_mult(quat_conjugate(q_est_prev),gradient);
		quat_scalar(&q_w_gradient,2*DELTA_T);
		quat_add(&q_w_gradient_integral, q_w_gradient_integral, q_w_gradient);
		quat_scalar(&q_w_gradient_integral,ZETA);
		quat_sub(&q_w, q_w, q_w_gradient_integral);
		//Orientation from angular rate
		quat_scalar(&q_w, 0.5);                 	// equation (12) dq/dt = (1/2)q*w
    q_w = quat_mult(q_est_prev, q_w);        	// equation (12)
		// quat_scalar(&q_w, deltaT);             // eq (13) integrates the angles velocity to position
    // quat_add(&q_w, q_w, q_est_prev);       // addition part of equation (13)
  
		// Combining
    quat_scalar(&gradient, BETA);             // multiply normalized gradient by beta
    quat_sub(&q_est_dot, q_w, gradient);        // subtract above from q_w, the integrated gyro quaternion
    quat_scalar(&q_est_dot, DELTA_T);
    quat_add(&q_est, q_est_prev, q_est_dot);     // Integrate orientation rate to find position
    quat_Normalization(&q_est);                 // normalize the orientation of the estimate
                                                //(shown in diagram, plus always use unit quaternions for orientation)
		eulerAngles(q_est,&Madgwick);
}
void Madgwick_imu(MPU6050_t *DataStruct)
{		
		static Quaternion_t q_est 	= {1,0,0,0};
		double ax = DataStruct->Ax;
    double ay = DataStruct->Ay;
		double az = DataStruct->Az;
		double gx = DataStruct->Gx;
		double gy = DataStruct->Gy;
		double gz = DataStruct->Gz;
    //Variables and constants
		double F_g [3] = {0};                      // eq(15/21/25) objective function for gravity
    double J_g [3][4] = {0};                   // jacobian matrix for gravity   
    Quaternion_t q_est_prev = q_est;
    Quaternion_t q_est_dot = {0};            	// eq 42 and 43
    Quaternion_t gradient = {0};
    //const Quaternion_t q_g_ref = {0, 0, 0, 1};// eq (23) not needed because I used eq 25 instead of eq 21
		
    Quaternion_t q_a = {0, ax, ay, az};    		// eq (24) raw acceleration values, needs to be normalized
    if (quat_Norm(q_a) == 0) return;
		quat_Normalization(&q_a);              // normalize the acceleration quaternion to be a unit quaternion
		    
    Quaternion_t q_w = {0, gx, gy, gz};  			// equation (10), places gyroscope readings in a quaternion                         
    
    quat_scalar(&q_w, 0.5);                 	// equation (12) dq/dt = (1/2)q*w
    q_w = quat_mult(q_est_prev, q_w);        	// equation (12)
		
    // quat_scalar(&q_w, deltaT);             // eq (13) integrates the angles velocity to position
    // quat_add(&q_w, q_w, q_est_prev);       // addition part of equation (13)
    
    //Compute the objective function for gravity, equation(15), simplified to equation (25) due to the 0's in the acceleration reference quaternion
    F_g[0] = 2*(q_est_prev.q2 * q_est_prev.q4 - q_est_prev.q1 * q_est_prev.q3) - q_a.q2;
    F_g[1] = 2*(q_est_prev.q1 * q_est_prev.q2 + q_est_prev.q3* q_est_prev.q4) - q_a.q3;
    F_g[2] = 2*(0.5 - q_est_prev.q2 * q_est_prev.q2 - q_est_prev.q3 * q_est_prev.q3) - q_a.q4;
    
    //Compute the Jacobian matrix, equation (26), for gravity
    J_g[0][0] = -2 * q_est_prev.q3;
    J_g[0][1] =  2 * q_est_prev.q4;
    J_g[0][2] = -2 * q_est_prev.q1;
    J_g[0][3] =  2 * q_est_prev.q2;
    
    J_g[1][0] = 2 * q_est_prev.q2;
    J_g[1][1] = 2 * q_est_prev.q1;
    J_g[1][2] = 2 * q_est_prev.q4;
    J_g[1][3] = 2 * q_est_prev.q3;
    
    J_g[2][0] = 0;
    J_g[2][1] = -4 * q_est_prev.q2;
    J_g[2][2] = -4 * q_est_prev.q3;
    J_g[2][3] = 0;
    
    // now computer the gradient, equation (20), gradient = J_g'*F_g
    gradient.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    gradient.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    gradient.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    gradient.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];
    
    // Normalize the gradient, equation (44)
    quat_Normalization(&gradient);
		// Combining
    quat_scalar(&gradient, BETA);             // multiply normalized gradient by beta
    quat_sub(&q_est_dot, q_w, gradient);        // subtract above from q_w, the integrated gyro quaternion
    quat_scalar(&q_est_dot, DELTA_T);
    quat_add(&q_est, q_est_prev, q_est_dot);     // Integrate orientation rate to find position
    quat_Normalization(&q_est);                 // normalize the orientation of the estimate
                                                //(shown in diagram, plus always use unit quaternions for orientation)
		eulerAngles(q_est,&Madgwick);
}
void eulerAngles(Quaternion_t q, Euler_t *Angle)
{
    
    Angle->yaw 		= atan2f((2*q.q2*q.q3 - 2*q.q1*q.q4), (2*q.q1*q.q1 + 2*q.q2*q.q2 -1));  // equation (7)
    Angle->pitch 	= -asinf(2*q.q2*q.q4 + 2*q.q1*q.q3);                                  // equatino (8)
    Angle->roll  	= atan2f((2*q.q3*q.q4 - 2*q.q1*q.q2), (2*q.q1*q.q1 + 2*q.q4*q.q4 -1));
    
    Angle->yaw 		*= (180.0f / PI);
    Angle->pitch 	*= (180.0f / PI);
    Angle->roll 	*= (180.0f / PI);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
 {
   HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	 MPU6050_Read_All(&hi2c1,&MPU6050);
	 Madgwick_update(&MPU6050);
	 
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
