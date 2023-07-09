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
#include "define.h"
#include "TD_GY86.h"
#include "TD_Madgwick.h"
#include "TD_Brushless.h"
#include "TD_LQR.h"
#include "TD_RF24L01.h"
#include "TD_Lowpass.h"
#include "TD_SMC.h"

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

// Sensor
MPU6050_t MPU6050;
MPU6050_t MPU6050_LP;
// Angle
Euler_t angle;
Euler_t Madgwick;
Euler_t Madgwick_new;
Euler_t Setpoint;
Euler_t Setpoint_pre;
Euler_t Setpoint_dot;
Euler_t Offset = {
    .roll = 0.0129,
    .pitch = 0.0875,
    .yaw = 0,
};
// Quaternions
Quaternion_t Q_est = {1, 0, 0, 0};
// Brushless
Motor_t M1 = {
    .Duty = 0,
    .Duty_base = 1100,
    .Duty_min = 800,
    .Duty_max = 2000,
};
Motor_t M2 = {
    .Duty = 0,
    .Duty_base = 1100,
    .Duty_min = 800,
    .Duty_max = 2000,
};
Motor_t M3 = {
    .Duty = 0,
    .Duty_base = 1100,
    .Duty_min = 800,
    .Duty_max = 2000,
};
Motor_t M4 = {
    .Duty = 0,
    .Duty_base = 1100,
    .Duty_min = 800,
    .Duty_max = 2000,
};
// RF24
Data_t data_Tx;
Data_t data_Rx;
uint64_t TxpipeAddrs = 0xAABBCCDDEE;
uint64_t RxpipeAddrs = 0x1122334455;
// LQR
const float a = 6.3246;
const float b = 2.4522;
double x[6];
double K[4][6] = {{0, 0, 0, 0, 0, 0}, {a, 0, 0, b, 0, 0}, {0, a, 0, 0, b, 0}, {0, 0, 0, 0, 0, 0}};
double u[4];
double T1, T2, T3, T4;
// Enable Run
int enable = 0;
int state = 0;
int run = 0;
uint64_t time;
uint64_t time_pre;
double i_e_roll = 0;
double i_e_pitch = 0;
double Ki = 5;
double LimitI = 0.5;
double aa = 0;
double outI_roll = 0;
double outF_roll = 0;
double outI_pitch = 0;
double outF_pitch = 0;
double m = 0;
double gama = 1.4;
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
  /**
   * Sends data using the NRF24L01.
   *
   * @param huart6 The UART handle.
   * @param GPIOB The GPIO port used for the NRF24L01.
   * @param nrf_CSN_PIN The chip select pin for the NRF24L01.
   * @param nrf_CE_PIN The chip enable pin for the NRF24L01.
   * @param hspi1 The SPI handle.
   * @param TxpipeAddrs The address of the transmitting pipe.
   */
  // Begin setup Rf24
  NRF24_begin(GPIOB, nrf_CSN_PIN, nrf_CE_PIN, hspi1);
  NRF24_stopListening();
  NRF24_openWritingPipe(TxpipeAddrs);
  NRF24_setAutoAck(true);
  NRF24_enableDynamicPayloads();
  NRF24_enableAckPayload();
  /**
   * Initializes the MPU6050 and HMC5883L sensors, and the MS5611 barometer.
   *
   * @param hi2c1 The I2C handle for communication with the sensors.
   */
  // Begin setup GY86
  while (MPU6050_Init(&hi2c1) == 1);
  MPU6050_Bypass(&hi2c1);
  HMC5883L_Init(&hi2c1);
  MPU6050_Master(&hi2c1);
  MPU6050_Addslave(&hi2c1);
  /**
   * Sets up the ESC (Electronic Speed Controller) for a motor using the specified timer.
   *
   * @param htim1 The timer used for the ESC.
   */
  // Begin setup ESC
  setupESC(&htim1);
  /**
   * Starts the timer interrupt for TIM2.
   *
   * @param htim2 A pointer to the TIM2 handle.
   * Simple time is 2ms
   */
  // RUN
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /**
     * Sends sensor data over NRF24L01 radio module.
     * The data is sent every 20ms.
     * If there is no response from the receiver for 1 second, the enable flag is set to 0.
     */
    if (HAL_GetTick() - time_pre >= 20)
    {
      // Pack Data_Tx
      data_Tx.a = Madgwick.roll;
      data_Tx.b = Madgwick.pitch;
      data_Tx.c = Madgwick.yaw;
      data_Tx.d = enable;
      // Send
      if (NRF24_write(&data_Tx, 32))
      {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // LED signal receiving RF24
        // Receive
        NRF24_read(&data_Rx, 32);
        // Unpack Data_Rx
        Setpoint.roll = data_Rx.a;
        Setpoint.pitch = data_Rx.b;
        Setpoint.yaw = data_Rx.c;
        enable = data_Rx.d;
        time_pre = HAL_GetTick();
      }
    }
    if (HAL_GetTick() - time_pre > 1000) // If the control signal is lost after 1s, turn off the power
      enable = 0;
    // chạy tăng tốc 1s trước khi chạy điều khiển
    if (enable == 1 && state == 0)
    {
      run = 2;
      time = HAL_GetTick();
    }
    if (enable == 0)
      run = 0;
    if (run == 2)
    {
      if (HAL_GetTick() - time > 1000)
        run = 1;
    }
    gama = (Madgwick.roll/1.57 + 0.7853)*0.52+1.35;

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void quaternion_to_euler(Quaternion_t q, Euler_t *angles)
{
	// Calculate Euler angles
	double sinr_cosp = 2.0 * (q.q4 * q.q1 + q.q2 * q.q3);
	double cosr_cosp = 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2);
	angles->roll = atan2(sinr_cosp, cosr_cosp);

	double sinp = 2.0 * (q.q4 * q.q2 - q.q3 * q.q1);
	if (fabs(sinp) >= 1.0) {
	    angles->pitch = copysign(M_PI / 2.0, sinp);
	} else {
	    angles->pitch = asin(sinp);
	}

	double siny_cosp = 2.0 * (q.q4 * q.q3 + q.q1 * q.q2);
	double cosy_cosp = 1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3);
	angles->yaw = atan2(siny_cosp, cosy_cosp);
}
// 2. Bo dieu khien LQR
void LQR_Control()
{
  /**
   * Computes the control signals for a quadcopter based on the difference between the setpoint and the current orientation.
   * The control signals are then used to adjust the duty cycle of the motors to achieve the desired orientation.
   */
  // r()
  x[0] = Setpoint.roll - Madgwick.roll;
  x[1] = Setpoint.pitch - Madgwick.pitch;
  x[2] = 0;
  x[3] = Setpoint_dot.roll - MPU6050.Gx;
  x[4] = Setpoint_dot.pitch - MPU6050.Gy;
  x[5] = 0;
  // Tích phân khâu I để bù trừ phần chưa kiểm soát
  if (run == 1)
  {
    i_e_roll += x[0];
    i_e_pitch += x[1];
  }
  else
  {
    i_e_roll = 0;
    i_e_pitch = 0;
  }
  if (Setpoint.roll != Setpoint_pre.roll)
  {
    i_e_roll = 0;
    i_e_pitch = 0;
    Setpoint_pre = Setpoint;
  }
  //
  aa = LimitI / (Ki * DELTA_T);
  if (i_e_roll > aa)
    i_e_roll = aa;
  if (i_e_roll < -aa)
    i_e_roll = -aa;
  if (i_e_pitch > aa)
    i_e_pitch = aa;
  if (i_e_pitch < -aa)
    i_e_pitch = -aa;
  //
  outI_roll = Ki * i_e_roll * DELTA_T;
  outF_roll = tan(Setpoint.roll) * 9.81 * m;
  outI_pitch = Ki * i_e_pitch * DELTA_T;
  outF_pitch = tan(Setpoint.pitch) * 9.81 * m;
  // LQR
  compute_control(x, K, u);
  u[1] += outI_roll;
  u[2] += outI_pitch;

  // Limit
  if (u[0] > 25)
    u[0] = 25;
  if (u[1] > 11.5)
    u[1] = 11.5;
  if (u[1] < -11.5)
    u[1] = -11.5;
  if (u[2] > 11.5)
    u[2] = 11.5;
  if (u[2] < -11.5)
    u[2] = -11.5;
  if (u[3] > 11.5)
    u[3] = 11.5;
  if (u[3] < -11.5)
    u[3] = -11.5;
  u[1] = u[1] * 0.5;
  u[2] = u[2] * 0.5;
  u[3] = u[3] * 0.1;
  // U -> T
  T1 = u[0] * 0.25 + u[3] * 0.25 - u[2] * 0.5; // T = Khối lượng/4 + độ cao + yaw - pitch;
  T2 = u[0] * 0.25 - u[3] * 0.25 + u[1] * 0.5; // T = Khối lượng/4 + độ cao - yaw + roll;
  T3 = u[0] * 0.25 + u[3] * 0.25 + u[2] * 0.5; // T = Khối lượng/4 + độ cao + yaw + pitch;
  T4 = u[0] * 0.25 - u[3] * 0.25 - u[1] * 0.5; // T = Khối lượng/4 + độ cao - yaw - roll;
  // T -> Duty
  M1.Duty = T1 / 11.5 * 1000;
  M2.Duty = T2 / 11.5 * 1000;
  M3.Duty = T3 / 11.5 * 1000;
  M4.Duty = T4 / 11.5 * 1000;
}
// 3. Read sensor and calculate value
void read_Sensor()
{
  /**
   * Reads data from the MPU6050 and MS5611 sensors, applies low-pass filtering and calculates the orientation and altitude.
   */
  // Read Data
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // LED indicates the sensor is read
  MPU6050_Read_All(&hi2c1, &MPU6050);
  // Filter
  Lowpass4MPU(MPU6050, &MPU6050_LP);
  Madgwick_imu(&MPU6050_LP, &Q_est);
  // Caculator
  MPU2Angle(MPU6050_LP, &angle);
  Quat2Angle(Q_est, &Madgwick);
  quaternion_to_euler(Q_est, &Madgwick_new);
  OffsetAngle(&Madgwick, Offset);
  Madgwick.pitch = Madgwick.pitch*gama;
}
// 4. Duration of 2ms cycle will be executed
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim2.Instance)
  {
    read_Sensor();
    LQR_Control();
    // Enable output
    if (run == 1)
    {
      ESC_Control(&htim1, M1.Duty + M1.Duty_base,
                  M2.Duty + M2.Duty_base,
                  M3.Duty + M3.Duty_base,
                  M4.Duty + M4.Duty_base);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
      state = 1;
    }
    else if (run == 2)
    {
      ESC_Control(&htim1, M1.Duty_base,
                  M2.Duty_base,
                  M3.Duty_base,
                  M4.Duty_base);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
      state = 1;
    }
    else
    {
      ESC_Control(&htim1, M1.Duty_min,
                  M2.Duty_min,
                  M3.Duty_min,
                  M4.Duty_min);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
      state = 0;
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

#ifdef USE_FULL_ASSERT
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
