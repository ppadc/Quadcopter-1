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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	double x;
	double y;
	double z;
}Vector_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define nrf_CSN_PORT GPIOB
#define nrf_CSN_PIN GPIO_PIN_0

#define nrf_CE_PORT GPIOB
#define nrf_CE_PIN GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int enable = 0;
// Sensor
MPU6050_t MPU6050;
MPU6050_t MPU6050_LP;
MS5611_t MS5611;
// Lowpass
Lowpass_t LP_height = {0};
// Height
double height;
double SetPoint_height = 0;
double Offset_height = 0;
// Quaternions
Quaternion_t Q_est = {1, 0, 0, 0};
// Brushless
Motor_t M1 = {
    .Duty = 0,
    .Duty_base = 979,
    .Duty_min = 800,
    .Duty_max = 1400,
};
Motor_t M2 = {
    .Duty = 0,
    .Duty_base = 955,
    .Duty_min = 800,
    .Duty_max = 1400,
};
Motor_t M3 = {
    .Duty = 0,
    .Duty_base = 955,
    .Duty_min = 800,
    .Duty_max = 1400,
};
Motor_t M4 = {
    .Duty = 0,
    .Duty_base = 955,
    .Duty_min = 800,
    .Duty_max = 1400,
};

// Angle
Euler_t angle = {
    .roll = 0,
    .pitch = 0,
    .yaw = 0,
};
Euler_t Madgwick = {
    .roll = 0,
    .pitch = 0,
    .yaw = 0,
};
Euler_t Setpoint = {
    .roll = 0,
    .pitch = 0,
    .yaw = 0,
};
Euler_t Setpoint_dot = {
    .roll = 0,
    .pitch = 0,
    .yaw = 0,
};
Euler_t Offset = {
    .roll = 1.5,
    .pitch = 0,
    .yaw = 0,
};
Vector_t Setpoint_xyz;
Data_t data_Tx;
Data_t data_Rx;
uint64_t TxpipeAddrs = 0xAABBCCDDEE;
uint64_t RxpipeAddrs = 0x1122334455;
char TxData[32] = "";
char RxData[32] = "";
uint64_t value;
const float a = 0.0316;
const float b = 1.321;
double x[6];
double K[4][6] = {{0, 0, 0, 0, 0, 0}, {a, 0, 0, b, 0, 0}, {0, a, 0, 0, b, 0}, {0, 0, a, 0, 0, b}};
double u[4];
double T1, T2, T3, T4;
double T_base = 1.7 * 9.81 / 4;
char q[9] = "";
char w[9] = "";
char e[9] = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void floatToString(float num, char *target);
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
     * Sends data using the NRF24L01+ module and UART.
     *
     * @param q The data to be transmitted via UART.
     * @param w An empty character array.
     * @param e An empty character array.
     * @param huart6 The UART handle.
     * @param GPIOB The GPIO port used for the NRF24L01+ module.
     * @param nrf_CSN_PIN The chip select pin for the NRF24L01+ module.
     * @param nrf_CE_PIN The chip enable pin for the NRF24L01+ module.
     * @param hspi1 The SPI handle.
     * @param TxpipeAddrs The address of the transmitting pipe.
     *
     * @returns None
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
     *
     * @returns None
     */
    // Begin setup GY86
    while (MPU6050_Init(&hi2c1) == 1);
    MPU6050_Bypass(&hi2c1);
    HMC5883L_Init(&hi2c1);
    MPU6050_Master(&hi2c1);
    MPU6050_Addslave(&hi2c1);
    MS5611_init(&hi2c1, &MS5611);
    /**
     * Sets up the ESC (Electronic Speed Controller) for a motor using the specified timer.
     *
     * @param htim1 The timer used for the ESC.
     *
     * @returns None
     */
    //	 Begin setup ESC
    setupESC(&htim1);
    /**
     * Starts the timer interrupt for TIM2.
     *
     * @param htim2 A pointer to the TIM2 handle.
     *
     * @returns None
     */
    //	 RUN
    HAL_TIM_Base_Start_IT(&htim2);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /**
         * Sends sensor data over NRF24L01 radio module.
         * The data is sent every 20ms and the enable flag is set to 1.
         * If the enable flag is set to 0, the data is not sent.
         * If there is a response from the receiver, the enable flag is set to 1 if the response is '1' and 0 if the response is '0'.
         * If there is no response from the receiver for 1 second, the enable flag is set to 0.
         */
        if (HAL_GetTick() - value >= 20)
        {
            //Pack Data_Tx
            data_Tx.a = Madgwick.roll;
            data_Tx.b = Madgwick.pitch;
            data_Tx.c = Madgwick.yaw;
            data_Tx.d = enable;
            if (NRF24_write(&data_Tx, 32))
            {
            	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
                // Receive
                NRF24_read(&data_Rx, 32);
                Setpoint_xyz.x = data_Rx.a;
                Setpoint_xyz.y = data_Rx.b;
               	Setpoint_xyz.z = data_Rx.c;
				enable		   = data_Rx.d;
				value = HAL_GetTick();
            }
        }
        if (HAL_GetTick() - value > 1000)
            enable = 0;
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
// Hàm chuyển float sang string
void floatToString(float num, char *target)
{
    int intPart = (int)num;                              // phần nguyên của số float
    float fracPart = num - intPart;                      // phần thập phân của số float
    char intStr[5];                                      // chuỗi chứa phần nguyên
    char fracStr[5];                                     // chuỗi chứa phần thập phân
    sprintf(intStr, "%d", intPart);                      // gọi hàm chuyển phần nguyên sang string
    sprintf(fracStr, "%d", (int16_t)(fracPart * 10000)); // gọi hàm chuyển phần thập phân sang string với độ chính xác là 6 chữ số sau dấu phẩy
    sprintf(target, "%s.%s", intStr, fracStr);           // nối hai chuỗi lại với nhau và thêm dấu chấm vào giữa
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance)
    {
        /**
         * Reads data from the MPU6050 and MS5611 sensors, applies low-pass filtering and calculates the orientation and altitude.
         */
        // Read Data
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        MPU6050_Read_All(&hi2c1, &MPU6050);
        MS5611_calculate(&hi2c1, &MS5611);
        // Filter
        Lowpass4MPU(MPU6050, &MPU6050_LP);
        Lowpass(MS5611.P, &LP_height, 0.5);
        MPU2Angle(MPU6050_LP, &angle);
        Madgwick_update(&MPU6050_LP, &Q_est);
        // Caculator
        Quat2Angle(Q_est, &Madgwick);
        OffsetAngle(&Madgwick, Offset);
        height = getAltitude(LP_height.out, 101325);
        /**
         * Computes the control signals for a quadcopter based on the difference between the setpoint and the current orientation.
         * The control signals are then used to adjust the duty cycle of the motors to achieve the desired orientation.
         */
        // r()
        x[0] = Setpoint.roll - Madgwick.roll;
        x[1] = Setpoint.pitch - Madgwick.pitch;
        x[2] = Setpoint.yaw - Madgwick.yaw;
        x[3] = Setpoint_dot.roll - MPU6050.Gx;
        x[4] = Setpoint_dot.pitch - MPU6050.Gy;
        x[5] = Setpoint_dot.yaw - MPU6050.Gz;
        //
        //		x[0] = 0;
        //		x[1] = 0;
        x[2] = 0;
        //		x[3] = 0;
        //		x[4] = 0;
        x[5] = 0;

        // LQR
        compute_control(x, K, u);
        // Limit
        if (u[0] > 8.7969)
            u[0] = 8.7969;
        if (u[0] < -16.677)
            u[0] = -16.677;
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
        u[1] = u[1] * 0.3;
        u[2] = u[2] * 0.3;
        u[3] = u[3] * 0.1;

        // U -> T
        T1 = T_base + u[0] * 0.25 + u[3] * 0.25 - u[2] * 0.5; // T = Khối lượng/4 + độ cao + yaw - pitch;
        T2 = T_base + u[0] * 0.25 - u[3] * 0.25 + u[1] * 0.5; // T = Khối lượng/4 + độ cao - yaw + sin;
        T3 = T_base + u[0] * 0.25 - u[3] * 0.25 + u[2] * 0.5; // T = Khối lượng/4 + độ cao - yaw + pitch;
        T4 = T_base + u[0] * 0.25 - u[3] * 0.25 - u[1] * 0.5; // T = Khối lượng/4 + độ cao - yaw - sin;

        // T -> Duty
        M1.Duty = T1 / 11.5 * 1000;
        M2.Duty = T2 / 11.5 * 1000;
        M3.Duty = T3 / 11.5 * 1000;
        M4.Duty = T4 / 11.5 * 1000;

        // Enable output PID
        if (enable)
        {
            ESC_Control(&htim1, M1.Duty + M1.Duty_base, M2.Duty + M2.Duty_base, M3.Duty + M3.Duty_base, M4.Duty + M4.Duty_base); // Duty + Duty_base
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