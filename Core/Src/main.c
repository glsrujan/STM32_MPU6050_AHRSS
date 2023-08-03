/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "sd_hal_mpu6050.h"
#include "MadgwickAHRS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SD_MPU6050 mpu1;
HAL_StatusTypeDef ret;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

// Accelerometer data raw

int16_t a_x;
int16_t a_y;
int16_t a_z;

int16_t g_x;
int16_t g_y;
int16_t g_z;

// Accelerometer data converted to +16 to -16 G

uint8_t Tim_Cnt = 1;
uint8_t prev_tim_val;

double Ax;
double Ay;
double Az;

double Gx;
double Gy;
double Gz;

//calib variables
double tot_ax;
double tot_ay;
double tot_az;

double tot_gx;
double tot_gy;
double tot_gz;

double ROLL;
double PITCH;
double YAW;

float Eu_Roll;
float Eu_Pitch;
float Eu_Yaw;
// Extras
SD_MPU6050_Result result ;
bool Bias_calib = true;
uint8_t calib = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
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

//	uint8_t buff[12];
    uint8_t mpu_ok[20] ={"MPU WORK FINE\r\n"};
    uint8_t mpu_not[20] ={"MPU NOT WORKING\r\n"};
    uint8_t tags[200] ={"Raw_ax,Raw_ay,Raw_az,Raw_gx,Raw_gy,Raw_gz,Ax,Ay,Az,Gx,Gy,Gz,Q0,Q1,Q2,Q3,ROLL,PITCH,YAW,Eu_Roll,Eu_Pitch,Eu_Yaw\r\n"};
    char data[1000];

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim6);

    result = SD_MPU6050_Init(&hi2c2, &mpu1, SD_MPU6050_Device_0, SD_MPU6050_Accelerometer_16G, SD_MPU6050_Gyroscope_2000s);
    HAL_Delay(500);
    if (result == SD_MPU6050_Result_Ok)
	{
	HAL_UART_Transmit(&huart3, mpu_ok, sizeof(mpu_ok), HAL_MAX_DELAY);
	}
    else
	{
	HAL_UART_Transmit(&huart3, mpu_not, sizeof(mpu_not), HAL_MAX_DELAY);
	}

    sprintf(data, "calibrating....\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);

    for (int calib_loop = 0; calib_loop < 2000; calib_loop++)
	{
	SD_MPU6050_ReadGyroscope(&hi2c2, &mpu1);
	g_x = mpu1.Gyroscope_X;
	g_y = mpu1.Gyroscope_Y;
	g_z = mpu1.Gyroscope_Z;

//	SD_MPU6050_ReadAccelerometer(&hi2c2, &mpu1);
//	a_x = mpu1.Accelerometer_X;
//	a_y = mpu1.Accelerometer_Y;
//	a_z = mpu1.Accelerometer_Z;

//	tot_ax = tot_ax + (double)a_x/2000.0;
//	tot_ay = tot_ax + (double)a_y/2000.0;
//	tot_az = tot_ax + (double)a_z/2000.0;

	tot_gx = tot_gx + (double) g_x / 2000.0;
	tot_gy = tot_gy + (double) g_y / 2000.0;
	tot_gz = tot_gz + (double) g_z / 2000.0;

	HAL_Delay(2);
	}
    sprintf(data, "calibrating done\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart3, (uint8_t*) tags, sizeof(tags), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (prev_tim_val != Tim_Cnt)
	    {
	    prev_tim_val = Tim_Cnt;

	    SD_MPU6050_ReadTemperature(&hi2c2, &mpu1);
//	  float temper = mpu1.Temperature;
	    SD_MPU6050_ReadGyroscope(&hi2c2, &mpu1);
	    g_x = mpu1.Gyroscope_X;
	    g_y = mpu1.Gyroscope_Y;
	    g_z = mpu1.Gyroscope_Z;

	    SD_MPU6050_ReadAccelerometer(&hi2c2, &mpu1);
	    a_x = mpu1.Accelerometer_X;
	    a_y = mpu1.Accelerometer_Y;
	    a_z = mpu1.Accelerometer_Z;

	    Bias_calib = false;
	    Ax = (((((double) a_x - tot_ax) + 32768.0) / 65535.0) * 32.0) - 16.0;
	    Ay = (((((double) a_y - tot_ay) + 32768.0) / 65535.0) * 32.0) - 16.0;
	    Az = (((((double) a_z - tot_az) + 32768.0) / 65535.0) * 32.0) - 16.0;

	    Gx = (((((double) g_x - tot_gx) + 32768.0) / 65535.0) * 4000.0) - 2000.0;
	    Gy = (((((double) g_y - tot_gy) + 32768.0) / 65535.0) * 4000.0) - 2000.0;
	    Gz = (((((double) g_z - tot_gz) + 32768.0) / 65535.0) * 4000.0) - 2000.0;

	    Gx = (Gx * M_PI) / 180.0;
	    Gy = (Gy * M_PI) / 180.0;
	    Gz = (Gz * M_PI) / 180.0;

	    MadgwickAHRSupdateIMU(Gx, Gy, Gz, Ax, Ay, Az);

//	  	Euler Angles

//	  	roll (x-axis rotation)
	    double sinr_cosp = 2 * ((q0 * q1) + (q2 * q3));
	    double cosr_cosp = 1 - (2 * ((q1 * q1) + (q2 * q2)));
	    ROLL = atan2(sinr_cosp, cosr_cosp);

//	    pitch (y-axis rotation)
	    double sinp = 2 * ((q0 * q2) - (q3 * q1));
	    if (abs(sinp) >= 1)
		PITCH = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	    else
		PITCH = asin(sinp);

	    // yaw (z-axis rotation)
	    double siny_cosp = 2 * ((q0 * q3) + (q1 * q2));
	    double cosy_cosp = 1 - (2 * ((q2 * q2) + (q3 * q3)));
	    YAW = atan2(siny_cosp, cosy_cosp);

	    Eu_Roll = ROLL * (180 / M_PI);
	    Eu_Pitch = PITCH * (180 / M_PI);
	    Eu_Yaw = YAW * (180 / M_PI);

	    /**********************************    Enable for Data analysis   ***************************/
	    sprintf(data, "%d ,%d ,%d ,%d ,%d ,%d ,%f ,%f ,%f ,%f ,%f ,%f ,", a_x, a_y, a_z, g_x, g_y, g_z, Ax, Ay, Az, Gx, Gy, Gz);
	    HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);
	    sprintf(data, "%f ,%f ,%f ,%f ,", q0, q1, q2, q3);
	    HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);
	    sprintf(data, "%f ,%f ,%f ,%f ,%f ,%f\r\n", ROLL, PITCH, YAW, Eu_Roll, Eu_Pitch, Eu_Yaw);
	    HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);
	    /*********************************************************************************************/

//	  	sprintf(data,"%f ,%f ,%f\r\n",Eu_Roll,Eu_Pitch,Eu_Yaw);
//	  	HAL_UART_Transmit(&huart3, (uint8_t*) data, strlen(data),HAL_MAX_DELAY);
	    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x6000030D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 108-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		if(Tim_Cnt < 200)
		{
			Tim_Cnt++;
		}
		else
		{
			Tim_Cnt = 1;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
