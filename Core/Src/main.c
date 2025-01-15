/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Device Address
//Please note that arduino uses 7 bit addresses, STM32 uses 8
#define BH1750_NO_GROUND_ADDR_WRITE     (0xB9 + 0)
#define BH1750_NO_GROUND_ADDR_READ      (0xB9 + 1)
#define BH1750_GROUND_ADDR_WRITE        (0x46 + 0)
#define BH1750_GROUND_ADDR_READ         (0x46 + 1)

//instructions
//datasheet ref http://cpre.kmutnb.ac.th/esl/learning/bh1750-light-sensor/bh1750fvi-e_datasheet.pdf
#define CMD_POWER_DOWN          0x00
#define CMD_POWER_ON            0x01
#define CMD_RESET               0x03
#define CMD_H_RES_MODE          0x10
#define CMD_H_RES_MODE2         0x11
#define CMD_L_RES_MODE          0x13
#define CMD_ONE_H_RES_MODE      0x20
#define CMD_ONE_H_RES_MODE2     0x21
#define CMD_ONE_L_RES_MODE      0x23
#define CMD_CNG_TIME_HIGH       0x30    // 3 LSB set time
#define CMD_CNG_TIME_LOW        0x60    // 5 LSB set time

#ifndef bool
#define bool    uint8_t
#endif

#ifndef true
#define true    1
#endif

#ifndef false
#define false   0
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static const uint16_t BH1750_ADDR = 0x23;
uint8_t buff[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
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

  uint8_t buf[12], i2c_buf[2], serialbuf[20];
  char str[32];
  uint32_t analog_value;
  int size_len;
  uint16_t addr = 4;
  HAL_StatusTypeDef ret;
  
  //uint32_t analog_value;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  strcpy((char*)buf, "Start!\r\n");
  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
  //strcpy((char*)buf, "Hello!\r\n");

  i2c_buf[0] = 0b00010000;
  /*ret = HAL_I2C_Master_Transmit(&hi2c1, BH1750_GROUND_ADDR_WRITE, i2c_buf, 1, 200);
  if(ret == HAL_OK){ 
      strcpy((char*)serialbuf, "");
      strcpy((char*)serialbuf, "Power ON\r\n");
      HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
  }else{
      strcpy((char*)serialbuf, "");
      strcpy((char*)serialbuf, "Error Power\r\n");
      HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //HAL_ADC_Start(&hadc1);
	  //if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){

		  //analog_value = HAL_ADC_GetValue(&hadc1);
      //size_len = sprintf(str, "%lu\r\n", analog_value);
      //HAL_UART_Transmit(&huart2, (uint8_t*)str, size_len, HAL_MAX_DELAY);
	  //}
	  //HAL_ADC_Stop(&hadc1);
	  //HAL_Delay(1000);


    /*
	  ret = BH1750_send_command(dev, CMD_RESET);
        if(ret == HAL_OK){ 
      strcpy((char*)serialbuf, "");
      strcpy((char*)serialbuf, "Reset\r\n");
      HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
      }else{
        strcpy((char*)serialbuf, "");
        strcpy((char*)serialbuf, "Reset Fail\r\n");
        HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
      }

	  ret = BH1750_send_command(dev, CMD_H_RES_MODE);
        if(ret == HAL_OK){ 
      strcpy((char*)serialbuf, "");
      strcpy((char*)serialbuf, "RES MODE ON\r\n");
      HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
      }else{
        strcpy((char*)serialbuf, "");
        strcpy((char*)serialbuf, "RES MODE FAIL\r\n");
        HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
      }

    ret = HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, i2c_buf, 1, 200);
    if(ret != HAL_OK){
      strcpy((char*)serialbuf, "Error Rx\r\n");
      HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
    }*/
  
    //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    //HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
    //HAL_Delay(500);
    
    ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0b01000110, i2c_buf, 1, 200);
    if(ret == HAL_OK){      
      strcpy((char*)serialbuf, "");
      strcpy((char*)serialbuf, "Transmited\r\n");
      HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
    }else{
        strcpy((char*)serialbuf, "");
        strcpy((char*)serialbuf, "Failed\r\n");
        HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
    }

    ret = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)0b01000111, i2c_buf, 2, 200);
    if(ret == HAL_OK){      
      size_len = sprintf(str, "%lu\r\n", i2c_buf);
      HAL_UART_Transmit(&huart2, (uint8_t*)str, size_len, HAL_MAX_DELAY);
    }else{
        strcpy((char*)serialbuf, "");
        strcpy((char*)serialbuf, "Failed\r\n");
        HAL_UART_Transmit(&huart2, serialbuf, strlen((char*)serialbuf), HAL_MAX_DELAY);
    }
    HAL_Delay(1000);
    
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void MX_SPI1_Init(){

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if(HAL_SPI_Init(&hspi1) != HAL_OK)
    Error_Handler();
}

/* START OF BH1750 COMMUNICATION FUNCTIONS*/
/*HAL_StatusTypeDef BH1750_send_command(BH1750_device_t* dev, uint8_t cmd)
{
	//TODO hal checks
	if(HAL_I2C_Master_Transmit(
			dev->i2c_handle,	//I2C Handle
			dev->address_w,		//I2C addr of dev
			&cmd,				//CMD to be executed
			1,					//8bit addr
			10					//Wait time
		) != HAL_OK) return HAL_ERROR;

	return HAL_OK;
}*/

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
