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
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Title1 "       SYSTEM       "
#define Title2 "     OGRZEWANIA     "
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

SRAM_HandleTypeDef hsram3;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t JOY_UP=0, JOY_DOWN=0, JOY_SEL=0;
volatile float temperatura_C;
volatile uint8_t znak=0;
volatile uint8_t symbol=99;
volatile uint8_t klawisz_0=0;
volatile uint8_t klawisz_1=0;
volatile uint8_t klawisz_2=0;
volatile uint8_t klawisz_3=0;
volatile uint8_t klawisz_4=0;
volatile uint8_t klawisz_5=0;
volatile uint8_t klawisz_6=0;
volatile uint8_t klawisz_7=0;
volatile uint8_t klawisz_8=0;
volatile uint8_t klawisz_9=0;
volatile uint8_t klawisz_A=0;
volatile uint8_t klawisz_B=0;
volatile uint8_t klawisz_C=0;
volatile uint8_t klawisz_D=0;
volatile uint8_t klawisz_K=0;
volatile uint8_t klawisz_G=0;
volatile uint8_t n=0;
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
  MX_TIM7_Init();
  MX_FSMC_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start(&htim7);
	HAL_ADC_Start(&hadc1);
	//while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim6);
	
	float licznik=20;
	float histereza=5;
	uint8_t temp_zad[20];
	uint8_t temp_akt[20];
	uint8_t histereza_disp[20];
	uint8_t z=0;
	
	STM3210E_LCD_Init();
	LCD_Clear(Black);
  LCD_SetBackColor(Black);
	LCD_SetTextColor(Green);
	LCD_DisplayStringLine(Line0, Title1);
	LCD_DisplayStringLine(Line1, Title2);
  LCD_SetBackColor(Black);                        // Set the Back Color */
  LCD_SetTextColor(Red);
	snprintf((char *)temp_zad, 20, "Temp. Zad. : %.1f*C   ", licznik);
	LCD_DisplayStringLine(Line3, temp_zad);
	snprintf((char *)histereza_disp, 20, "Histereza : %.1f*C   ", histereza);
	LCD_DisplayStringLine(Line7, histereza_disp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//klawiatura 
		if (symbol == 10)
		{
			LCD_SetBackColor(Black);
			LCD_DisplayStringLine(Line3, temp_zad);
			LCD_SetBackColor(White);
			LCD_DisplayStringLine(Line7, histereza_disp);
				switch (n) {
					case 1 :
						histereza=0.1*znak;
						z=0;
					break;
					case 2 :
						if (!(z&1)){
						histereza *= 10;
						histereza += 0.1*znak;
						z++;
						}
					break;
					case 3:
						if(z&1){
						histereza *= 10;
						histereza += 0.1*znak;
						z++;
						n=0;
						}
					break;
				}
				snprintf((char *)histereza_disp, 20, "Histereza : %.1f*C   ", histereza);
				LCD_DisplayStringLine(Line7, histereza_disp);
		}
		
		if (symbol == 11)
		{
			LCD_SetBackColor(White);
			LCD_DisplayStringLine(Line3, temp_zad);
			LCD_SetBackColor(Black);
			LCD_DisplayStringLine(Line7, histereza_disp);
				switch (n) {
					case 1 :
						licznik=0.1*znak;
						z=0;
					break;
					case 2 :
						if (!(z&1)){
						licznik *= 10;
						licznik += 0.1*znak;
						z++;
						}
					break;
					case 3:
						if(z&1){
						licznik *= 10;
						licznik += 0.1*znak;
						z++;
						n=0;
						}
					break;
				}
				LCD_SetBackColor(White);
				snprintf((char *)temp_zad, 20, "Temp. Zad. : %.1f*C   ", licznik);
				LCD_DisplayStringLine(Line3, temp_zad);
		}
		if(symbol !=10 && symbol !=11){
			znak=0;
			n=0;
			LCD_SetBackColor(Black);
			LCD_DisplayStringLine(Line3, temp_zad);
			LCD_SetBackColor(Black);
			LCD_DisplayStringLine(Line7, histereza_disp);
		}
		
			LCD_SetBackColor(Black);
			snprintf((char *)temp_akt, 20, "Temp. Akt. : %.1f*C     ", temperatura_C);
			LCD_DisplayStringLine(Line5, temp_akt);
			
		if (temperatura_C<licznik-histereza) 
			GPIOF->ODR |= (1<<6);
		if (temperatura_C>licznik+histereza)
			GPIOF->ODR &= ~(1<<6);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim6.Init.Prescaler = 4999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 7999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, key_w1_Pin|key_w2_Pin|key_w3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, key_w4_Pin|LED_Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : key_w1_Pin key_w2_Pin key_w3_Pin */
  GPIO_InitStruct.Pin = key_w1_Pin|key_w2_Pin|key_w3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : key_k1_Pin key_k2_Pin */
  GPIO_InitStruct.Pin = key_k1_Pin|key_k2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : key_k3_Pin key_k4_Pin */
  GPIO_InitStruct.Pin = key_k3_Pin|key_k4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : key_w4_Pin LED_Relay_Pin */
  GPIO_InitStruct.Pin = key_w4_Pin|LED_Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_SEL_Pin JOY_UP_Pin */
  GPIO_InitStruct.Pin = JOY_SEL_Pin|JOY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(JOY_DOWN_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM3 memory initialization sequence
  */
  hsram3.Instance = FSMC_NORSRAM_DEVICE;
  hsram3.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram3.Init */
  hsram3.Init.NSBank = FSMC_NORSRAM_BANK4;
  hsram3.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram3.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram3.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram3.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram3.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram3.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram3.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram3.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram3.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram3.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram3.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram3.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram3, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	  HAL_GPIO_TogglePin(GPIOF, LEDF7_Pin);
//	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
