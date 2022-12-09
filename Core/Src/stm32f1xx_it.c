/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t JOY_UP, JOY_DOWN, JOY_SEL;
extern float temperatura_C;
extern ADC_HandleTypeDef hadc1;
extern uint8_t znak;
extern uint8_t symbol;
extern uint8_t klawisz_0;
extern uint8_t klawisz_1;
extern uint8_t klawisz_2;
extern uint8_t klawisz_3;
extern uint8_t klawisz_4;
extern uint8_t klawisz_5;
extern uint8_t klawisz_6;
extern uint8_t klawisz_7;
extern uint8_t klawisz_8;
extern uint8_t klawisz_9;
extern uint8_t klawisz_G;
extern uint8_t klawisz_K;
extern uint8_t klawisz_A;
extern uint8_t klawisz_B;
extern uint8_t klawisz_C;
extern uint8_t klawisz_D;
extern uint8_t n;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
float temperatura;
  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */
			if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
			temperatura=(float)HAL_ADC_GetValue(&hadc1)/1.2433846153846153846153846153846;
			HAL_ADC_Start(&hadc1);
			temperatura_C=((float)temperatura-30)/10;
		}
  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
		//klawiatura
		if(n==4) n=1;
		for(int i=1;i<5;i++)
	{
		switch (i)
		{
case 1:
				HAL_GPIO_WritePin(key_w1_GPIO_Port, key_w1_Pin, GPIO_PIN_RESET);
			////////////////////////////   1   ////////////////////////////
			switch (klawisz_1)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_1++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_1++; else klawisz_1=0; break;
				case 2: znak=1; n++; klawisz_1++;
				case 3: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_1++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_1=0; else klawisz_1=3; break;
			}
			////////////////////////////   2   ////////////////////////////
			switch (klawisz_2)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_2++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_2++; else klawisz_2=0; break;
				case 2: znak=2; n++; klawisz_2++;
				case 3: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_2++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_2=0; else klawisz_2=3; break;
			}
			////////////////////////////   3   ////////////////////////////
			switch (klawisz_3)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_3++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_3++; else klawisz_3=0; break;
				case 2: znak=3; n++; klawisz_3++;
				case 3: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_3++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_3=0; else klawisz_3=3; break;
			}
			////////////////////////////   A   ////////////////////////////
			switch (klawisz_A)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_A++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_A++; else klawisz_A=0; break;
				case 2: symbol=10; klawisz_A++;
				case 3: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_A++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_A=0; else klawisz_A=3; break;
			}
				HAL_GPIO_WritePin(key_w1_GPIO_Port, key_w1_Pin, GPIO_PIN_SET);
			break;
case 2:
				HAL_GPIO_WritePin(key_w2_GPIO_Port, key_w2_Pin, GPIO_PIN_RESET);
			////////////////////////////   4   ////////////////////////////
			switch (klawisz_4)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_4++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_4++; else klawisz_4=0; break;
				case 2: znak=4; n++; klawisz_4++;
				case 3: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_4++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_4=0; else klawisz_4=3; break;
			}
			////////////////////////////   5   ////////////////////////////
			switch (klawisz_5)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_5++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_5++; else klawisz_5=0; break;
				case 2: znak=5; n++; klawisz_5++;
				case 3: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_5++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_5=0; else klawisz_5=3; break;
			}
			////////////////////////////   6   ////////////////////////////
			switch (klawisz_6)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_6++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_6++; else klawisz_6=0; break;
				case 2: znak=6; n++; klawisz_6++;
				case 3: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_6++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_6=0; else klawisz_6=3; break;
			}
			////////////////////////////   B   ////////////////////////////
			switch (klawisz_B)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_B++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_B++; else klawisz_B=0; break;
				case 2: symbol=11; klawisz_B++;
				case 3: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_B++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_B=0; else klawisz_B=3; break;
			}
				HAL_GPIO_WritePin(key_w2_GPIO_Port, key_w2_Pin, GPIO_PIN_SET);
			break;
case 3:
				HAL_GPIO_WritePin(key_w3_GPIO_Port, key_w3_Pin, GPIO_PIN_RESET);
			////////////////////////////   7   ////////////////////////////
			switch (klawisz_7)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_7++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_7++; else klawisz_7=0; break;
				case 2: znak=7; n++; klawisz_7++;
				case 3: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_7++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_7=0; else klawisz_7=3; break;
			}
			////////////////////////////   8   ////////////////////////////
			switch (klawisz_8)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_8++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_8++; else klawisz_8=0; break;
				case 2: znak=8; n++; klawisz_8++;
				case 3: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_8++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_8=0; else klawisz_8=3; break;
			}
			////////////////////////////   9   ////////////////////////////
			switch (klawisz_9)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_9++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_9++; else klawisz_9=0; break;
				case 2: znak=9; n++; klawisz_9++;
				case 3: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_9++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_9=0; else klawisz_9=3; break;
			}
			////////////////////////////   C   ////////////////////////////
			switch (klawisz_C)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_C++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_C++; else klawisz_C=0; break;
				case 2: symbol=12; klawisz_C++;
				case 3: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_C++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_C=0; else klawisz_C=3; break;
			}
				HAL_GPIO_WritePin(key_w3_GPIO_Port, key_w3_Pin, GPIO_PIN_SET);
			break;
case 4:
				HAL_GPIO_WritePin(key_w4_GPIO_Port, key_w4_Pin, GPIO_PIN_RESET);
			////////////////////////////   *   ////////////////////////////
			switch (klawisz_G)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_G++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_G++; else klawisz_G=0; break;
				case 2: symbol=13; klawisz_G++;
				case 3: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_G++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k1_GPIO_Port, key_k1_Pin)) klawisz_G=0; else klawisz_G=3; break;
			}
			////////////////////////////   0   ////////////////////////////
			switch (klawisz_0)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_0++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_0++; else klawisz_0=0; break;
				case 2: znak=0; n++; klawisz_0++;
				case 3: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_0++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k2_GPIO_Port, key_k2_Pin)) klawisz_0=0; else klawisz_0=3; break;
			}
			////////////////////////////   #   ////////////////////////////
			switch (klawisz_K)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_K++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_K++; else klawisz_K=0; break;
				case 2: symbol=14; klawisz_K++;
				case 3: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_K++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k3_GPIO_Port, key_k3_Pin)) klawisz_K=0; else klawisz_K=3; break;
			}
			////////////////////////////   D   ////////////////////////////
			switch (klawisz_D)
			{
				case 0: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_D++; break;
				case 1: if(!HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_D++; else klawisz_D=0; break;
				case 2: symbol=15; klawisz_D++;
				case 3: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_D++; break;
				case 4: if(HAL_GPIO_ReadPin(key_k4_GPIO_Port, key_k4_Pin)) klawisz_D=0; else klawisz_D=3; break;
			}
				HAL_GPIO_WritePin(key_w4_GPIO_Port, key_w4_Pin, GPIO_PIN_SET);
			break;
		}
	}
		//JOY_UP
		if (JOY_UP==0){
			if (!HAL_GPIO_ReadPin(JOY_UP_GPIO_Port,JOY_UP_Pin)) JOY_UP++;
		}
		else {
			if(JOY_UP==1){
				if(!HAL_GPIO_ReadPin(JOY_UP_GPIO_Port,JOY_UP_Pin)) JOY_UP++; else JOY_UP=0;
			}
		}
		
		if (JOY_UP==3){
			if (HAL_GPIO_ReadPin(JOY_UP_GPIO_Port,JOY_UP_Pin)) JOY_UP++;
		}
		else {
			if(JOY_UP==4){
				if(HAL_GPIO_ReadPin(JOY_UP_GPIO_Port,JOY_UP_Pin)) JOY_UP=0; else JOY_UP=3;
			}
		}
		//JOY_DOWN
		if (JOY_DOWN==0){
			if (!HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port,JOY_DOWN_Pin)) JOY_DOWN++;
		}
		else {
			if(JOY_DOWN==1){
				if(!HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port,JOY_DOWN_Pin)) JOY_DOWN++; else JOY_DOWN=0;
			}
		}
		
		if (JOY_DOWN==3){
			if (HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port,JOY_DOWN_Pin)) JOY_DOWN++;
		}
		else {
			if(JOY_DOWN==4){
				if(HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port,JOY_DOWN_Pin)) JOY_DOWN=0; else JOY_DOWN=3;
			}
		}
		//JOY_SEL
		if (JOY_SEL==0){
			if (!HAL_GPIO_ReadPin(JOY_SEL_GPIO_Port,JOY_SEL_Pin)) JOY_SEL++;
		}
		else {
			if(JOY_SEL==1){
				if(!HAL_GPIO_ReadPin(JOY_SEL_GPIO_Port,JOY_SEL_Pin)) JOY_SEL++; else JOY_SEL=0;
			}
		}
		
		if (JOY_SEL==3){
			if (HAL_GPIO_ReadPin(JOY_SEL_GPIO_Port,JOY_SEL_Pin)) JOY_SEL++;
		}
		else {
			if(JOY_SEL==4){
				if(HAL_GPIO_ReadPin(JOY_SEL_GPIO_Port,JOY_SEL_Pin)) JOY_SEL=0; else JOY_SEL=3;
			}
		}
  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
