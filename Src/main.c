/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usart.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SystemClock_Decrease(void);
void SystemPower_Config(void);
void BufferTransfer(int mode);
void LPUARTClock_Config(void);
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
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral 
  */
  LL_PWR_DisableDeadBatteryPD();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Polling USART initialisation */
	SystemPower_Config();
	LL_mDelay(1000);
  while (1)
  {
		SystemClock_Decrease();
		LPUARTClock_Config();
		LL_PWR_EnableLowPowerRunMode();
		BufferTransfer(0);
		LL_mDelay(1000);

		LL_PWR_DisableLowPowerRunMode();
		while(LL_PWR_IsActiveFlag_REGLPF() == 1);
		SystemClock_Config();
		LPUARTClock_Config();
		BufferTransfer(1);
		LL_mDelay(1000);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_8);
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_6, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  /* Insure 1?s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */
void SystemClock_Decrease(void)
{
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  LL_RCC_PLL_DisableDomain_SYS();
  LL_RCC_PLL_Disable();
  LL_RCC_HSE_Disable();

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_8);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(2000000);

  LL_SetSystemCoreClock(2000000);
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
    Error_Handler();  
  }	
}

void SystemPower_Config(void)
{
	/*** Peripharel Disable ***/
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI3);

	
	/*** GPIO Disable ***/
  LL_GPIO_InitTypeDef gpio_initstruct = {LL_GPIO_PIN_ALL, LL_GPIO_MODE_ANALOG, 
                                         LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_OUTPUT_PUSHPULL, 
                                         LL_GPIO_PULL_NO, LL_GPIO_AF_0};

  /* Set all GPIO in analog state to reduce power consumption,                */
  /* Note: Debug using ST-Link is not possible during the execution of this   */
  /*       example because communication between ST-link and the device       */
  /*       under test is done through UART. All GPIO pins are disabled (set   */
  /*       to analog input mode) including  UART I/O pins.                    */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA |
                            LL_AHB2_GRP1_PERIPH_GPIOB |
                            LL_AHB2_GRP1_PERIPH_GPIOC |
                            LL_AHB2_GRP1_PERIPH_GPIOD |
                            LL_AHB2_GRP1_PERIPH_GPIOE |
                            LL_AHB2_GRP1_PERIPH_GPIOF |
                            LL_AHB2_GRP1_PERIPH_GPIOG );

  LL_GPIO_Init(GPIOA, &gpio_initstruct);
  LL_GPIO_Init(GPIOB, &gpio_initstruct);
  LL_GPIO_Init(GPIOC, &gpio_initstruct);
  LL_GPIO_Init(GPIOD, &gpio_initstruct);
  LL_GPIO_Init(GPIOE, &gpio_initstruct);
  LL_GPIO_Init(GPIOF, &gpio_initstruct);
  LL_GPIO_Init(GPIOG, &gpio_initstruct);

  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOA |
                            LL_AHB2_GRP1_PERIPH_GPIOB |
                            LL_AHB2_GRP1_PERIPH_GPIOC |
                            LL_AHB2_GRP1_PERIPH_GPIOD |
                            LL_AHB2_GRP1_PERIPH_GPIOE |
                            LL_AHB2_GRP1_PERIPH_GPIOF |
                            LL_AHB2_GRP1_PERIPH_GPIOG );
														
	/** LPUART Enable **/
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**LPUART1 GPIO Configuration  
  PA2   ------> LPUART1_TX
  PA3   ------> LPUART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
}

uint8_t ubSend = 0;
const uint8_t aStringToSend0[] = "LP RUN Mode - STM32G4xx LPUART LL API Example : TX in Polling mode\r\nConfiguration LPUART 9600 bps, 8 data bit/1 stop bit/No parity/No HW flow control\r\n";
const uint8_t aStringToSend1[] = "RUN Mode - STM32G4xx LPUART LL API Example : TX in Polling mode\r\nConfiguration LPUART 9600 bps, 8 data bit/1 stop bit/No parity/No HW flow control\r\n";

void BufferTransfer(int mode)
{
	if(mode == 0)
	{
		/* Send characters one per one, until last char to be sent */
		while (ubSend < sizeof(aStringToSend0))
		{
			/* Wait for TXE flag to be raised */
			while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
			{
			}

			/* If last char to be sent, clear TC flag */
			if (ubSend == (sizeof(aStringToSend0) - 1))
			{
				LL_LPUART_ClearFlag_TC(LPUART1);
			}

			/* Write character in Transmit Data register.
				 TXE flag is cleared by writing data in TDR register */
			LL_LPUART_TransmitData8(LPUART1, aStringToSend0[ubSend++]);
		}
	}
	else
	{
		/* Send characters one per one, until last char to be sent */
		while (ubSend < sizeof(aStringToSend1))
		{
			/* Wait for TXE flag to be raised */
			while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
			{
			}

			/* If last char to be sent, clear TC flag */
			if (ubSend == (sizeof(aStringToSend1) - 1))
			{
				LL_LPUART_ClearFlag_TC(LPUART1);
			}

			/* Write character in Transmit Data register.
				 TXE flag is cleared by writing data in TDR register */
			LL_LPUART_TransmitData8(LPUART1, aStringToSend1[ubSend++]);
		}
	}

  /* Wait for TC flag to be raised for last char */
  while (!LL_LPUART_IsActiveFlag_TC(LPUART1))
  {
  }
	
	ubSend = 0;
}

void LPUARTClock_Config(void)
{
	uint32_t periphclk;
	
	periphclk = LL_RCC_GetLPUARTClockFreq(LL_RCC_LPUART1_CLKSOURCE);
  LL_LPUART_SetBaudRate(LPUART1, periphclk, LL_LPUART_PRESCALER_DIV1, 9600);
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
