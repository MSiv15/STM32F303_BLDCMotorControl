/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
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

void uart1_putc(uint8_t c)
{
  while (!LL_USART_IsActiveFlag_TXE(USART1));
  LL_USART_TransmitData8(USART1, c);
}

uint8_t uart1_getc(void)
{
  while (!LL_USART_IsActiveFlag_RXNE(USART1));
  return LL_USART_ReceiveData8(USART1);
}

uint16_t SPI1_TransmitReceive16(uint16_t TxData)
{
  uint16_t RxData;
  LL_GPIO_ResetOutputPin(SPI_NSS_GPIO_Port,SPI_NSS_Pin);
  while (!LL_SPI_IsActiveFlag_TXE(SPI1));
  LL_SPI_TransmitData16(SPI1, TxData);
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
  RxData = LL_SPI_ReceiveData16(SPI1);
  LL_GPIO_SetOutputPin(SPI_NSS_GPIO_Port,SPI_NSS_Pin);
  return RxData;
}

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

  /* USER CODE BEGIN Init */

  // xprintf allocation
  xdev_out(uart1_putc);
  xdev_in(uart1_getc);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  while (LL_GPIO_IsInputPinSet(USER_SW_GPIO_Port,USER_SW_Pin));
  xprintf("Hello world!\n");

  // controller initialization
  period = LL_TIM_GetAutoReload(TIM1);

  // ADC2 Initialization
  WRITE_REG(ADC2->DIFSEL,0U); // LL_ADC_SetChannelSingleDiffが未定義動作を起こすので、レジスタに直接書き込む
  LL_ADC_EnableInternalRegulator(ADC2);
  delay_us(10);
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC2));
  LL_ADC_Enable(ADC2);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC2));
  // JSQRをクリア
  LL_ADC_INJ_StopConversion(ADC2);
  while (LL_ADC_INJ_IsStopConversionOngoing(ADC2));
  // CubeMXが出力するコードが不適切なため、ADC2のJSQRを改めて設定する
  LL_ADC_DisableIT_JQOVF(ADC2);
  LL_ADC_INJ_ConfigQueueContext(ADC2,
                                LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2,
                                LL_ADC_INJ_TRIG_EXT_RISING,
                                LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS,
                                LL_ADC_CHANNEL_1,
                                LL_ADC_CHANNEL_2,
                                LL_ADC_CHANNEL_3,
                                LL_ADC_CHANNEL_4);
  // Inject変換の外部トリガを許可
  LL_ADC_INJ_StartConversion(ADC2);

  // Enable ADC Interrupt
  LL_ADC_EnableIT_JEOS(ADC2);
//  LL_TIM_EnableIT_UPDATE(TIM1);

  // Set ADC Sampling Timing
  LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(3599 - 90));

  // Enable PWM Career
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_SetRepetitionCounter(TIM1,1);

  // Enable PWM Channels
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

  // Enable SPI Channel
  LL_SPI_Enable(SPI1);

  // Enable EXTI Interrupt
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    FOC_LowFreqTask();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(72000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PLL);
  LL_RCC_SetADCClockSource(LL_RCC_ADC34_CLKSRC_PLL_DIV_1);
}

/* USER CODE BEGIN 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
