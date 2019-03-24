/*
 * peripheral_utility.h
 *
 *  Created on: 2019/03/24
 *      Author: Shibasaki
 */

#ifndef PERIPHERAL_UTILITY_H_
#define PERIPHERAL_UTILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_spi.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx.h"
#include "stm32f3xx_ll_gpio.h"

void uart1_putc(uint8_t c);
uint8_t uart1_getc(void);
uint16_t SPI1_TransmitReceive16(uint16_t TxData);

static inline void delay_us(uint16_t time)
{
  LL_TIM_SetCounter(TIM6,0);
  LL_TIM_EnableCounter(TIM6);
  while (LL_TIM_GetCounter(TIM6)<time);
  LL_TIM_DisableCounter(TIM6);
}

static inline void delay_ms(uint32_t time)
{
#ifdef USE_HAL_DRIVER
  HAL_Delay(time);
#else
  LL_mDelay(time);
#endif
}


#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_UTILITY_H_ */
