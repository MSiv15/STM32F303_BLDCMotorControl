/*
 * peripheral_utility.c
 *
 *  Created on: 2019/03/24
 *      Author: Shibasaki
 */


#include "peripheral_utility.h"
#include "main.h"

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

