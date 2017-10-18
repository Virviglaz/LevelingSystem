#include "STM8_SPI.h"

void SPI_Init_ClockLOW (void)
{
  CLK->PCKENR1 |= CLK_PCKENR1_SPI;
  SPI->CR2 = SPI_CR2_SSI|SPI_CR2_SSM;
  SPI->CR1 = SPI_CR1_SPE|SPI_CR1_MSTR;
}

void SPI_Init_ClockHigh (void)
{
  CLK->PCKENR1 |= CLK_PCKENR1_SPI;
  SPI->CR2 = SPI_CR2_SSI|SPI_CR2_SSM;
  SPI->CR1 = SPI_CR1_SPE|SPI_CR1_MSTR|SPI_CR1_CPOL|SPI_CR1_CPHA;
}

u8 SPI_ReadByte (u8 Data)
{
  SPI->DR = Data; 
  while (!(SPI->SR&SPI_SR_TXE));
  while (!(SPI->SR&SPI_SR_RXNE));
  return SPI->DR; 
}

void SPI_SendByte (u8 Data)
{
  SPI_ReadByte (Data);
}