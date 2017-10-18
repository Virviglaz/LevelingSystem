#include "FreeRTOS.h"
#include "task.h"
#include "error_collector.h"
#include "HW.h"
#include "data_collector.h"
#include "stm32_GPIO.h"

/* Variables */
UartRXTypeDef Uart;

int main (void)
{
	extern int GeneralInit (void);
	extern ErrorTypeDef Error;
	Error.TaskCreate = GeneralInit();
	
	vTaskStartScheduler();
	return 0;
}

void TIM4_IRQHandler (void)
{
	PIN_OFF(LED11);
	Uart.DataReady = SET;
	Uart.Buffer[Uart.BufferIndex] = 0;
	Uart.BufferIndex = 0;
	TIM4->SR &= ~TIM_SR_UIF;
	TIM4->CR1 &= ~TIM_CR1_CEN;
}

void USART1_IRQHandler (void)
{
	PIN_ON(LED11);
	if (Uart.BufferIndex < sizeof(Uart.Buffer))	
		Uart.Buffer[Uart.BufferIndex++] = (char)USART1->DR;

	TIM4->CR1 |= TIM_CR1_CEN;
	TIM4->CNT = 0;	
	USART1->SR &= ~USART_SR_RXNE;
}
