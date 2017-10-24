#include "FreeRTOS.h"
#include "task.h"
#include "error_collector.h"
#include "HW.h"
#include "data_collector.h"
#include "stm32_GPIO.h"
#include <stdio.h>

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

void vApplicationMallocFailedHook( void )
{
	/* Out of memory */

}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	/* Stack overflow */
	extern char PrintToFile (char * filename, char * buf);
	char * buf = pvPortMalloc(100);
	sprintf(buf, "Task: %s has reported stack overflow!", pcTaskName);
	PrintToFile("ERRLOG.TXT", buf);
	vPortFree(buf);
}

void AssertFailed (void)
{
	
}

/* Redefine stdlib functions */
void * malloc(size_t s) { return pvPortMalloc(s); }
void free( void *pv ) { vPortFree( pv ); }
