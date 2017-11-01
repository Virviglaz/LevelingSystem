#include "data_collector.h"
#include "error_collector.h"
#include "data_logger.h"
#include "local_db.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "MPU6050.h"
#include "BMP180.h"
#include "SI7005.h"
#include <math.h>
#include "strings.h"
#include "hid_comm.h"
#include "rtc.h"
#include "HID_Commands.h"
#include "BME280.h"
#include "Leveling.h"
#include "Positions.h"

/* Functions */
void UartDataHandler (void);

/* Tasks */
void SI7005_DataCollector (void * pvArg);
void BMP180_DataCollector (void * pvArg);
void BME280_DataCollector (void * pvArg);
void ButtonHandle (void);
void LevelingTask (void * pvArg);
void LebBlinkTaskD1 (void * pvArg);
void LebBlinkTaskD2 (void * pvArg);
void RTC_CorrectionTask (void * pvArg);
void RLY_SW_Switch (void * pvArg);
TaskHandle_t vLevelingTaskHandler;
TaskHandle_t vLebBlinkTaskD1Handler;
TaskHandle_t vLebBlinkTaskD2Handler;
TaskHandle_t vRTC_CorrectionTaskHandler;
TaskHandle_t vZeroPacketInitTaskHandler;
TaskHandle_t LogToSD_TaskHadler;

/* Variables */
extern BMP180_StructTypeDef BMP180_Struct;
extern SI7005_StructTypeDef SI7005_Struct;
extern ErrorTypeDef Error;
extern UartRXTypeDef Uart;
extern USBStructTypeDef USB;
extern RTC_t DateAndTime;
extern char * ErrLogFile;
extern LevelingConfigStructTypeDef LevConfig;
extern LogConfStructTypeDef LogConfig;
extern RTC_CorrectorStructTypeDef RTC_C;

const int BlinkFreqTablen[] = {1000, 500, 333, 250, 200, 167, 143, 125, 111, 100};
const int BlinkFreqTable[] = {100, 111, 125, 143, 167, 200, 250, 333, 500, 1000};
const char BFT_size = 9;
SensorListStructTypeDef SensList;

long AutoOffTimerInit (void)
{
	extern PositionTypeDef Position;
	vTaskResume(vLebBlinkTaskD1Handler);
	vTaskResume(vLebBlinkTaskD2Handler);
	vTaskResume(vLevelingTaskHandler);
	return Position.AutoOffTimerValueS * (1000 / LevConfig.delay);
}

void DataCollectorTask (void * pvArg)
{	
	BaseType_t TaskError = 1;	

	if (!Error.MPU6050)
	{
		
		TaskError &= xTaskCreate(LebBlinkTaskD1, "Leds Task 1",  70, NULL, tskIDLE_PRIORITY + 3 , &vLebBlinkTaskD1Handler);
		TaskError &= xTaskCreate(LebBlinkTaskD2, "Leds Task 2",  70, NULL, tskIDLE_PRIORITY + 3 , &vLebBlinkTaskD2Handler);
	}
	TaskError &= xTaskCreate(RTC_CorrectionTask, "RTC correction",  30, NULL, tskIDLE_PRIORITY + 1 , &vRTC_CorrectionTaskHandler);
	TaskError &= xTaskCreate(ZeroPacketInitTask, "ZERO buffer USB",  50, NULL, tskIDLE_PRIORITY + 1 , &vZeroPacketInitTaskHandler);
	TaskError &= xTaskCreate(LogToSD_Task, "Log to SD card",  1000, NULL, tskIDLE_PRIORITY + 1 , &LogToSD_TaskHadler);
	TaskError &= xTaskCreate(RLY_SW_Switch, "Relay Switch", 30, NULL, tskIDLE_PRIORITY + 1, NULL);
	
	if (!Error.SI7005)
		TaskError &= xTaskCreate(SI7005_DataCollector, "Int Humidity", 70, NULL, tskIDLE_PRIORITY + 1, NULL);
	if (!Error.BMP180)
		TaskError &= xTaskCreate(BMP180_DataCollector, "Int Pressure", 70, NULL, tskIDLE_PRIORITY + 1, NULL);
	if (!Error.BME280)
		TaskError &= xTaskCreate(BME280_DataCollector, "Int Pressure", 100, NULL, tskIDLE_PRIORITY + 1, NULL);
	
	if (!TaskError)
		ErrorHandle(Task, "Task: Not created! Not enought memory.\r\n");

	PrintIntDataToFile(ErrLogFile, "Free memory available: ", (int)xPortGetFreeHeapSize(), "bytes");
	
	while(1)
	{	
		UartDataHandler();
		//USB_RX_DataHandler();
		//USB_TX_DataHandler();
		ButtonHandle();
		IWDG->KR = 0xAAAA; //reset watchdog timer
		vTaskDelay(250);
	}
}

void LebBlinkTaskD1 (void * pvArg)
{
	extern PositionTypeDef Position;
	const PIN_TypeDef D1_PINs[] = {FLU_LED, FLM_LED, FLD_LED, RRU_LED, RRM_LED, RRD_LED};
	char cnt;

	while(1)
	{
		for (cnt = 0; cnt != 6; cnt++) //switch off all leds
			if (LevConfig.isLedTestEnabled)
				PIN_ON(D1_PINs[cnt].GPIOx, D1_PINs[cnt].PINx);
			else
				PIN_OFF(D1_PINs[cnt].GPIOx, D1_PINs[cnt].PINx);
			
		if (!Position.LevelingIsOn) vTaskSuspend( NULL );
			
		if ((Position.D1 >= -Position.D1_zero_t) && (Position.D1 <= Position.D1_zero_t))
		{
				PIN_ON(FLM_LED);
				PIN_ON(RRM_LED);			
		}
		else
		{
			Position.D1_DelayBlink = (char) fabs(Position.D1 / Position.D1_zero_t);
			if (Position.D1_DelayBlink > BFT_size) Position.D1_DelayBlink = BFT_size;
			vTaskDelay(BlinkFreqTable[Position.D1_DelayBlink]);
			if (Position.D1 > Position.D1_zero_t)
			{
				PIN_ON(FLU_LED);
				PIN_ON(RRD_LED);
			}
			else 
			{
				PIN_ON(RRU_LED);
				PIN_ON(FLD_LED);			
			}
		}
		vTaskDelay(BlinkFreqTable[Position.D1_DelayBlink]);
	}
}

void LebBlinkTaskD2 (void * pvArg)
{
	extern PositionTypeDef Position;
	const PIN_TypeDef D2_PINs[] = {FRU_LED, FRM_LED, FRD_LED, RLU_LED, RLM_LED, RLD_LED};
	char cnt;

	while(1)
	{
		for (cnt = 0; cnt != 6; cnt++) //switch off all leds
			if (LevConfig.isLedTestEnabled)
				PIN_ON(D2_PINs[cnt].GPIOx, D2_PINs[cnt].PINx);
			else
				PIN_OFF(D2_PINs[cnt].GPIOx, D2_PINs[cnt].PINx);
		if (!Position.LevelingIsOn) vTaskSuspend( NULL );
		
		if ((Position.D2 >= -Position.D2_zero_t) && (Position.D2 <= Position.D2_zero_t))
		{
				PIN_ON(FRM_LED);
				PIN_ON(RLM_LED);			
		}
		else
		{
			Position.D2_DelayBlink = (char) fabs(Position.D2 / Position.D2_zero_t);
			if (Position.D2_DelayBlink > BFT_size) Position.D2_DelayBlink = BFT_size;
			vTaskDelay(BlinkFreqTable[Position.D2_DelayBlink]);
			if (Position.D2 > Position.D2_zero_t)
			{
				PIN_ON(FRU_LED);
				PIN_ON(RLD_LED);
			}
			else 
			{
				PIN_ON(RLU_LED);
				PIN_ON(FRD_LED);			
			}
		}	
		vTaskDelay(BlinkFreqTable[Position.D2_DelayBlink]);
	}
}

void UartDataHandler (void)
{
	if (!Uart.DataReady) return;
	Uart.DataReady = 0;
	if (Uart.Buffer[0] == (char)CONV_RESULT && Uart.Buffer[1] < MaxSensors)
	{
		char SensNum;
		SensNum = Uart.Buffer[1];
		SensList.OneWireSensors[SensNum].Err = Uart.Buffer[2];			//error code
		SensList.OneWireSensors[SensNum].Res[0] = Uart.Buffer[3];	//H data
		SensList.OneWireSensors[SensNum].Res[1] = Uart.Buffer[4];	//L data
		SensList.OneWireSensors[SensNum].isDataUpdated = 1;				//set update flag
		SensList.OneWireSensors[SensNum].isDataNotLogged = 1;			//prepare for logging	
		return;
	}
	
	if (Uart.Buffer[0] == SearchSensorsRoutine)
	{
		char devcnt, bytecnt, cnt = 2;
		SensList.SensorsFound = Uart.Buffer[1];
		for (devcnt = 0; devcnt != SensList.SensorsFound; devcnt++)
			for (bytecnt = 0; bytecnt != 8; bytecnt++)
				SensList.SearchedSensors[devcnt][bytecnt] = Uart.Buffer[cnt++];
		return;
	}	
	
	if (Uart.Buffer[0] == GET_INSTALLED_SENSORS)
	{
		char devcnt, bytecnt, cnt = 2;
		SensList.SensorsIntalled = Uart.Buffer[1];
		for (devcnt = 0; devcnt != SensList.SensorsIntalled; devcnt++)
			for (bytecnt = 0; bytecnt != 8; bytecnt++)
				SensList.SearchedSensors[devcnt][bytecnt] = Uart.Buffer[cnt++];
		return;		
	}
}

void TIM3_IRQHandler (void)
{
	TIM3->SR &= ~TIM_SR_UIF;
	if (RTC_C.SecondsToMeasure--)
		return;
	RTC_C.EndValue = RTC_GetCounter();
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

void RTC_CorrectionTask (void * pvArg)
{
	//vTaskSuspend ( NULL );
	vTaskDelay(1000);
	while(1)
	{
		const long InitialValue = 24 * 60 * 60; //amount of seconds
		RTC_C.SecondsToMeasure = InitialValue; 
		RTC_C.InitialValue = RTC_GetCounter();
		TIM3->CNT = 0;
		TIM3->CR1 |= TIM_CR1_CEN;
		while (TIM3->CR1 & TIM_CR1_CEN)
			vTaskDelay(1000);
		RTC_C.Result = RTC_C.EndValue - RTC_C.InitialValue - InitialValue;// result hanlde
		PrintIntDataToFile(ErrLogFile, "Clock correction per day: ", RTC_C.Result, "s");
		vTaskDelete ( NULL );
	}	
}

void SI7005_DataCollector (void * pvArg)
{
	while(1)
	{
		/* Task delay */
		vTaskDelay(LogConfig.SI7005_ReadInt * 1000);
		
		/* Get temperature value */
		SI7005_Struct.MeasType = MeasTemp;
		SI7005_StartConversion(&SI7005_Struct);
	
		vTaskDelay(500);		
		
		while (SI7005_DataReady(&SI7005_Struct));
		SensList.intSI7005_Humdt.Err = SI7005_GetResult(&SI7005_Struct);
	
		/* Get humidity */
		SI7005_Struct.MeasType = MeasHum;
		SI7005_StartConversion(&SI7005_Struct);
		
		vTaskDelay(500);
		while (SI7005_DataReady(&SI7005_Struct));			
		SensList.intSI7005_Humdt.Err = SI7005_GetResult(&SI7005_Struct);
		
		SensList.intSI7005_Humdt.Humidity = SI7005_Struct.Humidity;
		
		/* Update flags */
		SensList.intSI7005_Humdt.isDataNotLogged = 1;
		SensList.intSI7005_Humdt.isDataUpdated = 1;
		
		/* Back compatibility */
		SensList.OneWireSensors[SI7005_SensorNumberINT].Err = SensList.intSI7005_Humdt.Err;
		SensList.OneWireSensors[SI7005_SensorNumberINT].Res[1] = SI7005_Struct.Humidity;
		SensList.OneWireSensors[SI7005_SensorNumberINT].isDataUpdated = SET;
		SensList.OneWireSensors[SI7005_SensorNumberINT].isDataNotLogged = SET;
	}
}

void BMP180_DataCollector (void * pvArg)
{
	BMP180_Init(&BMP180_Struct);
	
	while(1)
	{
		/* Task delay */
		vTaskDelay(LogConfig.BMP180_ReadInt * 1000);
		/* Get data */
		BMP180_Get_Result(&BMP180_Struct);
		
		SensList.intBMP180.Pressure = BMP180_Struct.Pressure;
		SensList.intBMP180.Temperature = BMP180_Struct.Temperature;
		SensList.intBMP180.Altitude = Altitude(BMP180_Struct.Pressure);
		SensList.intBMP180.mmHg = Pa_To_Hg(BMP180_Struct.Pressure);

		/* Update flags */
		SensList.intBMP180.isDataNotLogged = 1;
		SensList.intBMP180.isDataUpdated = 1;
	}
}

void BME280_DataCollector (void * pvArg)
{
	extern BME280_StructTypeDef BME280_Struct;
	while(1)
	{
		vTaskDelay(LogConfig.BMP180_ReadInt * 1000);
		//if (!BME280_Busy(&BME280_Struct))
		{
			BME280_Get_Result(&BME280_Struct);
			SensList.intBMP180.Pressure = BME280_Struct.Pressure;
			SensList.intBMP180.Temperature = BME280_Struct.Temperature;
			SensList.intBMP180.Altitude = Altitude(BME280_Struct.Pressure);
			SensList.intBMP180.mmHg = Pa_To_Hg(BME280_Struct.Pressure);
			
			/* Update flags */
			SensList.intBMP180.isDataNotLogged = 1;
			SensList.intBMP180.isDataUpdated = 1;			
		}			
	}
}

short ADC_Get_Result (char channel)
{
  	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
  	// Start the conversion
  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  	// Wait until conversion completion
  	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  	// Get the conversion value
  	return ADC_GetConversionValue(ADC1);
}

void STM8_SendData (char * buf, char size)
{
	while(size--)
	{
		while (!(USART1->SR & USART_SR_TXE));
		USART1->DR = *buf++;
	}
}

void ButtonHandle (void)
{
	extern void EnableFloatMode (char FloatState);
	static char PrevState = SET;
	if (PIN_SYG(BTN) && !PrevState)
	{
		//released
		PrevState = SET;
		EnableFloatMode(DISABLE);
		ChangeLevelingState(DISABLE);
	}
	if (!PIN_SYG(BTN) && PrevState)
	{
		//pressed
		PrevState = RESET;
		EnableFloatMode(DISABLE);
		ChangeLevelingState(ENABLE);
	}	
}

void RLY_SW_Switch (void * pvArg)
{
	char cnt;
	static char PrevState = 0;
	while(1)
	{
		vTaskDelay(1000);
		PIN_OFF(RCS);
		vTaskDelay(10);
		if (LevConfig.RelaysState != PrevState)
		{
			PrevState = LevConfig.RelaysState;
			PWR_BackupAccessCmd(ENABLE);
			BKP_WriteBackupRegister(RelayStateAddress, PrevState);
			PWR_BackupAccessCmd(DISABLE);
		}
		cnt = 8;
		while (cnt)
		{
			cnt--;
			if ((LevConfig.RelaysState & (1 << cnt))) 
				PIN_ON(RMOSI);
			else 
				PIN_OFF(RMOSI);
			
			PIN_OFF(RMSCK);
			vTaskDelay(10);
			PIN_ON(RMSCK);
			vTaskDelay(10);
		}
		PIN_OFF(RMOSI);
		PIN_ON(RCS);
		vTaskDelay(10);
	}
}

