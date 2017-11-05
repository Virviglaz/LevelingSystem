#include "stm32f10x.h"                  // Device header
#include "hid_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "rtc.h"
#include "data_collector.h"
#include "error_collector.h"
#include "data_logger.h"
#include "Positions.h"
#include "MPU6050.h"
#include "BMP180.h"
#include "SI7005.h"
#include "HID_Commands.h"
#include "local_db.h"
#include "kalman.h"
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "ff.h"
#include "diskio.h"

/* Functions */
void InstallNewSensorHandle (void);
void DELETE_SENSOR_COMM_Handle (void);
void SET_LEDS_BRIGHTNESS_Handle (void);
void SEARCH_SENSORS_Handle (void);
void EnableFloatMode (char FloatState);
void LED_TEST_ROUTINE_Handle (void);
void TurnOnAddLEDn (char LedNum, char R, char G, char B);
void GetInstalledSensors_Command (void);
void IR_Handle (void);
void ChangeSettingsHandler (void);
void GetSettingsHandler (void);
void SaveFileHandler (void);
void DefineFileName (void);
void FileFinish (void);
extern void CallibrateZeroPositionHandle (void);

/* Variables */
USBStructTypeDef USB;
BYTE * USB_TX_Buffer = 0;
extern BYTE EP0Buf[USB_MAX_PACKET0];
extern MPU6050_StructTypeDef MPU6050_Struct;
extern BMP180_StructTypeDef BMP180_Struct;
extern SI7005_StructTypeDef SI7005_Struct;
extern ErrorTypeDef Error;
extern UartRXTypeDef Uart;
extern USBStructTypeDef USB;
extern RTC_t DateAndTime;
extern PositionTypeDef Position;
extern char * ErrLogFile;
extern LevelingConfigStructTypeDef LevConfig;
extern SensorListStructTypeDef SensList;
extern LogConfStructTypeDef LogConfig;
V32_TypeDef V32;
char FileName[15];
FIL FileObject;       /* File object */
QueueHandle_t xUSB_TX_TextQueue;

/* Tasks */
void ChangeSettingsTask (void * pvArg);
void SaveFileTask (void * pvArg);

/* Mutex */
extern SemaphoreHandle_t xFLASH_Semaphore;
extern SemaphoreHandle_t xUSB_RX_Semaphore;
extern SemaphoreHandle_t xUSB_TX_Semaphore;
extern SemaphoreHandle_t xUSB_ZX_Semaphore;

void GetReportToPC (void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	
	if (xSemaphoreTakeFromISR(xUSB_TX_Semaphore, &xHigherPriorityTaskWoken) == pdTRUE)
	{
		USB_WriteEP(EP1_IN, USB.TX_Buffer, USB_PACKET_LEN);
		return;
	}
	
	if (xSemaphoreTakeFromISR(xUSB_ZX_Semaphore, &xHigherPriorityTaskWoken) == pdTRUE)
	{
		USB_WriteEP(EP1_IN, USB.ZERO_Buffer, USB_PACKET_LEN);
		return;
	}
	
	USB_WriteEP(EP1_IN, USB.NOP_Buffer, USB_PACKET_LEN);
}

void ReportFromPCHandle (void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	memcpy(USB.RX_Buffer, EP0Buf, USB_PACKET_LEN);
	xSemaphoreGiveFromISR(xUSB_RX_Semaphore, &xHigherPriorityTaskWoken);
}

void ZeroPacketInitTask (void * pvArg)
{
	memset(USB.NOP_Buffer, 0, USB_PACKET_LEN);
	
	while(1)
	{
		short ADC_Res;
		
		vTaskDelay(250);
		
		rtc_gettime(&DateAndTime);
		ADC_Res = ADC_Get_Result(USB_VOLTAGE);
	
		/* Identification packet */
		USB.ZERO_Buffer[0] = 48;
		USB.ZERO_Buffer[1] = 49;
		USB.ZERO_Buffer[2] = 50;
		/* Errors report */
		USB.ZERO_Buffer[3] = 	Error.BMP180;
		USB.ZERO_Buffer[4] = 	Error.DB;
		USB.ZERO_Buffer[7] = 	Error.EEPROM;
		USB.ZERO_Buffer[8] = 	0;
		USB.ZERO_Buffer[9] = 	Error.MPU6050;
		USB.ZERO_Buffer[10] = Error.DB;
		USB.ZERO_Buffer[11] = Error.Reset;
		USB.ZERO_Buffer[12] = Error.SI7005;
		
		USB.ZERO_Buffer[13] = LevConfig.RelaysState;
		//USB.ZERO_Buffer[14] = Error.Flash;
		//USB.ZERO_Buffer[15] = Error.SD_Card;
		//USB.ZERO_Buffer[16] = Error.LOG;
		USB.ZERO_Buffer[17] = ADC_Res >> 8;
		USB.ZERO_Buffer[18] = ADC_Res;
		USB.ZERO_Buffer[19] = Error.FAT32;
		/* RTC Report */
		USB.ZERO_Buffer[20] = DateAndTime.year - 2000;
		USB.ZERO_Buffer[21] = DateAndTime.month;
		USB.ZERO_Buffer[22] = DateAndTime.mday;
		USB.ZERO_Buffer[23] = DateAndTime.wday;
		USB.ZERO_Buffer[24] = DateAndTime.hour;
		USB.ZERO_Buffer[25] = DateAndTime.min;
		USB.ZERO_Buffer[26] = DateAndTime.sec;
		//27-29 reserved
		
		/* Leveling report */
		V32.sVar[0] = (short)Position.D1;
		USB.ZERO_Buffer[30] = V32.bVar[1];
		USB.ZERO_Buffer[31] = V32.bVar[0];
		
		V32.sVar[0] = (short)Position.D2;
		USB.ZERO_Buffer[32] = V32.bVar[1];
		USB.ZERO_Buffer[33] = V32.bVar[0];	
	
		USB.ZERO_Buffer[34] = Position.LevelingIsOn;
		USB.ZERO_Buffer[35] = LevConfig.isFloatingModeOn;
		USB.ZERO_Buffer[36] = (LevConfig.delay * LevConfig.AutoOffTimerS / 1000) >> 8;
		USB.ZERO_Buffer[37] = LevConfig.delay * LevConfig.AutoOffTimerS / 1000;
		USB.ZERO_Buffer[38] = LevConfig.isLedTestEnabled;
		USB.ZERO_Buffer[39] = LevConfig.LedBrightness;

		V32.lVar = Error.I2C;
		USB.ZERO_Buffer[40] = V32.bVar[3];
		USB.ZERO_Buffer[41] = V32.bVar[2];
		USB.ZERO_Buffer[42] = V32.bVar[1];
		USB.ZERO_Buffer[43] = V32.bVar[0];

		xSemaphoreGive(xUSB_ZX_Semaphore);
	}
}

void USB_RX_DataHandler (void * pvArg)
{
	while(1)
	{
		if (xSemaphoreTake(xUSB_RX_Semaphore, portMAX_DELAY) == pdTRUE) 
			switch (USB.RX_Buffer[0])
			{
				case TurnOnLevelingSystem: 				ChangeLevelingState(ENABLE);				break;
				case TurnOffLevelingSystem: 			ChangeLevelingState(DISABLE);				break;
				case InstallNewSensor:						InstallNewSensorHandle();						break;
				case SetTime: 										USB_SetTime();											break;
				case CallibrateZeroPosition:			CallibrateZeroPositionHandle(); 		break;
				case DELETE_SENSOR_COMM:					DELETE_SENSOR_COMM_Handle();				break;
				case SET_LEDS_BRIGHTNESS:					SET_LEDS_BRIGHTNESS_Handle();				break;
				case SearchSensorsRoutine:				SEARCH_SENSORS_Handle();						break;
				case EnableFloatingMode:					EnableFloatMode(ENABLE);						break;
				case DisableFloatingMode:					EnableFloatMode(DISABLE);						break;
				case LED_TEST_ROUTINE:						LED_TEST_ROUTINE_Handle();					break;
				case ADDLEDSChangeState:					TurnOnAddLEDn(USB.RX_Buffer[1], USB.RX_Buffer[2], USB.RX_Buffer[3], USB.RX_Buffer[4]); break;
				case GET_INSTALLED_SENSORS:				GetInstalledSensors_Command();			break;
				case RelayChangeState:						LevConfig.RelaysState = USB.RX_Buffer[1]; break;
				case IR_COMMAND:									IR_Handle();												break;
				case ChangeSettingsCommand:				ChangeSettingsHandler();						break;
				case GetSettingsCommand:					GetSettingsHandler();								break;
				case SaveFileDefineNameCommand:		SaveFileHandler();									break;
				case PERFORM_SW_RESET:						NVIC_SystemReset();									break;
				case SaveFileDefineName:					DefineFileName(); 									break;
				case SaveFileFinishedCommand:			FileFinish();												break;
			}
	}
}

void USB_TX_DataHelper (void)
{
	char cnt, * buf;

	if (xQueueReceive(xUSB_TX_TextQueue, &buf, 0) == pdTRUE)
	{
		USB.TX_Buffer[0] = SendTextOverUSB_Command;
		strcpy((char*)USB.TX_Buffer + 1, buf);
		vPortFree(buf);
		xSemaphoreGive(xUSB_TX_Semaphore);
		return;
	}
	
	for (cnt = 0; cnt != MaxSensors; cnt++)
		if (SensList.OneWireSensors[cnt].isDataUpdated)
		{
				USB.TX_Buffer[0] = GetTempResultSenseNum;
				USB.TX_Buffer[1] = SensList.OneWireSensors[cnt].Err | (cnt << 4); //SNum + ErrCode
				USB.TX_Buffer[2] = SensList.OneWireSensors[cnt].Res[0]; //H temp data
				USB.TX_Buffer[3] = SensList.OneWireSensors[cnt].Res[1]; //L temp data
				USB.TX_Buffer[4] = 0;
				SensList.OneWireSensors[cnt].isDataUpdated = RESET;
				xSemaphoreGive(xUSB_TX_Semaphore);
				return;
		}
		
	if (SensList.intBMP180.isDataUpdated)
		{
			 USB.TX_Buffer[0] =  BMP180_ResultReady;
			 
			 V32.lVar = (long)SensList.intBMP180.Pressure;
			 USB.TX_Buffer[1] =  V32.bVar[3];
			 USB.TX_Buffer[2] =  V32.bVar[2];
			 USB.TX_Buffer[3] =  V32.bVar[1];
			 USB.TX_Buffer[4] =  V32.bVar[0];
			 
			 V32.lVar = (long)SensList.intBMP180.Altitude;
			 USB.TX_Buffer[5] =  V32.bVar[3];
			 USB.TX_Buffer[6] =  V32.bVar[2];
			 USB.TX_Buffer[7] =  V32.bVar[1];
			 USB.TX_Buffer[8] =  V32.bVar[0];

			 V32.sVar[0] = (short)SensList.intBMP180.mmHg;
			 USB.TX_Buffer[9] =  V32.bVar[1];
			 USB.TX_Buffer[10] = V32.bVar[0];

			 V32.lVar = (long)(SensList.intBMP180.Temperature * 100.0);
			 USB.TX_Buffer[11] = V32.bVar[3];
			 USB.TX_Buffer[12] = V32.bVar[2];
			 USB.TX_Buffer[13] = V32.bVar[1];
			 USB.TX_Buffer[14] = V32.bVar[0];
			 
			 SensList.intBMP180.isDataUpdated = RESET;
			 xSemaphoreGive(xUSB_TX_Semaphore);
			 return;
	}

	if (SensList.SensorsFound)
	{
			USB.TX_Buffer[0] = SearchSensorsRoutine;
			USB.TX_Buffer[1] = SensList.SensorsFound;
			for (cnt = 0; cnt != 8; cnt++)
				USB.TX_Buffer[cnt+2] = SensList.SearchedSensors[(SensList.SensorsFound - 1)][cnt];
			SensList.SensorsFound--;
			xSemaphoreGive(xUSB_TX_Semaphore);
			return;
	}

	if (SensList.SensorsIntalled)
	{
			USB.TX_Buffer[0] = GET_INSTALLED_SENSORS;
			USB.TX_Buffer[1] = SensList.SensorsIntalled;
			for (cnt = 0; cnt != 8; cnt++)
				USB.TX_Buffer[cnt+2] = SensList.SearchedSensors[(SensList.SensorsIntalled - 1)][cnt];
			SensList.SensorsIntalled--;
			xSemaphoreGive(xUSB_TX_Semaphore);
			return;
	}
}

void USB_TX_DataHandler (void * pvArg)
{
	while(1)
	{
		vTaskDelay(250);
		USB_TX_DataHelper();
	}
}

void USB_SetTime (void)
{
	DateAndTime.hour = 	USB.RX_Buffer[1];
	DateAndTime.min = 	USB.RX_Buffer[2];
	DateAndTime.sec = 	USB.RX_Buffer[3];
	DateAndTime.year = 	USB.RX_Buffer[4] + 2000;
	DateAndTime.month = USB.RX_Buffer[5];
	DateAndTime.mday = 	USB.RX_Buffer[6];
	DateAndTime.wday = 	USB.RX_Buffer[7];
	rtc_settime(&DateAndTime);
	PRINT("Time updated: %u:%u:%u", DateAndTime.hour, DateAndTime.min, DateAndTime.sec);
}

void InstallNewSensorHandle (void)
{
	char dat[2];
	char * buf;
	dat[0] = REG_NEW_SENSOR;
	dat[1] = USB.RX_Buffer[1];
	STM8_SendData(dat, 2);
	STM8_SendData((char*)(USB.RX_Buffer+2), 8);
	
	buf = pvPortMalloc(30);
	if (buf != NULL)
	{
		sprintf(buf, "Sensor added. S/N: %X%X%X%X%X%X%X%X\r\n%c", 
			USB.RX_Buffer[2], USB.RX_Buffer[3], USB.RX_Buffer[4], USB.RX_Buffer[5], 
			USB.RX_Buffer[6], USB.RX_Buffer[7], USB.RX_Buffer[8], USB.RX_Buffer[9], 0);
		PrintToFile(ErrLogFile, buf);
	}
	vPortFree(buf);
}

void DELETE_SENSOR_COMM_Handle (void)
{
	char dat[2];
	char * buf;
	dat[0] = DELETE_SENSOR;
	dat[1] = USB.RX_Buffer[1];
	STM8_SendData(dat, 2);

	buf = pvPortMalloc(40);
	if (buf != NULL)
	{
		sprintf(buf, "Sensor %d deleted.\r\n%c", dat[1], 0);
		PrintToFile(ErrLogFile, buf);
	}
	vPortFree(buf);	
}

void SET_LEDS_BRIGHTNESS_Handle (void)
{
	if (USB.RX_Buffer[1] > 100 || USB.RX_Buffer[1] < 4) return;
	LevConfig.LedBrightness = USB.RX_Buffer[1];
	TIM_SetCompare3(TIM2, USB.RX_Buffer[1]);
}

void SEARCH_SENSORS_Handle (void)
{
	char buf = SearchSensorsRoutine;
	STM8_SendData(&buf, 1);
}

void EnableFloatMode (char FloatState)
{
	extern KalmanFloatStructTypeDef Kx, Ky, Kz;
	if (FloatState)
	{
		/* Floating values rude mode */
		LevConfig.isFloatingModeOn = SET;
		Kx.K = LevConfig.Kr;
		Ky.K = LevConfig.Kr;
		Kz.K = LevConfig.Kr;
		Position.D1_zero_t = Position.D1_zero_def_r;
		Position.D2_zero_t = Position.D2_zero_def_r;
	}
	else
	{
		/* Accurate leveling values */
		LevConfig.isFloatingModeOn = RESET;
		Kx.K = LevConfig.Kp;
		Ky.K = LevConfig.Kp;
		Kz.K = LevConfig.Kp;
		Position.D1_zero_t = Position.D1_zero_def_p;
		Position.D2_zero_t = Position.D2_zero_def_p;
	}
	ChangeLevelingState(ENABLE);
}

void LED_TEST_ROUTINE_Handle (void)
{
	LevConfig.isLedTestEnabled = USB.RX_Buffer[1];
	ChangeLevelingState(ENABLE);
}

void TurnOnAddLEDn (char LedNum, char R, char G, char B)
{
	extern u8 SPI_ReadByte(SPI_TypeDef * SPIx, u8 byte);
	char cnt;
	for (cnt = 0; cnt != MaxADDLEDsAmount; cnt++)
	{
		if (cnt == LedNum)
		{
			SPI_ReadByte(AddLedsSPI, R);
			SPI_ReadByte(AddLedsSPI, G);
			SPI_ReadByte(AddLedsSPI, B);
		}
		else
		{
			SPI_ReadByte(AddLedsSPI, 0);
			SPI_ReadByte(AddLedsSPI, 0);
			SPI_ReadByte(AddLedsSPI, 0);			
		}
	}
}

void GetInstalledSensors_Command (void)
{
	char buf[1]; 
	buf[0] = GET_INSTALLED_SENSORS;
	STM8_SendData(buf, 1);
}

void IR_Handle (void)
{
	char datasize = USB.RX_Buffer[5];
	char buf[16];
	if (USB.RX_Buffer[5] > sizeof(buf)) return; //unsupported
	memcpy(buf, USB.RX_Buffer, datasize + 5);
	STM8_SendData(buf, datasize + 5);
}

void ChangeSettingsHandler (void)
{
	if (xTaskCreate(ChangeSettingsTask, NULL, 100, USB.RX_Buffer, tskIDLE_PRIORITY + 1 , NULL) != pdPASS)
		ErrorHandle(Task, "Change Settings Task: Not created! Not enought memory.\r\n");
}

void ChangeSettingsTask (void * pvArg)
{
	long * Arg = (long*) pvArg;
	char Pointer = (char)(Arg[0] >> 8);
	switch(Pointer)
	{
		case sAutoOffTimerValueS: Position.AutoOffTimerValueS = 	 Arg[1];					break;
		case sPositionNum:				LevConfig.PositionNum = 	 (char)Arg[1];					break;
		case sSI7005_Int:					LogConfig.SI7005_ReadInt = 	(int)Arg[1];					break;
		case sBMP180_Int:					LogConfig.BMP180_ReadInt = 	(int)Arg[1];					break;
		case sLogToSDint:					LogConfig.LogToSDint = 			(int)Arg[1];					break;
		case sNormZeroAreaSize:		Position.D1_zero_def_p = (float)Arg[1] / 1000;
															Position.D2_zero_def_p = (float)Arg[1] / 1000;		break;
		case sNormFilterValue:		LevConfig.Kp = (float)Arg[1] / 1000;							break;
		case sCoarseZeroAreaSize: Position.D1_zero_def_r = (float)Arg[1] / 1000;
															Position.D2_zero_def_r = (float)Arg[1] / 1000;		break;
		case sCoarseFilterValue:  LevConfig.Kr = (float)Arg[1] / 1000;							break;
	}
	UpdateDB();
	vTaskDelete( NULL );
}

void GetSettingsHandler (void)
{
	char i = 0;
	USB.TX_Buffer[i++] =  GetSettingsCommand;
	
	//1-4
	V32.lVar = Position.AutoOffTimerValueS;
	USB.TX_Buffer[i++] =  V32.bVar[3];
	USB.TX_Buffer[i++] =  V32.bVar[2];
	USB.TX_Buffer[i++] =  V32.bVar[1];
	USB.TX_Buffer[i++] =  V32.bVar[0];
	
	//5
	USB.TX_Buffer[i++] = LevConfig.PositionNum;

	//6-7
	USB.TX_Buffer[i++] = (char)(LogConfig.SI7005_ReadInt >> 8);
	USB.TX_Buffer[i++] = (char)(LogConfig.SI7005_ReadInt & 0xFF);
	
	//8-9
	USB.TX_Buffer[i++] = (char)(LogConfig.BMP180_ReadInt >> 8);
	USB.TX_Buffer[i++] = (char)(LogConfig.BMP180_ReadInt & 0xFF);

	//10-11
	USB.TX_Buffer[i++] = (char)(LogConfig.LogToSDint >> 8);
	USB.TX_Buffer[i++] = (char)(LogConfig.LogToSDint & 0xFF);
	
	//12-15
	V32.lVar = (long)(Position.D1_zero_def_p * 1000);
	USB.TX_Buffer[i++] =  V32.bVar[3];
	USB.TX_Buffer[i++] =  V32.bVar[2];
	USB.TX_Buffer[i++] =  V32.bVar[1];
	USB.TX_Buffer[i++] =  V32.bVar[0];
	
	//16-19
	V32.lVar = (long)(LevConfig.Kp * 1000);
	USB.TX_Buffer[i++] =  V32.bVar[3];
	USB.TX_Buffer[i++] =  V32.bVar[2];
	USB.TX_Buffer[i++] =  V32.bVar[1];
	USB.TX_Buffer[i++] =  V32.bVar[0];
	
	//20-23
	V32.lVar = (long)(Position.D1_zero_def_r * 1000);
	USB.TX_Buffer[i++] =  V32.bVar[3];
	USB.TX_Buffer[i++] =  V32.bVar[2];
	USB.TX_Buffer[i++] =  V32.bVar[1];
	USB.TX_Buffer[i++] =  V32.bVar[0];
	
	//24-27
	V32.lVar = (long)(LevConfig.Kr * 1000);
	USB.TX_Buffer[i++] =  V32.bVar[3];
	USB.TX_Buffer[i++] =  V32.bVar[2];
	USB.TX_Buffer[i++] =  V32.bVar[1];
	USB.TX_Buffer[i++] =  V32.bVar[0];

	xSemaphoreGive(xUSB_TX_Semaphore);
}

void SaveFileHandler (void)
{
	SaveFileTask(NULL);
}

void SaveFileTask (void * pvArg)	//this not a task. Just a void to save file without args
{
	extern SemaphoreHandle_t xSD_Semaphore;
	char * Buffer = pvPortMalloc(USB.RX_Buffer[1]);
	if (xSemaphoreTake (xSD_Semaphore, portMAX_DELAY) == pdTRUE)
	{		
		if (USB.RX_Buffer[1] && Buffer)
		{
			char i;
			UINT BytesSend;
			for (i = 0; i < USB.RX_Buffer[1]; i++)
				Buffer[i] = USB.RX_Buffer[i + 2];
			f_lseek(&FileObject, f_size(&FileObject));
			f_write(&FileObject, Buffer, USB.RX_Buffer[1], &BytesSend);
		}
		xSemaphoreGive(xSD_Semaphore);
	}
	vPortFree(Buffer);
	//vTaskDelete( NULL );
}

void DefineFileName (void)
{
	char i;
	extern SemaphoreHandle_t xSD_Semaphore;
	extern FATFS fs;
	if (USB.RX_Buffer[1] >= sizeof(FileName)) return;
	
	if (xSemaphoreTake (xSD_Semaphore, portMAX_DELAY) == pdTRUE)
	{
		for (i = 0; i < USB.RX_Buffer[1]; i++)	
			FileName[i] = USB.RX_Buffer[i + 2];
		
		//disk_initialize(0);
		f_mount(&fs, "", 1);
		f_unlink(FileName);
		f_open(&FileObject, FileName, FA_CREATE_NEW | FA_WRITE);
		xSemaphoreGive(xSD_Semaphore);
	}
}

void FileFinish (void)
{
	f_close(&FileObject);
}
