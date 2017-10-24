#include "data_logger.h"
#include "data_collector.h"
#include "error_collector.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "diskio.h"
#include "sd_spi_stm32.h"
#include "ff.h"
#include "diskio.h"
#include "rtc.h"
#include "MPU6050.h"
#include "BMP180.h"
#include "SI7005.h"
#include <stdio.h>
#include <string.h>
#include "HW.h"

/* Variables */
SemaphoreHandle_t xSD_Semaphore;
FATFS fs;
FRESULT res;
FIL fsrc;
DSTATUS SDMMC_Status;
FRESULT fr;    /* FatFs return code */
FIL fil;       /* File object */
FILINFO fno;
RTC_t DateAndTime;
extern LevelingConfigStructTypeDef LevConfig;
extern LogConfStructTypeDef LogConfig;
extern SensorListStructTypeDef SensList;
extern char * DataLogFile;
extern char * CSVLogFile;

char LoggerInit (void)
{
	xSD_Semaphore = xSemaphoreCreateMutex();
	if (xSD_Semaphore) return 0;
	return 1;
}

char DeleteFile (char * filename)
{
	char Result;
	xSemaphoreTake (xSD_Semaphore, portMAX_DELAY);
	Result = (char)f_unlink(filename);
	xSemaphoreGive(xSD_Semaphore);
	return Result;
}

char LogToSD (char * filename, char * buf, unsigned int size, char CreateNew)
{
	extern ErrorTypeDef Error;
	unsigned int BytesSend;
	
	if (xSemaphoreTake (xSD_Semaphore, portMAX_DELAY) != pdTRUE)
		return 1;
	
	SDMMC_Status = disk_initialize(0);
	Error.SD_Card |= SDMMC_Status;
	if (SDMMC_Status == RES_OK) 
	{
		f_mount(&fs, "", 1);
		
		if (CreateNew && f_stat(filename, &fno) == FR_OK)
			f_unlink(filename);
		
		if (f_stat(filename, &fno) == FR_NO_FILE)
			fr = f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE);
		else
			fr = f_open(&fil, filename, FA_WRITE);
		f_lseek(&fil, f_size(&fil));
		fr = f_write(&fil, buf, size, &BytesSend);
		f_close(&fil);	
		Error.FAT32 |= fr;
	}
	xSemaphoreGive(xSD_Semaphore);
	return (BytesSend == size) ? 0 : 1;
}

char PrintToFile (char * filename, char * buf)
{
	char Res;
	Res = LogToSD(filename, buf, strlen(buf), 0);
	return Res;
}

char PrintFloatDataToFile (char * filename, const char * Text, float Data, const char * units)
{
	char * buf;
	char * DateStamp;
	char Result;
	buf = pvPortMalloc(30 + 7 + strlen((char*)Text) + strlen((char*)units));
	DateStamp = pvPortMalloc(30);
	DateAndTimePrint(DateStamp);
	sprintf(buf, "%s %s %.2f %s \r\n%c", DateStamp, Text, Data, units, 0);
	Result = LogToSD(filename, buf, strlen(buf), 0);
	vPortFree(DateStamp);
	vPortFree(buf);
	return Result;
}

char PrintIntDataToFile (char * filename, const char * Text, int Data, const char * units)
{
	char * buf;
	char * DateStamp;
	char Result;
	buf = pvPortMalloc(30 + 7 + strlen((char*)Text) + strlen((char*)units));
	DateStamp = pvPortMalloc(30);
	DateAndTimePrint(DateStamp);
	sprintf(buf, "%s %s %i %s \r\n%c", DateStamp, Text, Data, units, 0);
	Result = LogToSD(filename, buf, strlen(buf), 0);
	vPortFree(DateStamp);
	vPortFree(buf);
	return Result;
}

char PrintFloatResultToFile (char * filename, const char * Text, char num, float Data, const char * units)
{
	char * buf;
	char * DateStamp;
	char Result;
	buf = pvPortMalloc(35 + 7 + strlen((char*)Text) + strlen((char*)units));
	DateStamp = pvPortMalloc(30);
	DateAndTimePrint(DateStamp);
	sprintf(buf, "%s %s %d %.2f %s \r\n%c", DateStamp, Text, num, Data, units, 0);
	Result = LogToSD(filename, buf, strlen(buf), 0);
	vPortFree(DateStamp);
	vPortFree(buf);
	return Result;
}

char PrintIntResultToFile (char * filename, const char * Text, char num, int Data, const char * units)
{
	char * buf;
	char * DateStamp;
	char Result;
	buf = pvPortMalloc(35 + 7 + strlen((char*)Text) + strlen((char*)units));
	DateStamp = pvPortMalloc(30);
	DateAndTimePrint(DateStamp);
	sprintf(buf, "%s %s %d %i %s \r\n%c", DateStamp, Text, num, Data, units, 0);
	Result = LogToSD(filename, buf, strlen(buf), 0);
	vPortFree(DateStamp);
	vPortFree(buf);
	return Result;
}

void DateAndTimePrint (char * string)
{
	rtc_gettime(&DateAndTime);

	sprintf(string, "%02d.%02d.%4d %02d:%02d:%02d %c", 
	DateAndTime.mday, 
	DateAndTime.month, 
	DateAndTime.year, 
	DateAndTime.hour, 
	DateAndTime.min, 
	DateAndTime.sec, 0);
}

void DatePrint (char * string)
{
	rtc_gettime(&DateAndTime);

	sprintf(string, "%02d_%02d_%2d%c", 
	DateAndTime.mday, 
	DateAndTime.month, 
	DateAndTime.year - 2000, 0);
}

char ReadFromSD (const char * filename, char * buf, unsigned int * ByteReaded)
{
	extern ErrorTypeDef Error;
	char Result;
	xSemaphoreTake (xSD_Semaphore, portMAX_DELAY);
	SDMMC_Status = disk_initialize(0);
	Error.SD_Card |= SDMMC_Status;
	Result = SDMMC_Status;
	if (SDMMC_Status == RES_OK)
	{
		f_mount(&fs, "", 1);
		if (f_stat(filename, &fno) == FR_NO_FILE)
			Result = (char)FR_NO_FILE;
		else
		{
			fr = f_open(&fil, filename, FA_READ);
			fr = f_read(&fil, buf, f_size(&fil), ByteReaded);
		}
		f_close(&fil);	
	}
	
	xSemaphoreGive(xSD_Semaphore);
	return Result;
}

void LogToSD_Task (void * pvArg)
{
	char cnt;
	const char Csign[] = {0xB0, 'C'};
	const char HumU[] = {0x25};
	float TempConvRes;
	static char csv_out[10];
	static char txtlogDate[15];
	//char csvLogDate[15];
	static char csvDateAndTime[20];
	union
	{
		short Svar;
		char Bytes[2];
	} W16;

	while(1)
	{
		vTaskDelay(LogConfig.LogToSDint * 1000);	
		DatePrint(txtlogDate);
		//DatePrint(csvLogDate);
		memcpy(txtlogDate + 8, ".TXT\0", 5);
		//memcpy(csvLogDate + 8, ".CSV\0", 5);
		
		DateAndTimePrint(csvDateAndTime);
		csvDateAndTime[strlen(csvDateAndTime) - 1] = ',';
		csvDateAndTime[10] = ',';
		LogToSD(CSVLogFile, csvDateAndTime, strlen(csvDateAndTime), 0);	
		
		for (cnt = 0; cnt != 10; cnt++)
			if (SensList.OneWireSensors[cnt].isDataNotLogged && !SensList.OneWireSensors[cnt].Err)
			{
				W16.Bytes[0] = SensList.OneWireSensors[cnt].Res[1];
				W16.Bytes[1] = SensList.OneWireSensors[cnt].Res[0];
				TempConvRes = (float)W16.Svar / 16;
				PrintFloatResultToFile(txtlogDate, "Sensor", cnt, TempConvRes, Csign);	
				
				sprintf(csv_out, "%.2f,%s,%c", TempConvRes, Csign, 0);
				LogToSD(CSVLogFile, csv_out, strlen(csv_out), 0);					
				
				SensList.OneWireSensors[cnt].isDataNotLogged = RESET;
			}
			
		if (SensList.OneWireSensors[SI7005_SensorNumberINT].isDataNotLogged)
		{
			PrintIntDataToFile(txtlogDate, "Internal humidity sensor", SensList.OneWireSensors[SI7005_SensorNumberINT].Res[1], HumU);
			SensList.OneWireSensors[SI7005_SensorNumberINT].isDataNotLogged = RESET;
		}

		if (SensList.OneWireSensors[Humidity_EXT1_Number].isDataNotLogged)
		{
			PrintIntDataToFile(txtlogDate, "External humidity sensor 1", SensList.OneWireSensors[Humidity_EXT1_Number].Res[1], HumU);
			SensList.OneWireSensors[Humidity_EXT1_Number].isDataNotLogged = RESET;
		}

		if (SensList.OneWireSensors[Humidity_EXT2_Number].isDataNotLogged)
		{
			PrintIntDataToFile(txtlogDate, "External humidity sensor 2", SensList.OneWireSensors[Humidity_EXT2_Number].Res[1], HumU);
			SensList.OneWireSensors[Humidity_EXT2_Number].isDataNotLogged = RESET;
		}
		
		if (SensList.intBMP180.isDataNotLogged)
		{
			PrintIntDataToFile(txtlogDate,   "Internal pressure sensor", SensList.intBMP180.Pressure, "Pa");
			PrintIntDataToFile(txtlogDate,   "Internal pressure sensor", SensList.intBMP180.mmHg, "mmHg");
			PrintFloatDataToFile(txtlogDate, "Internal temperature", SensList.intBMP180.Temperature, Csign);
			PrintFloatDataToFile(txtlogDate, "Internal altitude", SensList.intBMP180.Altitude, "m");	
			SensList.intBMP180.isDataNotLogged = RESET;
		}
				
		/* CSV print humidity */
		sprintf(csv_out, "%d,%s,%c", SensList.OneWireSensors[SI7005_SensorNumberINT].Res[1], "%", 0);
		LogToSD(CSVLogFile, csv_out, strlen(csv_out), 0);	
		
		if (SensList.OneWireSensors[Humidity_EXT1_Number].Res[1])
		{
			sprintf(csv_out, "%d,%s,%c", SensList.OneWireSensors[Humidity_EXT1_Number].Res[1], "%", 0);
			LogToSD(CSVLogFile, csv_out, strlen(csv_out), 0);
		}

		if (SensList.OneWireSensors[Humidity_EXT2_Number].Res[1])
		{
			sprintf(csv_out, "%d,%s,%c", SensList.OneWireSensors[Humidity_EXT2_Number].Res[1], "%", 0);
			LogToSD(CSVLogFile, csv_out, strlen(csv_out), 0);
		}

		/* CSV print mmHg */
		sprintf(csv_out, "%d,%s,%c", SensList.intBMP180.mmHg, "mmHg", 0);
		LogToSD(CSVLogFile, csv_out, strlen(csv_out), 0);			

		/* CSV print internal temperature */
		sprintf(csv_out, "%.2f,%s%c", SensList.intBMP180.Temperature, Csign, 0);
		LogToSD(CSVLogFile, csv_out, strlen(csv_out), 0);		

		LogToSD(CSVLogFile, "\r\n", 2, 0);
	}
}
