#include "local_db.h"
#include "data_collector.h"
#include "data_logger.h"
#include "error_collector.h"
#include "stm32f10x_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "HW.h"
#include "EEPROM.h"
#include "MPU6050.h"
#include <string.h>
#include "db.h"

/* Internal functions */
char ValidateValue (float value, float low_lim, float hi_lim);
char ValidateDB (void);

/* Extern variables */
extern LevelingConfigStructTypeDef LevConfig;
extern PositionTypeDef Position;
extern EEPROM_StructTypeDef EEPROM_Struct;
extern MPU6050_StructTypeDef MPU6050_Struct;
extern RTC_CorrectorStructTypeDef RTC_C;
extern LogConfStructTypeDef LogConfig;
extern char * ErrLogFile;
			char DB_FileName[] = 					"DB.BIN";
const char LevConfTagName[] = 			"LevConfig";
const char PositionTagName[] = 			"Position";
const char RTC_CorrectorTagName[] = "ClockCorr";
const char LogConfigTagName[]			= "LogConfig";
const char MPU6050_TagName[] = 			"Acc";

/* Default values */
const LevelingConfigStructTypeDef LevConfigDefault = {0.1, 0.25, 100, 100, 0, 3600, 0,0,0};
const PositionTypeDef PositionDefault = {0,0,0,0,0, 0,0, 30,30, 30,30, 50,50, 1,3600};
const LogConfStructTypeDef LogConfigDefault = {5, 5, 60};
const RTC_CorrectorStructTypeDef RTC_C_Default = {0, 0, 0, 0};
const int MaxDB_size = 300;

DB_ErrorTypeDef StoreDB (LogSourceTypeDef Source)
{
	DB_ErrorTypeDef Result = Err_DB_Success;
	char * db;
	int db_size = MaxDB_size;								
	db = pvPortMalloc(db_size + 20);
	if (db == NULL) return Err_OutOfMemory;
	memset(db, 0, db_size);
	Result = (DB_ErrorTypeDef) dbStoreData(LevConfTagName, (char*)&LevConfig, sizeof(LevConfig), db);								//24
	Result |= (DB_ErrorTypeDef) dbStoreData(PositionTagName, (char*)&Position, sizeof(Position), db);								//44
	Result |= (DB_ErrorTypeDef) dbStoreData(RTC_CorrectorTagName, (char*)&RTC_C, sizeof(RTC_C), db);								//16
	Result |= (DB_ErrorTypeDef) dbStoreData(LogConfigTagName, (char*)&LogConfig, sizeof(LogConfig), db);						//8
	Result |= (DB_ErrorTypeDef) dbStoreData(MPU6050_TagName, (char*)&MPU6050_Struct, sizeof(MPU6050_Struct), db);		//52
	
  db_size = dbGetSize(db);
	
	if (!Result)
	{		
		if (Source == LogSourceEEPROM)
		{
			EEPROM_Struct.isWriting = 1;
			EEPROM_Struct.buf = db;
			EEPROM_Struct.Mem_adrs = 0;
			EEPROM_Struct.size = db_size;
			if (EEPROM_RW(&EEPROM_Struct)) 
				Result = Err_EEPROM_WriteError;
		}
		else 	
			if (LogToSD(DB_FileName, db, db_size, 1))
				Result = Err_SDCard_WriteError;			
	}
	vPortFree(db);
	return Result;
}

DB_ErrorTypeDef RestoreDB (LogSourceTypeDef Source)
{
	static DB_ErrorTypeDef Result;
	
	char * db;
	long db_size;
	long BytesReaded;
	
	Result = Err_DB_Success;
	
	/* Allocate memory for db */
	db_size = MaxDB_size;
	db = pvPortMalloc(db_size);
	if (db == NULL) return Err_OutOfMemory;
	
	/* Fetch db */
	if (Source == LogSourceEEPROM)
	{	
		EEPROM_Struct.isWriting = 0;
		EEPROM_Struct.buf = db;
		EEPROM_Struct.Mem_adrs = 0;
		EEPROM_Struct.size = db_size;
		if (EEPROM_RW(&EEPROM_Struct))
			Result = Err_EEPROM_ReadError;
	}
	else
	{		
		if (ReadFromSD(DB_FileName, db, (unsigned int*)&BytesReaded))
			Result = Err_SDCard_ReadError;
	}	
	
	if (!Result)
	{
		if (dbCheckCRC(db) == DB_Success)
		{
			Result = (DB_ErrorTypeDef)dbReadData(LevConfTagName, (char*)&LevConfig, &BytesReaded, db);		//24
			Result |= (DB_ErrorTypeDef)dbReadData(PositionTagName, (char*)&Position, &BytesReaded, db);		//44
			Result |= (DB_ErrorTypeDef)dbReadData(RTC_CorrectorTagName, (char*)&RTC_C, &BytesReaded, db);	//16	
			Result |= (DB_ErrorTypeDef)dbReadData(LogConfigTagName, (char*)&LogConfig, &BytesReaded, db);	//8
			Result |= (DB_ErrorTypeDef)dbReadData(MPU6050_TagName, (char*)&MPU6050_Struct, &BytesReaded, db);
			PrintIntDataToFile(ErrLogFile, "DB size: ", (int)dbGetSize(db), "bytes");
			/*if (ValidateDB())
			{
				Result = Err_ValidationFailed;
				PrintIntDataToFile(ErrLogFile, "DB Validation Error! ", Result, " ");
				RollBackToDefaultValues();
			}*/
		}	
		else
			Result = Err_DB_WrongCRC;
	}	
	else
		PrintIntDataToFile(ErrLogFile, "DB Error: ", Result, " ");
	
	vPortFree(db);
	return Result;
}

void UpdateDB (void)
{
	extern ErrorTypeDef Error;
	Error.EEPROM = StoreDB(LogSourceEEPROM);
	Error.DB = StoreDB(LogSourceSDcard);
	if (Error.EEPROM) PrintIntDataToFile(ErrLogFile, "Error saving DB to EEPROM.  Error num:", (int) Error.EEPROM, " ");
	if (Error.DB) 		PrintIntDataToFile(ErrLogFile, "Error saving DB to SD card. Error num:", (int) Error.DB, " ");
}

void RollBackToDefaultValues (void)
{
	memcpy(&LevConfig, &LevConfigDefault, sizeof(LevConfigDefault));
	memcpy(&Position, &PositionDefault, sizeof(PositionDefault));
	memcpy(&LogConfig, &LogConfigDefault, sizeof(LogConfigDefault));
	memcpy(&RTC_C, &RTC_C_Default, sizeof(RTC_C_Default));
}

char ValidateValue (float value, float low_lim, float hi_lim)
{
	if (value > hi_lim || value < low_lim) return 1;
	return 0;
}

char ValidateDB (void)
{
	static char Result = 0;
	Result |= ValidateValue(LevConfig.Kp, 0, 1);
	Result |= ValidateValue(LevConfig.Kr, 0, 1);
	Result |= ValidateValue(LevConfig.delay, 10, 1000);
	Result |= ValidateValue(LevConfig.LedBrightness, 0, 100);
	Result |= ValidateValue(LevConfig.PositionNum, 0, 16);
	Result |= ValidateValue(LevConfig.AutoOffTimerS, 0, 1000000);
	
	Result |= ValidateValue(Position.D1_zero_t, 0, 1000);
	Result |= ValidateValue(Position.D2_zero_t, 0, 1000);
	Result |= ValidateValue(Position.D1_zero_def_p, 0, 1000);
	Result |= ValidateValue(Position.D2_zero_def_p, 0, 1000);
	Result |= ValidateValue(Position.D1_zero_def_r, 0, 1000);
	Result |= ValidateValue(Position.D2_zero_def_r, 0, 1000);

	Result |= ValidateValue(RTC_C.Result, -1000, +1000);

	return Result;
}
