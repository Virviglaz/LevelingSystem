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
#include "simple_db.h"
#include "CRC.h"

/* Internal functions */
char ValidateValue (float value, float low_lim, float hi_lim);
char ValidateDB (void);

/* Extern variables */
extern LevelingConfigStructTypeDef LevConfig;
extern PositionTypeDef Position;
extern EEPROM_StructTypeDef EEPROM_Struct;
extern MPU6050_ZeroCalTypeDef MPU6050_ZeroCal;
extern RTC_CorrectorStructTypeDef RTC_C;
extern LogConfStructTypeDef LogConfig;
extern char * ErrLogFile;
			char DB_FileName[] = 					"DB.BIN";
const char LevConfTagName[] = 			"LevConfig";
const char PositionTagName[] = 			"Position";
const char RTC_CorrectorTagName[] = "ClockCorr";
const char LogConfigTagName[]			= "LogConfig";
const char MPU6050_TagName[] = 			"AccellZeroCal";

/* Default values */
const LevelingConfigStructTypeDef LevConfigDefault = {0.1, 0.25, 100, 100, 0, 3600, 0,0,0};
const PositionTypeDef PositionDefault = {0,0,0,0,0, 0,0, 30,30, 30,30, 50,50, 1,3600};
const LogConfStructTypeDef LogConfigDefault = {5, 5, 60};
const RTC_CorrectorStructTypeDef RTC_C_Default = {0, 0, 0, 0};
const int MaxDB_size = 300;

uint32_t CRC_Func(void * buf, uint32_t size)
{
	return crc32(buf, size);
}

LocalDB_ErrorTypeDef StoreDB (LogSourceTypeDef Source)
{
	LocalDB_ErrorTypeDef Result = Err_DB_Success;
	char * db;
	int db_size = MaxDB_size;								
	db = pvPortMalloc(db_size + 20);
	if (db == NULL) return Err_OutOfMemory;
	memset(db, 0, db_size);
	
	SimpleDB.Init(CRC_Func);
	
	Result = (LocalDB_ErrorTypeDef)  SimpleDB.Write(LevConfTagName, (char*)&LevConfig, sizeof(LevConfig), DB_8b, db);								//24
	Result |= (LocalDB_ErrorTypeDef) SimpleDB.Write(PositionTagName, (char*)&Position, sizeof(Position), DB_8b, db);								//44
	Result |= (LocalDB_ErrorTypeDef) SimpleDB.Write(RTC_CorrectorTagName, (char*)&RTC_C, sizeof(RTC_C), DB_8b, db);								//16
	Result |= (LocalDB_ErrorTypeDef) SimpleDB.Write(LogConfigTagName, (char*)&LogConfig, sizeof(LogConfig), DB_8b, db);						//8
	Result |= (LocalDB_ErrorTypeDef) SimpleDB.Write(MPU6050_TagName, (char*)&MPU6050_ZeroCal, sizeof(MPU6050_ZeroCal), DB_8b, db);
	
  db_size = SimpleDB.GetSize(db);
	
	if (!Result)
	{		
		if (Source == LogSourceEEPROM)
		{
			EEPROM_Struct.isWriting = 1;
			EEPROM_Struct.buf = (uint8_t*)db;
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

LocalDB_ErrorTypeDef RestoreDB (LogSourceTypeDef Source)
{
	LocalDB_ErrorTypeDef Result = Err_DB_Success;
	char * db;
	long db_size;
	long BytesReaded;
	
	SimpleDB.Init(CRC_Func);
	
	/* Allocate memory for db */
	db_size = MaxDB_size;
	db = pvPortMalloc(db_size);
	if (db == NULL) return Err_OutOfMemory;
	memset(db, 0, MaxDB_size);
	
	/* Fetch db */
	if (Source == LogSourceEEPROM)
	{	
		EEPROM_Struct.isWriting = 0;
		EEPROM_Struct.buf = (uint8_t*)db;
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
	
	if (Result == DB_Success)
		if (SimpleDB.GetSize(db) == 0 || SimpleDB.GetSize(db) > MaxDB_size)
			Result = Err_ValidationFailed;
		
	if (SimpleDB.Validate(db))
			Result = Err_ValidationFailed;
	
	if (Result == DB_Success)
	{		

			Result = (LocalDB_ErrorTypeDef)SimpleDB.Read(LevConfTagName, (char*)&LevConfig, db);		//24
			Result |= (LocalDB_ErrorTypeDef)SimpleDB.Read(PositionTagName, (char*)&Position, db);		//44
			Result |= (LocalDB_ErrorTypeDef)SimpleDB.Read(RTC_CorrectorTagName, (char*)&RTC_C, db);	//16	
			Result |= (LocalDB_ErrorTypeDef)SimpleDB.Read(LogConfigTagName, (char*)&LogConfig, db);	//8
			Result |= (LocalDB_ErrorTypeDef)SimpleDB.Read(MPU6050_TagName, (char*)&MPU6050_ZeroCal, db);
			PrintIntDataToFile(ErrLogFile, "DB size: ", (int)SimpleDB.GetSize(db), "bytes");
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
