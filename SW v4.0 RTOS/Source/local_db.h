typedef enum
{
	LogSourceSDcard,
	LogSourceEEPROM
}LogSourceTypeDef;

typedef enum
{
	Err_DB_Success = 					0,
	Err_DB_WrongCRC = 				1,
  Err_DB_TagNotFound = 			2,
  Err_DB_TagExist = 				3,
	Err_DB_NoCRC_Func = 			4,
	Err_EEPROM_WriteError = 	5,
	Err_EEPROM_ReadError = 		6,
	Err_SDCard_WriteError = 	7,
	Err_SDCard_ReadError = 		8,
	Err_SDCard_DB_SizeError = 9,
	Err_CRC_Mismatch = 				10,
	Err_OutOfMemory = 				11,
	Err_ValidationFailed = 		12
}LocalDB_ErrorTypeDef;

LocalDB_ErrorTypeDef StoreDB (LogSourceTypeDef Source);
LocalDB_ErrorTypeDef RestoreDB (LogSourceTypeDef Source);
void UpdateDB (void);
void RollBackToDefaultValues (void);
