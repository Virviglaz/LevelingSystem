typedef enum
{
	Reset = 0,
	I2C,
	MPU6050,
	SI7005,
	EEPROM,
	BMP180,
	Config,
	DB_Err,
	Flash,
	SDMMC,
	LOG,
	FS,
	Mutex,
	Task,
	Memory,
	OneWireSensors
}ErrorGroupTypeDef;

enum OneWireErrors
{
	One_Wire_Success = 0,
	One_Wire_Error_No_Echo,
	One_Wire_Bus_Low_Error,
	One_Wire_Device_Busy,
	One_Wire_CRC_Error
};

typedef struct
{
	char TaskCreate;
	char MPU6050;
	char BMP180;
	char BME280;
	char SI7005;
	char SD_Card;
	char FAT32;
	char Reset;
	char EEPROM;
	char DB;
	long I2C;
}ErrorTypeDef;

typedef enum 
{
	LOW_POWER = 0,
	WINDOW_WATCHDOG,
	INDEPENDANT_WATCHDOG,
	SOFTWARE,
	POD_PDR,
	NRST_PIN,
	UNKNOWN
}RESET_Result;

void ErrorHandle (ErrorGroupTypeDef Group, const char * Error);
