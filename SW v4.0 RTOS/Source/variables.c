#include "data_logger.h"
#include "data_collector.h"
#include "error_collector.h"
#include "FreeRTOS.h"
#include "MPU6050.h"
#include "BMP180.h"
#include "SI7005.h"
#include "SW_I2C_Driver.h"
#include "EEPROM.h"
#include "kalman.h"
#include "BME280.h"

char * ErrLogFile  = 	{"ERRLOG.TXT"};
char * DataLogFile = 	{"LOG.TXT"};
char * CSVLogFile  =  {"LOG.CSV"};

LevelingConfigStructTypeDef LevConfig = {0.01, 0.1, 100, 100, 0, 3600, 0,0,0};
LogConfStructTypeDef LogConfig = {5, 5, 60};
RTC_CorrectorStructTypeDef RTC_C = {0,0,0,0};

SW_I2C_DriverStructTypeDef I2C_Struct;
BMP180_StructTypeDef BMP180_Struct;
SI7005_StructTypeDef SI7005_Struct;
EEPROM_StructTypeDef EEPROM_Struct;
BME280_StructTypeDef BME280_Struct;
