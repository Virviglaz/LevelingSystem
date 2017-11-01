#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "HW.h"
#include "stm32_GPIO.h"

typedef struct
{
	/* actual position */
	float x,y,z,D1,D2;		
	
	/* blink freq */
	int D1_DelayBlink, D2_DelayBlink;
	
	/* zero point accuracy */
	float D1_zero_t, D2_zero_t;
	
	/* default values for precise and rude leveling */
	float D1_zero_def_p, D2_zero_def_p;
	float D1_zero_def_r, D2_zero_def_r;
	
	/* Leveling on flag */
	char LevelingIsOn;
	
	/* Auto off timer in seconds */
	long AutoOffTimerValueS;
}PositionTypeDef;

typedef struct 
{
	GPIO_TypeDef * GPIOx;
	u16 PINx;
}PIN_TypeDef;

/*typedef struct
{
	PIN_TypeDef * D_Pins;
	
}LebBlinkTypeDef;*/

typedef struct
{
	/* Data */
	float Kp, Kr;
	int delay;
	char LedBrightness;
	char PositionNum;
	long AutoOffTimerS;
	char isFloatingModeOn;
	char isLedTestEnabled;
	char RelaysState;
	
	/* Functions */
	void (*PosCalc)(PositionTypeDef * LevConfig);
}LevelingConfigStructTypeDef;

typedef struct
{
	char Buffer[100];
	volatile char BufferIndex;
	volatile char DataReady;
}UartRXTypeDef;

typedef struct
{
	//char SN[8];
	char Res[2];
	char Err;
	char isDataUpdated;
	char isDataNotLogged;
}OneWireSensorStructTypeDef;

typedef struct
{
	char Humidity;
	char Err;
	char isDataUpdated;
	char isDataNotLogged;
}SI7005_SensorStructTypeDef;

typedef struct
{
	long Pressure;
	float Altitude;
	float Temperature;
	char isDataUpdated;
	char isDataNotLogged;
	int mmHg;
}BMP180_SensorStructTypeDef;

typedef struct
{
	OneWireSensorStructTypeDef OneWireSensors[MaxSensors];
	float intMPU6050_Temp;
	long  intBMP180_Press;
	char SensorsFound;
	char SensorsIntalled;
	char SearchedSensors[MaxSensors][8];
	SI7005_SensorStructTypeDef  intSI7005_Humdt;
	SI7005_SensorStructTypeDef  extSI7005_Humdt[2];
	BMP180_SensorStructTypeDef	intBMP180;
}SensorListStructTypeDef;

typedef struct
{
	long InitialValue;
	volatile long EndValue;
	volatile long SecondsToMeasure;
	long Result;
}RTC_CorrectorStructTypeDef;

typedef struct
{
	int SI7005_ReadInt;
	int BMP180_ReadInt;
	int LogToSDint;
}LogConfStructTypeDef;

void InitHW (void);
long AutoOffTimerInit (void);
short ADC_Get_Result (char channel);
void CallibrateZeroPositionHandle (void);
void STM8_SendData (char * buf, char size);

#endif
