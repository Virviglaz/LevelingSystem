#include "Leveling.h"
#include "MPU6050.h"
#include "data_collector.h"
#include "kalman.h"
#include "Positions.h"
#include "stm32f10x_tim.h"
#include "data_logger.h"
#include "error_collector.h"
#include "local_db.h"

/* RTOS Specific include */
#include "FreeRTOS.h"
#include "task.h"

/* Local variables */
PositionTypeDef Position = {0,0,0,0,0, 0,0, 50,50, 50,50, 100,100, 1,3600};
KalmanFloatStructTypeDef Kx, Ky, Kz;

/* Local pointers */
MPU6050_StructTypeDef * MPU6050_Struct;
MPU6050_ResultTypeDef * MPU6050_Result;
MPU6050_ZeroCalTypeDef MPU6050_ZeroCal;

/* Local functions */
void LevelingTask (void * pvArg);

uint8_t LevelingInit (void)
{
	/* Extern functions */
	extern uint16_t MPU6050_CheckReadyPin (void);
	extern uint8_t I2C_WriteReg (uint8_t I2C_Adrs, uint8_t Reg, uint8_t Value);
	extern uint8_t I2C_ReadReg  (uint8_t I2C_Adrs, uint8_t Reg, uint8_t * buf, uint16_t size);
	extern TaskHandle_t vLevelingTaskHandler;

	static uint8_t Result;
	
	MPU6050_Struct = pvPortMalloc(sizeof(* MPU6050_Struct));
	
	// Init accelerometer
	MPU6050_Struct->delay_func = vTaskDelay;
	MPU6050_Struct->ReadReg = I2C_ReadReg;
	MPU6050_Struct->WriteReg = I2C_WriteReg;
	MPU6050_Struct->I2C_Adrs = MPU6050_I2C_AddressOnBoard;
	MPU6050_Struct->GyroScale = GYRO_0250d_s;
	MPU6050_Struct->AccelScale = Scale_2g;
	MPU6050_Struct->FilterOrder = 6;
	MPU6050_Struct->GyroSampleRateHz = 10;
	MPU6050_Struct->CheckRDY_pin = MPU6050_CheckReadyPin;
	
	Result = MPU6050_Init(MPU6050_Struct);
	
	/* Try search external acc */
	if (Result)
	{
		MPU6050_Struct->I2C_Adrs = MPU6050_I2C_AddressExternal;
		Result = MPU6050_Init(MPU6050_Struct);
	}

	if (Result)
		vPortFree(MPU6050_Struct);
	else
		xTaskCreate(LevelingTask, "Leveling Task",  120, NULL, tskIDLE_PRIORITY + 2, &vLevelingTaskHandler);
	
	return Result;
}

void LevelingTask (void * pvArg)
{
	extern LevelingConfigStructTypeDef LevConfig;
	extern void Position_0 (PositionTypeDef * Pos);
	extern void (*const PositionTable[])(PositionTypeDef * Pos);
	
	MPU6050_Result = pvPortMalloc(sizeof(* MPU6050_Result));
	//MPU6050_ZeroCal = pvPortMalloc(sizeof(* MPU6050_ZeroCal));
	
	MPU6050_Struct->MPU6050_Result = MPU6050_Result;
	MPU6050_Struct->MPU6050_ZeroCal = &MPU6050_ZeroCal;
	
	LevConfig.AutoOffTimerS = AutoOffTimerInit();
	vTaskDelay(1000);
	while(MPU6050_GetResult(MPU6050_Struct));
	Kx.Previous = (float)MPU6050_Struct->MPU6050_Result->X;
	Ky.Previous = (float)MPU6050_Struct->MPU6050_Result->Y;
	Kz.Previous = (float)MPU6050_Struct->MPU6050_Result->Z;
	Kx.K = LevConfig.Kp;
	Ky.K = LevConfig.Kp;
	Kz.K = LevConfig.Kp;
	if (LevConfig.PositionNum > 15) LevConfig.PositionNum = 0;
	LevConfig.PosCalc = PositionTable[LevConfig.PositionNum];
	TIM_SetCompare3(TIM2, LevConfig.LedBrightness);
	
	while(1)
	{
		vTaskDelay(LevConfig.delay);
		if (Position.LevelingIsOn)
		{			
			if (!MPU6050_GetResult(MPU6050_Struct))
			{
				Kx.Value = (float)MPU6050_Struct->MPU6050_Result->X;
				Ky.Value = (float)MPU6050_Struct->MPU6050_Result->Y;
				Kz.Value = (float)MPU6050_Struct->MPU6050_Result->Z;
				
				KalmanFloatCalc(&Kx);
				KalmanFloatCalc(&Ky);
				KalmanFloatCalc(&Kz);
				
				Position.x = Kx.Result;
				Position.y = Ky.Result;
				Position.z = Kz.Result;

				LevConfig.PosCalc(&Position);
			}

			if (LevConfig.AutoOffTimerS) 
				LevConfig.AutoOffTimerS--;
			else 
				Position.LevelingIsOn = 0;			
		}
		else
			vTaskSuspend( NULL );
	}
}

void CallibrateZeroPositionHandle (void)
{
	extern ErrorTypeDef Error;
	extern char * ErrLogFile;
	char * buf;
	
	if (MPU6050_Struct == 0) return;
	if (Error.MPU6050) return;
	
	MPU6050_CalibrateZero(MPU6050_Struct);
	UpdateDB();
	buf = pvPortMalloc(70);
	
	if (buf == 0) return;

	sprintf(buf, "Calibrated to Zero position. Offsets: %i, %i, %i\n", 
		MPU6050_Struct->MPU6050_ZeroCal->x_offset, 
		MPU6050_Struct->MPU6050_ZeroCal->y_offset, 
		MPU6050_Struct->MPU6050_ZeroCal->z_offset);
	
	PrintToFile(ErrLogFile, buf);
	
	vPortFree(buf);
}
