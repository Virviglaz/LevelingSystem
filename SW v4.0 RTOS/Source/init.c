#include "data_collector.h"
#include "data_logger.h"
#include "error_collector.h"
#include "stm32f10x_gpio.h"
#include "strings.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "rtc.h"
#include "stm32_GPIO.h"
#include "HW.h"
#include "SW_I2C_Driver.h"
#include "MPU6050.h"
#include "BMP180.h"
#include "SI7005.h"
#include "EEPROM.h"
#include "local_db.h"
#include "type.h"
#include "usbhw.h"
#include "db.h"
#include "diskio.h"
#include "CRC.h"
#include "BME280.h"
#include "Leveling.h"
#include "hid_comm.h"
#include <stdio.h>
#include <string.h>

/* Variables */
extern SW_I2C_DriverStructTypeDef I2C_Struct;
extern LevelingConfigStructTypeDef LevConfig;
extern MPU6050_StructTypeDef MPU6050_Struct;
extern BMP180_StructTypeDef BMP180_Struct;
extern BMP180_CalibrationStructTypeDef BMP180_Calibration;
extern BME280_StructTypeDef BME280_Struct;
extern BME280_CalibrationStructTypeDef BME280_Calibration;
extern SI7005_StructTypeDef SI7005_Struct;
extern ErrorTypeDef Error;
extern char * ErrLogFile;
extern char * DataLogFile;
extern QueueHandle_t xUSB_TX_TextQueue;
SemaphoreHandle_t xUSB_RX_Semaphore;
SemaphoreHandle_t xUSB_TX_Semaphore;
SemaphoreHandle_t xUSB_ZX_Semaphore;
SemaphoreHandle_t xI2C_Semaphore;
SemaphoreHandle_t xFLASH_Semaphore;
struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

/* Functions */
void InitHW (void);
void Init_SPI1 (void);
void Init_TIM2 (void);
RESET_Result CheckResetCause (void);
uint8_t I2C_WriteReg (uint8_t I2C_Adrs, uint8_t Reg, uint8_t Value);
uint8_t I2C_ReadReg  (uint8_t I2C_Adrs, uint8_t Reg, uint8_t * buf, uint16_t size);
uint8_t I2C_WritePage (uint8_t I2C_Adrs, uint8_t * MemPos, uint8_t MemPosSize, uint8_t * buf, uint16_t size);
uint8_t I2C_ReadPage  (uint8_t I2C_Adrs, uint8_t * MemPos, uint8_t MemPosSize, uint8_t * buf, uint16_t size);
uint16_t MPU6050_CheckReadyPin (void);
void delay (unsigned int cycles);
void Get_SerialNum(void);
void Init_TIM4 (void);
void Init_USART1 (void);
void Init_TIM3 (void);
void ADC_Configure (u8 Channel);
char PrintDB_Error (char Err);
long CalcCRC32 (char * buf, long size);
void Init_SPI2 (void);

/* Tasks */
TaskHandle_t  vSD_ServiceTaskHandler;
TaskHandle_t 	vDataCollectorTaskHandler;

void InitTask (void * pvArg)
{
	InitHW();
	vTaskDelete( NULL );
}

void SD_ServiceTask (void * pvArg)
{
	extern void disk_timerproc (void);
	while(1)
	{
		disk_timerproc();
		vTaskDelay(100);
	}
}

int GeneralInit (void)
{
	static BaseType_t TaskError = 1;
	extern void stm32_Init();
	//extern void BootLoader (void);
	
	stm32_Init();
	//BootLoader();
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
											 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1 
											 | RCC_APB2Periph_SPI1  | RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2  | RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	RCC_AHBPeriphClockCmd (RCC_AHBPeriph_CRC, ENABLE);	
	
	/* GPIO Init */
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE | AFIO_MAPR_SPI1_REMAP;	//disable JTAG & SPI1 remap
	PIN_IN_PU(SD_Present);
	PIN_OUT_PP(SDCS);
	PIN_ON(SDCS);
	PIN_OUT_OD(RMSCK);
	PIN_OUT_OD(RMOSI);
	PIN_OUT_OD(RCS);
	PIN_IN_PU(BTN);
	
	rtc_init();
	Init_SPI1();
	Init_SPI2();
	Init_TIM2();
	LoggerInit();
	Get_SerialNum();
	Init_TIM4();
	Init_USART1();
	Init_TIM3();
	ADC_Configure(USB_VOLTAGE);
	
	TaskError &= xTaskCreate(InitTask, 			"Init Task",  100, NULL, tskIDLE_PRIORITY + 1, NULL);
	TaskError &= xTaskCreate(SD_ServiceTask, "SD Handler", 50, NULL, tskIDLE_PRIORITY + 1, &vSD_ServiceTaskHandler);
	return TaskError ? 0 : 1;
}

void I2C_SCL_IO_Write (uint8_t state)
{
	state ? PIN_ON(I2C_SCL) : PIN_OFF(I2C_SCL);
}

void I2C_SDA_IO_Write (uint8_t state)
{
	state ? PIN_ON(I2C_SDA) : PIN_OFF(I2C_SDA);
}

uint16_t I2C_SDA_IO_Read (void)
{
	return PIN_SYG(I2C_SDA);
}

void I2C_DelayFunc (uint16_t ms)
{
	delay(ms);
}

void RTOS_Delay (uint16_t ms)
{
	vTaskDelay(ms);
}

void InitHW (void)
{
	extern void DataCollectorTask (void * pvArg);
	RESET_Result ResetCause = CheckResetCause();
	const char * ResetText[] = {"Reseted by low power mode!\r\n", 
															"Reseted by window watchdog!\r\n", 
															"Reseted by independant watchdog!\r\n",
															"Reseted by POD/PDR!\r\n",
															"Reseted by nRESET pin!\r\n",
															"Unknown reset source!\r\n"};
	
	PrintToFile(ErrLogFile, "\r\nInitialisation started!\r\n");
	PrintToFile(ErrLogFile, (char*)ResetText[ResetCause]);			
	vTaskDelay(500);
						
	// Create semaphore and queues
	xI2C_Semaphore = xSemaphoreCreateMutex();
	xFLASH_Semaphore = xSemaphoreCreateMutex();
	xUSB_RX_Semaphore = xSemaphoreCreateMutex();
	xUSB_TX_Semaphore = xSemaphoreCreateMutex();
	xUSB_ZX_Semaphore = xSemaphoreCreateMutex();
	xUSB_TX_TextQueue = xQueueCreate(10, sizeof(char*));
															
	// Init I2C
	I2C_Struct.Delay_func = I2C_DelayFunc;
	I2C_Struct.DelayValue = 100; //50 -> 100kHz clock
	I2C_Struct.IO_SCL_Write = I2C_SCL_IO_Write;
	I2C_Struct.IO_SDA_Write = I2C_SDA_IO_Write;
	I2C_Struct.IO_SDA_Read = I2C_SDA_IO_Read;
	SW_I2C_ASSIGN(&I2C_Struct);												

	// Init pressure sensor
	BMP180_Struct.delay_func = RTOS_Delay;
	BMP180_Struct.ReadReg = I2C_ReadReg;
	BMP180_Struct.WriteReg = I2C_WriteReg;
	BMP180_Struct.P_Oversampling = BMP180_OversamplingX8;
	BMP180_Struct.I2C_Adrs = BMP180_I2C_Address;
	BMP180_Struct.BMP180_Calibration = &BMP180_Calibration;
	Error.BMP180 = BMP180_Check_ID(&BMP180_Struct);
	
	// Init BME280
	BME280_Struct.ReadReg = I2C_ReadReg;
	BME280_Struct.WriteReg = I2C_WriteReg;
	BME280_Struct.I2C_Adrs = BME280_I2C_Address;
	BME280_Struct.HumidityOversampling = BME280_OversamplingX16;
	BME280_Struct.TemperatureOversampling = BME280_OversamplingX16;
	BME280_Struct.PressureOversampling = BME280_OversamplingX16;
	BME280_Struct.BME280_Calibration = &BME280_Calibration;
	Error.BME280 = BME280_Init(&BME280_Struct);
	
	// Init humididy sensor
	SI7005_Struct.ReadReg = I2C_ReadReg;
	SI7005_Struct.WriteReg = I2C_WriteReg;
	SI7005_Struct.I2C_Adrs = SI7005_I2C_Address;
	SI7005_Struct.UseRelativeMeas = ENABLE;
	Error.SI7005 = SI7005_CheckID(&SI7005_Struct);
	if (Error.SI7005 == I2C_ADD_NOT_EXIST)
		ErrorHandle(SI7005, "SI7005 does not answer!\r\n");
	else if (Error.SI7005)
		ErrorHandle(SI7005, "SI7005 unknown error!\r\n");	

	/* DB restore */
#ifdef DEBUG
	if (RCC->CSR & RCC_CSR_IWDGRSTF)	
		UpdateDB();		
	else
#endif
	{
		/* First try fetch from EEPROM */
		Error.DB = PrintDB_Error((char)RestoreDB(LogSourceEEPROM));
		
		/* If failed, fetch from SD card */
		if (Error.DB)
		{
			PrintIntDataToFile(ErrLogFile, "Error fetching DB from EEPROM.  Error num:", (int) Error.DB, " ");
			Error.DB = PrintDB_Error((char)RestoreDB(LogSourceSDcard));
		}
		
		/* if no db found create default one */
		if (Error.DB)
		{
			PrintIntDataToFile(ErrLogFile, "No DB found. Will create a default one.  Error num:", (int) Error.DB, " ");
			UpdateDB();
		}
	}
	
	// Init accelerometer
	Error.MPU6050 = LevelingInit();
	if (Error.MPU6050 == I2C_ADD_NOT_EXIST)
		ErrorHandle(MPU6050, "MPU6050 does not answer!\r\n");
	else if (Error.MPU6050)
		ErrorHandle(MPU6050, "MPU6050 unknown error!\r\n");
	
	xTaskCreate(DataCollectorTask, "Data collector", 150, NULL, tskIDLE_PRIORITY + 1 , &vDataCollectorTaskHandler);
	xTaskCreate(USB_RX_DataHandler, "USB RX", 150, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(USB_TX_DataHandler, "USB TX", 150, NULL, tskIDLE_PRIORITY + 1, NULL);

	PrintToFile(ErrLogFile, "Initialisation passed successfully!\r\n");
	
	LevConfig.RelaysState = (char)BKP_ReadBackupRegister(RelayStateAddress);
	
	USB_Init();
	USB_Connect(1);
	PIN_ON(USB_ON);
}

char PrintDB_Error (char Err)
{
	if (!Err) return Err;
	switch (Err)
	{
		case Err_EEPROM_ReadError: 		ErrorHandle(DB_Err, "Failed restore DB from EEPROM!\r\n"); break;
		case Err_SDCard_ReadError:		ErrorHandle(DB_Err, "Failed restore DB from SDCard!\r\n"); break;
		case Err_SDCard_DB_SizeError: ErrorHandle(DB_Err, "Failed restore DB! SD card DB size incorrect.\r\n"); break;
		case Err_DB_WrongCRC: 				ErrorHandle(DB_Err, "Failed restore DB! CRC mismatch.\r\n"); break;
		case Err_OutOfMemory:					ErrorHandle(DB_Err, "Failed restore DB! Out of memory.\r\n"); break;
		case Err_ValidationFailed:		ErrorHandle(DB_Err, "DB: Validation failed. Default settings loaded.\r\n"); break;
		default: ErrorHandle(DB_Err, "DB: Undefined error.\r\n"); break;
	}
	return Err;
}

void Init_SPI1 (void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;		 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Cmd(SPI1, ENABLE);
}

void Init_TIM2 (void)
{
	extern LevelingConfigStructTypeDef LevConfig;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 100;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 4 * RCC_ClocksStatus.HCLK_Frequency/(TIM_TimeBaseInitStruct.TIM_Period * TIM2_Freq);
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse = TIM_TimeBaseInitStruct.TIM_Period;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC3Init(TIM2, &TIM_OCInitStruct);
	//LV.Brightness = TIM_TimeBaseInitStruct.TIM_Period;
	TIM_Cmd(TIM2, ENABLE);
}

RESET_Result CheckResetCause (void)
{
	if (RCC->CSR & RCC_CSR_LPWRRSTF) 	return LOW_POWER;
	if (RCC->CSR & RCC_CSR_WWDGRSTF) 	return WINDOW_WATCHDOG;
	if (RCC->CSR & RCC_CSR_IWDGRSTF) 	return INDEPENDANT_WATCHDOG;
	if (RCC->CSR & RCC_CSR_SFTRSTF) 	return SOFTWARE;
	if (RCC->CSR & RCC_CSR_PORRSTF) 	return POD_PDR;
	if (RCC->CSR & RCC_CSR_PINRSTF) 	return NRST_PIN;
	return UNKNOWN;
}

void I2C_ResetBus (void)
{
	PIN_OUT_PP(I2C_SDA);
	PIN_OUT_PP(I2C_SCL);
	PIN_ON(I2C_SDA);
	PIN_ON(I2C_SCL);
	vTaskDelay(50);
	PIN_OUT_OD(I2C_SDA);
	PIN_OUT_OD(I2C_SCL);
	PIN_ON(I2C_SDA);
	PIN_ON(I2C_SCL);
}

uint8_t I2C_WriteReg (uint8_t I2C_Adrs, uint8_t Reg, uint8_t Value)
{
	uint8_t Result;
	xSemaphoreTake (xI2C_Semaphore, portMAX_DELAY);
	Result = SW_I2C_WR(I2C_Adrs, &Reg, 1, &Value, 1);
	if (Result) 
	{
		Error.I2C++;
		SW_I2C_RESET_BUS();
		//I2C_ResetBus();
	}
	xSemaphoreGive (xI2C_Semaphore);
	return Result;
}

uint8_t I2C_ReadReg (uint8_t I2C_Adrs, uint8_t Reg, uint8_t * buf, uint16_t size)
{
	uint8_t Result;
	xSemaphoreTake (xI2C_Semaphore, portMAX_DELAY);
	Result = SW_I2C_RD(I2C_Adrs, &Reg, 1, buf, size);
	if (Result) 
	{
		Error.I2C++;
		SW_I2C_RESET_BUS();
		//I2C_ResetBus();
	}
	xSemaphoreGive (xI2C_Semaphore);
	return Result;	
}

uint8_t I2C_WritePage (uint8_t I2C_Adrs, uint8_t * MemPos, uint8_t MemPosSize, uint8_t * buf, uint16_t size)
{
	uint8_t Result;
	xSemaphoreTake (xI2C_Semaphore, portMAX_DELAY);
	Result = SW_I2C_WR(I2C_Adrs, MemPos, MemPosSize, buf, size);
	if (Result) 
	{
		Error.I2C++;
		SW_I2C_RESET_BUS();
		//I2C_ResetBus();
	}
	xSemaphoreGive (xI2C_Semaphore);
	return Result;
}

uint8_t I2C_ReadPage (uint8_t I2C_Adrs, uint8_t * MemPos, uint8_t MemPosSize, uint8_t * buf, uint16_t size)
{
	uint8_t Result;
	xSemaphoreTake (xI2C_Semaphore, portMAX_DELAY);
	Result = SW_I2C_RD(I2C_Adrs, MemPos, MemPosSize, buf, size);
	if (Result) 
	{
		Error.I2C++;
		SW_I2C_RESET_BUS();
		//I2C_ResetBus();
	}
	xSemaphoreGive (xI2C_Semaphore);
	return Result;	
}

uint16_t MPU6050_CheckReadyPin (void)
{
	return PIN_SYG(DRY);
}

void delay (unsigned int cycles)
{
	while(cycles--)
		__nop();
}

void Get_SerialNum(void)
{
	char SerialString[8];
	extern BYTE USB_StringDescriptor[];
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;
	u8 cnt, cnt2 = 0;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

	Device_Serial0 = (Device_Serial0+Device_Serial1)^Device_Serial2;
	ValueToStringHEX_Long(Device_Serial0, SerialString); 
	for (cnt=70;cnt!=(70+16);cnt+=2)
		USB_StringDescriptor[cnt] = SerialString[cnt2++];
}

void Init_TIM4 (void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 18000;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 100;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn);
}

void Init_USART1 (void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);		
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

void Init_TIM3 (void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 36000;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 2000;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);
}

void ADC_Configure (u8 Channel)
{
	ADC_InitTypeDef ADC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = Channel;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
}

long CalcCRC32 (char * buf, long size)
{
	CRC_ResetDR();
	return CRC_CalcBlockCRC((uint32_t*)buf, size);
}

void Init_SPI2 (void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;		 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
			
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_Cmd(SPI2, ENABLE);
}

uint8_t EEPROM_ReadWrite (uint16_t Mem_adrs, void * buf, uint16_t size, uint8_t isWriting)
{
	uint8_t Result;
	EEPROM_StructTypeDef * EEPROM;
	EEPROM = pvPortMalloc(sizeof(*EEPROM));
	EEPROM->buf = buf;
	EEPROM->delay_func = vTaskDelay;
	EEPROM->I2C_Adrs = I2c_EEPROM_Address;
	EEPROM->isWriting = isWriting;
	EEPROM->Mem_adrs = Mem_adrs;
	EEPROM->PageSize = PageSizeBytes;
	EEPROM->PageWriteTime = I2c_EEPROM_WriteTime_ms;
	EEPROM->size = size;
	EEPROM->ReadPage = I2C_ReadPage;
	EEPROM->WritePage = I2C_WritePage;
	
	Result = EEPROM_RW(EEPROM);
	vPortFree(EEPROM);
	return Result;
}

int fputc(int ch, FILE *f)
{
	static char text[60], i = 0;
	
	text[i++] = ch;
	if (i == sizeof(text) || ch == 0 || ch == '\n')
	{
		char * buf = pvPortMalloc(i + 2);
		memcpy(buf, text, i++);
		text[i] = 0; //null terminate
		i = 0;
		
		if (uxQueueSpacesAvailable(xUSB_TX_TextQueue))
			xQueueSend(xUSB_TX_TextQueue, &buf, NULL);
		else
			vPortFree(buf);
	}
	
  return(ch);
}
