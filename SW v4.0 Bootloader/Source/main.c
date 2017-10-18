#include "rtc.h"
#include "HW.h"
#include "stm32_GPIO.h"
#include "diskio.h"
#include "sd_spi_stm32.h"
#include "ff.h"
#include "diskio.h"

typedef enum
{
	DataToWriteCMD = 0,
	LastStringCMD = 1,
	DefineStartCMD = 2,
	DefineInSegCMD = 4
}HEXcmdTypeDef;

typedef enum 
{
	Success = 0,
	NoDiskFound,
	NoFileFound,
	FileStringTooLong,
	FormatError,
	EraseError,
	WriteError
}
BT_ErrorTypeDef;

BT_ErrorTypeDef BootLoader (void);
void Init_SPI1 (void);
void Init_TIM4 (void);
BT_ErrorTypeDef UpdateFlash (void);
char ByteParse (char * buf, char pos);
u16 uIntParse (char * buf, char pos);
u16 uIntParseReverse (char * buf, char pos);
BT_ErrorTypeDef FlashErase (void);
void ParseToUINT_Array (char * Source, u16 * Dist, char Len, char StartPos);
void Internal_Flash_Write(u16 * data, unsigned int address, unsigned int count);

/* Variables */
BT_ErrorTypeDef FW_Error;
DSTATUS SDMMC_Status;
FATFS fs;
FILINFO fno;
FRESULT fr;    /* FatFs return code */
FIL fil;       /* File object */
const char filename[] = "FIRMWARE.HEX";

int main (void)
{
	typedef void (*pFunction)(void);
	pFunction Jump_To_Application;
	uint32_t JumpAddress;
	extern void stm32_Init();
	stm32_Init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
											 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
											 | RCC_APB2Periph_SPI1, ENABLE);
	/* GPIO Init */
	AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE | AFIO_MAPR_SPI1_REMAP;	//disable JTAG & SPI1 remap
	PIN_IN_PU(SD_Present);
	PIN_OUT_PP(SDCS);
	PIN_ON(SDCS);
	
	Init_SPI1();
	Init_TIM4();
	rtc_init();
	
	FW_Error = BootLoader();
	
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, MAIN_FLASH_OFFSET);
	__disable_irq();
  JumpAddress = *(__IO uint32_t*) (MAIN_FLASH_OFFSET + 4);
  Jump_To_Application = (pFunction) JumpAddress;
  __set_MSP(*(__IO uint32_t*) MAIN_FLASH_OFFSET);
  Jump_To_Application();
	return 0;
}

BT_ErrorTypeDef BootLoader (void)
{
	BT_ErrorTypeDef Result;
	SDMMC_Status = disk_initialize(0);
	if (SDMMC_Status != RES_OK) return NoDiskFound; //no disk
	f_mount(&fs, "", 1);
	if (f_stat(filename, &fno) == FR_NO_FILE) return NoFileFound;	//no file
	fr = f_open(&fil, filename, FA_READ);
	Result = UpdateFlash();
	f_unlink(filename);
	f_close(&fil);
	return Result;
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
	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler (void)
{
	extern void disk_timerproc (void);
	disk_timerproc();
	PIN_INV(LED11);
	TIM4->SR &= ~TIM_SR_UIF;
}

BT_ErrorTypeDef UpdateFlash (void)
{
	static BT_ErrorTypeDef Result;
	static char String[60];
	static u16 Buf[16];
	static char rByte, i;
	static UINT BytesToRead = 1;
	static char BytesToWrite = 1;
	static u32 CurrentWritePos;
	static u16 MemOffset;
	static HEXcmdTypeDef Command;
	
	Result = FlashErase();
	if (Result != Success) return Result;
	
	while (BytesToWrite)
	{
		/* Read string */
		i = 0;
		do
		{
			fr = f_read(&fil, &rByte, 1, &BytesToRead);
			String[i++] = rByte;
		}while (rByte != 10 && i < sizeof(String));
		
		if (i == sizeof(String)) return FileStringTooLong; //overlimit
		if (String[0] != ':') return FormatError;
		BytesToWrite = ByteParse(String, 1);
		if (BytesToWrite > (sizeof(Buf) * 2)) return FileStringTooLong;
		IWDG->KR = 0xAAAA; //reset watchdog timer
		
		if (BytesToWrite)
		{
			Command = (HEXcmdTypeDef)ByteParse(String, 7);
			
			/* Get first address offset */
			if (Command == DefineInSegCMD)
				MemOffset = uIntParse(String, 9);
			
			/* Write string to flash */
			if (Command == DataToWriteCMD)
			{
				CurrentWritePos = MemOffset << 16 | uIntParse(String, 3);
				ParseToUINT_Array(String, Buf, BytesToWrite, 9);
				Internal_Flash_Write(Buf, CurrentWritePos, BytesToWrite / 2);
			}
		}
	}
	FLASH_Lock();
	return Success;
}

BT_ErrorTypeDef FlashErase (void)
{
	u32 FlashStastAddress = MAIN_FLASH_OFFSET;
	FLASH_Status Result = FLASH_COMPLETE;
	
	FLASH_Unlock();
	
	while (FlashStastAddress < MAIN_FLASH_OFFSET + MAIN_FLASH_SIZE && Result == FLASH_COMPLETE)
	{
		Result = FLASH_ErasePage(FlashStastAddress);
		FlashStastAddress += FLASH_PAGE_SIZE;
	}
	
	if (Result == FLASH_COMPLETE) return Success;
	return EraseError;
}

char ByteParse (char * buf, char pos)
{
	const char ASCII_Table[] = {0,1,2,3,4,5,6,7,8,9,':',';','<','=','>','?','@',10,11,12,13,14,15};
	return (ASCII_Table[(buf[pos] - '0')] << 4) | ASCII_Table[buf[pos + 1] - '0'];
}

u16 uIntParse (char * buf, char pos)
{
	return (ByteParse(buf, pos) << 8) | ByteParse(buf, pos + 2);
}

u16 uIntParseReverse (char * buf, char pos)
{
	return (ByteParse(buf, pos + 2) << 8) | ByteParse(buf, pos);
}

void ParseToUINT_Array (char * Source, u16 * Dist, char Len, char StartPos)
{
	char i;
	for (i = 0; i < Len / 2; i++)
		Dist[i] = uIntParseReverse(Source, StartPos + i * 4);
}

void Internal_Flash_Write(u16 * data, unsigned int address, unsigned int count) 
{
	unsigned int i;
	
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) 	
		FLASH->SR = FLASH_SR_EOP;
	
	FLASH->CR |= FLASH_CR_PG;

	for (i = 0; i < count; i++) 
	{
		while (FLASH->SR & FLASH_SR_BSY);
		*(__IO uint16_t*)(address + i * 2) = data[i];
		while (!(FLASH->SR & FLASH_SR_EOP));
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR &= ~FLASH_CR_PG;
}
