#include "HW.h"
#include "stm32f10x.h"                  // Device header

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

void Boot_Flash_Write(void) ;

void BootLoader (void)
{
	u32 i = 0; 
	u32 dif_cnt = 0;
	u32 FlashStastAddress = MAIN_FLASH_OFFSET;
	
	/* Check new firmware exist */
	while (i < BOOT_ADD_SIZE && *(volatile unsigned short*)(i + BOOT_ADD_OFFSET) == 0xFFFF)
		i += 2;
	if (i == BOOT_ADD_SIZE) 
		return; //no boot loader
	
	/* Compare firmwares */
	for (i = 0; i < BOOT_ADD_SIZE - 2; i+=2)
	{
		if (*(volatile unsigned short*)(i + MAIN_FLASH_OFFSET) != *(volatile unsigned short*)(i + BOOT_ADD_OFFSET))
		{
			dif_cnt++;
			//break;
		}
	}
	if (dif_cnt < 10) 
		return; //flash equal

	/* Some pre-boot delay */
	i = 0x00FFFFFF;
	while(i--) __nop();
		
  /* Authorize the FPEC of Bank1 Access */
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
		
	/* ERASE FLASH */
	while (FlashStastAddress < MAIN_FLASH_OFFSET + BOOT_ADD_SIZE)
	{
		FLASH->CR |= FLASH_CR_PER; /* Page erase */
		FLASH->AR = FlashStastAddress;
		FLASH->CR|= FLASH_CR_STRT; /* Start erase */
		while (FLASH->SR & FLASH_SR_BSY); /* Wait end of eraze */		
		FlashStastAddress += FLASH_PAGE_SIZE;
	}	
	FLASH->CR &= ~FLASH_CR_PER;

	while (FLASH->SR & FLASH_SR_BSY){}
	
	Boot_Flash_Write();
	NVIC_SystemReset();
}

void Boot_Flash_Write(void) 
{
	unsigned int i;
	volatile unsigned short FlashWord;
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) 	
		FLASH->SR = FLASH_SR_EOP;	

	for (i = 0; i < BOOT_ADD_SIZE; i += 2) 
	{
		FlashWord = *(volatile unsigned short*)(BOOT_ADD_OFFSET + i);
		FLASH->CR |= FLASH_CR_PG;
		while (FLASH->SR & FLASH_SR_BSY);
		*(volatile unsigned short*)(MAIN_FLASH_OFFSET + i) = *(volatile unsigned short*)(BOOT_ADD_OFFSET + i);
		//while (!(FLASH->SR & FLASH_SR_EOP));
		//FLASH->SR = FLASH_SR_EOP;
		while (FLASH->SR & FLASH_SR_BSY);
		FLASH->CR &= ~(FLASH_CR_PG);
	}	
}

