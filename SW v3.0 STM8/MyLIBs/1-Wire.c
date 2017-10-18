#include "HW.h"
#include "STM8_GPIO.h"
#include "1-Wire.h"
#include "STM8_Delays.h"

unsigned int One_Wire_Reset(void)								
{
	unsigned int tmp;
	if ((PIN_SYG(One_Wire_RX)))	return One_Wire_Bus_Low_Error;
	PIN_OFF(One_Wire_TX);
	delay_us(250);
        delay_us(250);
	PIN_ON(One_Wire_TX);
	delay_us(Time_Pulse_Delay_High);
	if ((PIN_SYG(One_Wire_RX))) tmp = One_Wire_Success;
		else tmp = One_Wire_Error_No_Echo;
	delay_us(250);
        delay_us(200);
	return tmp;
}

void One_Wire_Write_Byte(unsigned char Byte)
{
	unsigned char cnt;
	for (cnt=0;cnt!=8;cnt++) One_Wire_Write_Bit(Byte&(1<<cnt));
}

void One_Wire_Write_Bit (unsigned char Bit)
{
	PIN_OFF(One_Wire_TX);
	if (Bit==0)
	{
		delay_us(Time_Pulse_Delay_High);
		PIN_ON(One_Wire_TX);
		delay_us(Time_Pulse_Delay_Low);
	}
	else
	{
		delay_us(Time_Pulse_Delay_Low);
		PIN_ON(One_Wire_TX);
		delay_us(Time_Pulse_Delay_High);
	}
} 

unsigned char One_Wire_Read_Byte(void)
{
	unsigned char tmp=0;
	unsigned char cnt;
	for (cnt=0;cnt!=8;cnt++)
		if (One_Wire_Read_Bit()!=0)	tmp|=(1<<cnt);
	delay_us(Time_Pulse_Delay_High);
	return tmp;
}

unsigned char One_Wire_Read_Bit (void)
{
		unsigned char tmp;
		PIN_OFF(One_Wire_TX);
		delay_us(Time_Hold_Down);
		PIN_ON(One_Wire_TX);
		delay_us(Time_Pulse_Delay_Low);
		if ((PIN_SYG(One_Wire_RX))!=0)	tmp = 0;
			else tmp = 1;
		delay_us(Time_Pulse_Delay_High);
		return tmp;
}
