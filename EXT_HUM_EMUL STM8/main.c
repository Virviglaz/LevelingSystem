#include "main.h"
#include "CRC.h"
#include "STM8_GPIO.h"
#include "STM8S_Delays.h"
#include "SW_I2C_Driver.h"
#include "SI7005.h"
#include "BME280.h"
#include <string.h>

#define I2C_SDA                 PB4
#define I2C_SCL                 PB5

/* Local variables and drivers */
const u8 EXT1_SN[8] = {0x28, 0x55, 0x11, 0x74, 0xA0, 0xE1, 0xBA, 0xC7};
const u8 EXT2_SN[8] = {0x28, 0xA1, 0xE8, 0x33, 0xBE, 0x05, 0x23, 0x27};
OW_ModeTypeDef OW;
I2C_Result SI7005_Error;
I2C_Result BME280_Error;
uint16_t Humidity, Temperature;
float Temp180; 
long Pressure;

/* Drivers */
SW_I2C_DriverStructTypeDef SW_I2C;
BME280_StructTypeDef BME280_Struct;
BME280_CalibrationStructTypeDef BME280_Calibration;
SI7005_StructTypeDef SI7005;

/* Local functions */
void Init (void);
void PrepareData (u16 HumidyValue, u16 TempValue);
void PROTECT_FW_Handle (void);
void SW_I2C_Driver_Init (void);
I2C_Result BME280_TryInit (void);
I2C_Result SI7005_TryInit (void);

int main( void )
{
  Init();
  while(1);
}

void Init (void)
{
  CLK->CKDIVR=0;  //16Mhz OSC RC
  delays_init(TIM4_PRSC_16);
  SW_I2C_Driver_Init();
  delay_ms(3000);
  //PROTECT_FW_Handle();
  
  /* TIM4 Init for 4uS time generation */
  CLK->PCKENR1 |= CLK_PCKENR1_TIM4;
  TIM4->PSCR = TIM4_PRESCALER_64;
  TIM4->EGR |= TIM4_EGR_UG;
  TIM4->CR1 = TIM4_CR1_CEN;
  
  /* TIM2 Init for RESET detection sequence */
  CLK->PCKENR1 |= CLK_PCKENR1_TIM2;// | CLK_PCKENR1_I2C;
  TIM2->CR1 = TIM2_CR1_ARPE;
  TIM2->PSCR = TIM2_PRESCALER_16; //1uS
  TIM2->ARRH = 0xFF - 0x01;    //480uS Reset pulse measure
  TIM2->ARRL = 0xFF - 0xE0;
  TIM2->EGR |= TIM2_EGR_UG;
  TIM2->CR1 = TIM2_CR1_CEN;
  
  /* GPIO Init */
  OW_GPIO->DDR &= ~OW_PIN;      //input
  OW_GPIO->ODR &= ~OW_PIN;      //low level
  OW_GPIO->CR1 |= OW_PIN;       //pull up
  OW_GPIO->CR2 |= OW_PIN;       //IT enable
  EXTI->CR1 = FallC | RiseC;     //IT for rising and falling edge
  GPIO_Init(LED, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(JP1, GPIO_MODE_IN_PU_NO_IT);
  //GPIOC->CR1 |= GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;              //pull up for external inputs
  
  GPIO_Init(I2C_SDA, GPIO_MODE_OUT_OD_HIZ_FAST);
  GPIO_Init(I2C_SCL, GPIO_MODE_OUT_OD_HIZ_FAST);
  
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);
  
  BME280_Error = BME280_TryInit();
  SI7005_Error = SI7005_TryInit();
  if (BME280_Error == I2C_SUCCESS)
    BME280_Init(&BME280_Struct);

  GPIO_ReadInputPin(JP1) ? memcpy(OW.SN, EXT1_SN, 7) : memcpy(OW.SN, EXT2_SN, 7);
  OW.SN[7] = Crc8Dallas(7, OW.SN); 
  OW.Mode = OW_RESET;
  OW.StartConvFlag = SET;
  
  rim();
  
  while(1)
  {
     if (OW.StartConvFlag == SET)
     {
       GPIO_WriteHigh(LED);
       
       /* Try get data from SI7005 */
       if (SI7005_Error == I2C_SUCCESS)
       {
         SI7005.MeasType = MeasHum;
         SI7005_Error = (I2C_Result)SI7005_StartConversion(&SI7005);
         delay_ms(10);
         SI7005_Error = (I2C_Result)SI7005_GetResult(&SI7005);
         Humidity = (uint16_t)SI7005.Humidity;
         delay_ms(10);
         
         SI7005.MeasType = MeasTemp;
         SI7005_Error = (I2C_Result)SI7005_StartConversion(&SI7005);
         delay_ms(10);
         SI7005_Error = (I2C_Result)SI7005_GetResult(&SI7005);        
         Temperature = (uint16_t)SI7005.Temperature;
         delay_ms(10);
       }
       
       /* Try get data from BME280 */
       if (BME280_Error == I2C_SUCCESS)
       {
         BME280_Get_Result(&BME280_Struct);
         Humidity = (uint16_t)BME280_Struct.Humidity;
         Temperature = (uint16_t)BME280_Struct.Temperature;
       }
       
       PrepareData(Humidity, Temperature);
       GPIO_WriteLow(LED);
       OW.StartConvFlag = RESET;
     }
  }
}

void PrepareData (u16 HumidyValue, u16 TempValue)
{
  OW.DATA[0] = (u8)HumidyValue;
  OW.DATA[1] = (u8)(HumidyValue >> 8);
  OW.DATA[2] = GPIOC->IDR & 0xF0;
  OW.DATA[3] = (u8)TempValue;
  OW.DATA[4] = (u8)(TempValue >> 8);

  OW.DATA[8] = Crc8Dallas(8, OW.DATA); 
}

void PROTECT_FW_Handle (void)
{
  if (FLASH_ReadOptionByte(0x4800) == 0xAA) return;
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  FLASH_ProgramOptionByte(0x4800, 0xAA);
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void I2C_Delay_func (uint16_t ms)
{
  delay_us(ms);
}

void I2C_SCL_Write (uint8_t state)
{
  state ? PIN_ON(I2C_SCL) : PIN_OFF(I2C_SCL);
}

void I2C_SDA_Write (uint8_t state)
{
  state ? PIN_ON(I2C_SDA) : PIN_OFF(I2C_SDA);
}

uint16_t I2C_SDA_Read (void)
{
  return PIN_SYG(I2C_SDA);
}

void SW_I2C_Driver_Init (void)
{
  SW_I2C.Delay_func = I2C_Delay_func;
  SW_I2C.IO_SCL_Write = I2C_SCL_Write;
  SW_I2C.IO_SDA_Write = I2C_SDA_Write;
  SW_I2C.IO_SDA_Read = I2C_SDA_Read;
  SW_I2C.DelayValue = 100;
  SW_I2C_ASSIGN(&SW_I2C);
}

uint8_t I2C_WriteReg (uint8_t I2C_Adrs, uint8_t Reg, uint8_t Value)
{
  return (uint8_t)SW_I2C_WR (I2C_Adrs, &Reg, 1, &Value, 1);
}

uint8_t I2C_ReadReg  (uint8_t I2C_Adrs, uint8_t Reg, uint8_t * buf, uint16_t size)
{
  return (uint8_t)SW_I2C_RD (I2C_Adrs, &Reg, 1, buf, size);
}

I2C_Result BME280_TryInit (void)
{
  BME280_Struct.WriteReg = I2C_WriteReg;
  BME280_Struct.ReadReg  = I2C_ReadReg;
  BME280_Struct.I2C_Adrs = 0xEC;
  BME280_Struct.BME280_Calibration = &BME280_Calibration;
  BME280_Struct.PressureOversampling = BME280_OversamplingX16;
  BME280_Struct.TemperatureOversampling = BME280_OversamplingX16;
  BME280_Struct.HumidityOversampling = BME280_OversamplingX16;
  return (I2C_Result)BME280_Check_ID(&BME280_Struct);
}

I2C_Result SI7005_TryInit (void)
{
  SI7005.WriteReg = I2C_WriteReg;
  SI7005.ReadReg = I2C_ReadReg;
  SI7005.I2C_Adrs = 0x80;
  SI7005.UseRelativeMeas = SET;
  SI7005.MeasType = MeasHum;
  return (I2C_Result)SI7005_CheckID(&SI7005);
}
