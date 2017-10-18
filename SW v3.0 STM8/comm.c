#include "comm.h"
#include "HW.h"
#include "CRC.h"
#include "DS1822.h"
#include "STM8_Delays.h"

extern char inBuf[MAX_CommandLen];
void RegNewSensor (void);
extern void Uart_TXIT (char * data, u16 Len);
void IR_WriteByte (char Data);
__no_init static unsigned char SN[MAX_Sensors][8];
u8 CurrentSensor = 0;
u8 ConvertingFlag = 0;
u8 OutDat[OutDataLen];
const u8 EXT1_SN[8] = {0x28, 0x55, 0x11, 0x74, 0xA0, 0xE1, 0xBA, 0xC7};
const u8 EXT2_SN[8] = {0x28, 0xA1, 0xE8, 0x33, 0xBE, 0x05, 0x23, 0x27};
u8 RF_KEY[5];
const u8 RF_KEY_Def[5] = {53, 64, 2, 0, 1};
IR_TypeDefStruct IR_Packet;

void RecoverSNData (void)
{
  u16 adcnt = EEPROM_ADDRESS0;
  u8 cnt1, cnt2;
  for (cnt1 = 0; cnt1 != 10; cnt1++)
    for (cnt2= 0; cnt2 != 8; cnt2++)
      SN[cnt1][cnt2] = FLASH_ReadByte(adcnt++);
  for (cnt2 = 0; cnt2 != 8; cnt2++)
  {
    SN[10][cnt2] = EXT1_SN[cnt2];
    SN[11][cnt2] = EXT2_SN[cnt2];
  }
  adcnt = EEPROM_ADDRESS0 + RF_KEY_EE_ADD;
  for (cnt1 = 0; cnt1 != sizeof(RF_KEY); cnt1++)
  {
    RF_KEY[cnt1] = FLASH_ReadByte(adcnt++);
    if (RF_KEY[cnt1] == 0xFF || RF_KEY[cnt1] == 0) RF_KEY[cnt1] = RF_KEY_Def[cnt1];
  }
  //SN[10][7] = Crc8Dallas(7, SN[10]); 
  //SN[11][7] = Crc8Dallas(7, SN[11]); 
}

void SaveSNData (u8 SNum)
{
  u8 bcnt;
  if (SNum>MAX_Sensors) return;
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  for (bcnt=0; bcnt!=8; bcnt++)
  {
    while(FLASH->IAPSR&FLASH_IAPSR_EOP == 0);
    FLASH_ProgramByte((EEPROM_ADDRESS0+SNum*8+bcnt), SN[SNum][bcnt]);
  }
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void inCommHandle (void)
{
  switch(inBuf[0])
  {
    case REG_NEW_SENSOR:                RegNewSensor();                 break;
    case PROTECT_FW:                    PROTECT_FW_Handle();            break;
    case DELETE_SENSOR:                 DELETE_SENSOR_Handle();         break;
    case SEARCH_SENSORS:                SEARCH_SENSORS_Handle();        break;
    case GET_INSTALLED_SENSORS:         GET_INSTALLED_SENSORS_Handle(); break; 
    case INSTALL_NEY_KEY:               RegNewKey_Handle();             break;
    case BOOTLOADER_STM8:               {sim();Bootloader();}           break;
    case IR_COMMAND:                    IR_Handle();                    break;
  }
}

void RegNewSensor (void)
{
  u8 tmp;
  if (Crc8Dallas(8, (unsigned char*)inBuf+2)) return;                           //CRC missmatch
  if (inBuf[1]>10) return;                                             //SN out of range
  for (tmp=0; tmp!=8;tmp++)
    SN[inBuf[1]][tmp] = inBuf[tmp+2];
  SaveSNData(inBuf[1]);
  OutDat[0] = SENSOR_SAVED;                                                     //send ACK
  OutDat[1] = inBuf[1];
  Uart_TXIT((char*)OutDat, 2);
}

void SecondInterruptHandle (void)
{ 
  //0 - Command
  //1 - Sensor Number
  //2 - Error
  //3 - MSB Result
  //4 - LSB Result 
  //7 - EXT pin state
  if (SN[CurrentSensor][0] == 0x28)      //family code match, DS18B20
  {
    if (ConvertingFlag == RESET)
    {
      DS1822_Start_Conversion_by_ROM(&SN[CurrentSensor]);
      ConvertingFlag = SET;
    }
    else
    {
      unsigned int TempCode;
      char opcode;
      OutDat[0] = CONV_RESULT;
      OutDat[1] = CurrentSensor;
      OutDat[2] = DS1822_Get_Conversion_Result_by_ROM_CRC(&SN[CurrentSensor], &TempCode, &opcode);
      OutDat[3] = TempCode>>8;
      OutDat[4] = TempCode&0xFF;
      OutDat[5] = opcode;
      Uart_TXIT((char*)OutDat, 6);
      ConvertingFlag = RESET;
      CurrentSensor++;
    }
  }
  else CurrentSensor++;
  if (CurrentSensor == 12) CurrentSensor = 0;
}

void NRF_DATA_Handle (char * Data)
{
  u8 cnt;
  OutDat[0] = RF_DATA_RES;
  OutDat[1] = 0;
  for (cnt=2; cnt!=(16+2); cnt++)
    OutDat[cnt] = Data[cnt-2];
  Uart_TXIT((char*)OutDat, 16+2);
  delay_ms(100);
}

void PROTECT_FW_Handle (void)
{
  if (inBuf[1] != 0xAA) return;
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  FLASH_ProgramOptionByte(0x4800, 0xAA);
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}

void DELETE_SENSOR_Handle (void)
{
  if (inBuf[1]>10) return;
  SN[(inBuf[1])][0] = 0;
  SaveSNData(inBuf[1]);
}

void SEARCH_SENSORS_Handle (void)
{
  unsigned char SN_TEMP[MAX_Sensors][8];
  unsigned char SensorsFound;
  unsigned char devcnt, bytecnt, cnt = 2;
  if (DS1822_Search_Rom2(&SensorsFound, &SN_TEMP) != One_Wire_Success) return;
  OutDat[0] = SEARCH_SENSORS;
  OutDat[1] = SensorsFound;
  for (devcnt=0; devcnt!=SensorsFound; devcnt++)
    for (bytecnt=0; bytecnt!=8; bytecnt++)
      OutDat[cnt++] = SN_TEMP[devcnt][bytecnt];
  Uart_TXIT((char*)OutDat, cnt);
  delay_ms(300);
}

void GET_INSTALLED_SENSORS_Handle (void)
{
  unsigned char devcnt, bytecnt, sensors_installed = 0, cnt = 2;
  for (devcnt=0; devcnt!=10; devcnt++)
    if (SN[devcnt][0] == 0x28)
    {
      sensors_installed++;
      OutDat[cnt++] = devcnt;
      for (bytecnt=1; bytecnt!=8;bytecnt++)
        OutDat[cnt++] = SN[devcnt][bytecnt];     
    }
  OutDat[0] = GET_INSTALLED_SENSORS;
  OutDat[1] = sensors_installed;
  Uart_TXIT((char*)OutDat, cnt);
  delay_ms(300);  
}

void RegNewKey_Handle (void)
{
  u8 cnt;
  extern unsigned char Init_NRF24L01 (void);
  
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  for (cnt = 0; cnt!= sizeof(RF_KEY); cnt++)
  {
    RF_KEY[cnt] = inBuf[cnt+1];
    while(FLASH->IAPSR&FLASH_IAPSR_EOP == 0);
    FLASH_ProgramByte((EEPROM_ADDRESS0 + cnt + RF_KEY_EE_ADD), RF_KEY[cnt]);
  }
  FLASH_Lock(FLASH_MEMTYPE_DATA); 
  Init_NRF24L01();
}

IN_RAM(void Bootloader (void))
{
  u16 MemArea = 0x8000;
  UART1->CR2 = UART1_CR2_TEN|UART1_CR2_REN; //UART IT off
  TIM1->CR1 &= ~TIM1_CR1_CEN;               //TIM1 off
  FLASH->PUKR = FLASH_RASS_KEY1;            //flash unlock
  FLASH->PUKR = FLASH_RASS_KEY2;            //flash unlock
  IWDG->PR = IWDG_Prescaler_256;
  IWDG->KR = IWDG_KEY_ENABLE;               //IWDG ON
  IWDG->KR = IWDG_KEY_REFRESH;
  while(1)
  {
    static u8 cnt = 0;
    while (!UART1->SR & UART1_SR_RXNE);
    cnt = UART1->DR;
    *(PointerAttr uint8_t*) (uint16_t)MemArea = cnt;//UART1->DR;
    MemArea++;
    IWDG->KR = IWDG_KEY_REFRESH;
  }
}

void IR_Handle (void)
{
  extern void IR_Enable (FunctionalState NewState);
  u8 cnt;
  
  IR_Packet.DataLen = inBuf[5];
  IR_Packet.HighDataDelay = inBuf[1] << 8 | inBuf[2];
  IR_Packet.LowDataDelay = inBuf[3] << 8 | inBuf[4];
  
  IR_Enable(ENABLE);
  TIM2->CR1 |= TIM2_CR1_CEN;
  delay_us(3000);
  TIM2->CR1 &= ~TIM2_CR1_CEN;
  delay_us(2000);
  for (cnt = 0; cnt < IR_Packet.DataLen; cnt++)
    IR_WriteByte(inBuf[cnt + 6]);
  TIM2->CR1 |= TIM2_CR1_CEN;
  delay_us(IR_Packet.HighDataDelay);
  TIM2->CR1 &= ~TIM2_CR1_CEN;
  IR_Enable(DISABLE);
}

void IR_WriteByte (char Data)
{
  u8 cnt;
  //while (cnt--)
  for (cnt = 0; cnt != 8; cnt++)
  {
    TIM2->CR1 |= TIM2_CR1_CEN;
    //TIM2->CR1 &= ~TIM2_CR1_CEN;
    delay_us(IR_Packet.LowDataDelay);
    TIM2->CR1 &= ~TIM2_CR1_CEN;
    //TIM2->CR1 |= TIM2_CR1_CEN;
    if (Data & (1 << cnt))
      delay_us(IR_Packet.HighDataDelay);
    else
      delay_us(IR_Packet.LowDataDelay);
    //TIM2->CR1 &= ~TIM2_CR1_CEN;
  }
}
