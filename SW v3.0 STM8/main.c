#include "STM8_Delays.h"
#include "HW.h"
#include "STM8_SPI.h"
#include "nrf24l01.h"
#include "comm.h"

void init (void);
unsigned char Init_NRF24L01 (void);
void Uart_TXIT (char * data, u16 Len);
void External_Clock_Init (void);
void IR_Enable (FunctionalState NewState);
volatile u8 DataReceivedFlag = 0;
char inBuf[MAX_CommandLen];
u8 inBufi = 0;
volatile char * TX_Data;
volatile u16 TX_DataLen = 0;
u8 SecondFlag = 0;
RF_TypeDef RF;
extern u8 RF_KEY[5];
u8 BootloaderFlag = 0;

int main (void)
{
  External_Clock_Init();
  init();
  while(1)
  {
    if (DataReceivedFlag){inCommHandle(); inBufi = 0; DataReceivedFlag = 0;}
    if (SecondFlag) {SecondInterruptHandle(); SecondFlag = 0;}
    if ((RF.Error != 0x00) && (RF.Error != 0xFF))
    {
      RF.PipeNum = RF_Receive_Data(RF.Data, Pipe0Len);
      if (RF.PipeNum != RF_NO_DATA_RECEIVED)
        NRF_DATA_Handle(RF.Data);
    }
  }
}

void init (void)
{
  CLK->PCKENR1|=CLK_PCKENR1_SPI|CLK_PCKENR1_UART1|CLK_PCKENR1_TIM2;
  CLK->CKDIVR=0;  //16Mhz OSC RC
  delays_init(TIM4_PRESCALER_16);
  GPIO_Init(RF_CE_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(RF_CSN_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(One_Wire_TX, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(One_Wire_RX, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(IR, GPIO_MODE_OUT_PP_HIGH_FAST);
  
  /* UART Init */
  UART1->BRR1 = 0x68;     //9600 @ 16MHz 
  UART1->BRR2 = 0x03;     //1ms time slot
  //UART1->BRR1 = 0x08;   //115200 @ 16MHz
  //UART1->BRR2 = 0x0B;   //70uS time slot
  UART1->CR2 = UART1_CR2_RIEN|UART1_CR2_TEN|UART1_CR2_REN;
  
  /* TIM1 Init */
  TIM1_TimeBaseInit(16000, TIM1_COUNTERMODE_UP, 1000, 0);
  TIM1_Cmd(ENABLE);
  TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
  
  /* TIM2 Init */
  TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,
                102, TIM2_OCPOLARITY_LOW); 
  IR_Enable(DISABLE);
  //IR_Enable(ENABLE);

  SPI_Init_ClockLOW();
  delay_ms(100);
  RF.Error = Init_NRF24L01();
  RecoverSNData();
  rim();
}

unsigned char Init_NRF24L01 (void)
{
    /*24L01 INIT*/
  RF_InitTypeDef RF_InitStruct;

  RF_InitStruct.RF_Power_State = RF_Power_On;
  RF_InitStruct.RF_Config = RF_Config_IRQ_RX_On|RF_Config_IRQ_TX_Off|RF_Confing_IRQ_Max_Rt_Off;
  RF_InitStruct.RF_CRC_Mode = RF_CRC16_On;
  RF_InitStruct.RF_Mode = RF_Mode_RX;
  RF_InitStruct.RF_Pipe_Auto_Ack = RF_Pipe0_Ack_Enable | RF_Pipe1_Ack_Enable;
  RF_InitStruct.RF_Enable_Pipe = RF_Pipe0_Enable | RF_Pipe1_Enable;
  RF_InitStruct.RF_Setup = RF_Setup_5_Byte_Adress;
  RF_InitStruct.RF_TX_Power = RF_TX_Power_High;
  RF_InitStruct.RF_Data_Rate = RF_Data_Rate_1Mbs;
  RF_InitStruct.RF_Channel = 77;
  
  RF_InitStruct.RF_RX_Adress_Pipe0[0] = RF_KEY[0];
  RF_InitStruct.RF_RX_Adress_Pipe0[1] = RF_KEY[1];
  RF_InitStruct.RF_RX_Adress_Pipe0[2] = RF_KEY[2];
  RF_InitStruct.RF_RX_Adress_Pipe0[3] = RF_KEY[3];
  RF_InitStruct.RF_RX_Adress_Pipe0[4] = RF_KEY[4];

  RF_InitStruct.RF_RX_Adress_Pipe1[0] = 'L';
  RF_InitStruct.RF_RX_Adress_Pipe1[1] = 'V';
  RF_InitStruct.RF_RX_Adress_Pipe1[2] = 'T';
  RF_InitStruct.RF_RX_Adress_Pipe1[3] = 'M';
  RF_InitStruct.RF_RX_Adress_Pipe1[4] = 'P';
 
  RF_InitStruct.RF_TX_Adress[0] = 'L';
  RF_InitStruct.RF_TX_Adress[1] = 'V';
  RF_InitStruct.RF_TX_Adress[2] = 'S';
  RF_InitStruct.RF_TX_Adress[3] = 'Y';
  RF_InitStruct.RF_TX_Adress[4] = 'S';
  RF_InitStruct.RF_Payload_Size_Pipe0 = Pipe0Len;
  RF_InitStruct.RF_Payload_Size_Pipe1 = Pipe1Len;
  RF_InitStruct.RF_Auto_Retransmit_Count = 2;
  RF_InitStruct.RF_Auto_Retransmit_Delay = 15;
  return RF_Init(&RF_InitStruct);
}

void Uart_TXIT (char * data, u16 Len)
{
  TX_Data = data;
  UART1->DR = * TX_Data++;
  TX_DataLen = Len;
  UART1->CR2|= UART1_CR2_TIEN;
}

INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)                                      //uart receive interrupt
{
  if (DataReceivedFlag == 0)
    if (inBufi<sizeof(inBuf))
    {
      inBuf[inBufi] = UART1->DR;
      inBufi++;
      TIM2->CNTRH = 0;
      TIM2->CNTRL = 0;
      TIM2->CR1 |= TIM2_CR1_CEN;
    }
  UART1->SR&= ~UART1_SR_RXNE;                                                   //clear IT pending bit
}

INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{
  if (TX_DataLen)
  {
    UART1->DR = * TX_Data++;
    TX_DataLen--;
  }
  else UART1->CR2&= ~UART1_CR2_TIEN;
}

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)                              //tim2 RX timeout interrupt
{
  TIM2->CR1 &= ~TIM2_CR1_CEN;
  DataReceivedFlag = ENABLE;
  TIM2->SR1 &= ~TIM2_IT_UPDATE;
}

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  SecondFlag = 1;
  TIM1->SR1 &= ~TIM1_IT_UPDATE;
}


void External_Clock_Init (void)
{
  CLK->ECKR |= CLK_ECKR_HSEEN;                                                  //HSE enable
  CLK->SWCR |= CLK_SWCR_SWEN;                                                   //switching start
  while(!CLK->ECKR&CLK_ECKR_HSERDY);                                            //wait for HSE ready
  CLK->CKDIVR = 0;                                                              //divider = 0
  CLK->SWR = 0xB4;                                                              //HSE clock
  while(!CLK->SWCR&CLK_SWCR_SWIF);                                              //wait for switching
}

void IR_Enable (FunctionalState NewState)
{
  if (NewState != DISABLE) //ENABLE IR
  {
    TIM2->PSCR = TIM2_PRESCALER_2;
    TIM2->ARRH = 0;
    TIM2->ARRL = 203;
    TIM2->CCR3H = TIM2->ARRH>>1;
    TIM2->CCR3L = TIM2->ARRL>>1;
    TIM2->IER &= ~TIM2_IT_UPDATE;
    TIM2->EGR |= TIM2_EGR_UG;
    TIM2->CR1 |= TIM2_CR1_CEN;
    return;
  }
  else  //DISABLE IR
  {      
    TIM2->PSCR = TIM2_PRESCALER_16;
    TIM2->EGR |= TIM2_EGR_UG;
    TIM2_SetAutoreload(3000);
    TIM2->IER |= TIM2_IT_UPDATE;
    return;
  }  
}
