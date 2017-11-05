#include "main.h"
#include "stm8s.h"

extern OW_ModeTypeDef OW;

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  TIM4->SR1 = (uint8_t)(~TIM4_IT_UPDATE);
  
  if (OW.Mode == OW_START_PRESENTS_PULSE)  //start presents pulse
  {
    OW_LOW();
    TIM4->CNTR = 0xFF - 27; //100uS
    OW.Mode = OW_END_PRESENTS_PULSE;
    return;
  }
  
  if (OW.Mode == OW_END_PRESENTS_PULSE)  //end presents pulse
  {
    OW_HIGH();
    TIM4_OFF;    
    OW.Mode = OW_COMM;              //switch next phase 
    OW.COMMAND_CNT = 0;
    OW.COMMAND = 0;
    OW.SNi = 0;
    OW.SN_COMP = 0;
    OW.BitCNT = 0;
    OW.ByteCNT = 0;
    return;
  }
      
  if ((OW.Mode == OW_COMM) || (OW.Mode == OW_SN_READ) || (OW.Mode == OW_DATA_READ))
  {
    if (OW.COMMAND_CNT == 0) OW.COMMAND = 0;
    if (OW_GPIO->IDR & OW_PIN) OW.COMMAND |= (1 << OW.COMMAND_CNT);
    OW.COMMAND_CNT ++;
    TIM4_OFF; 
    if (OW.COMMAND_CNT == 8)
    {
      OW.COMMAND_CNT = 0;
      
      if (OW.Mode == OW_COMM)
      {
        if (OW.COMMAND == MATCH_ROM)        
          OW.Mode = OW_SN_READ;  
        else
          OW.Mode = OW_RESET;
        return;
      }
      
      if (OW.Mode == OW_SN_READ)
      {             
        if (OW.COMMAND == OW.SN[OW.SNi]) OW.SN_COMP ++;        
        OW.SNi++;     
        if (OW.SNi == 8)
        {
          if (OW.SN_COMP == 8) OW.Mode = OW_DATA_READ;
          else OW.Mode = OW_RESET;            
        }
        return;
      }
      
      if (OW.Mode == OW_DATA_READ)
      {
        if (OW.COMMAND == CONVERT_CMD)  OW.StartConvFlag = SET;        
        if (OW.COMMAND == READ_STRATCHPAD) {OW.Mode = OW_DATA_WRITE; return;}
        OW.Mode = OW_RESET;
        return;
      }
    }
  }
  
  if (OW.Mode == OW_DATA_WRITE)
  {
    OW_HIGH();
    TIM4_OFF;
    OW.BitCNT++;
    if (OW.BitCNT == 8) { OW.BitCNT = 0; OW.ByteCNT ++; }
    if (OW.ByteCNT == 9) 
      OW.Mode = OW_RESET;
    return;
  }
}

 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
 {
  TIM2->SR1 = (uint8_t)(~TIM2_IT_UPDATE);
  OW.Mode = OW_RESET;
  OW.COMMAND_CNT = 0;
 }

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  u8 PinState = OW_GPIO->IDR & OW_PIN;
  if (PinState) TIM2->IER &= ~TIM2_IER_UIE;
  else 
  {
    TIM4_ON; 
    TIM2->CNTRH = 0; 
    TIM2->CNTRL = 0;
  }

  if (OW.Mode == OW_RESET)                                                      //RESET
  {
    if (PinState)    //rise                                                     //END RESET PULSE
    {
      if (TIM4->CNTR > 120) OW.Mode = OW_START_PRESENTS_PULSE;       
      TIM4->CNTR = 0xFF - 10;  
      EXTI->CR1 = FallC;
    }
    else         //fall                                                         //START RESET PULSE
    {
      TIM4->CNTR = 0;  
      TIM4_ON;
      EXTI->CR1 = FallC | RiseC;
    }
    return;
  }
  
  if ((OW.Mode == OW_COMM) || (OW.Mode == OW_SN_READ) || (OW.Mode == OW_DATA_READ)) //READ ONE WIRE
  {
    if (PinState) return;
    TIM4->CNTR = 0xFF - 5; //20us (5x4)
    TIM4_ON;      
    return;
  }
  
  if (OW.Mode == OW_DATA_WRITE)                                                  //WRITE ONE WIRE
  {
    if (PinState) return;
    if ((OW.DATA[OW.ByteCNT] & (1 << OW.BitCNT)) == 0) OW_LOW();
    TIM4->CNTR = 0xFF - 10; //40us (10x4)
    TIM4_ON;
    return;
  }
}
