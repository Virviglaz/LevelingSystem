#ifndef MAIN_H
#define MAIN_H

#include "stm8s.h"
#define _STDINT

#define LED             GPIOD, GPIO_PIN_5
#define JP1             GPIOD, GPIO_PIN_4
#define OW_PIN          GPIO_PIN_3
#define OW_GPIO         GPIOC
#define OW_LOW()        OW_GPIO->DDR |= OW_PIN
#define OW_HIGH()       OW_GPIO->DDR &= ~OW_PIN
#define FallC           0x20
#define RiseC           0x10
#define TIM4_ON         TIM4->IER |= TIM4_IT_UPDATE
#define TIM4_OFF        TIM4->IER &= ~TIM4_IT_UPDATE;

#define SEARCH_ROM      0xF0
#define MATCH_ROM       0x55
#define READ_STRATCHPAD	0xBE
#define CONVERT_CMD	0x44

#define SI7005_I2C_Address      0x80

typedef enum {
  OW_RESET = 1,
  OW_START_PRESENTS_PULSE,      //2
  OW_END_PRESENTS_PULSE,        //3
  OW_COMM,                      //4
  OW_SN_READ,                   //5
  OW_DATA_READ,                 //6
  OW_DATA_WRITE                 //7
} OW_TypeDef;

typedef struct {
  volatile OW_TypeDef Mode;
  uint8_t COMMAND;
  volatile uint8_t COMMAND_CNT;
  uint8_t ERROR;
  uint8_t DATA[9];
  uint8_t SN[8];
  uint8_t SN_COMP;
  uint8_t SNi;
  uint8_t ByteCNT;
  uint8_t BitCNT;
  volatile uint8_t StartConvFlag;
} OW_ModeTypeDef;

#endif

