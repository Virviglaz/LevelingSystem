#include "stm8s.h"

void PIN_ON(GPIO_TypeDef * GPIOx,u8 PINx);
void PIN_OFF(GPIO_TypeDef * GPIOx,u8 PINx);
u8 PIN_SYG(GPIO_TypeDef * GPIOx, u8 PINx);
void PIN_IN (GPIO_TypeDef * GPIOx,u8 PINx);
void PIN_OUT_PP (GPIO_TypeDef * GPIOx,u8 PINx);
void PIN_INV(GPIO_TypeDef * GPIOx, u8 PINx);
void PIN_IN_PU (GPIO_TypeDef * GPIOx,u8 PINx);
u8 PIN_SYG_FILTERED (GPIO_TypeDef * GPIOx,u8 PINx);
u8 PIN_SYG_FILTERED_WORD (GPIO_TypeDef * GPIOx,u8 PINx);