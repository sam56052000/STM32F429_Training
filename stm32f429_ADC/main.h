
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

 #include "stm32f4xx.h"
// #include "stm32f429i_discovery.h"
// #include "stm32f429i_discovery_lcd.h"
// #include "stm32f429i_discovery_ioe.h"
// #include "stm32f429i_discovery_lcd.c"

// void DrawThickCircle(uint32_t x,uint32_t y,uint32_t radius, uint32_t thickness);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Initialization(void);
void LED_Initialization(void);
void LED3_Toggle(void);
void USART1_Configuration(void);
void USART1_puts(char* s);
static inline void Delay_1us(uint32_t);

#endif /* __MAIN_H */


