
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

 #include "stm32f4xx.h"

#include "stm32f4xx_tim.h"


void RCC_Configuration(void);
void GPIO_Configuration(void);
void LED_Initialization(void);
void LED3_Toggle(void);
void LED3_On(void);
void PWM_Initialization(void);
void USART1_Configuration(void);
void USART1_puts(char* s);
void USART1_IRQHandler(void);
void Trans16to10(void);

static inline void Delay_1us(uint32_t);


#endif /* __MAIN_H */


