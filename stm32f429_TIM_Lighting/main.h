
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
void LED3_Off(void);
void LED4_Toggle(void);
void LED4_On(void);
void LED4_Off(void);
void Timer4_Initialization(void);
void Timer5_Initialization(void);
void TIM5_IRQHandler(void);
void TIM4_IRQHandler(void);

static inline void Delay_1us(uint32_t);


#endif /* __MAIN_H */


