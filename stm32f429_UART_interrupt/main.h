
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

 #include "stm32f4xx.h"
#include <stdio.h>

uint8_t uart1_data;


void RCC_Configuration(void);
void GPIO_Configuration(void);
void LED_Initialization(void);
void LED3_Toggle(void);
void LED3_On(void);
void LED3_Off(void);
void LED4_Toggle(void);
void LED4_On(void);
void LED4_Off(void);
void USART1_Configuration(void);
void USART1_puts(char* s);
void USART1_IRQHandler(void);

void Trans_SBUS_data(void);
void Inv_Trans_SBUS_data(void);
void Send_SBUS_signal(void);

static inline void Delay_1us(uint32_t);
// static inline void Delay_1us(uint32_t nCnt_1us)
// {
//   volatile uint32_t nCnt;

//   for (; nCnt_1us != 0; nCnt_1us--)
//     for (nCnt = 45; nCnt != 0; nCnt--);
// }

#endif /* __MAIN_H */


