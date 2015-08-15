
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "terminal.h"
#include <stdio.h>
#include "lcd.h"

void USART1_Configuration(void);
void USART1_puts(char* s);
void USART1_IRQHandler(void);
void SPI_Initialization(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);
static inline void Delay_1us(uint32_t nCnt_1us);
uint8_t uart1_data; 

#endif /* __MAIN_H */


