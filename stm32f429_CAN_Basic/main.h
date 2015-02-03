/**
  ******************************************************************************
  * @file    Touch_Panel/main.h 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include <stdio.h>
#include <math.h>
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ioe.h"
#include "stm32f429i_discovery_l3gd20.h"
#include "lcd_glitchless.h"
#include "lcd.h"
#include "can.h"

/* Private define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/ 
/* Exported functions ------------------------------------------------------- */

  
void ID_Pin_Initialization(void);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Initialization(void);
void LED_Initialization(void);
void LED3_Toggle(void);
void USART1_Configuration(void);
void USART1_puts(char* s);
void USART1_IRQHandler(void);
void SPI_Initialization(void);
void PWM_Initialization(void);
//void Timer5_Initialization(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM2_Initialization(void);
void TIM3_Initialization(void);
void LED_Initialization(void);

void Timer5_Initialization(void);
void TIM5_IRQHandler(void);

uint8_t PIN_ID_Read(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
