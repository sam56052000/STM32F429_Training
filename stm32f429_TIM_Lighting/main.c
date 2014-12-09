#include "main.h"

static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

 
void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
 
 
/**************************************************************************************/
 
void LED_Initialization(void){

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE); //LED3/4 GPIO Port

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;  // LED is connected to PG13/PG14
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

}


void Timer4_Initialization(void)
{

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =  TIM4_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM4);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = 1000 - 1 ;  //250ms  --> 4Hz
  TIM_TimeBaseStruct.TIM_Prescaler = 135 - 1; // Prescaled by 1800 -> = 0.1M(10us)
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Div by one -> 90 MHz (Now RCC_DCKCFGR_TIMPRE is configured to divide clock by two)
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Down;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}


void Timer5_Initialization(void)
{

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM5 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =  TIM5_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM5);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = 10000 - 1 ;  //250ms  --> 4Hz
  TIM_TimeBaseStruct.TIM_Prescaler = 180 - 1; // Prescaled by 90 -> = 0.1M(10us)
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Div by one -> 90 MHz (Now RCC_DCKCFGR_TIMPRE is configured to divide clock by two)
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM5, DISABLE);       //Put ARR value into register
  TIM_Cmd(TIM5, ENABLE);
}

void Timer6_Initialization(void)
{

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM5 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel =  TIM6_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM5);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = 10000 - 1 ;  //250ms  --> 4Hz
  TIM_TimeBaseStruct.TIM_Prescaler = 180 - 1; // Prescaled by 90 -> = 0.1M(10us)
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Div by one -> 90 MHz (Now RCC_DCKCFGR_TIMPRE is configured to divide clock by two)
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStruct);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM6, DISABLE);       //Put ARR value into register
  TIM_Cmd(TIM6, ENABLE);
}


void LED3_On(void){

  GPIO_SetBits(GPIOG,GPIO_Pin_13);

}

void LED3_Off(void){

  GPIO_ResetBits(GPIOG,GPIO_Pin_13);

}

void LED4_On(void){

  GPIO_SetBits(GPIOG,GPIO_Pin_14);

}

void LED4_Off(void){

  GPIO_ResetBits(GPIOG,GPIO_Pin_14);

}

void LED3_Toggle(void){


  GPIOG->ODR ^= GPIO_Pin_13;

}

void LED4_Toggle(void){


  GPIOG->ODR ^= GPIO_Pin_14;

}

/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    LED_Initialization();
    Timer4_Initialization();
    Timer5_Initialization();
    while(1)
    {
        // this loop is doing nothing but smiling at you :)
    }
}



void TIM4_IRQHandler()
{
        if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){

            TIM_Cmd(TIM4, DISABLE);
            GPIO_ResetBits(GPIOG,GPIO_Pin_13);
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        }
}

uint16_t HighTime=999; //control high time
uint8_t Direction=0,i=1;     // control direction of amplitude change

void TIM5_IRQHandler()
{
        if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
          //  if(Direction == 0){

          //        HighTime -= 100;

          //        if(HighTime < 300){
          //         Direction =1;
          //        }
          // }else if(Direction == 1){

          //        HighTime += 100;
          //        if(HighTime > 23000){
          //         Direction =0;
          //        }
          }
            GPIO_SetBits(GPIOG,GPIO_Pin_13);
            GPIO_SetBits(GPIOG,GPIO_Pin_14);
           TIM_SetCounter(TIM4, HighTime);
            TIM_Cmd(TIM4, ENABLE);
            if(i==1)
            {
              TIM_SetCounter(TIM6, 666);
              TIM_Cmd(TIM6, ENABLE);
              i=2;
            }
            else if(i==2)
            {
              TIM_SetCounter(TIM6, 999);
              TIM_Cmd(TIM6, ENABLE);
              i=3;
            }
            else if(i==3)
            {
              TIM_SetCounter(TIM6, 1332);
              TIM_Cmd(TIM6, ENABLE);
              i=1;
            }
            

        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        
}

void TIM6_IRQHandler()
{
        if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){

            TIM_Cmd(TIM6, DISABLE);
            GPIO_ResetBits(GPIOG,GPIO_Pin_14);
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        }
}

