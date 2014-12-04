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
 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration for Push Button ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD ;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
 
/**************************************************************************************/
 
void LED_Initialization(void){

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE); //LED3/4 GPIO Port

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;  // LED is connected to PG13/PG14
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

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


  GPIO_ToggleBits(GPIOG,GPIO_Pin_13);

}

void LED4_Toggle(void){


  GPIO_ToggleBits(GPIOG,GPIO_Pin_14);

}
///////////////////////////////////////////////////////////
void v13_on(void)
{
  GPIO_SetBits(GPIOG,GPIO_Pin_13);
}

void v13_off(void)
{
  GPIO_ResetBits(GPIOG,GPIO_Pin_13);
}

void v14_on(void)
{
  GPIO_SetBits(GPIOG,GPIO_Pin_14);
}

void v14_off(void)
{
  GPIO_ResetBits(GPIOG,GPIO_Pin_14);
}

void v15_on(void)
{
  GPIO_SetBits(GPIOG,GPIO_Pin_15);
}

void v15_off(void)
{
  GPIO_ResetBits(GPIOG,GPIO_Pin_15);
}

uint8_t PushButton_Read(void){

    return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

}
/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    LED_Initialization();
    uint8_t i=1,c=1;
    while(1)
    {

      //sevro
      if(PushButton_Read())
      {
        if(c==1)
        {
          c=0;
          if(PushButton_Read())
          {
            if(i==1)
            {
              i=2;
            }
            else if(i==2)
            {
              i=3;
            }
            else
            {
              i=1;
            }
          }
        }
      }
      if(PushButton_Read())
      {
        c=0;
      }
      else
      {
        c=1;
      }
      

      //
        if(i==1)
        {
          v13_on();
          v14_on();
          v15_on();
          Delay_1us(1000);
          v13_off();
          Delay_1us(500);
          v14_off();
          Delay_1us(500);
          v15_off();
          Delay_1us(18500);
        }
        if(i==2)
        {
          v13_on();
          v14_on();
          v15_on();
          Delay_1us(1000);
          v14_off();
          Delay_1us(500);
          v15_off();
          Delay_1us(500);
          v13_off();
          Delay_1us(18500);
        }
        if(i==3)
        {
          v13_on();
          v14_on();
          v15_on();
          Delay_1us(1000);
          v15_off();
          Delay_1us(500);
          v13_off();
          Delay_1us(500);
          v14_off();
          Delay_1us(18500);
        }
      
      // if(PushButton_Read())
      // {
      //   LED4_Off();
      //   for(i=0;i<90;i++)
      //   {
      //     LED3_Toggle();
      //     Delay_1us(100000-i*1000);

      //     if (PushButton_Read())
      //     {

      //     }
      //     else
      //       break;
      //   }
      // }
      // else
      // {

      //   LED4_Toggle();
      // Delay_1us(100000);
      //   LED3_Off();
      // }
      

    }

    while(1); // Don't want to exit
}

