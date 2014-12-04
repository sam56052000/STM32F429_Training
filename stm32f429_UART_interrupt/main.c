
#include "main.h"

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
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

void LED3_Toggle(void){


  GPIOG->ODR ^= GPIO_Pin_13;

}

void LED4_Toggle(void){


  GPIOG->ODR ^= GPIO_Pin_14;

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

char delay_check=0;
void Delay_1us(uint32_t nCnt_1us)
{
  uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
  {
    for (nCnt = 45; nCnt != 0; nCnt--)
    {
      if(delay_check==1)
      {
        break;
      }
    }
    if(delay_check==1)
    {
      delay_check=0;
      break;
    }
  }
}


void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 57600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* NVIC Initialization */
    NVIC_InitTypeDef NVIC_InitStruct = {
      .NVIC_IRQChannel = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 0,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&NVIC_InitStruct);

}

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}
char q[5],t=0,t3=0,t4=0;
/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    LED_Initialization();
    USART1_puts("HW2 interrupt\r\n");
    USART1_puts("Just for STM32F429I Discovery verify USART1 with USB TTL Cable\r\n");
    while(1)
    {
      //set LED On and Off
      // if(t=='a')//check a
      // {
      //   while(1)//wait for interrupt
      //   {
      //     if(b==1)
      //     {
      //       b=0;
      //       break;
      //     }
      //   }
      //   if(t=='b')//check b
      //   {
      //     while(1)//wait for interrupt
      //     {
      //       if(b==1)
      //       {
      //         b=0;
      //         break;
      //       }
      //     }
      //     if(t=='c')//check c
      //     {
      //       while(1)//wait for interrupt
      //       {
      //         if(b==1)
      //         {
      //           b=0;
      //           break;
      //         }
      //       }
      //       if(t=='3')
      //       {
      //         while(1)//wait for interrupt
      //         {
      //           if(b==1)
      //           {
      //             b=0;
      //             break;
      //           }
      //         }
      //         if(t=='o')
      //         {
      //           //LED3 ON
      //           LED3_On();
      //           t3=0;
      //           t3check=0;
      //         }
      //         else if(t=='f')
      //         {
      //           //LED3 OFF
      //           LED3_Off();
      //           t3=0;
      //           t3check=0;
      //         }
      //       }
      //       else if(t=='4')
      //       {
      //         while(1)//wait for interrupt
      //         {
      //           if(b==1)
      //           {
      //             b=0;
      //             break;
      //           }
      //         }
      //         if(t=='o')
      //         {
      //           //LED4 ON
      //           LED4_On();
      //           t4=0;
      //           t4check=0;
      //         }
      //         else if(t=='f')
      //         {
      //           //LED4 OFF
      //           LED4_Off();
      //           t4=0;
      //           t4check=0;
      //         }
      //       }
      //     }
      //   }

      // }

      // if(t=='d')//check d
      // {
      //   while(1)//wait for interrupt
      //   {
      //     if(b==1)
      //     {
      //       b=0;
      //       break;
      //     }
      //   }
      //   if(t=='e')//check e
      //   {
      //     while(1)//wait for interrupt
      //     {
      //       if(b==1)
      //       {
      //         b=0;
      //         break;
      //       }
      //     }
      //     if(t=='f')//check f
      //     {
      //       while(1)//wait for interrupt
      //       {
      //         if(b==1)
      //         {
      //           b=0;
      //           break;
      //         }
      //       }
      //       if(t=='3')
      //       {
      //         t3=1;
      //       }
      //       else if(t=='4')
      //       {
      //         t4=1;
      //       }
      //     }
      //   }
      // }
      if(q[0]==97)
      {
        if(q[1]==98)
        {
          if(q[2]==99)
          {
            if(q[3]=='3')
            {
              if(q[4]==111)
              {
                LED3_On();
                t3=0;
              }
              else if(q[4]==102)
              {
                LED3_Off();
                t3=0;
              }
            }
            else if(q[3]=='4')
            {
              if(q[4]==111)
              {
                LED4_On();
                t4=0;
              }
              else if(q[4]==102)
              {
                LED4_Off();
                t4=0;
              }
            }
          }
        }
      }
      if(q[1]==100)
      {
        if(q[2]==101)
        {
          if(q[3]==102)
          {
            if(q[4]=='3')
            {
              t3=1;
            }
            else if(q[4]=='4')
            {
              t4=1;
            }
          }
        }
      }
      if(q[0]==100)
      {
        if(q[1]==101)
        {
          if(q[2]==102)
          {
            if(q[3]=='3')
            {
              t3=1;
            }
            else if(q[4]=='4')
            {
              t4=1;
            }
          }
        }
      }
      if(t3==1)
      {
        LED3_Toggle();
        if(t4==1)
        {
          Delay_1us(5000);
        }
        else
        {
          Delay_1us(10000);
        }
      }
      if(t4==1)
      {
        LED4_Toggle();
        if(t3==1)
        {
          Delay_1us(5000);
        }
        else
        {
          Delay_1us(10000);
        }
      }
    }
}


void USART1_IRQHandler(void)
{
  
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
  {
    uart1_data = USART_ReceiveData(USART1);
    USART_SendData(USART1, uart1_data);
    t = USART_ReceiveData(USART1);
    delay_check=1;//break to delay
    q[0]=q[1];
    q[1]=q[2];
    q[2]=q[3];
    q[3]=q[4];
    q[4]=t;
  }

}