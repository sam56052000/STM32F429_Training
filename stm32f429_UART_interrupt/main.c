
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

// char delay_check=0;
void Delay_1us(uint32_t nCnt_1us)
{
  uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
  {
    for (nCnt = 14; nCnt != 0; nCnt--)
    {
      // if(delay_check==1)
      // {
      //   break;
      // }
    }
    // if(delay_check==1)
    // {
    //   delay_check=0;
    //   break;
    // }
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
    USART_InitStructure.USART_BaudRate = 100000;
    //USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
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

static uint8_t sbusFrameIndex=0;
static uint8_t sBUScapturingFlag=0;
static uint8_t receivedFrameFlag=0;
static uint8_t sbusBuffer[25];

static int16_t sbus_rc_data[17];
static int16_t sbus_rc_data_RX[17];
static uint8_t sbusSendBuffer[25];
static uint8_t sbusReceivedBuffer[25];
uint8_t buff_transmit[100];
uint8_t SBUS_start_byte=0x0F;
uint8_t SBUS_end_byte=0x00;
uint8_t SBUS_flags=0x00;

/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    LED_Initialization();
    USART1_puts("SBUS interrupt\r\n");
    USART1_puts("Just for STM32F429I Discovery verify USART1 with USB TTL Cable\r\n");

    //Set RC Data
    // sbus_rc_data[1]=1500;
    // sbus_rc_data[2]=1501;
    // sbus_rc_data[3]=1502;
    // sbus_rc_data[4]=1503;
    // sbus_rc_data[5]=1504;
    // sbus_rc_data[6]=1505;
    // sbus_rc_data[7]=1506;
    // sbus_rc_data[8]=1507;
    // sbus_rc_data[9]=1508;
    // sbus_rc_data[10]=1500;
    // sbus_rc_data[11]=1500;
    // sbus_rc_data[12]=1500;
    // sbus_rc_data[13]=1500;
    // sbus_rc_data[14]=1500;
    // sbus_rc_data[15]=1500;
    // sbus_rc_data[16]=1500;



    //Set Start,End Byte
    sbusSendBuffer[0]=SBUS_start_byte;
    sbusSendBuffer[23]=SBUS_flags;
    sbusSendBuffer[24]=SBUS_end_byte;


    while(1)
    {
      //================SBUS TEST Read Receiver and Output signal===================
      if(receivedFrameFlag==1)
      {
        receivedFrameFlag=0;
        Inv_Trans_SBUS_data();

        uint8_t qi;
        for(qi=1;qi<17;qi++)
        {
          sbus_rc_data[qi]=sbus_rc_data_RX[qi];
        }
      }
      Trans_SBUS_data();
      Send_SBUS_signal();
      Delay_1us(12000);

      //================Test Buffer Trans and Inv-trans=============================
      // sbusReceivedBuffer[1]=0xA1;
      // sbusReceivedBuffer[2]=0xA2;
      // sbusReceivedBuffer[3]=0xA3;
      // sbusReceivedBuffer[4]=0xA4;
      // sbusReceivedBuffer[5]=0xA5;
      // sbusReceivedBuffer[6]=0xA6;
      // sbusReceivedBuffer[7]=0xA7;
      // sbusReceivedBuffer[8]=0xA8;
      // sbusReceivedBuffer[9]=0xA9;
      // sbusReceivedBuffer[10]=0xB0;
      // sbusReceivedBuffer[11]=0xB1;

      // sprintf((char *)buff_transmit, "RX=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",sbusReceivedBuffer[1],sbusReceivedBuffer[2],sbusReceivedBuffer[3],sbusReceivedBuffer[4],sbusReceivedBuffer[5],sbusReceivedBuffer[6],sbusReceivedBuffer[7],sbusReceivedBuffer[8],sbusReceivedBuffer[9],sbusReceivedBuffer[10],sbusReceivedBuffer[11]);
      // USART1_puts((char *)buff_transmit);
      // Inv_Trans_SBUS_data();

      // sbus_rc_data[1]=sbus_rc_data_RX[1];
      // sbus_rc_data[2]=sbus_rc_data_RX[2];
      // sbus_rc_data[3]=sbus_rc_data_RX[3];
      // sbus_rc_data[4]=sbus_rc_data_RX[4];
      // sbus_rc_data[5]=sbus_rc_data_RX[5];
      // sbus_rc_data[6]=sbus_rc_data_RX[6];
      // sbus_rc_data[7]=sbus_rc_data_RX[7];
      // sbus_rc_data[8]=sbus_rc_data_RX[8];

      // Trans_SBUS_data();
      // sprintf((char *)buff_transmit, "TX=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",sbusSendBuffer[1],sbusSendBuffer[2],sbusSendBuffer[3],sbusSendBuffer[4],sbusSendBuffer[5],sbusSendBuffer[6],sbusSendBuffer[7],sbusSendBuffer[8],sbusSendBuffer[9],sbusSendBuffer[10],sbusSendBuffer[11]);
      // USART1_puts((char *)buff_transmit);
     
      
      
    //=====================USART Homework===========================================
    //   if(q[0]==97)
    //   {
    //     if(q[1]==98)
    //     {
    //       if(q[2]==99)
    //       {
    //         if(q[3]=='3')
    //         {
    //           if(q[4]==111)
    //           {
    //             LED3_On();
    //             t3=0;
    //           }
    //           else if(q[4]==102)
    //           {
    //             LED3_Off();
    //             t3=0;
    //           }
    //         }
    //         else if(q[3]=='4')
    //         {
    //           if(q[4]==111)
    //           {
    //             LED4_On();
    //             t4=0;
    //           }
    //           else if(q[4]==102)
    //           {
    //             LED4_Off();
    //             t4=0;
    //           }
    //         }
    //       }
    //     }
    //   }
    //   if(q[1]==100)
    //   {
    //     if(q[2]==101)
    //     {
    //       if(q[3]==102)
    //       {
    //         if(q[4]=='3')
    //         {
    //           t3=1;
    //         }
    //         else if(q[4]=='4')
    //         {
    //           t4=1;
    //         }
    //       }
    //     }
    //   }
    //   if(q[0]==100)
    //   {
    //     if(q[1]==101)
    //     {
    //       if(q[2]==102)
    //       {
    //         if(q[3]=='3')
    //         {
    //           t3=1;
    //         }
    //         else if(q[4]=='4')
    //         {
    //           t4=1;
    //         }
    //       }
    //     }
    //   }
    //   if(t3==1)
    //   {
    //     LED3_Toggle();
    //     if(t4==1)
    //     {
    //       Delay_1us(5000);
    //     }
    //     else
    //     {
    //       Delay_1us(10000);
    //     }
    //   }
    //   if(t4==1)
    //   {
    //     LED4_Toggle();
    //     if(t3==1)
    //     {
    //       Delay_1us(5000);
    //     }
    //     else
    //     {
    //       Delay_1us(10000);
    //     }
    //   }
     }
}


void USART1_IRQHandler(void)
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
  {
    char c;
    uint8_t i;
    c = USART_ReceiveData(USART1);

    if(c == 0x0F)
    {
      sBUScapturingFlag=1;

      sbusBuffer[sbusFrameIndex] = c;
      sbusFrameIndex++;
    }
    else if((sBUScapturingFlag ==1) && (sbusFrameIndex<24))
    {
      sbusBuffer[sbusFrameIndex] = c;
      sbusFrameIndex++;

    }
    else if((sBUScapturingFlag ==1) && (sbusFrameIndex==24) && ((c == 0x04) || (c==0x14) || (c==0x24) || (c==0x34)))
    {
        sbusBuffer[sbusFrameIndex] = c;
        for(i=0;i<25;i++)
        {
          sbusReceivedBuffer[i] = sbusBuffer[i];
        }
        receivedFrameFlag=1;
        sbusFrameIndex=0;
        sBUScapturingFlag=0;
        LED3_Toggle();
    }
    else
    {
      sbusFrameIndex=0;
      sBUScapturingFlag=0;
    }
  }
}

void Trans_SBUS_data(void)
{
  sbusSendBuffer[1]=(sbus_rc_data[1] & 0x00FF);
  sbusSendBuffer[2]=((sbus_rc_data[1] & 0x0700)>>8)|((sbus_rc_data[2] & 0x001F)<<3);
  sbusSendBuffer[3]=((sbus_rc_data[2] & 0x07E0)>>5)|((sbus_rc_data[3] & 0x0003)<<6);
  sbusSendBuffer[4]=((sbus_rc_data[3] & 0x03FC)>>2);
  sbusSendBuffer[5]=((sbus_rc_data[3] & 0x0400)>>10)|((sbus_rc_data[4] & 0x007F)<<1);
  sbusSendBuffer[6]=((sbus_rc_data[4] & 0x0780)>>7)|((sbus_rc_data[5] & 0x000F)<<4);
  sbusSendBuffer[7]=((sbus_rc_data[5] & 0x07F0)>>4)|((sbus_rc_data[6] & 0x0001)<<7);
  sbusSendBuffer[8]=((sbus_rc_data[6] & 0x01FE)>>1);
  sbusSendBuffer[9]=((sbus_rc_data[6] & 0x0600)>>9)|((sbus_rc_data[7] & 0x003F)<<2);
  sbusSendBuffer[10]=((sbus_rc_data[7] & 0x07C0)>>6)|((sbus_rc_data[8] & 0x0007)<<5);
  sbusSendBuffer[11]=((sbus_rc_data[8] & 0x07F8)>>3);

  sbusSendBuffer[12]=(sbus_rc_data[9] & 0x00FF);
  sbusSendBuffer[13]=((sbus_rc_data[9] & 0x0700)>>8)|((sbus_rc_data[10] & 0x001F)<<3);
  sbusSendBuffer[14]=((sbus_rc_data[10] & 0x07E0)>>5)|((sbus_rc_data[11] & 0x0003)<<6);
  sbusSendBuffer[15]=((sbus_rc_data[11] & 0x03FC)>>2);
  sbusSendBuffer[16]=((sbus_rc_data[11] & 0x0400)>>10)|((sbus_rc_data[12] & 0x007F)<<1);
  sbusSendBuffer[17]=((sbus_rc_data[12] & 0x0780)>>7)|((sbus_rc_data[13] & 0x000F)<<4);
  sbusSendBuffer[18]=((sbus_rc_data[13] & 0x07F0)>>4)|((sbus_rc_data[14] & 0x0001)<<7);
  sbusSendBuffer[19]=((sbus_rc_data[14] & 0x01FE)>>1);
  sbusSendBuffer[20]=((sbus_rc_data[14] & 0x0600)>>9)|((sbus_rc_data[15] & 0x003F)<<2);
  sbusSendBuffer[21]=((sbus_rc_data[15] & 0x07C0)>>6)|((sbus_rc_data[16] & 0x0007)<<5);
  sbusSendBuffer[22]=((sbus_rc_data[16] & 0x07F8)>>3);
}

void Inv_Trans_SBUS_data(void)
{
  sbus_rc_data_RX[1] = ((int16_t)sbusReceivedBuffer[1] & 0xFFFF) | ((int16_t) (sbusReceivedBuffer[2] & 0x0007) << 8);
  sbus_rc_data_RX[2] = ((int16_t)sbusReceivedBuffer[2] & 0x00F8 ) >> 3 | ((int16_t) (sbusReceivedBuffer[3] & 0x003F) << 5);
  sbus_rc_data_RX[3] = ((int16_t)sbusReceivedBuffer[3] & 0x00C0 ) >> 6 | ((int16_t) (sbusReceivedBuffer[4]) << 2 ) | ((int16_t) (sbusReceivedBuffer[5] & 0x0001) << 10);
  sbus_rc_data_RX[4] = ((int16_t)sbusReceivedBuffer[5] & 0x00FE ) >> 1 | ((int16_t) (sbusReceivedBuffer[6] & 0x000F) << 7);
  sbus_rc_data_RX[5] = ((int16_t)sbusReceivedBuffer[6] & 0x00F0 ) >> 4 | ((int16_t) (sbusReceivedBuffer[7] & 0x007F) << 4);
  sbus_rc_data_RX[6] = ((int16_t)sbusReceivedBuffer[7] & 0x0080 ) >> 7 | ((int16_t) (sbusReceivedBuffer[8]) << 1 ) | ((int16_t) (sbusReceivedBuffer[9] & 0x0003) << 9);
  sbus_rc_data_RX[7] = ((int16_t)sbusReceivedBuffer[9] & 0x00FC ) >> 2 | ((int16_t) (sbusReceivedBuffer[10] & 0x001F) << 6);
  sbus_rc_data_RX[8] = ((int16_t)sbusReceivedBuffer[10] & 0x00E0 ) >> 5 | ((int16_t) (sbusReceivedBuffer[11] ) << 3);

  sbus_rc_data_RX[9] = ((int16_t)sbusReceivedBuffer[12] & 0xFFFF) | ((int16_t) (sbusReceivedBuffer[13] & 0x0007) << 8);
  sbus_rc_data_RX[10] = ((int16_t)sbusReceivedBuffer[13] & 0x00F8 ) >> 3 | ((int16_t) (sbusReceivedBuffer[14] & 0x003F) << 5);
  sbus_rc_data_RX[11] = ((int16_t)sbusReceivedBuffer[14] & 0x00C0 ) >> 6 | ((int16_t) (sbusReceivedBuffer[15]) << 2 ) | ((int16_t) (sbusReceivedBuffer[5] & 0x0001) << 10);
  sbus_rc_data_RX[12] = ((int16_t)sbusReceivedBuffer[16] & 0x00FE ) >> 1 | ((int16_t) (sbusReceivedBuffer[17] & 0x000F) << 7);
  sbus_rc_data_RX[13] = ((int16_t)sbusReceivedBuffer[17] & 0x00F0 ) >> 4 | ((int16_t) (sbusReceivedBuffer[18] & 0x007F) << 4);
  sbus_rc_data_RX[14] = ((int16_t)sbusReceivedBuffer[18] & 0x0080 ) >> 7 | ((int16_t) (sbusReceivedBuffer[19]) << 1 ) | ((int16_t) (sbusReceivedBuffer[9] & 0x0003) << 9);
  sbus_rc_data_RX[15] = ((int16_t)sbusReceivedBuffer[20] & 0x00FC ) >> 2 | ((int16_t) (sbusReceivedBuffer[21] & 0x001F) << 6);
  sbus_rc_data_RX[16] = ((int16_t)sbusReceivedBuffer[21] & 0x00E0 ) >> 5 | ((int16_t) (sbusReceivedBuffer[22] ) << 3);
}

void Send_SBUS_signal(void)
{
  uint8_t ii;
  for(ii=0;ii<25;ii++)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1,sbusSendBuffer[ii]);
  }
}