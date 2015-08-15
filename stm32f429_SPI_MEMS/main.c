#include "main.h"

static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

void SPI_Initialization(void){

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);


  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPI5);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz)  */

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI5, &SPI_InitStructure);

  /* Enable SPI4  */
  SPI_Cmd(SPI5, ENABLE);
  
  /* Configure GPIO PIN for Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_SPI5);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_SPI5);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_SPI5);

  /* Configure GPIO PIN for SPI4 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOF, &GPIO_InitStructure);


}

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

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

uint8_t buff_transmit[100],i=0;
uint8_t XL,XH,YL,YH,ZL,ZH;


int main(void)
{

  //char lcd_text_main[100];
  //uint32_t runner=0;
  uint8_t receivedData=0;

    // lcd_init();
    // lcd_drawBackground(20,60,250);
    // lcd_drawBGPersimmon(20, 60, 250);
    // LCD_SetColors(LCD_COLOR_WHITE-1,LCD_COLOR_WHITE);
    // LCD_SetFont(&Font8x12); 
    // terminalBufferInitilization();
    
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    SPI_Initialization();

    //Write CTRL_REG5
    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    SPI_I2S_SendData(SPI5,0x24);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);

    SPI_I2S_SendData(SPI5,0x40);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    GPIO_SetBits(GPIOC,GPIO_Pin_1);

    //Read CTRL_REG5
    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    SPI_I2S_SendData(SPI5,0xa4);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    SPI_I2S_SendData(SPI5,0xff);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    GPIO_SetBits(GPIOC,GPIO_Pin_1);

    sprintf((char *)buff_transmit, "CTRL_REG5 Value: %d \r\n",receivedData);
    USART1_puts((char *)buff_transmit);

    //Write FIFO_CTRL_REG(Bypass mode)
    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    SPI_I2S_SendData(SPI5,0x2e);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);

    SPI_I2S_SendData(SPI5,0x00);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    GPIO_SetBits(GPIOC,GPIO_Pin_1);

    //Read FIFO_CTRL_REG
    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    SPI_I2S_SendData(SPI5,0xae);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    SPI_I2S_SendData(SPI5,0xff);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    GPIO_SetBits(GPIOC,GPIO_Pin_1);

    sprintf((char *)buff_transmit, "FIFO_CTRL_REG Value: %d \r\n",receivedData);
    USART1_puts((char *)buff_transmit);

    //Write CTRL_REG1
    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    SPI_I2S_SendData(SPI5,0x20);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);

    SPI_I2S_SendData(SPI5,0x0f);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    GPIO_SetBits(GPIOC,GPIO_Pin_1);

    //Read CTRL_REG1
    GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    SPI_I2S_SendData(SPI5,0xa0);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    SPI_I2S_SendData(SPI5,0xff);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    receivedData=SPI_I2S_ReceiveData(SPI5);
    GPIO_SetBits(GPIOC,GPIO_Pin_1);

    sprintf((char *)buff_transmit, "CTRL_REG1 Value: %d \r\n",receivedData);
    USART1_puts((char *)buff_transmit);


    // //Write CTRL_REG3
    // GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    // SPI_I2S_SendData(SPI5,0x23);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    // receivedData=SPI_I2S_ReceiveData(SPI5);

    // SPI_I2S_SendData(SPI5,0xe0);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    // receivedData=SPI_I2S_ReceiveData(SPI5);
    // GPIO_SetBits(GPIOC,GPIO_Pin_1);

    // //Read CTRL_REG3
    // GPIO_ResetBits(GPIOC,GPIO_Pin_1);
    // SPI_I2S_SendData(SPI5,0xa3);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    // receivedData=SPI_I2S_ReceiveData(SPI5);
    // SPI_I2S_SendData(SPI5,0xff);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
    // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
    // receivedData=SPI_I2S_ReceiveData(SPI5);
    // GPIO_SetBits(GPIOC,GPIO_Pin_1);

    // sprintf((char *)buff_transmit, "CTRL_REG1 Value: %d \r\n",receivedData);
    // USART1_puts((char *)buff_transmit);

    while(1){

      receivedData=0;
      // sprintf(lcd_text_main,"SPI Getdata\n\n"); terminalWrite(lcd_text_main);

      //Get XL Data
      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      SPI_I2S_SendData(SPI5,0xa8);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);

      SPI_I2S_SendData(SPI5,0xff);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);
      
      GPIO_SetBits(GPIOC,GPIO_Pin_1);
      //sprintf(lcd_text_main,"XLData : %X    \n\n", receivedData); terminalWrite(lcd_text_main);
      XL=receivedData;


      //Get XH Data
      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      SPI_I2S_SendData(SPI5,0xa9);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);

      SPI_I2S_SendData(SPI5,0xff);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);
      
      GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // sprintf(lcd_text_main,"XHData : %X    \n\n\n", receivedData); terminalWrite(lcd_text_main);
      XH=receivedData; 


      //Get YL Data
      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      SPI_I2S_SendData(SPI5,0xaa);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);

      SPI_I2S_SendData(SPI5,0xff);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);
      
      GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // sprintf(lcd_text_main,"YLData : %X    \n\n", receivedData); terminalWrite(lcd_text_main);
      YL=receivedData; 


      //Get YH Data
      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      SPI_I2S_SendData(SPI5,0xab);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);

      SPI_I2S_SendData(SPI5,0xff);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);
      
      GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // sprintf(lcd_text_main,"YHData : %X    \n\n\n", receivedData); terminalWrite(lcd_text_main);
      YH=receivedData; 


      //Get ZL Data
      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      SPI_I2S_SendData(SPI5,0xac);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);

      SPI_I2S_SendData(SPI5,0xff);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);
      
      GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // sprintf(lcd_text_main,"ZLData : %X    \n\n", receivedData); terminalWrite(lcd_text_main);
      ZL=receivedData; 

      //Get ZH Data
      GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      SPI_I2S_SendData(SPI5,0xad);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);

      SPI_I2S_SendData(SPI5,0xff);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      receivedData=SPI_I2S_ReceiveData(SPI5);
      
      GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // sprintf(lcd_text_main,"ZHData : %X    \n\n\n", receivedData); terminalWrite(lcd_text_main);
      ZH=receivedData; 


      //sprintf(lcd_text_main,"END SPI Getdata\n\n\n\n\n"); terminalWrite(lcd_text_main);


      //USART put data
      //sprintf((char *)buff_transmit, "XL=%d,XH=%d\r\n",XL,XH);
      int16_t X=0x0000;
      int16_t Y=0x0000;
      int16_t Z=0x0000;
      X=(int16_t)XH*256+(int16_t)XL;
      Y=(int16_t)YH*256+(int16_t)YL;
      Z=(int16_t)ZH*256+(int16_t)ZL;

      sprintf((char *)buff_transmit, "XL=%d,XH=%d,YL=%d,YH=%d,ZL=%d,ZH=%d \r\n",XL,XH,YL,YH,ZL,ZH);
      USART1_puts((char *)buff_transmit);
      for (i=0;i<50;i++)
      {

        buff_transmit[i]=0;
      }
      sprintf((char *)buff_transmit, "X=%d,Y=%d,Z=%d \r\n",X,Y,Z);
      USART1_puts((char *)buff_transmit);
      for (i=0;i<50;i++)
      {

        buff_transmit[i]=0;
      }
      // SPI_I2S_SendData(SPI5,0xFF);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);

      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      

      // sprintf(lcd_text_main,"receivedData : %x    \n", receivedData); terminalWrite(lcd_text_main); 
   
      Delay_1us(1000000);
    }
    
}

uint8_t t=0;
void USART1_IRQHandler(void)
{
  
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
  {
    uart1_data = USART_ReceiveData(USART1);
    USART_SendData(USART1, uart1_data);
    t = USART_ReceiveData(USART1);
  }

}