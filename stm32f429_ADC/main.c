
#include "main.h"
#include <stdio.h>
#include <math.h>

static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

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
void ADC_Initialization(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  //ADC1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //ADC2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);



  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                     // No external trigger is connected
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;                  // ADC clock prescaler
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;      // No DMA (polling mode)
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                       // Resolution 12 bits
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;                                // Use single channel 
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                           // Continue conversion
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                       // Data bits shifted to right hand side (Low)
  ADC_InitStructure.ADC_NbrOfConversion = 1;                                   // Convert only once
  ADC_Init(ADC3, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
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


//USART==========================================================================
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
     *  - Receivnel_6, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC3);
        Delay_1us(50);
        adc_data1 = ADC_GetConversionValue(ADC3);e and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}



//PWM======================================================================
void PWM_Initialization(void)
{

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE); 

  /* -- GPIO Configuration ---------------------------------------------------- */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM1);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = (uint32_t)(10000 - 1);  //2.5ms , 400Hz
  TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)(180 - 1); //84 = 1M(1us)
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;    // No division, so 180MHz
  TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;           // Not used
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);


  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;               //PWM Edge mode
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = 1000-1;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;        // Output polarity High
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;      // Complementary output polarity :Not used
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;     // No output polarity : reset (low)
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;    // Complementary idle output : reset (not used)

  TIM_OC2Init(TIM1, &TIM_OCInitStruct);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);       //Put ARR value into register
  TIM_Cmd(TIM1, ENABLE);                    // Enable Timer 1
  TIM_CtrlPWMOutputs(TIM1, ENABLE);         // Enable output (To GPIO)

//===========================================================================
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE); 

  /* -- GPIO Configuration ---------------------------------------------------- */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);

  //GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* -- Timer Configuration --------------------------------------------------- */
  TIM_DeInit(TIM2);

  //TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_TimeBaseStruct.TIM_Period = (uint32_t)(5000 - 1);  //2.5ms , 400Hz
  TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)(180 - 1); //84 = 1M(1us)
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;    // No division, so 180MHz
  TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;           // Not used
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);


  //TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;               //PWM Edge mode
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = 1000-1;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;        // Output polarity High
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;      // Complementary output polarity :Not used
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;     // No output polarity : reset (low)
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;    // Complementary idle output : reset (not used)

  TIM_OC3Init(TIM2, &TIM_OCInitStruct);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE);       //Put ARR value into register
  TIM_Cmd(TIM2, ENABLE);                    // Enable Timer 1
  TIM_CtrlPWMOutputs(TIM2, ENABLE);         // Enable output (To GPIO)
}


//SPI======================================================================
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
  TIM_TimeBaseStruct.TIM_Period = 250 - 1 ;  //250ms  --> 4Hz
  TIM_TimeBaseStruct.TIM_Prescaler = 90 - 1; // Prescaled by 90 -> = 0.1M(10us)
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Div by one -> 90 MHz (Now RCC_DCKCFGR_TIMPRE is configured to divide clock by two)
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM5, DISABLE);       //Put ARR value into register
  TIM_Cmd(TIM5, ENABLE);
}

/**************************************************************************************/
//uint16_t adc_datax=0;
//uint16_t adc_dataz=0;
uint16_t adc_datay=0;
double adcy=0.0f,angy=0.0f;
double deltime=0.00025;
uint8_t receivedData=0;
int i=0,j=100;
uint8_t XL,XH;
//,YL,YH,ZL,ZH;
double Xreg=0.0f,Xchange=0.0f,Xsta=0.0f,fXchange=0.0f;
// double Yreg=0x0000;
// double Zreg=0x0000;
double areg=0,t=0;
double ctrldata=0;
uint16_t trustdata,trust1,trust2;
//uint8_t buff_transmit[100];
char buff_transmit[100];
char *ps;//while loop usart
char freqflag;//timer IQR flag


//PID control value
double P=12,I=2.05,D=2.5;
double II=0;

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}


//main function==================================================================
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    LED_Initialization();
    ADC_Initialization();
    SPI_Initialization();
    PWM_Initialization();
    
    ADC_SoftwareStartConv(ADC3);
    ADC_SoftwareStartConv(ADC2);
    ADC_SoftwareStartConv(ADC1);

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

    sprintf((char *)buff_transmit, "GYO FIFO_CTRL_REG Value: %d \r\n",receivedData);
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

    //clalbrate
    TIM2->CCR3 =1000;
    TIM1->CCR2 =2000;
    Delay_1us(5000000);

    sprintf((char *)buff_transmit, "system just aready");
    USART1_puts((char *)buff_transmit);

    //  Timer5_Initialization();
    // while(1)
    // {
    //   if(freqflag==1)
    //   {
    //   LED3_Toggle();
    //   if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
    //   {
    //   USART_SendData(USART1, *ps);
    //   ps++;
    //   j++;
    //   if(*ps==0x00)
    //   {
    //     j=50;
    //   }
    //   }
    //   if(j==50)
    //   {
    //     for (i=0;i<50;i++)
    //     {
    //       buff_transmit[i]=0;
    //     }
    //     sprintf((char *)buff_transmit, "usart test aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
    //     j=0;
    //     ps=(char *)buff_transmit;
    //   }
    //   freqflag=0;
    //   }
    // }


    TIM2->CCR3 =500;
    TIM1->CCR2 =1000;
    Delay_1us(10000000);

    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
    ADC_SoftwareStartConv(ADC2);
    Delay_1us(50);
    adc_datay = ADC_GetConversionValue(ADC2);
    adcy=((double)adc_datay-1790)/380;
    if(adcy>0.707)
    {
      adcy=0.707;
    }
    if(adcy<-0.707)
    {
      adcy=-0.707;
    }
    angy=asin(adcy)*180/3.141593;
    Xsta=angy-0.5;

    //each trust open
    trustdata=1300;

    if(Xsta>0)
    {
      trust1=trustdata;
      TIM1->CCR2 =trust1;
      TIM2->CCR3 =550;
      while(1)
      {
        ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC2);
        Delay_1us(50);
        adc_datay = ADC_GetConversionValue(ADC2);
        adcy=((double)adc_datay-1790)/380;
        if(adcy>0.707)
        {
          adcy=0.707;
        }
        if(adcy<-0.707)
        {
          adcy=-0.707;
        }
        angy=asin(adcy)*180/3.141593;
        Xsta=angy-0.5;

        if(Xsta-ctrldata<20)
        {
          break;
        }
      }
      TIM2->CCR3 =750;
    }
    else
    {
      trust2=trustdata;
      trust2=trust2/2;
      TIM2->CCR3 =trust2;
      TIM1->CCR2 =1100;
      while(1)
      {
        ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC2);
        Delay_1us(50);
        adc_datay = ADC_GetConversionValue(ADC2);
        adcy=((double)adc_datay-1790)/380;
        if(adcy>0.707)
        {
          adcy=0.707;
        }
        if(adcy<-0.707)
        {
          adcy=-0.707;
        }
        angy=asin(adcy)*180/3.141593;
        Xsta=angy-0.5;
        
        if(Xsta-ctrldata>-20)
        {
          break;
        }
      }
      TIM1->CCR2 =1500;
    }
    //Delay_1us(500000);
    Timer5_Initialization();

    while(1)
    {
      if(freqflag==1)
      {
      LED3_Toggle();
     
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
      XH=receivedData; 

      //Get YL Data
      // GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      // SPI_I2S_SendData(SPI5,0xaa);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // SPI_I2S_SendData(SPI5,0xff);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // YL=receivedData; 

      // //Get YH Data
      // GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      // SPI_I2S_SendData(SPI5,0xab);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // SPI_I2S_SendData(SPI5,0xff);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // YH=receivedData; 

      // //Get ZL Data
      // GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      // SPI_I2S_SendData(SPI5,0xac);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // SPI_I2S_SendData(SPI5,0xff);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // ZL=receivedData; 

      // //Get ZH Data
      // GPIO_ResetBits(GPIOC,GPIO_Pin_1);
      // SPI_I2S_SendData(SPI5,0xad);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // SPI_I2S_SendData(SPI5,0xff);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET);
      // while (SPI_I2S_GetFlagStatus(SPI5, SPI_FLAG_RXNE) == RESET);
      // receivedData=SPI_I2S_ReceiveData(SPI5);
      // GPIO_SetBits(GPIOC,GPIO_Pin_1);
      // ZH=receivedData;

      // sprintf((char *)buff_transmit, "SPI Data :XL=%d,XH=%d,YL=%d,YH=%d,ZL=%d,ZH=%d \r\n",XL,XH,YL,YH,ZL,ZH);
      // USART1_puts((char *)buff_transmit);
      // for (i=0;i<50;i++)
      // {

      //   buff_transmit[i]=0;
      // }

      //Get ADC data

      // ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
      // ADC_SoftwareStartConv(ADC3);
      // Delay_1us(50);
      // adc_datax = ADC_GetConversionValue(ADC3);

      ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
      ADC_SoftwareStartConv(ADC2);
      Delay_1us(50);
      adc_datay = ADC_GetConversionValue(ADC2);

      // ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
      // ADC_SoftwareStartConv(ADC1);
      // Delay_1us(50);
      // adc_dataz = ADC_GetConversionValue(ADC1);

      // sprintf((char *)buff_transmit, "ADC Datax = %d, ADC Datay = %d, ADC Dataz = %d \r\n",adc_datax,adc_datay,adc_dataz);
      // USART1_puts((char *)buff_transmit);
      //areg=0.03*(Xreg/32767*250*3.141592/180)*(Xreg/32767*250*3.141592/180);
      adcy=((double)adc_datay-1790)/380;
      if(adcy>0.707)
      {
        adcy=0.707;
      }
      if(adcy<-0.707)
      {
        adcy=-0.707;
      }
      angy=asin(adcy)*180/3.141593;
      // sprintf((char *)buff_transmit, "ADC_angle_read=%lf\n",angy);
      // USART1_puts((char *)buff_transmit);

      // for (i=0;i<50;i++)
      // {
      //   buff_transmit[i]=0;
      // }

      Xreg=(double)XH*256+(double)XL;

      if(Xreg>32768)
      {
        Xreg=Xreg-65535;
      }
      Xchange=Xreg/32767*250*deltime;
      fXchange=Xsta;
      Xsta=(Xsta+Xchange)*(1.0-0.00014)+(angy-0.5)*0.00014;
      // Yreg=(double)YH*256+(double)YL;
      // Zreg=(double)ZH*256+(double)ZL;
      
      //trust============================================
      II=II+(Xsta-ctrldata)*deltime;
      if(II>200)
      {
        II=200;
      }

      trustdata=1400;
      trust1=trustdata+P*(Xsta-ctrldata)+I*II+D*Xchange/deltime+0;
      trust2=trustdata-P*(Xsta-ctrldata)-I*II-D*Xchange/deltime+0;

      // trust1=trustdata+(P+I*deltime)*(Xsta-ctrldata)+0;
      // trust2=trustdata-(P+I*deltime)*(Xsta-ctrldata)+50;
      

      if(trust1>1600)
      {
        trust1=1600;
      }
      if(trust2>1600)
      {
        trust2=1600;
      }
      if(trust1<1200)
      {
        trust1=1200;
      }
      if(trust2<1200)
      {
        trust2=1200;
      }
      //=================================================
      TIM1->CCR2 =trust1;
      //=================================================
      trust2=trust2/2;
      TIM2->CCR3 =trust2;

      //USART put DATA
      if(j==100)
      {
        for (i=0;i<100;i++)
        {
          buff_transmit[i]=0x00;
        }
        sprintf((char *)buff_transmit,"Xsta=%lf,speed=%lf,trust1=%d,trust2=%d,II=%lf\n",Xsta,Xreg/32767*250,trust1,trust2,II);
        j=0;
        ps=(char *)buff_transmit;
      }
      if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
      {
        USART_SendData(USART1, *ps);
        ps++;
        j++;
        if(*ps==0x00)
        {
          if(*(ps+1)==0x00)
          {
            if(*(ps+2)==0x00)
            {
              j=100;
            }
          }
        }
      }
      freqflag=0;
    }
//function end
    }
}

void USART1_IRQHandler(void)//USART interrupt function
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
  {
    t = USART_ReceiveData(USART1);
    USART_SendData(USART1, t);
  }
}

void TIM5_IRQHandler()
{
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
  }
    //
  freqflag=1;
  TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 
}