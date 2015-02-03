
#include "gyro.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "stm32f4xx_can.h"
#include <stdint.h>

float Buffer[6];
//board_ID data
uint8_t board_ID;
//USART data
char buff_transmit[100];

CanRxMsg can2RxMessage;
TIM_ICInitTypeDef  TIM_ICInitStructure;

//Delay_1us code
static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

/**************************************************************************************/
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

void LED_Initialization(void)
{
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

void TIM3_Initialization(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  //TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* TIM2  PA5 */  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM pin to AF5 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_DeInit(TIM3);
  TIM_TimeBaseStruct.TIM_Period = 65535-1;//65535 is count number             
  TIM_TimeBaseStruct.TIM_Prescaler = 90-1;//50-1          
  TIM_TimeBaseStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    // Counter Up
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //POLARITY!!!!
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;             //Prescaler
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);
  /* Enable the CC1 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
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

void ID_Pin_Initialization(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE); //LED3/4 GPIO Port

    /* Configure the GPIO ID pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;  // LED is connected to PG13/PG14
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* Configure GPIO pin for Jumper purpose */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4;  // LED is connected to PG13/PG14
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_SetBits(GPIOE,GPIO_Pin_2 | GPIO_Pin_4);
}

uint8_t PIN_ID_Read(void)
{
  return (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3))+ 2*(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5));
}

void USART1_puts(char* s)
{
  while(*s)
  {
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, *s);
    s++;
  }
}

void LED3_Toggle(void)
{
  GPIOG->ODR ^= GPIO_Pin_13;
}

//time capture
uint32_t timebaseCapture_prev = 0;
uint32_t timebaseCapture_current =0;
uint32_t timebaseCapture_output = 0;
uint8_t hlstatus=1;

double adcy=0.0f,angy=0.0f;
double deltime=0.0005;
uint8_t receivedData=0;
int i=0,j=100,countcan;
uint8_t XL,XH;
//,YL,YH,ZL,ZH;
double Xreg=0.0f,Xchange=0.0f,Xsta=0.0f,fXchange=0.0f;
// double Yreg=0x0000;
// double Zreg=0x0000;
double areg=0,t=0;
double ctrldata=0;
uint16_t trustdata,trust1,trust2;
uint16_t trust1h,trust2h,trust1l,trust2l,attnh,attnl,attn;
double nowxta;
char freqflag;//timer IQR flag
char *ps;//while loop usart


//PID control value
double P=12,I=2.05,D=2.5;
double II=0;

int main(void)
{
  CanTxMsg TxMessage;
  char lcd_text_main[100];

  RCC_Configuration();
  GPIO_Configuration();
  ADC_Initialization();
  USART1_Configuration();
  TIM3_Initialization();
  PWM_Initialization();
  SPI_Initialization();

  ADC_SoftwareStartConv(ADC3);
  ADC_SoftwareStartConv(ADC2);
  ADC_SoftwareStartConv(ADC1);


  /* LED Initialization */
  LED_Initialization();

  /* CAN Initialization */
  CAN2_Config();
  CAN2_NVIC_Config();

  //Read board ID
  board_ID = PIN_ID_Read();

  //ADC data
  uint16_t adc_datax,adc_datay,adc_dataz;
  //CANBUS data
  uint16_t accxh,accxl,accyh,accyl,acczh,acczl;
  uint16_t accxh2,accxl2,accyh2,accyl2,acczh2,acczl2;
  uint16_t atth,attl,AttitPWM;
  uint8_t checksum=0x00;



  while(1)
  {
    if(board_ID == 1)
    {
      /* LCD Initialization */
      lcd_init();
      lcd_drawBackground(20,60,250);
      while(1)
      {
        //Freq test
        LED3_Toggle();

        //LCD set title
        LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
        sprintf(lcd_text_main,"---CAN ID: %d---",board_ID);
        LCD_DisplayStringLine(LINE(0), (uint8_t*)lcd_text_main);
        sprintf((char *)lcd_text_main, "Vibra DATA DOWN      ");
        LCD_DisplayStringLine(LINE(1), (uint8_t*)lcd_text_main);
        sprintf((char *)lcd_text_main, "Vibra DATA ON        ");
        LCD_DisplayStringLine(LINE(5), (uint8_t*)lcd_text_main);

        //ADC read data
        ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC3);
        Delay_1us(50);
        adc_datax = ADC_GetConversionValue(ADC3);

        ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC2);
        Delay_1us(50);
        adc_datay = ADC_GetConversionValue(ADC2);

        ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
        ADC_SoftwareStartConv(ADC1);
        Delay_1us(50);
        adc_dataz = ADC_GetConversionValue(ADC1);

        sprintf((char *)lcd_text_main, "ADC_accx= %d     ",adc_datax);
        LCD_DisplayStringLine(LINE(2), (uint8_t*)lcd_text_main);
        sprintf((char *)lcd_text_main, "ADC_accy= %d     ",adc_datay);
        LCD_DisplayStringLine(LINE(3), (uint8_t*)lcd_text_main);
        sprintf((char *)lcd_text_main, "ADC_accz= %d     ",adc_dataz);
        LCD_DisplayStringLine(LINE(4), (uint8_t*)lcd_text_main);


        if( can2_rx_isr_flag ==1)
        {
          if(CAN_MessagePending(CAN2, CAN_FIFO0) > 0)
          {
            CAN_Receive(CAN2, CAN_FIFO0, &can2RxMessage);
            GPIO_ToggleBits(GPIOG,GPIO_Pin_14);
        //checksum to check buffer is correct
            checksum=can2RxMessage.Data[0]+can2RxMessage.Data[1]+can2RxMessage.Data[2]+can2RxMessage.Data[3]+can2RxMessage.Data[4]+can2RxMessage.Data[5]+can2RxMessage.Data[6];
            checksum=checksum%256;
            if(checksum==can2RxMessage.Data[7])
            {
              //from board ID 2 data buffer
              if(can2RxMessage.ExtId==0x0002FF)
              {
                //buffer type 2
                if(can2RxMessage.Data[0]==0x02)
                {
                  accxh2=can2RxMessage.Data[1];
                  accxl2=can2RxMessage.Data[2];
                  accyh2=can2RxMessage.Data[3];
                  accyl2=can2RxMessage.Data[4];
                  acczh2=can2RxMessage.Data[5];
                  acczl2=can2RxMessage.Data[6];

                  adc_datax=accxh2*256+accxl2;
                  adc_datay=accyh2*256+accyl2;
                  adc_dataz=acczh2*256+acczl2;
                  sprintf((char *)lcd_text_main, "ADC_accx= %d     ",adc_datax);
                  LCD_DisplayStringLine(LINE(6), (uint8_t*)lcd_text_main);
                  sprintf((char *)lcd_text_main, "ADC_accy= %d     ",adc_datay);
                  LCD_DisplayStringLine(LINE(7), (uint8_t*)lcd_text_main);
                  sprintf((char *)lcd_text_main, "ADC_accz= %d     ",adc_dataz);
                  LCD_DisplayStringLine(LINE(8), (uint8_t*)lcd_text_main);
                }
              }
            }
          }
        }

        accxh=adc_datax/256;
        accxl=adc_datax%256;
        accyh=adc_datay/256;
        accyl=adc_datay%256;
        acczh=adc_dataz/256;
        acczl=adc_dataz%256;

        /* Transmit Structure preparation */
        TxMessage.StdId = 0;
        //TxMessage.ExtId = (uint32_t)board_ID ;
        TxMessage.ExtId = 0x0001FF;
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.DLC = 8;
        TxMessage.Data[0] = 0x01;
        TxMessage.Data[1] = accxh;
        TxMessage.Data[2] = accxl;
        TxMessage.Data[3] = accyh;
        TxMessage.Data[4] = accyl;
        TxMessage.Data[5] = acczh;
        TxMessage.Data[6] = acczl;

        checksum=TxMessage.Data[0]+TxMessage.Data[1]+TxMessage.Data[2]+TxMessage.Data[3]+TxMessage.Data[4]+TxMessage.Data[5]+TxMessage.Data[6];
        TxMessage.Data[7] = checksum;

        CAN_Transmit(CAN2, &TxMessage);
        Delay_1us(10000);

        //PWM read data
        sprintf((char *)lcd_text_main, "Receiver DATA       ");
        LCD_DisplayStringLine(LINE(9), (uint8_t*)lcd_text_main);
        sprintf((char *)lcd_text_main, "AttitPWM=%ldus      ",timebaseCapture_output);
        LCD_DisplayStringLine(LINE(10), (uint8_t*)lcd_text_main);

        atth=timebaseCapture_output/256;
        attl=timebaseCapture_output%256;
        /* Transmit Structure preparation */
        TxMessage.StdId = 0;
        //TxMessage.ExtId = (uint32_t)board_ID ;
        TxMessage.ExtId = 0x000101;
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.DLC = 8;
        TxMessage.Data[0] = 0x00;
        TxMessage.Data[1] = 0;
        TxMessage.Data[2] = 0;
        TxMessage.Data[3] = atth;
        TxMessage.Data[4] = attl;
        TxMessage.Data[5] = 0;
        TxMessage.Data[6] = 0;

        checksum=TxMessage.Data[0]+TxMessage.Data[1]+TxMessage.Data[2]+TxMessage.Data[3]+TxMessage.Data[4]+TxMessage.Data[5]+TxMessage.Data[6];
        TxMessage.Data[7] = checksum;

        CAN_Transmit(CAN2, &TxMessage);

        Delay_1us(1000);
      }
    }
    else if(board_ID == 3)
    {
      /* LCD Initialization */
      lcd_init();
      lcd_drawBackground(20,60,250);
      while(1)
      {
        //Freq test
        LED3_Toggle();
        //LCD set title
        LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
        sprintf(lcd_text_main,"---CAN ID: %d---",board_ID);
        LCD_DisplayStringLine(LINE(0), (uint8_t*)lcd_text_main);
        sprintf((char *)lcd_text_main, "SYSTEM DATA NOW  ");
        LCD_DisplayStringLine(LINE(1), (uint8_t*)lcd_text_main);

        //Received Data
        if( can2_rx_isr_flag ==1)
        {
          if(CAN_MessagePending(CAN2, CAN_FIFO0) > 0)
          {
            CAN_Receive(CAN2, CAN_FIFO0, &can2RxMessage);
            GPIO_ToggleBits(GPIOG,GPIO_Pin_14);

            //checksum to check buffer is correct
            checksum=can2RxMessage.Data[0]+can2RxMessage.Data[1]+can2RxMessage.Data[2]+can2RxMessage.Data[3]+can2RxMessage.Data[4]+can2RxMessage.Data[5]+can2RxMessage.Data[6];
            checksum=checksum%256;
            if(checksum==can2RxMessage.Data[7])
            {
              //from board ID 2 data buffer
              if(can2RxMessage.ExtId==0x0002FE)
              {
                //buffer type 1
                if(can2RxMessage.Data[0]==0x01)
                {
                  attnh=can2RxMessage.Data[1];
                  attnl=can2RxMessage.Data[2];
                  trust1h=can2RxMessage.Data[3];
                  trust1l=can2RxMessage.Data[4];
                  trust2h=can2RxMessage.Data[5];
                  trust2l=can2RxMessage.Data[6];

                  trust1=trust1h*256+trust1l;
                  trust2=trust2h*256+trust2l;
                  attn=attnh*256+attnl;
                  nowxta=((double)attn/100)-50;

                  sprintf((char *)lcd_text_main, "AttitNOW=%lf      ",nowxta);
                  LCD_DisplayStringLine(LINE(3), (uint8_t*)lcd_text_main);
                  sprintf((char *)lcd_text_main, "trust1= %d     ",trust1);
                  LCD_DisplayStringLine(LINE(4), (uint8_t*)lcd_text_main);
                  sprintf((char *)lcd_text_main, "trust2= %d     ",trust2*2);
                  LCD_DisplayStringLine(LINE(5), (uint8_t*)lcd_text_main);
                }
              }
              //from board ID 1 command buffer
              if(can2RxMessage.ExtId==0x000101)
              {
                //buffer type 1
                if(can2RxMessage.Data[0]==0x00)
                {
                  atth=can2RxMessage.Data[3];
                  attl=can2RxMessage.Data[4];
                  AttitPWM=atth*256+attl;
                  ctrldata=-40+((double)AttitPWM-1000)/12.5;
                  //PWM read data
                  sprintf((char *)lcd_text_main, "ctrldata=%lfus      ",ctrldata);
                  LCD_DisplayStringLine(LINE(2), (uint8_t*)lcd_text_main);
                }
              }
            }
            else
            {
              sprintf((char *)buff_transmit, "checksum error: checksum=%X RX7=%X \r\n",checksum,can2RxMessage.Data[7]);
              USART1_puts((char *)buff_transmit);
            }
          }
        }
        //USART put DATA
        // sprintf((char *)buff_transmit, "Vibration DATA-- X: %d Y: %d Z:%d\r\n",adc_datax,adc_datay,adc_dataz);
        // USART1_puts((char *)buff_transmit);
      }
    }
    else
    {
      board_ID = 2;

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
      // TIM2->CCR3 =1000;
      // TIM1->CCR2 =2000;
      // Delay_1us(5000000);

      sprintf((char *)buff_transmit, "system just aready");
      USART1_puts((char *)buff_transmit);

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



          //Received Data
        if( can2_rx_isr_flag ==1)
        {
          if(CAN_MessagePending(CAN2, CAN_FIFO0) > 0)
          {
            CAN_Receive(CAN2, CAN_FIFO0, &can2RxMessage);
            GPIO_ToggleBits(GPIOG,GPIO_Pin_14);

            //checksum to check buffer is correct
            checksum=can2RxMessage.Data[0]+can2RxMessage.Data[1]+can2RxMessage.Data[2]+can2RxMessage.Data[3]+can2RxMessage.Data[4]+can2RxMessage.Data[5]+can2RxMessage.Data[6];
            checksum=checksum%256;
            if(checksum==can2RxMessage.Data[7])
            {
              
              //from board ID 1 command buffer
              if(can2RxMessage.ExtId==0x000101)
              {
                //buffer type 1
                if(can2RxMessage.Data[0]==0x00)
                {
                  atth=can2RxMessage.Data[3];
                  attl=can2RxMessage.Data[4];
                  AttitPWM=atth*256+attl;
                  ctrldata=-40+((double)AttitPWM-1000)/12.5;
                }
              }
            }
          }
        }
     
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

          ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
          ADC_SoftwareStartConv(ADC3);
          Delay_1us(50);
          adc_datax = ADC_GetConversionValue(ADC3);

          ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
          ADC_SoftwareStartConv(ADC2);
          Delay_1us(50);
          adc_datay = ADC_GetConversionValue(ADC2);

          ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_3Cycles);
          ADC_SoftwareStartConv(ADC1);
          Delay_1us(50);
          adc_dataz = ADC_GetConversionValue(ADC1);

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
          Xsta=(Xsta+Xchange)*(1.0-0.0003)+(angy-0.5)*0.0003;
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

          
          nowxta=Xsta;
          attnh=(uint16_t)((nowxta+50)*100)/256;
          attnl=(uint16_t)((nowxta+50)*100)%256;
          trust1h=trust1/256;
          trust1l=trust1%256;
          trust2h=trust2/256;
          trust2l=trust2%256;

          accxh2=adc_datax/256;
          accxl2=adc_datax%256;
          accyh2=adc_datay/256;
          accyl2=adc_datay%256;
          acczh2=adc_dataz/256;
          acczl2=adc_dataz%256;

          countcan++;
          if(countcan==100)
          {
          /* Transmit Structure preparation */
          TxMessage.StdId = 0;
          //TxMessage.ExtId = (uint32_t)board_ID ;
          TxMessage.ExtId = 0x0002FE;
          TxMessage.RTR = CAN_RTR_DATA;
          TxMessage.IDE = CAN_ID_EXT;
          TxMessage.DLC = 8;
          TxMessage.Data[0] = 0x01;
          TxMessage.Data[1] = attnh;
          TxMessage.Data[2] = attnl;
          TxMessage.Data[3] = trust1h;
          TxMessage.Data[4] = trust1l;
          TxMessage.Data[5] = trust2h;
          TxMessage.Data[6] = trust2l;

          checksum=TxMessage.Data[0]+TxMessage.Data[1]+TxMessage.Data[2]+TxMessage.Data[3]+TxMessage.Data[4]+TxMessage.Data[5]+TxMessage.Data[6];
          TxMessage.Data[7] = checksum;

          CAN_Transmit(CAN2, &TxMessage);
          Delay_1us(10);

          /* Transmit Structure preparation */
          TxMessage.StdId = 0;
          //TxMessage.ExtId = (uint32_t)board_ID ;
          TxMessage.ExtId = 0x0002FF;
          TxMessage.RTR = CAN_RTR_DATA;
          TxMessage.IDE = CAN_ID_EXT;
          TxMessage.DLC = 8;
          TxMessage.Data[0] = 0x02;
          TxMessage.Data[1] = accxh2;
          TxMessage.Data[2] = accxl2;
          TxMessage.Data[3] = accyh2;
          TxMessage.Data[4] = accyl2;
          TxMessage.Data[5] = acczh2;
          TxMessage.Data[6] = acczl2;

          checksum=TxMessage.Data[0]+TxMessage.Data[1]+TxMessage.Data[2]+TxMessage.Data[3]+TxMessage.Data[4]+TxMessage.Data[5]+TxMessage.Data[6];
          TxMessage.Data[7] = checksum;

          CAN_Transmit(CAN2, &TxMessage);
          Delay_1us(10);
          countcan=0;
          }
          // //USART put DATA
          // if(j==100)
          // {
          //   for (i=0;i<100;i++)
          //   {
          //     buff_transmit[i]=0x00;
          //   }
          //   sprintf((char *)buff_transmit,"Xsta=%lf,speed=%lf,trust1=%d,trust2=%d,II=%lf\n",Xsta,Xreg/32767*250,trust1,trust2,II);
          //   j=0;
          //   ps=(char *)buff_transmit;
          // }
          // if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
          // {
          //   USART_SendData(USART1, *ps);
          //   ps++;
          //   j++;
          //   if(*ps==0x00)
          //   {
          //     if(*(ps+1)==0x00)
          //     {
          //       if(*(ps+2)==0x00)
          //       {
          //         j=100;
          //       }
          //     }
          //   }
          // }
          freqflag=0;
        }
      }
    }//board 2 if

        // LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
        // sprintf(lcd_text_main," CAN Demo ID:%d    ",board_ID);
        // LCD_DisplayStringLine(LINE(0), (uint8_t*)lcd_text_main);

        /* Transmit Structure preparation */
//         TxMessage.StdId = 0;
//         TxMessage.ExtId = (uint32_t)board_ID ;
//         TxMessage.RTR = CAN_RTR_DATA;
//         TxMessage.IDE = CAN_ID_EXT;
//         TxMessage.DLC = 8;
//         //TxMessage.Data[0] = 0xAA;
//         TxMessage.Data[1] = 0x55;
//         TxMessage.Data[2] = 0x55;
//         TxMessage.Data[3] = 0x55;
//         TxMessage.Data[4] = 0x55;
//         TxMessage.Data[5] = 0x55;
//         TxMessage.Data[6] = 0x55;
//         TxMessage.Data[7] = 0x55;

//         TxMessage.Data[0] = TxMessage.Data[0]+1;

//          CAN_Transmit(CAN2, &TxMessage);
//           Delay_1us(10000);

//             /* Received Data */
//             if( can2_rx_isr_flag ==1){
// if(CAN_MessagePending(CAN2, CAN_FIFO0) > 0)
// {
//                   CAN_Receive(CAN2, CAN_FIFO0, &can2RxMessage);
//                   GPIO_ToggleBits(GPIOG,GPIO_Pin_14);
//                   LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
//                   sprintf(lcd_text_main," DATA ID:%lX    ",can2RxMessage.ExtId);
//                   LCD_DisplayStringLine(LINE(1), (uint8_t*)lcd_text_main);

//                   LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
//                   sprintf(lcd_text_main," DATA[0]: 0x%X    ",can2RxMessage.Data[0]);
//                   LCD_DisplayStringLine(LINE(2), (uint8_t*)lcd_text_main);
//                 }

                // do{
                //   CAN_Receive(CAN2, CAN_FIFO0, &can2RxMessage);
                //   GPIO_ToggleBits(GPIOG,GPIO_Pin_14);

                //   LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
                //   sprintf(lcd_text_main," DATA ID:%lX    ",can2RxMessage.ExtId);
                //   LCD_DisplayStringLine(LINE(1), (uint8_t*)lcd_text_main);

                //   LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);
                //   sprintf(lcd_text_main," DATA[0]: 0x%X    ",can2RxMessage.Data[0]);
                //   LCD_DisplayStringLine(LINE(2), (uint8_t*)lcd_text_main);

                // }while(CAN_MessagePending(CAN2, CAN_FIFO0) > 0);

            //}
      }

}

void TIM3_IRQHandler()
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)//chanel 1's intrrupt
  {
    
    if(hlstatus==1)
    {
      TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
      timebaseCapture_prev = timebaseCapture_current;
      timebaseCapture_current = TIM_GetCapture1(TIM3);
      TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      TIM_ICInit(TIM3, &TIM_ICInitStructure);
      hlstatus=0;
      GPIO_ToggleBits(GPIOG,GPIO_Pin_14);
    }
    else
    {
      TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
      timebaseCapture_prev = timebaseCapture_current;
      timebaseCapture_current = TIM_GetCapture1(TIM3);
      TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      TIM_ICInit(TIM3, &TIM_ICInitStructure);
      hlstatus=1;
      if(timebaseCapture_current > timebaseCapture_prev)
      {

        timebaseCapture_output  = (timebaseCapture_current- timebaseCapture_prev);//*5/18;
      }
      else
      {
        timebaseCapture_output  =  (0xFFFF - timebaseCapture_prev + timebaseCapture_current);//*5/18;
      }
    }
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