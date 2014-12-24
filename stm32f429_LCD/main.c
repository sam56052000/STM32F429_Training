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

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

void DrawThickCircle(uint32_t x,uint32_t y,uint32_t radius, uint32_t thickness){


    LCD_SetTextColor(LCD_COLOR_BLACK);
    LCD_DrawFullCircle(x, y, radius);
    LCD_SetColors(LCD_COLOR_WHITE-1,LCD_COLOR_WHITE);
    LCD_DrawFullCircle(x, y, radius-thickness);


}

int main(void)
{
      RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    LED_Initialization();
    ADC_Initialization();

    double adc_datax=0,adc_datay=0,adc_dataz=0;
    int i=0;
    double voltagex =0,voltagey =0,voltagez =0;
    double accx =0,accy =0,accz =0;
    double ata,rola,pi=3.1415926;

    ADC_SoftwareStartConv(ADC3);
    ADC_SoftwareStartConv(ADC2);
    ADC_SoftwareStartConv(ADC1);

  //uint8_t colorR =0 ,colorG =0 ,colorB =0 ;
  //uint8_t colorR_dir =0 ,colorG_dir =0 ,colorB_dir =0 ;
  //char lcd_text_buff[100]; 


  /* LCD initialization */
  LCD_Init();
  
  /* LCD Layer initialization */
  LCD_LayerInit();
    
  LCD_SetLayer(LCD_FOREGROUND_LAYER);
  LCD_SetColorKeying(0xFFFFFF);

  /* Need to reload */
  LTDC_ReloadConfig(LTDC_IMReload);

  /* Enable the LTDC */
  LTDC_Cmd(ENABLE);
  
  /* Set LCD foreground layer */

  /* Clear the LCD */ 
  LCD_Clear(LCD_COLOR_WHITE);
  LCD_SetFont(&Font16x24);

  LCD_SetLayer(LCD_BACKGROUND_LAYER);
  LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE);

  LCD_DisplayStringLine(LINE(1), (uint8_t*)" LCD text print example ");
  LCD_DisplayStringLine(LINE(2), (uint8_t*)" Ming6842 @ github");

    LCD_SetLayer(LCD_FOREGROUND_LAYER);

    LCD_SetColors(LCD_COLOR_WHITE,LCD_COLOR_WHITE);
    LCD_DrawFullRect(0,0,240,320);

    LCD_Clear(LCD_COLOR_WHITE);

// #define X_MIDDLE 120
// #define Y_MIDDLE 180

//     LCD_SetTextColor(LCD_COLOR_BLACK);
//     LCD_DrawUniLine(X_MIDDLE+75, Y_MIDDLE-75  , X_MIDDLE-75, Y_MIDDLE+75);
//     LCD_DrawUniLine(X_MIDDLE+75, Y_MIDDLE-75-1, X_MIDDLE-75, Y_MIDDLE+75-1);
//     LCD_DrawUniLine(X_MIDDLE+75, Y_MIDDLE-75+1, X_MIDDLE-75, Y_MIDDLE+75+1);

//     LCD_DrawUniLine(X_MIDDLE-75, Y_MIDDLE-75  , X_MIDDLE+75, Y_MIDDLE+75);
//     LCD_DrawUniLine(X_MIDDLE-75, Y_MIDDLE-75-1, X_MIDDLE+75, Y_MIDDLE+75-1);
//     LCD_DrawUniLine(X_MIDDLE-75, Y_MIDDLE-75+1, X_MIDDLE+75, Y_MIDDLE+75+1);


//     LCD_DrawFullRect(X_MIDDLE-60,Y_MIDDLE-5,120,10);
//     LCD_DrawFullRect(X_MIDDLE-5,Y_MIDDLE-60,10,120);


//     DrawThickCircle(X_MIDDLE,Y_MIDDLE,30,7);

//     DrawThickCircle(X_MIDDLE+60,Y_MIDDLE,22,   5);
//     DrawThickCircle(X_MIDDLE-60,Y_MIDDLE,22,   5);
//     DrawThickCircle(X_MIDDLE   ,Y_MIDDLE+60,22,5);
//     DrawThickCircle(X_MIDDLE   ,Y_MIDDLE-60,22,5);

//     DrawThickCircle(X_MIDDLE+75,Y_MIDDLE+75,32,5);
//     DrawThickCircle(X_MIDDLE-75,Y_MIDDLE-75,32,5);
//     DrawThickCircle(X_MIDDLE+75,Y_MIDDLE-75,32,5);
//     DrawThickCircle(X_MIDDLE-75,Y_MIDDLE+75,32,5);

//     LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);

//     LCD_DisplayStringLine(LINE(1), (uint8_t*)"     Ce-91     ");

//     LCD_SetLayer(LCD_BACKGROUND_LAYER);
    double buff_transmit[100];//num=0; 
    //ADC add


  while (1)
  {
    LED3_Toggle();

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

    // voltage1 = (float)adc_datax*3.3f/4095.0f;
    // voltage2 = (float)adc_datay*3.3f/4095.0f;

    sprintf((char *)buff_transmit, "ADC Datax = %lf, ADC Datay = %lf, ADC Dataz = %lf \r\n",adc_datax,adc_datay,adc_dataz);
    USART1_puts((char *)buff_transmit);

    for (i=0;i<50;i++)
    {
        buff_transmit[i]=0;
    }

    //LCD install




    #define X_MIDDLE 120
    #define Y_MIDDLE 180
    //print ADC data
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
    LCD_SetColors(LCD_COLOR_BLACK,LCD_COLOR_WHITE-1);

    LCD_DisplayStringLine(LINE(1), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "ADCX = %lf",adc_datax);
    LCD_DisplayStringLine(LINE(1), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(2), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "ADCY = %lf",adc_datay);
    LCD_DisplayStringLine(LINE(2), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(3), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "ADCZ = %lf",adc_dataz);
    LCD_DisplayStringLine(LINE(3), (uint8_t*)buff_transmit);

    //print V
    voltagex=adc_datax*3/4095;
    voltagey=adc_datay*3/4095;
    voltagez=adc_dataz*3/4095;

    LCD_DisplayStringLine(LINE(4), (uint8_t*)"                                    ");
    sprintf((char *)buff_transmit, "Vx = %lf V",voltagex);
    LCD_DisplayStringLine(LINE(4), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(5), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "Vy = %lf V",voltagey);
    LCD_DisplayStringLine(LINE(5), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(6), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "Vz = %lf V",voltagez);
    LCD_DisplayStringLine(LINE(6), (uint8_t*)buff_transmit);


    //print g
    accx=(voltagex-1.34)/0.25;
    accy=(voltagey-1.31)/0.28;
    accz=(voltagez-1.31)/0.28;
    LCD_DisplayStringLine(LINE(7), (uint8_t*)"                                    ");
    sprintf((char *)buff_transmit, "Accx = %lf V",accx);
    LCD_DisplayStringLine(LINE(7), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(8), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "Accy = %lf V",accy);
    LCD_DisplayStringLine(LINE(8), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(9), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "Accz = %lf V",accz);
    LCD_DisplayStringLine(LINE(9), (uint8_t*)buff_transmit);

    ata=90-(180/pi)*acos((voltagex-1.34)/0.25);
    rola=90-(180/pi)*acos((voltagey-1.31)/0.28);

    // if(ata>90)
    // {
    //   ata=90;
    // }
    // if(rola>90)
    // {
    //   rola=90;
    // }
    LCD_DisplayStringLine(LINE(10), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "Attack = %lf ",ata);
    LCD_DisplayStringLine(LINE(10), (uint8_t*)buff_transmit);

    LCD_DisplayStringLine(LINE(11), (uint8_t*)"                                   ");
    sprintf((char *)buff_transmit, "Roll = %lf ",rola);
    LCD_DisplayStringLine(LINE(11), (uint8_t*)buff_transmit);
    // //num++;
    // LCD_SetLayer(LCD_BACKGROUND_LAYER);


    // if(colorR_dir){

    //       colorR += 1;

    //   if(colorR > 250) colorR_dir=0;
      
    // }else{

    //   colorR -= 1;

    //   if(colorR<20) colorR_dir=1;
    // }

    // if(colorG_dir){

    //       colorG += 2;

    //   if(colorG > 250) colorG_dir=0;
      
    // }else{

    //   colorG -= 2;

    //   if(colorG<25) colorG_dir=1;
    // }

    // if(colorB_dir){

    //       colorB += 3;

    //   if(colorB > 250) colorB_dir=0;
      
    // }else{

    //   colorB -= 3;

    //   if(colorB<25) colorB_dir=1;
    // }

    // LCD_SetColors(ASSEMBLE_RGB(colorR, colorG, colorB),LCD_COLOR_BLACK);
    // LCD_DrawFullRect(0,0,240,320);

    Delay_1us(5000);


    //----------------------------------------------------------------

    

  }
  
}

