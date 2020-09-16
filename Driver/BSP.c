#include "BSP.h"

BSP_DATA bspdata;

void BSP_Init(void)
{
	BSP_USART_Init();
	BSP_GPIO_Init();
	BSP_SPI_Init();
	
	BSP_DMA_Init();
	BSP_ADC_Init();
	
	ADXL313_Init();
	
	BSP_EXTI_Init();
}

void BSP_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_ITD;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIOA->ODR = 0x00000800;
	
	//ADXL313 Connect Pin
	
	//PB15 SDI
	//PB14 SDO
	//PB13 SCLK
	//PB12 CS
	
	//PB0 INT1 
	//PA7 INT2
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_ITD);
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_12;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_ITD);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_0;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_ITD);
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_7;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	//BuzzerPin PA8
	GPIO_ITD.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_8;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
	//电压采集配置
	//市电5V PA5
	//电池电压 PA6
	GPIO_ITD.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	//WH-L102-L-C 引脚配置
	//PA0 M0
	//PA1 M1
	//PA2 S_RXD
	//PA3 S_TXD
	//PA4 AUX
	GPIO_ITD.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_3;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_2;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	
	//relay PA11
	GPIO_ITD.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_11;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	
	//Charger Switch PB3
	GPIO_ITD.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_3;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_ITD);
	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
	
	//LED1 LED2
	GPIO_ITD.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_ITD);
	
	//USART1
	GPIO_ITD.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_10;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
	
	GPIO_ITD.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_ITD.GPIO_Pin = GPIO_Pin_9;
	GPIO_ITD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_ITD);
}

void BSP_SPI_Init(void)
{
	SPI_InitTypeDef SPI_ITD;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	
	SPI_ITD.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_ITD.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_ITD.SPI_CPOL = SPI_CPOL_High;
	SPI_ITD.SPI_CRCPolynomial = 7;
	SPI_ITD.SPI_DataSize = SPI_DataSize_8b;
	SPI_ITD.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_ITD.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_ITD.SPI_Mode = SPI_Mode_Master;
	SPI_ITD.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2,&SPI_ITD);
	
	SPI_Cmd(SPI2,ENABLE);
}

void BSP_EXTI_Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void BSP_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&ADC1->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&mainData.PowerAdcData;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)mainData.U2ReceiveData;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)U2_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6,&DMA_InitStructure);
	
	DMA_Cmd(DMA1_Channel6,ENABLE);
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)mainData.U1ReceiveData;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5,&DMA_InitStructure);
	
	DMA_Cmd(DMA1_Channel5,ENABLE);
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
}

void BSP_ADC_Init(void)
{
	ADC_InitTypeDef ADC_ITD;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
	
	ADC_ITD.ADC_ContinuousConvMode = ENABLE;
	ADC_ITD.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_ITD.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_ITD.ADC_Mode = ADC_Mode_Independent;
	ADC_ITD.ADC_NbrOfChannel = 2;
	ADC_ITD.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_ITD);
	
	/* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_55Cycles5);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void BSP_USART_Init(void)
{
	USART_InitTypeDef USART_ITD;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	USART_ITD.USART_BaudRate = 115200;
	USART_ITD.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_ITD.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_ITD.USART_Parity = USART_Parity_No;
	USART_ITD.USART_StopBits = USART_StopBits_1;
	USART_ITD.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&USART_ITD);
	USART_Cmd(USART2,ENABLE);
	
	USART_ClearFlag(USART2,USART_FLAG_TC); 
	
	USART_ITD.USART_BaudRate = 115200;
	USART_ITD.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_ITD.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_ITD.USART_Parity = USART_Parity_No;
	USART_ITD.USART_StopBits = USART_StopBits_1;
	USART_ITD.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_ITD);
	USART_Cmd(USART1,ENABLE);
	
	USART_ClearFlag(USART1,USART_FLAG_TC); 
}

void bsp_delayms(u32 counter)
{
	u32 temp = counter + bspdata.systime;
	while(bspdata.systime <  temp)
	{
		
	}
	 bspdata.systime = 0;
}

void fun_usartSend(USART_TypeDef *com ,u8 data)
{
  while(USART_GetFlagStatus(com, USART_FLAG_TC) == RESET)
  {}
  USART_SendData(com, (u8) data);
}

void fun_usartSends(USART_TypeDef *com ,u8 *data, u8 num)
{
  uint16_t i=0;
  for(i=0;i<num;i++)
  {
    fun_usartSend(com, *(data+i));
  }
  
//  while( *(data+i) != '\0')
//  {
//    fun_usartSend(com, *(data+i));
//    i++;
//  }
}

u8 fun_usartRec(USART_TypeDef *com)
{
  while(USART_GetFlagStatus(com, USART_FLAG_RXNE) == RESET)
  {}
  return USART_ReceiveData(com);
}

u8 fun_spiSwap(SPI_TypeDef* spix,u8 data)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET)      //?????????
  {

  }			  
  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(spix, data);                                    //????SPI1??????

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET)   //??????????
  {

  }	  						    
  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(spix);                  
}

int fun_printf(USART_TypeDef* com, char * plog,...)
{
    int nret = 0;
    va_list args;
    char data[30];
    char *p;
    if (plog == NULL) {
      return nret;
    }	
    va_start(args, plog);
    nret = vsprintf(data,plog,args);
    p = data;
    while(*p)
    {
      fun_usartSend(com, *p++);
    }
    va_end(args);
    return nret;
}


