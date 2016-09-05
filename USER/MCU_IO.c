/**
  ******************************************************************************
  * @file    MCU_IO.c
  * @author  TeReSol
  * @version V1.0.0
  * @date    24-July-2013
  * @brief   This file contains all the initilization functions for USART,CAN,I2C,ADC,DMA,DAC,NVIC and SPI.
  ******************************************************************************
*/
#include "Global_Defines.h"

/* GPIO Variables ------------------------------------------------------------------*/


/* USART Variables ------------------------------------------------------------------*/
USART_TypeDef* COM_NO[COMn] 	  			= {COM_NO1,0,0,COM_NO4,COM_NO5}; 

GPIO_TypeDef*  COM_TX_PORT[COMn]		  = {COM1_TX_PORT,0,0,COM4_TX_PORT,COM5_TX_PORT};
GPIO_TypeDef*  COM_RX_PORT[COMn]      = {COM1_RX_PORT,0,0,COM4_RX_PORT,COM5_RX_PORT};
 
const uint32_t COM_CLK[COMn]    			= {COM1_CLK,0,0,COM4_CLK,COM5_CLK};

const uint32_t COM_TX_CLK[COMn]  			= {COM1_TX_CLK,0,0,COM4_TX_CLK,COM5_TX_CLK};
const uint32_t COM_RX_CLK[COMn]  			= {COM1_RX_CLK,0,0,COM4_RX_CLK,COM5_RX_CLK};

const uint16_t COM_TX_PIN[COMn]       = {COM1_TX_PIN,0,0,COM4_TX_PIN,COM5_TX_PIN};
const uint16_t COM_RX_PIN[COMn]       = {COM1_RX_PIN,0,0,COM4_RX_PIN,COM5_RX_PIN};

const uint16_t COM_TX_PINSOURCE[COMn] = {COM1_TX_PINSOURCE,0,0,COM4_TX_PINSOURCE,COM5_TX_PINSOURCE};
const uint16_t COM_RX_PINSOURCE[COMn] = {COM1_RX_PINSOURCE,0,0,COM4_RX_PINSOURCE,COM5_RX_PINSOURCE};

const uint16_t COM_IRQn[COMn]       	= {COM1_IRQn,0,0,COM4_IRQn,COM5_IRQn};

const uint32_t COM_BAUDRATE[COMn]    	= {COM1_BAUDRATE,0,0,COM4_BAUDRATE,COM5_BAUDRATE};


/* I2C Variables ------------------------------------------------------------------*/
I2C_TypeDef* 	 I2C_NO[I2Cn] 	  			= {I2C_NO1,I2C_NO2}; 

GPIO_TypeDef*  I2C_SCL_PORT[I2Cn]		  = {I2C1_SCL_PORT,I2C2_SCL_PORT};
GPIO_TypeDef*  I2C_SDA_PORT[I2Cn]     = {I2C1_SDA_PORT,I2C2_SDA_PORT};
 
const uint32_t I2C_CLK[I2Cn]    			= {I2C1_CLK,I2C2_CLK};

const uint32_t I2C_SCL_CLK[I2Cn]  		= {I2C1_SCL_CLK,I2C2_SCL_CLK};
const uint32_t I2C_SDA_CLK[I2Cn]  		= {I2C1_SDA_CLK,I2C2_SDA_CLK};

const uint16_t I2C_SCL_PIN[I2Cn]      = {I2C1_SCL_PIN,I2C2_SCL_PIN};
const uint16_t I2C_SDA_PIN[I2Cn]      = {I2C1_SDA_PIN,I2C2_SDA_PIN};

const uint8_t  I2C_OwnAddress[I2Cn]   = {0x00,0x01};

const uint32_t I2C_Speed[I2Cn]    		= {I2C1_SPEED,I2C2_SPEED};

const uint16_t I2C_IRQn[I2Cn]         = {I2C1_ER_IRQn,I2C2_ER_IRQn};

/* SPI Variables ------------------------------------------------------------------*/
SPI_TypeDef* 	 SPI_NO[SPIn] 	  			= {SPI_NO1,SPI_NO2}; 

GPIO_TypeDef*  SPI_SCK_PORT[SPIn]		  = {SPI1_SCK_PORT,SPI2_SCK_PORT};
GPIO_TypeDef*  SPI_MOSI_PORT[SPIn]    = {SPI1_MOSI_PORT,SPI2_MOSI_PORT};
GPIO_TypeDef*  SPI_MISO_PORT[SPIn]		= {SPI1_MISO_PORT,SPI2_MISO_PORT};
GPIO_TypeDef*  SPI_CS_PORT[SPIn]      = {SPI1_CS_PORT,SPI2_CS_PORT};
 
const uint32_t SPI_CLK[SPIn]    			= {SPI1_CLK,SPI2_CLK};

const uint32_t SPI_SCK_CLK[SPIn]  		= {SPI1_SCK_CLK,SPI2_SCK_CLK};
const uint32_t SPI_MOSI_CLK[SPIn]  		= {SPI1_MOSI_CLK,SPI2_MOSI_CLK};
const uint32_t SPI_MISO_CLK[SPIn]  		= {SPI1_MISO_CLK,SPI2_MISO_CLK};
const uint32_t SPI_CS_CLK[SPIn]  		  = {SPI1_CS_CLK,SPI2_CS_CLK};

const uint16_t SPI_SCK_PIN[SPIn]      = {SPI1_SCK_PIN,SPI2_SCK_PIN};
const uint16_t SPI_MOSI_PIN[SPIn]     = {SPI1_MOSI_PIN,SPI2_MOSI_PIN};
const uint16_t SPI_MISO_PIN[SPIn]     = {SPI1_MISO_PIN,SPI2_MISO_PIN};
const uint16_t SPI_CS_PIN[SPIn]       = {SPI1_CS_PIN,SPI2_CS_PIN};

/* Timer Variables ------------------------------------------------------------------*/
TIM_TypeDef* 	 TIMER_NO[TIMERn]   		= {TIMER_NO2,TIMER_NO3,TIMER_NO4}; 
 
const uint32_t TIMER_CLK[TIMERn]    	= {TIMER2_CLK,TIMER3_CLK,TIMER4_CLK};
const uint16_t TIMER_IRQn[TIMERn]     = {TIMER2_IRQn,TIMER3_IRQn,TIMER4_IRQn};

const uint16_t TIMER_AUTORELOAD[TIMERn]= {TIMER2_AUTORELOAD_VLAUE,TIMER3_AUTORELOAD_VLAUE,TIMER4_AUTORELOAD_VLAUE};
/* Private define ------------------------------------------------------------*/

#define ADC1_DR_ADDRESS    ((uint32_t)0x4001244C)

/**
  * @brief  Initiate specified ADC
  * @param  None
  * @retval None
  */
void ADC_Channel_Init(FunctionalState NewState)
{
	// Enable ADC1 clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, NewState);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, NewState);
	 
	DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ioRead.ADCConvertedValue;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel1, NewState);
	
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = NO_OF_ADC_CHANNELS;
  ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channel12 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_MAIN_12V_CH, 1, ADC_SampleTime_1Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_INT_BATT_CH, 2, ADC_SampleTime_1Cycles5);
	
	ADC_RegularChannelConfig(ADC1, ADC_IO1_CH, 3, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_IO2_CH, 4, ADC_SampleTime_1Cycles5);
	
}
/**
  * @brief  Start ADC
  * @param  None
  * @retval None
  */
void ADC_StartConversion(void  )
{
	
  ADC_DMACmd(ADC1, ENABLE);											/* Enable ADC3 DMA */
  
  ADC_Cmd(ADC1, ENABLE);												/* Enable ADC3 */
	  
//  ADC_ResetCalibration(ADC1); 									/* Enable ADC1 reset calibration register */ 
//  
//  while(ADC_GetResetCalibrationStatus(ADC1));		/* Check the end of ADC1 reset calibration register */

//  ADC_StartCalibration(ADC1); 									/* Start ADC1 calibration */

//  while(ADC_GetCalibrationStatus(ADC1));  			/* Check the end of ADC1 calibration */
	
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);				/* Start ADC3 Software Conversion */ 
}

/**
  * @brief  Stop specified Timer
  * @param  None
  * @retval None
  */
void ADC_StopConversion(void  )
{
	/* Enable ADC3 DMA */
  ADC_DMACmd(ADC1, DISABLE);
  /* Enable ADC3 */
  ADC_Cmd(ADC1, DISABLE);
}

/**
  * @brief  Stop specified Timer
  * @param  None
  * @retval None
  */
/*void Timer_Stop(TIMER_TypeDef  TIM)
{
	TIM_Cmd(TIMER_NO[TIM], DISABLE);
}*/
/**
  * @brief  Initiate specified Timer and configure it
	* @param  NewState: new state of the specified peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void Timer_Init(TIMER_TypeDef  TIM,  FunctionalState NewState )
{
  /* Enable the TIM2 global Interrupt */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQn[TIM];
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure);	
	
	if(TIMER_NO[TIM] == TIM2 || TIMER_NO[TIM] == TIM3 || TIMER_NO[TIM] == TIM4)
		RCC_APB1PeriphClockCmd(TIMER_CLK[TIM], NewState);
	
	/* As APB1_Prescaler=4 so according to Clock tree; if APB1 prescaler !=1 then Timer_Input_Clock will be x2 i.e; 84MHz  */
  /* Time base configuration for 8sec i.e; TIM_Period=(8sec*84MHz)/(TIM_CKD_DIV1*65000)=10338   */
  TIM_TimeBaseStructure.TIM_Period = TIMER_AUTORELOAD[TIM];
  TIM_TimeBaseStructure.TIM_Prescaler = 65000;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIMER_NO[TIM], &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIMER_NO[TIM], NewState);
	
	TIM_SetAutoreload(TIMER_NO[TIM],TIMER_AUTORELOAD[TIM]);
  /* TIM IT enable */
  TIM_ITConfig(TIMER_NO[TIM], TIM_IT_Update , NewState);
	TIM_Cmd(TIMER_NO[TIM], NewState);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  None
  * @retval None
  */
void GPIO_EXTI_Init(void)
{
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOE);

  /* Enable the PORT Clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
													RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_All  ;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_CLOCK_SPEED;

  //GPIO_Init(GPIOA, &GPIO_InitStructure);//Debug port 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

  /************************* Configure GPIO pins ****************************/
	
//	GPIO_InitStructure.GPIO_Pin =  GSM_DCD_PIN ;				/****************GSM DCD PIN***********************/
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(GSM_DCD_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RELAY1_PIN | RELAY2_PIN;/****************RELAY 1 & 2***********************/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GSM_PWRKEY_PIN;					/****************GSM POWER KEY***********************/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GSM_PWRKEY_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPS_RESET_PIN ;					/****************GPS RESET PIN***********************/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPS_RESET_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  MAIN_POWER_DETECT_PIN;	/****************MAIN POWER DETECT*******************/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPS_FIX_LED_PIN ;				/****************GPS FIX LED*************************/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = STATE_MACHINE_LED_PIN ;	/****************STATE MACHINE LED*******************/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  D1_OPTO_PIN | D2_OPTO_PIN |	/****************DIGITAL IO*******************/
																 D3_OPTO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/************************* Configure ADC pins ****************************/
	
	GPIO_InitStructure.GPIO_Pin = ADC1_LN10_PIN | ADC1_LN11_PIN /****************ADC PINS*******************/
															| MAIN_12V_ADC_PIN | INT_BATT_ADC_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/************************* Configure EXTI GPIOs ****************************/	
	
	GPIO_InitStructure.GPIO_Pin =  GSM_RI_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
  GPIO_Init(GSM_RI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  BMA_INT_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;
  GPIO_Init(BMA_INT_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GSM_STATUS_PIN ;//if GSM module is ON this PIN is high and if module is OFF this PIN is low
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GSM_STATUS_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  SOS_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
  GPIO_Init(SOS_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  CAR_IGNITION_ON_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
  GPIO_Init(CAR_IGNITION_ON_PORT, &GPIO_InitStructure);
	
	/************************* Connect EXTI pins ********************************/	
	
//	GPIO_EXTILineConfig(GSM_STATUS_PORTSOURCE ,GSM_STATUS_PINSOURCE);
	GPIO_EXTILineConfig(GSM_RI_PORTSOURCE ,GSM_RI_PINSOURCE);
	GPIO_EXTILineConfig(BMA_INT_PORTSOURCE ,BMA_INT_PINSOURCE);
	GPIO_EXTILineConfig(SOS_PORTSOURCE ,SOS_PINSOURCE);
	GPIO_EXTILineConfig(CAR_IGNITION_ON_PORTSOURCE ,CAR_IGNITION_ON_PINSOURCE);
	
//	// Configure GSM_STATUS_EXTI_LINE   
//  EXTI_InitStructure.EXTI_Line = GSM_STATUS_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
//  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
//  EXTI_Init(&EXTI_InitStructure);
	
	// Configure GSM_RI EXTI line 
  EXTI_InitStructure.EXTI_Line = GSM_RI_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	// Configure BMC_INT1 EXTI line 
  EXTI_InitStructure.EXTI_Line = BMA_INT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	// Configure SOS_PIN EXTI line 
  EXTI_InitStructure.EXTI_Line = SOS_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	// Configure CAR_IGNITION_ON_EXTI line 
  EXTI_InitStructure.EXTI_Line = CAR_IGNITION_ON_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
}
/**
  * @brief  Usually used in sleep mode,Enables GSM_RI interrupt for detecting any SMS received,GSM_STATUS 
						and CAR_IGNITION for exiting sleep mode
	*					Disable all interrupts when system exit the sleep mode
	* @param  NewState: new state of the specified peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SleepMode_Interrupt( FunctionalState NewState )
{
//	// Configure GSM_RI EXTI line 
//  EXTI_InitStructure.EXTI_Line = GSM_RI_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
//  EXTI_InitStructure.EXTI_LineCmd = NewState;
//  EXTI_Init(&EXTI_InitStructure);
//	
//	// Configure GSM_STATUS_EXTI_LINE   
//  EXTI_InitStructure.EXTI_Line = GSM_STATUS_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
//  EXTI_InitStructure.EXTI_LineCmd = NewState;
//  EXTI_Init(&EXTI_InitStructure);
//	
//	// Configure CAR_IGNITION_ON_EXTI line 
//  EXTI_InitStructure.EXTI_Line = CAR_IGNITION_ON_EXTI_LINE;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
//  EXTI_InitStructure.EXTI_LineCmd = NewState;
//  EXTI_Init(&EXTI_InitStructure);
}
/**
  * @brief  Enable External Interrupt lines
	* @param  NewState: new state of the specified peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void NVIC_Interrupt(FunctionalState NewState )
{
	// Enable and set (BMA_INT1) Interrupt priority 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = BMA_INT_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure);
	
//	// Enable and set GSM_STATUS
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	NVIC_InitStructure.NVIC_IRQChannel = GSM_STATUS_EXTI_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
//  NVIC_Init(&NVIC_InitStructure);
	
	// Enable and set GSM_RI_EXTI_LINE
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = GSM_RI_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure);
	
	// Enable and set CAR_IGNITION_ON_PIN Interrupt priority 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = CAR_IGNITION_ON_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure);
	
		// Enable and set SOS_PIN Interrupt priority 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures USART COM port.
  * @param  COM: Specifies the USART to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2
  *     @arg COM3
  *     @arg COM4
  *     @arg COM5 
	* @param  NewState: new state of the specified peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */

void COM_Init(COM_TypeDef COM, FunctionalState NewState )
{
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(COM_TX_CLK[COM] | COM_RX_CLK[COM] | RCC_APB2Periph_AFIO, NewState);

	
  /* Enable UART clock */
  if (COM == COM1)
  {
    RCC_APB2PeriphClockCmd(COM_CLK[COM], NewState); 
  }
  else
  {
    RCC_APB1PeriphClockCmd(COM_CLK[COM], NewState);
  }
	
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_CLOCK_SPEED;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);
	
	/* USART configured as follow:
	            - BaudRate = 115200 baud
	            - Word Length = 8 Bits
	            - One Stop Bit
	            - No parity
	            - Hardware flow control disabled (RTS and CTS signals)
	            - Receive and transmit enabled
	            - USART Clock disabled
	            - USART CPOL: Clock is active low
	            - USART CPHA: Data is captured on the middle
	            - USART LastBit: The clock pulse of the last data bit is not output to
	                             the SCLK pin
	*/
	USART_InitStructure.USART_BaudRate = COM_BAUDRATE[COM];
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx |USART_Mode_Tx;
  USART_Init(COM_NO[COM], &USART_InitStructure);
		
	if(COM == COM5)
	{
	USART_ITConfig(COM_NO[COM], USART_IT_RXNE , NewState);
	USART_ITConfig(COM_NO[COM],  USART_IT_ERR , NewState);
	}
  /* Enable USART */
  USART_Cmd(COM_NO[COM], NewState);
	
	if(COM == COM5)
	{
  /* Enable the USARTx Interrupt */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel =COM_IRQn[COM];
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = NewState;
  NVIC_Init(&NVIC_InitStructure); 
	}
}


/**
  * @brief  Send string through USART
  * @param  COM: Specifies the USART to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM4  
  * @param  *pucBuffer: pointer to the string 
  * @retval None
  */
void USART_SendData_s(COM_TypeDef COM, unsigned char *pucBuffer)
{
	while(USART_GetFlagStatus(COM_NO[COM], USART_FLAG_TC) == RESET);
	// Loop while there are more characters to send.
  while(*pucBuffer)
	{
		USART_SendData(COM_NO[COM], (uint16_t) *pucBuffer++);
    /* Loop until the end of transmission */
    while(USART_GetFlagStatus(COM_NO[COM], USART_FLAG_TC) == RESET);
	}
}
/**
  * @brief  Send a byte through USART
  * @param  COM: Specifies the USART to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM4  
  * @param  ch: byte to be sent 
  * @retval None
  */
void USART_Put(COM_TypeDef COM, uint8_t ch)
{
	while(USART_GetFlagStatus(COM_NO[COM], USART_FLAG_TC) == RESET);
	USART_SendData(COM_NO[COM], (uint8_t) ch);
  //Loop until the end of transmission
  while(USART_GetFlagStatus(COM_NO[COM], USART_FLAG_TC) == RESET);
	
}
/**
  * @brief  Get a byte through USART
  * @param  COM: Specifies the USART to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM4  
  * @retval uint8_t: returns single byte received
  */
uint8_t USART_Get(COM_TypeDef COM)
{
	int32_t timer=10000000;
	while ( USART_GetFlagStatus(COM_NO[COM], USART_FLAG_RXNE) == RESET)
	{
		timer--;
		if(timer<=0)
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"USART_GET_BREAK\r\n");
			COM_Init(GPS_COM,DISABLE);	
			GPS_RESET_ON();
			Delay_ms(1000);	
			GPS_RESET_OFF();
			COM_Init(GPS_COM,ENABLE);
			break;
		}
	}
	
	return (uint8_t)USART_ReceiveData(COM_NO[COM]);
}
/**
  * @brief  Configures I2C  port.
  * @param  I2C: Specifies the I2C to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg I2C1
  *     @arg I2C2 
	* @param  NewState: new state of the specified peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
//void I2CBus_Init(I2CBus_TypeDef I2C, FunctionalState NewState )
//{
//  /* Enable GPIO clock */
//  RCC_APB2PeriphClockCmd(I2C_SCL_CLK[I2C] | I2C_SDA_CLK[I2C] | RCC_APB2Periph_AFIO, ENABLE);

//  /* Enable I2C clock */
//  RCC_APB1PeriphClockCmd(I2C_CLK[I2C], ENABLE);
//  
//	/* Configure I2C pins: SCL  : Open Drain, I2C bus pulled high externally */
//  GPIO_InitStructure.GPIO_Pin =  I2C_SCL_PIN[I2C];
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//  GPIO_InitStructure.GPIO_Speed = GPIO_CLOCK_SPEED;
//  GPIO_Init(I2C_SCL_PORT[I2C], &GPIO_InitStructure);

//  /* Configure USART Rx as input floating */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//  GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN[I2C];
//  GPIO_Init(I2C_SDA_PORT[I2C], &GPIO_InitStructure);
//	
//	/* Reset the Peripheral */
//  RCC_APB1PeriphResetCmd(I2C_CLK[I2C], ENABLE);
//  RCC_APB1PeriphResetCmd(I2C_CLK[I2C], DISABLE);
//	
//	/* I2C configuration */
//  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//  I2C_InitStructure.I2C_OwnAddress1 = I2C_OwnAddress[I2C];
//  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed[I2C];
//  I2C_Init(I2C_NO[I2C] , &I2C_InitStructure);
//  /* Enable I2C */
//  I2C_Cmd(I2C_NO[I2C] , ENABLE);
//}

/**
  * @brief  Configures SPI  port.
  * @param  I2C: Specifies the SPI to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg SPI1
  *     @arg SPI2
  *     @arg SPI3
	* @param  NewState: new state of the specified peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */

void SPIBus_Init(SPIBus_TypeDef SPI, FunctionalState NewState )
{
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(SPI_SCK_CLK[SPI] | SPI_MOSI_CLK[SPI] | SPI_MISO_CLK[SPI] | SPI_CS_CLK[SPI] | RCC_APB2Periph_AFIO, ENABLE);

 /* Enable SPI clock */
  if (SPI == SPIBus1)
  {
    RCC_APB2PeriphClockCmd(SPI_CLK[SPI], ENABLE); 
  }
  else
  {
    RCC_APB1PeriphClockCmd(SPI_CLK[SPI], ENABLE);
  }
  
  /*!< Configure SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN[SPI];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI_SCK_PORT[SPI], &GPIO_InitStructure);

  /*!< Configure SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN[SPI];
  GPIO_Init(SPI_MOSI_PORT[SPI], &GPIO_InitStructure);

  /*!< Configure SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN[SPI];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_Init(SPI_MISO_PORT[SPI], &GPIO_InitStructure);
  
  /*!< Configure SPI pin: CS pin */
  GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN[SPI];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_CS_PORT[SPI], &GPIO_InitStructure);
	
	
	/*!< Deselect the FLASH: Chip Select high */
  SPIBus_CS_high_low(SPI,SET);

  /*!< SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_NO[SPI], &SPI_InitStructure);

  /*!< Enable the sFLASH_SPI  */
  SPI_Cmd(SPI_NO[SPI], ENABLE);
}

/**
  * @brief  Configures SPI CS Pin high or low.
  * @param  SPI: Specifies the SPI to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg SPI1
  *     @arg SPI2
  *     @arg SPI3
  * @param  CS_state: Specifies the SPI CS pin new state.
  *   This parameter can be one of following parameters:    
  *     @arg SET
  *     @arg RESET
  * @retval None
  */
void SPIBus_CS_high_low(SPIBus_TypeDef SPI, uint8_t CS_state)
{
	if(CS_state==RESET)
	{
		GPIO_ResetBits(SPI_CS_PORT[SPI], SPI_CS_PIN[SPI]);
	}
	else
	{
		GPIO_SetBits(SPI_CS_PORT[SPI], SPI_CS_PIN[SPI]);
	}
}

/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
