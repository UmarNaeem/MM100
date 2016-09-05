/****************************************Copyright (c)****************************************************
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               main.c
** Descriptions:            IN DASH application 
**
**--------------------------------------------------------------------------------------------------------
** Created by:              TeReSol
** Created date:            2013-07-24
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Defines & Includes ------------------------------------------------------------------*/
#define VAR_DECLS

#include "Global_Defines.h"
#include "main.h"

/* Private functions ---------------------------------------------------------*/
void stateMachine(void);
/* EXTERN variables ---------------------------------------------------------*/
extern unsigned char rxBufferLen;
extern unsigned char smsRead;
extern uint8_t l_gsmServerVoiceCallFlag;
extern const char cell1C[];
extern uint8_t l_cellNumber[15];
extern uint8_t l_counter;
uint8_t l_callVerifyLoop=0;
uint16_t gpsBufferLen = 0;
uint16_t gpsTempBufferLen = 0;
extern USART_TypeDef* COM_NO[COMn];

uint32_t l_ackpacketno=1;
uint32_t l_ackdelay=0;
extern uint32_t l_stringCounter;
extern uint32_t l_SendPktCounter;	

/* Private variables ---------------------------------------------------------*/
unsigned char configMode = 0;
uint8_t g_byteCounter=0;
uint16_t gprsRxBufferLen=0;

static __IO uint32_t uwLsiFreq = 0;
__IO uint32_t uwTimingDelay_ms = 0;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;
uint16_t tmpCC4[2] = {0, 0};


uint32_t size_of_file = 0;
uint8_t file_size_index = 0;
uint8_t file_size[10];
uint8_t file_download_complete = 0;
/* Private Functions ------------------------------------------------------------------*/
uint32_t GetLSIFrequency(void);
void WWDT_Init(void);
void System_Init(void);
void RTC_Config(void);

/**
  * @brief  This function handles TIM5 global interrupt request used to measure the LSI oscillator frequency .
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
  {    
    tmpCC4[uwCaptureNumber++] = TIM_GetCapture4(TIM5);									// Get the Input Capture value
   
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);														// Clear CC4 Interrupt pending bit

    if (uwCaptureNumber >= 2)
    {
      uwPeriodValue = (uint16_t)(0xFFFF - tmpCC4[0] + tmpCC4[1] + 1);		// Compute the period length
    }
  }
}
/**
  * @brief  Configures TIM5 to measure the LSI oscillator frequency. 
  * @param  None
  * @retval LSI Frequency
  */
uint32_t GetLSIFrequency(void)
{	
  RCC_LSICmd(ENABLE);																						// Enable the LSI oscillator
  
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){}					// Wait till LSI is ready 

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);					// Enable TIM5 clock
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;								// Enable TIM5 Interrupt channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);		// Configure TIM5 presclaer
	
  GPIO_PinRemapConfig(GPIO_Remap_TIM5CH4_LSI, ENABLE);					// Connect internally the TM5_CH4 Input Capture to the LSI clock output

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;										/* TIM5 configuration: Input Capture mode ---- */
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;					/* The LSI oscillator is connected to TIM5 CH4 */
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;			/* The Rising edge is used as active edge,TIM5 */
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;								/* CCR4 is used to compute the frequency value */
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);
  
  TIM_Cmd(TIM5, ENABLE);																				// Enable TIM5 counter

  TIM5->SR = 0;																									// Reset the flags
     
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);												// Enable the CC4 Interrupt Request
  
  while(uwCaptureNumber != 2){}																	// Wait until the TIM5 get 2 LSI edges

  TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);											// Disable TIM5 CC4 Interrupt Request
	
  RCC_GetClocksFreq(&Clock_Frequency);													// Get SYSCLK, HCLK and PCLKx frequency

  if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0)												// Get PCLK1 prescaler
  { 
    return ((Clock_Frequency.PCLK1_Frequency / uwPeriodValue) * 8);	// PCLK1 prescaler equal to 1 => TIMCLK = PCLK1
  }
  else
  { 
    return (((2 * Clock_Frequency.PCLK1_Frequency) / uwPeriodValue) * 8) ;	// PCLK1 prescaler different from 1 => TIMCLK = 2 * PCLK1
  }
}

/**
  * @brief  Initialize Watchdog timer peripheral
  * @param  None
  * @retval None
  */

void WWDT_Init(void)
{
//	unsigned char l_tempPacket[20];	
	
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)		// Check if the system has resumed from IWDG reset
  {
																											// IWDGRST flag set
		USART_SendData_s(DEBUG_COM,(unsigned char*)"Sytem has resumed after IWDT Reset\r\n"); 
		states.sleepModeACKFlag = 1;
    RCC_ClearFlag();																	// Clear reset flags 
  }
	else
  {   																								// IWDGRST flag is not set
		USART_SendData_s(DEBUG_COM,(unsigned char*)"Sytem has resumed after Normal Power ON Reset\r\n");
		states.sleepModeACKFlag = 2;
  }

  uwLsiFreq = GetLSIFrequency();											// Get the LSI frequency:  TIM5 is used to measure the LSI frequency

//	USART_SendData_s(DEBUG_COM,(unsigned char*)"IWDG LSI Frequency=");
//	sprintf(( char*)l_tempPacket,"%d",uwLsiFreq);	
//	USART_SendData_s(DEBUG_COM,l_tempPacket);
//	USART_SendData_s(DEBUG_COM,(unsigned char*)" Hz\r\n");
	
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);				// IWDG timeout may varies due to LSI frequency dispersion 
																											// Enable write access to IWDG_PR and IWDG_RLR registers
  
  IWDG_SetPrescaler(IWDG_Prescaler_256);							// IWDG counter clock: LSI/256
   
	IWDG_SetReload(0x0FFF-1);														//IWDG_SetReload((uwLsiFreq*28)/256);
																											// Max 0x0FFF(4095) can be entered. At 36.568kHz max Timeout willl be 28sec
  
  IWDG_ReloadCounter();																// Reload IWDG counter

  IWDG_Enable();																			// Enable IWDG (the LSI oscillator will be enabled by hardware)
}

/**
  * @brief  Initialize necessary system peripherals
  * @param  None
  * @retval None
  */
void System_Init(void)
{
	GPIO_EXTI_Init();											// Initiate GPIO and EXTI
	
	COM_Init(GSM_COM,ENABLE);							// Initiate UART5 for GSM Module communication 
	
	SPIBus_Init(EEPROM_SPI,ENABLE);				// Initiate SPI1 for EEPROM 
	
	SPIBus_Init(BMA150_SPI,ENABLE);				// Initiate SPI2 for BMA150 
	
	COM_Init(GPS_COM,ENABLE);							// Initiate UART4 for GPS Module
	
	COM_Init(DEBUG_COM,ENABLE);						// Initiate USART1 for Debugging
	
	ADC_Channel_Init(ENABLE);							// Initiate ADC
}

/**
  * @brief  Configures the RTC clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);		// Enable the PWR clock 
	
	EXTI_ClearITPendingBit(EXTI_Line17);  									// Connect EXTI_Line17 to the RTC Wakeup event
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  PWR_BackupAccessCmd(ENABLE);														// Allow access to RTC 
	
  BKP_DeInit();																						// Reset Backup Domain 

	RCC_LSEConfig(ENABLE);																	// Enable the LSE OSC 
	 
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){}			// Wait till LSE is ready 			
	
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);									// Select LSE as RTC Clock Source 
		
	RCC_RTCCLKCmd(ENABLE);																	// Enable the RTC Clock 
	
	RTC_WaitForSynchro();																		// Wait for RTC APB registers synchronisation
	
	RTC_SetPrescaler(32767); 																// RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
	
	RTC_WaitForLastTask();																	// Wait until last write operation on RTC registers has finished 
	
	RTC_ITConfig(RTC_IT_ALR, ENABLE);												// Enable the RTC Second
	
	RTC_WaitForLastTask();																	// Wait until last write operation on RTC registers has finished

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);					// Enable the RTC Wakeup Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;			
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	Delay(1500);
	
  RTC_ClearFlag(RTC_FLAG_SEC);														// Wait till RTC Second event occurs
  while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
	
	RTC_SetAlarm(RTC_GetCounter()+ 19);											//Set RTC Alarm for 20 sec

	RTC_WaitForLastTask();																	// Wait until last write operation on RTC registers has finished 
}

/**
  * @brief  Main Programme.
  * @param  None
  * @retval None
  */
int main(void)
{
	uint16_t send=0;
	/* Set the Vector Table base location at 0x4000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
	
	IAP.FTPConnectCounter=0;
	
	GPS_LOG.IgnitionOffSaveFlag = FALSE;
	GPS_LOG.MDLatLongUpdateFlag = FALSE;
	
	gpsTempBufferIndex = 0;
	GPS_LOG.DataReceiveFlag=FALSE;
	GPS_LOG.NoFixDetectCounter=0;
	states.gpsBreakFlag=FALSE;
	states.gsmIMEIFixed=FALSE;
	IAP.AfterIAPBootFlag=FALSE;
	
	GPS_LOG.TimerSelectFlag=FALSE;
	
	Ack.DataHandlerFlag=FALSE;
	Ack.LoginAckRetransmitSend=FALSE;
	
	states.JammingDetectFlag=FALSE;
	states.FTPPUTConnectFlag=FALSE;
	states.AckGprsErrorCounter=0;
	states.AckPacketErrorCounter=0;
	states.AckTimeOutCounter=0;
	
	ioRead.NextMotionAlertCount=0;
	states.AuthenticationOkFlag=FALSE;
	states.ReAuthenticationFlag=FALSE;
	states.ReAuthenticationCounter=0;

	GPS_LOG.geoFencingType[0] = '0';	
	GPS_LOG.geoFenceName[0] = '0';	
	config.mcuRebootFlag = TRUE;// reason code for server
	states.HeartBeatFlag = FALSE;
	config.IAPUpdatedFlag = FALSE;
	eeprom.eepromReadPointer = 0;
	eeprom.eepromWritePointer = 0;
	eeprom.eepromWriteLogicAlgo = NEW_DATA_DISCARD_ALGO ;
	
	eeprom.eepromRecoveryEnable = FALSE ;

	states.CallOngoingFlag=FALSE;
	ioRead.trackerOnFlag = TRUE;
	ioRead.trackerFirstOn = TRUE;
	
	ioRead.mainPowerFlagStateClear = TRUE;
	
	g_oneTouchDialEventFlag=FALSE;
	
	GPS_LOG.distanceAasci[0] = '0'; // to make odometer start with '0'
	config.totalTimeTravelledAASCI[0] = '0';// to make time start with '0'
	
	GPS_LOG.numberOfSatellite[0] = '0';
	GPS_LOG.TimerFlag=FALSE;
	
	Ack.LoginPackFlag=TRUE;
	Ack.LoginAckRetransmit=2;
	Ack.AckRecFlag=FALSE;
	
	Ack.PacketSendNo=0;
	states.IgnitionOnFlag = FALSE;
	
	ioRead.ignitionStateClear = FALSE;
	ioRead.DIO1StateClear = FALSE;
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);	
	arrayInit2Zero(config.IMEI,IMEISize);

	/* Setup SysTick Timer for 1 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.
    
    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
       call. The NVIC_SetPriority() is defined inside the core_cm4.h file.

    4. To adjust the SysTick time base, use the following formula:
                            
         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
    
       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF
   */
	/* Setup SysTick Timer for 1 msec interrupts(SysTick_CTRL_CLKSOURCE_Msk=(1UL << SysTick_CTRL_CLKSOURCE_Pos) 
	sets 3rd bit by default by calling SysTick_Config() which enables HCLK clock to be configured as SysTick 
	clock source. SysTick_CLKSourceConfig() function actually sets or clears this bit in SysTick->CTRL register, 
	at 3rd bit position which ontrols SysTick clock source,to configure HCLK or HCLK/8 as SysTick Clock source 
	respectively.) */
	SysTick_Config((SystemCoreClock / 1000));
	NVIC_Interrupt(DISABLE);	
	SleepMode_Interrupt(DISABLE);

	System_Init();
	
	/* Set Tracker LED banks(Setting bit turn off LEDs) */
	GPS_FIX_LED_ON();
	STATE_MACHINE_LED_OFF();

	GPIO_ResetBits(GSM_PWRKEY_PORT , GSM_PWRKEY_PIN);
	GPS_RESET_OFF();
		
	USART_SendData_s( DEBUG_COM, (unsigned char*)"\r\nFirmware Version 2.1\r\n");
	USART_SendData_s( DEBUG_COM, (unsigned char*)"Date: 03/10/2014\r\n");
	
	USART_SendData_s( DEBUG_COM,(unsigned char*) "Your Main Start\r\n");
	
	displaySysClocks();
	
	gsmPowerOn();/////////
	
	if(Accelerometer_Test()==TRUE)	
	{
		BMC_init(ACC_RANGE_2g,ACC_BW_25Hz);
		AnyMotionCriteriaSet();		
	}
	
	Config_FactorySettings_SIM1();
	//EEPROM_Config_Reset_Enable();

	multiByteContentRead(IMEI_START,IMEI_LEN,config.IMEI); /// read IMEI everytime from eeprom
	Factory_Config_Write();
	Factory_Config_Read(TRUE,config.simNumber);

	WWDT_Init();	
	NVIC_Interrupt(ENABLE);	
	Start_Blink();
	// RTC configuration 
	RTC_Config();
	
	ADC_StartConversion();
	Delay_ms(2);
	
	Timer_Init(Timer2,ENABLE);		/* 8 sec Timer for Ack timeout */
	
	Timer_Init(Timer4,ENABLE);		/* 1sec Timer,For OBD2 & polling state machine */
	
 USART_SendData_s( GPS_COM,(unsigned char*)"$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");		//Turn OFF GPGLL string
 Delay_ms(200);
 USART_SendData_s( GPS_COM,(unsigned char*)"$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");		//Turn OFF GPGSV string
 Delay_ms(200);
 USART_SendData_s( GPS_COM,(unsigned char*)"$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");		//Turn OFF GPGSA string
 Delay_ms(200);
 USART_SendData_s( GPS_COM,(unsigned char*)"$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");		//Turn OFF GPVTG string
 Delay_ms(200);

	while (1)
	{		
		if(states.AuthenticationOkFlag==FALSE && config.AuthenticateFuelKSEnDs==TRUE &&  states.CarKillFlag==FALSE)
		{
			// Fuel cutoff relay ON
			USART_SendData_s( DEBUG_COM,(unsigned char*)"Fuel-CutOff-Relay KILL\r\n");
			FUEL_KS_CUT;
			states.CarKillFlag=TRUE;
		}
		else if(states.AuthenticationOkFlag==TRUE && config.AuthenticateFuelKSEnDs==TRUE && states.CarKillFlag==TRUE)
		{
			// Fuel cutoff relay OFF
			USART_SendData_s( DEBUG_COM,(unsigned char*)"Fuel-CutOff-Relay RELEASE\r\n");
			FUEL_KS_MAKE;
			states.CarKillFlag=FALSE;
		}	

		if(ioRead.trackerFirstOn == TRUE)
		{
			ioRead.trackerFirstOn = FALSE;
			for ( send = 0;send<5;send++)
			{				
				USART_SendData_s( DEBUG_COM,(unsigned char*)">\r\n");				
				Delay_ms(500);	
			}	
		}				
		if(ioRead.trackerOnFlag == TRUE)
		{	
			Main_StateMachine();	// Calls Main state Machine							
			
		}
		else if(ioRead.trackerOnFlag == FALSE && ioRead.powerStateChange == TRUE)		
		{
			 gsmPowerOff();				
			 states.gsmFirstInit = FALSE;	
			 states.GPRSStateOk = FALSE;		 	
			 states.packetAck = FALSE;
			 ioRead.powerStateChange = FALSE;			 
		}
		// Reload IWDG counter 
		IWDG_ReloadCounter();
	}
}

/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */    
void RTCAlarm_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {
		IWDG_ReloadCounter();															// Reload IWDG counter
			
		SleepModeAlgo();																	// Call sleep mode function
		
    EXTI_ClearITPendingBit(EXTI_Line17);							// Clear EXTI line17 pending bit

    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)				// Check if the Wake-Up flag is set
    {
      PWR_ClearFlag(PWR_FLAG_WU);											// Clear Wake Up flag
    }

    RTC_WaitForLastTask();   													// Wait until last write operation on RTC registers has finished
    
    RTC_ClearITPendingBit(RTC_IT_ALR);								// Clear RTC Alarm interrupt pending bit
    
    RTC_WaitForLastTask();														// Wait until last write operation on RTC registers has finished
	
		RTC_SetAlarm(RTC_GetCounter()+ 19);								// Set RTC Alarm for 20 sec
	
		USART_SendData_s( DEBUG_COM,(unsigned char*)"RTC-20\r\n");	

    RTC_WaitForLastTask();														// Wait until last write operation on RTC registers has finished 
	
	}
}

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
//		USART_SendData_s( DEBUG_COM,(unsigned char*)"TIM2-IN\r\n");
		if(Ack.DataHandlerFlag==TRUE && config.AckLayerEnDs==TRUE && states.JammingDetectFlag==FALSE)
		{
			states.AckTimeOutCounter++;
			if(states.AckTimeOutCounter>config.AckTimeout  )
			{
				states.AckTimeOutCounter=0;
				//USART_SendData_s( DEBUG_COM,"ACK-Timeout///\n\r");
				states.AckPacketErrorCounter++;//increment no ack packet error counter
				
				if(states.GPRSStateOk == TRUE && states.FTPPUTConnectFlag==FALSE )
				{		
//					USART_SendData_s( DEBUG_COM,g_ReTransmissionPaket);//Comment due to overload
					Command_Send_SEND_Get(g_ReTransmissionPaket);							
				}	
				
			}
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"TIM2\r\n");
	}
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
//		USART_SendData_s( DEBUG_COM,(unsigned char*)"TIM4-IN\r\n");
		
		#ifdef GPS_ENABLED
		if(GPS_LOG.TimerFlag==TRUE && GPS_LOG.TimerSelectFlag==FALSE)//If gps no fix and 5 sec timer is on(not in sleep mode)
		{
			GPS_LOG.TimerPollCounter++;
			if(GPS_LOG.TimerPollCounter>=5)//5sec update
			{
				TimeDate_Update();
				GPS_LOG.TimerPollCounter=0;
			}
		}
		#endif
		
		states.mainPowerPollCounter++;
		if(states.mainPowerPollCounter>=config.MainPwrHysPoll)	
		{
			MainPowerHys();	
			states.mainPowerPollCounter = 0;
		}
		
		states.ignitionPollCounter++;
		if(states.ignitionPollCounter>=config.ignitionHysPoll)	
		{
			Ignition_module();	//For car transmission algo
			IgnitionHys();//For SMS algo
			states.ignitionPollCounter = 0;
		}

		
		states.DIO1PollCounter++;
		if(states.DIO1PollCounter>=config.DIO1HysPoll)	
		{
			DIO1Hys();	
			states.DIO1PollCounter = 0;
		}
		
		states.DIO2PollCounter++;
		if(states.DIO2PollCounter>=config.DIO2HysPoll)	
		{
			DIO2Hys();	
			states.DIO2PollCounter = 0;
		}
		
		states.DIO3PollCounter++;
		if(states.DIO3PollCounter>=config.DIO3HysPoll)	
		{
			DIO3Hys(); 	
			states.DIO3PollCounter = 0;
		}
		
		states.AIO1PollCounter++;
		if(states.AIO1PollCounter>=config.AIO1HysPoll)	
		{
			AIO1Hys();	
			states.AIO1PollCounter = 0;
		}
		
		states.AIO2PollCounter++;
		if(states.AIO2PollCounter>=config.AIO2HysPoll)	
		{
			AIO2Hys();	
			states.AIO2PollCounter = 0;
		}	
		
		states.SharpTurnPollCounter++;
		if(states.SharpTurnPollCounter>=config.SharpTurnHysPoll)	
		{
			if(!GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))
			{
				magnetometer_Heading();
			}
			states.SharpTurnPollCounter = 0;
		}
		
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"TIM4\r\n");
  }
}

/**
  * @brief  USART1(GSM module) IRQ handler 
  * @param  None
  * @retval None
  */
void GSM_USART_IRQHandler (void)
{
	unsigned char temp;
	if(USART_GetITStatus(GSM_USART, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		temp= USART_ReceiveData(GSM_USART);
		//USART_Put( DEBUG_COM,temp);
		
		if(IAP.Data_Start_Flag==FALSE)
		{
			rxBuffer[rxBufferIndex] = temp;// copy the data to rx buffer 	
			rxBufferIndex++;
			if(rxBuffer[rxBufferIndex-2] == 0x0D && rxBuffer[rxBufferIndex-1] == 0x0A)// check for valid message footer
			{		
				if(rxBufferIndex<700)// filter the messages > 160 bytes
				{		
					rxBuffer[rxBufferIndex-1]='\0';// reset the buffer ending
					rxBuffer[rxBufferIndex-2]='\0';
					rxBufferIndex=0;// reset the buffer index		
					//USART_SendData_s( DEBUG_COM,"GSM:");	////	
					//USART_SendData_s( DEBUG_COM,(unsigned char*)rxBuffer);	////	
					//USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");		////
			
					rxBufferLen = strlen((const char*)rxBuffer);
				
				
					if(!strncmp((const char*)rxBuffer,(const char*)SMS_LOC_HEADER,4))// if message is received +CMTI process and stores index to buffer
					{	
						//USART_SendData_s( DEBUG_COM,"Server SMS received:\r\n");
						smsRecBuffer[smsRecBufferIndex++] = rxBuffer[12];				
						if(rxBuffer[13] != 0x0D)
						{					
							smsRecBuffer[smsRecBufferIndex++] = rxBuffer[13];						
							smsRecBuffer[smsRecBufferIndex++] = ',';						
						}
						else
						{
							smsRecBuffer[smsRecBufferIndex++] = ',';										
						}	
						noOfSmsRec++;// sms received counter								
						sms.messageReceivedFlag = TRUE;
						//USART_SendData_s( DEBUG_COM,"+CMTI\r\n");
					}		
					else if(strstr((const char*)rxBuffer,(const char*)SERVER_PKT_HEADER)  && ((650-strlen((const char*)gprsRequestBuffer))>rxBufferLen))// if @@ valid packet is received from server
					{				
						//USART_Put( DEBUG_COM,'%');
						//USART_SendData_s( DEBUG_COM,"Server CMD received:\r\n");
						strcat((char*)gprsRequestBuffer,(const char*)rxBuffer);// concatenate the data to command buffer								
						//gprsRequestCounter++;
					}			
					else if((strstr((const char*)rxBuffer,(const char*)"CMGR") || smsRead == TRUE) && ((180-strlen((const char*)smsReadBuffer))>rxBufferLen))// if sms read is requested
					{					
						strcat((char*)smsReadBuffer,(const char*)rxBuffer);// concatenate the data to command buffer
						smsRead = TRUE;// set sms read flag true
						gsmComm.smsHeaderCounter++;// count for the valid message completion
						if(gsmComm.smsHeaderCounter >=3)
						{
							smsRead = FALSE;
							gsmComm.smsHeaderCounter = 0;
						}								
					}		

					else if(strstr((const char*)rxBuffer,(const char*)"+CRING: VOICE") )// if call ring is received from server
					{				
						USART_SendData_s( DEBUG_COM,(unsigned char*)"calling\r\n");
						l_gsmServerVoiceCallFlag=TRUE;
					}	
					else if(strstr((const char*)rxBuffer,(const char*)"+CLIP:")  &&  l_gsmServerVoiceCallFlag==TRUE)// if incomming call info is received from gsm
					{										
						if(strstr((const char*)config.ServerCallCellNo1,(const char*)"0000000") && 
							strstr((const char*)config.ServerCallCellNo2,(const char*)"0000000") &&
							strstr((const char*)config.ServerCallCellNo3,(const char*)"0000000")) 
						{
							USART_SendData_s( DEBUG_COM,(unsigned char*)"Number verified:\r\n");
							
							USART_SendData_s( GSM_COM,(unsigned char*)ATTEND_CALL);	
							l_gsmServerVoiceCallFlag=FALSE;
							states.CallOngoingFlag=TRUE;
						}
						else
						{
							for(l_callVerifyLoop=0;l_callVerifyLoop<3;l_callVerifyLoop++)
							{
								for(l_counter=0;l_counter<=9;l_counter++)// Exclude area code from number
								{
									if(l_callVerifyLoop==0)
									{
										l_cellNumber[l_counter]=config.ServerCallCellNo1[l_counter+3];
									}
									else if(l_callVerifyLoop==1)
									{
										l_cellNumber[l_counter]=config.ServerCallCellNo2[l_counter+3];
									}
									else if(l_callVerifyLoop==2)
									{
										l_cellNumber[l_counter]=config.ServerCallCellNo3[l_counter+3];
									}
								}
								l_cellNumber[l_counter]=0x00;
						
								if(strstr((const char*)rxBuffer,(const char*)l_cellNumber))
								{
									USART_SendData_s( DEBUG_COM,(unsigned char*)"Server number verified:\r\n");
							
									//////////////////////////////USART_SendData_s( GSM_COM,(unsigned char*)ATTEND_CALL);								
									l_gsmServerVoiceCallFlag=FALSE;
								
									states.CallOngoingFlag=TRUE;
									break;
								}
								//else
									//USART_SendData_s( DEBUG_COM,"Server number not verified:\r\n");
							}
						}
					}	
				
					else if(strstr((const char*)rxBuffer,(const char*)"+QJDR: JAMMED"/*"+QLJDC: 1"*/))// if Jamming burst is detected
					{	
						USART_SendData_s( DEBUG_COM,(unsigned char*)"Jamming Detected:\r\n");
						
						states.JammingDetectFlag=TRUE;
					}
					
					else if(strstr((const char*)rxBuffer,(const char*)"+QJDR: NO JAMMING"/*"+QLJDC: 1"*/))// if Jamming burst is detected
					{	
						USART_SendData_s( DEBUG_COM,(unsigned char*)"Jamming Removed:\r\n");
						
						states.JammingDetectFlag=2;
					}
					
					else if((650-strlen((const char*)gsmCommandsBuffer))>rxBufferLen)
					{				
						smsRead = FALSE;
						gsmComm.smsHeaderCounter = 0;
					
						strcat((char*)gsmCommandsBuffer,(const char*)rxBuffer);// concatenate the data to command buffer, only gsm and MCU data

						if(strstr((const char*)gsmCommandsBuffer,(const char*)"CONNECT") && states.FTPPUTConnectFlag==TRUE)// if @@ valid packet is received from server
						{		
							USART_SendData_s( DEBUG_COM,(unsigned char*)"FTP CONNECT OK\r\n");
							IAP.BufferNo=0;
							IAP.sFLASH_ADD_Counter=0;
							IAP.Data_Start_Flag=TRUE;	
							IAP.USART_ISR_Count=0;
							IAP.Array=IAP.USART_ISR_buffer1;
							DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)IAP.USART_ISR_buffer1;
							DMA_Init(DMA1_Channel1, &DMA_InitStructure);
							
							///Timer_Init(Timer4,DISABLE);
						
						}
						else if(strstr((const char*)gsmCommandsBuffer,(const char*)"+QFTPGET") && states.FTPPUTConnectFlag==TRUE)
						{
							USART_SendData_s( DEBUG_COM,(unsigned char*)"FTP CONNECT ERROR\r\n");
							USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
							USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
							USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
			
							arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
				
							IAP.Data_Start_Flag=FALSE;
							IAP.File_download_status=FALSE;
							states.FTPPUTConnectFlag=FALSE;
							
							///Timer_Init(Timer4,ENABLE);
						}
		
					}
					if(strstr((const char*)rxBuffer,(const char*)"@@,A") )// if server ack is received 
					{	
//						USART_SendData_s( DEBUG_COM,"\r\n\r\nServer ACK received\r\n");
						Ack.ServerAckNoBuff[0]=rxBuffer[4];
						Ack.ServerAckNoBuff[1]=rxBuffer[5];
						Ack.ServerAckNoBuff[2]=rxBuffer[6];
						Ack.ServerAckNoBuff[3]=0x00;
						Ack.ServerAckLenBuff[0]=rxBuffer[8];
						Ack.ServerAckLenBuff[1]=rxBuffer[9];	
						Ack.ServerAckLenBuff[2]=0x00;	
						
						
						l_ackpacketno = atoi((const char*)Ack.ServerAckNoBuff);
					  l_ackdelay=atoi((const char*)Ack.ServerAckLenBuff);
 		
						
//						sprintf((char*)G_sprintfBuffer,"%d",l_SendPktCounter );
//						USART_SendData_s( DEBUG_COM,"l_SendPktCounter 1=");
//						USART_SendData_s( DEBUG_COM,G_sprintfBuffer);
//						USART_SendData_s( DEBUG_COM,"\r\n");
//						
//						sprintf((char*)G_sprintfBuffer,"%d",l_ackpacketno );
//						USART_SendData_s( DEBUG_COM,"l_ackpacketno 1=");
//						USART_SendData_s( DEBUG_COM,G_sprintfBuffer);
//						USART_SendData_s( DEBUG_COM,"\r\n");
						
						if(l_SendPktCounter==l_ackpacketno)
						{
							USART_SendData_s( DEBUG_COM,(unsigned char*)"A-O\n\r");
							states.packetAck=TRUE;
							states.AckPacketErrorCounter = 0;
							states.AckGprsErrorCounter = 0;
							Ack.DataHandlerFlag=FALSE;
						}
						else
						{
							USART_SendData_s( DEBUG_COM,(unsigned char*)"A-P\n\r");
						}
					}
				}
			
				else// if the packet is > 160 bytes reset the receive bufffer
				{
					smsRead = FALSE;
					gsmComm.smsHeaderCounter = 0;
					rxBuffer[rxBufferIndex-1]='\0';
					rxBuffer[rxBufferIndex-2]='\0';
					rxBufferIndex=0;						
				}			
			}
			
		}
		
		else //FTP IAP Section
		{
			#ifdef  FLASH_ENABLED
	if(!file_download_complete)
	{		
			IAP.USART_ISR_Count++;
			
			IAP.Array_Track=IAP.Array;
			*IAP.Array++=temp;
			if(IAP.USART_ISR_Count==EEPROM_PAGE_SIZE)
			{
				if(IAP.BufferNo==0)
				{
					IAP.BufferNo=1;
					IAP.Array=IAP.USART_ISR_buffer2;
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)IAP.USART_ISR_buffer1;
					DMA_Init(DMA1_Channel3, &DMA_InitStructure); 
				}
				else 
				{
					IAP.BufferNo=0;
					IAP.Array=IAP.USART_ISR_buffer1;
					DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)IAP.USART_ISR_buffer2;
					DMA_Init(DMA1_Channel3, &DMA_InitStructure);
				}
				//USART_SendData_s(DEBUG_COM,(unsigned char*)"*\r\n");
				sFLASH_DMA_WritePage( IAP.sFLASH_ADD_Counter);
				
				IAP.sFLASH_ADD_Counter+=EEPROM_PAGE_SIZE;
				IAP.USART_ISR_Count=0;
				Delay_viaCounter(DELAY_MICRO_SEC,150);
				
			}
			
			if((temp==':' ) && (*(IAP.Array_Track-4)=='P') && (*(IAP.Array_Track-5)=='T') && (*(IAP.Array_Track-6)=='F') && (*(IAP.Array_Track-7)=='Q')&& (*(IAP.Array_Track-8)=='+'))
			{
					if(IAP.BufferNo==0)
					{
						DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)IAP.USART_ISR_buffer1;
						DMA_Init(DMA1_Channel3, &DMA_InitStructure);
					}		
					else if(IAP.BufferNo==1)
					{
						DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)IAP.USART_ISR_buffer2;
						DMA_Init(DMA1_Channel3, &DMA_InitStructure);
					}						
					sFLASH_DMA_WritePage( IAP.sFLASH_ADD_Counter);
					
					IAP.sFLASH_ADD_Counter+=EEPROM_PAGE_SIZE;
					Delay_viaCounter(DELAY_MICRO_SEC,150);
					
					file_download_complete = TRUE;
					USART_SendData_s(DEBUG_COM,(unsigned char*)"C");
			}
		}
	
			else
			{
				file_size[file_size_index] = temp;
				file_size_index++;
				
				if(file_size[file_size_index-2] == 0x0D && file_size[file_size_index-1] == 0x0A)
				{
					file_size[file_size_index-2] = 0;
					file_size[file_size_index-1] = 0;
					file_download_complete = FALSE;
					file_size_index = 0;
					
					size_of_file = atoi((const char*)file_size);	
					
					IAP.Data_Start_Flag=0;
					IAP.USART_ISR_Count=0;
		
					states.FTPPUTConnectFlag=FALSE;
					config.IAPSetting=FALSE;
					eeprom_write_byte(IAP_SETTING,config.IAPSetting);
					FTP.PUTConnectFlag2=TRUE;
					IAP.File_download_status=TRUE;
					USART_SendData_s(DEBUG_COM,(unsigned char*)"file_End\r\n");
				}
			}
		#endif
		}		
	}
	
	/* If Overrun occures, clear the OVR condition */
	else if(USART_GetITStatus(GSM_USART, USART_IT_ORE) != RESET)
	{
	  // Read one byte from the receive data register 
	  USART_ReceiveData(GSM_USART);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"GSM_USART Overrun:\r\n");
	}
	else
	{
		USART_GetFlagStatus(GSM_USART, USART_IT_ORE);//Read status register followed by read data to clear flag
		USART_ReceiveData(GSM_USART);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"GSM_USART Else\r\n");
	}
}

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)  /* BMC_INT1  */
  {
//			USART_SendData_s( DEBUG_COM,(unsigned char*)"BMA_INT1\r\n");
			ioRead.motionDetect = TRUE;
			ioRead.lastMotionDetected = TRUE;
			ioRead.lastMotionDetDist = TRUE;
			config.sleepModeCounter = 0;
			ioRead.anyIOChangeFlag = TRUE;
			GPS_LOG.geofenceMnCount++;
			ioRead.NextMotionAlertCount++;
			GPS_LOG.MDLatLongUpdateFlag = TRUE;
		
			if(!GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN) && (config.AccelerationDetectTHR != 0 || config.HarshBreakDetectTHR != 0 || config.ImpactDetectTHR != 0))			
			{	
				AccelrationXYZ();
			}

    EXTI_ClearITPendingBit(EXTI_Line0);	// Clear the  EXTI line 0 pending bit
  }
}

/**
  * @brief  This function handles External line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)  /* SOS_PIN  */  
  {
		if(!GPIO_ReadInputDataBit(SOS_PORT,SOS_PIN))
		{	
			//Start_Blink();	
			ioRead.sosActive = TRUE;
			ioRead.sosFlagSMS = TRUE;
			USART_SendData_s( DEBUG_COM,(unsigned char*)"SOS_PIN\r\n");
		}
		
    EXTI_ClearITPendingBit(EXTI_Line4);	// Clear the  EXTI line 4 pending bit 
  }
}

/**
  * @brief  This function handles External lines 5-9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)  /* CAR_IGNITION_ON_PIN  */
  {
		if(!GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))
		{	
			states.StopModeExitFlag=TRUE;
			USART_SendData_s( DEBUG_COM,(unsigned char*)"CAR_IGN\r\n");
		}	
    
    EXTI_ClearITPendingBit(EXTI_Line8);	// Clear the  EXTI line 8 pending bit 
  }
}

/**
  * @brief  This function handles External lines 10-15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)  /* GSM_RI_PIN  */
  {
		//interrupt generated when any call or sms is received in sleep mode
		gsmComm.interruptCallFlag = TRUE;
		USART_SendData_s( DEBUG_COM,(unsigned char*)"GSM_RI INTERRUPT:\r\n");
				
    EXTI_ClearITPendingBit(EXTI_Line13);	// Clear the  EXTI line 8 pending bit 
  }
}

/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
