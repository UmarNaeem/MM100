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

#include "Global_Defines.h"

extern uint32_t size_of_file;

/* Private variables ---------------------------------------------------------*/
float Vref = 3.29; // Reference voltage Vref of ADC
float Low_Batt = 3600; // Reference voltage Vref of ADC
float divider_Add = 4.33;// for 3.3 v input supply

float Int_Bat_divider_Add = 2.82;// for 3.3 v input supply//1.75
float Main_Bat_divider_Add = 9.99;// for 3.3 v input supply


const char GOOGLE_TOP[]  		= "http://maps.google.com/maps?f=q&hl=en&q=";
const char GOOGLE_BOTTOM[]  = "&ie=UTF8&z=16&iwloc=addr&om=1";

unsigned char l_BufferStringCounter[6];	
unsigned char l_BufferTripIdCounter[6];	
unsigned char l_tempCheckSum[4];	
unsigned char l_tempPacketLength[4];	
unsigned char l_simInUse[2];
unsigned char rxBufferLenCm;

static unsigned char lastStateFlag = 0;

unsigned char IOSMSBuffer[50];
unsigned char tempData[10];	

const char isIn[]  = " is Inside, ";
const char isOut[]  = " is Outside, ";
const char igOn[]  = " is turned on.";
const char igOff[]  = " is turned off.";
const char opened[]  = " opened.";
const char closed[]  = " closed.";
const char sosAler[]  = ",SOS Alert!";

uint8_t g_OBDRequestCounter=0; 
//extern const uint8_t g_mainbPktTemplate[MAINB_PKT_LENGTH];
uint32_t l_stringCounter = 0;	
uint32_t l_SendPktCounter = 0;	

extern uint32_t l_ackdelay;

static unsigned char ADCavgCounter = 0;		
static float ADCbatVoltAvg = 0;

#define ALERT_NAME_DISPLAY
/* Private functions ---------------------------------------------------------*/
static void IOMessageGenerator(unsigned char ioType,unsigned char hiLowType);
void DataRecoveryHandler(void);
void DataStringHandler(void);
uint16_t PacketNoExtract(void);

ErrorStatus HSEStartUpStatus;
/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
	
	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
	if(HSEStartUpStatus == SUCCESS)
  {

#ifdef STM32F10X_CL
    /* Enable PLL2 */ 
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {
    }

#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}
/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void SleepModeAlgo(void)
{
	//USART_SendData_s( DEBUG_COM, "SS////////////////////\r\n");
	
  #ifdef GPS_ENABLED
	if(GPS_LOG.TimerFlag==TRUE && GPS_LOG.TimerSelectFlag==TRUE)//if gps is no fix and 20sec update timer is on
	{
		TimeDate_Update();
	}
	#endif
				
	if(states.sleepModeActiveFlag == FALSE)
	{
		config.timeFlag++;	
		TimeComputeController();
	}		
	
	if(ioRead.ignitionOnOffState == TRUE)//If ignition is on
	{
		g_packetSendCounter++;	
		config.sleepModeCounter = 0;
	}		
	else//If ignition is off
	{
		config.sleepModeCounter++;
		g_packetSendCounterIGOff++;
	}			
	
	if(g_packetSendCounter >= config.transmissionTimeIGon && config.transmissionTimeIGon != 0)
	{		
		//USART_SendData_s( DEBUG_COM, "RTC ISR transmissionTimeIGon\r\n");
		states.transmissionFlagIGOn = TRUE;	
		g_packetSendCounter = 0;
			
		states.StopModeExitFlag=TRUE;/////
	}	
		
	if(g_packetSendCounterIGOff >= config.transmissionTimeIGOff && config.transmissionTimeIGOff != 0)
	{		
		//USART_SendData_s( DEBUG_COM, "RTC ISR transmissionFlagIGOff\r\n");
		states.transmissionFlagIGOff = TRUE;	
		g_packetSendCounterIGOff = 0;
	}	
		
	g_HeartbeatSendCounter++;
	if(g_HeartbeatSendCounter >= config.heartBeatTime)
	{
		states.HeartBeatFlag = TRUE;
		g_HeartbeatSendCounter = 0;		
	}	
	g_SmsTrackSendCounter++;
		
	if(g_SmsTrackSendCounter >= config.transmissionTimeSMS && config.transmissionTimeSMS != 0)
	{
		states.smsTrackFlag = TRUE;
		g_SmsTrackSendCounter = 0;

		states.StopModeExitFlag=TRUE;/////
	}	
	
	if(config.sleepModeCounter>=config.sleepModeTimeout)
	{
		//USART_SendData_s( DEBUG_COM, "sleepModeTimeout/////////\r\n");
		config.sleepModeCounter = 0;
		sleepModeEnterFlag = TRUE;
	}
		
	if(states.FTPPUTConnectFlag==TRUE)// Wait 20 mint for IAP download
	{
		IAP.FTPConnectCounter++;
	}
	else
	{
		IAP.FTPConnectCounter=0;
	}
}
/**
  * @brief  sleep module is used to enter the device in sleep mode and then recover if any interrupt is generated
  * @param  None
  * @retval None
  */
void SleepModeController(void)
{
	uint16_t l_count=0;
	// if sleep mode is enabled	sleep mode timer overflow, and tracker is not in data recovery mode						
	if(config.sleepModeSetting[0] == '1'     &&   sleepModeEnterFlag == TRUE   && 
		 eeprom.eepromRecoveryEnable == FALSE  &&   Ack.DataHandlerFlag== FALSE  && 
	   config.IAPSetting==FALSE              &&   states.GPRSStateOk == TRUE   && 
	   ioRead.ignitionOnOffState == FALSE    &&   states.FTPPUTConnectFlag==FALSE ) // if car ignition is on donot sleep 
	{	
		states.sleepModeACKFlag = TRUE;
		dataTransmissionController();// Data Transmission controller						
		
		
		while(Ack.DataHandlerFlag== TRUE && l_count<60)
		{
			Delay_ms(1000);
			l_count++;
		}

		
		if(config.sleepModeSetting[1] == '1')// if gprs off enabled
		{
			Delay_ms(5000);				
			USART_SendData_s( GSM_COM,(unsigned char*)GPRS_DEACT);// sends GPRS deactivate command to modem						
			//USART_SendData_s(Debug_COM,"GPRS DEACTIVE\r\n");
			states.GPRSStateOk =FALSE;
		}		
		if(config.sleepModeSetting[2] == '1')// if GPS module off enabled
		{
			if(GPS_LOG.firstGpsGet == TRUE)	
			{
				GPS_LOG.firstGpsGet = FALSE;
				Init_TimeDate();		
			}
			GPS_LOG.TimerFlag=TRUE;//Start GPS time increment if GPS is no fix after First fix	
			sprintf((char*)GPS_LOG.gpsTimeDateConverted,(const char*)"%02d%02d%02d%04d%02d%02d",TD.hh,TD.mm,TD.ss,TD.year,TD.mo,TD.dd);// generate buffer for date time		
			
			GPS_LOG.TimerSelectFlag=TRUE;//increment 20sec GPS time update
			GPS_LOG.gpsFix[0]='V';
			
			
			GPS_RESET_ON();
			//GPS_PWR_DISABLE();
			//USART_SendData_s(Debug_COM,"GPS OFF\r\n");
			GPS_LOG.seconds = 0;
			GPS_LOG.minutes = 0;
			GPS_LOG.hour = 0;
			GPS_LOG.year = 0;
			GPS_LOG.month = 0;
			GPS_LOG.day = 0;	
			GPS_LOG.firstGpsGet = FALSE;// RESET GPS Flags
			GPS_LOG.gpsFixClear = TRUE;//Gps no fix event send only one time
			GPS_LOG.gpsFixNoFix = 2;//send gps no fix event after exiting sleep mode
		}	
		
		ADC_Channel_Init(DISABLE);/////
		ADC_StopConversion();////
		
		USART_SendData_s( DEBUG_COM, (unsigned char*)"Sleep mode start\r\n");
		ioRead.anyIOChangeFlag = FALSE;// reset the flags only used for sleep		
		USART_SendData_s( DEBUG_COM, (unsigned char*)"Sleep mode start\r\n");
		SleepMode_Interrupt(ENABLE);//Turn on RI+Ignition interrupt for sleep mode
		gsmComm.interruptCallFlag = FALSE;// reset the flags only used for sleep				
		states.sleepModeActiveFlag = TRUE; // high till sleep mode is active
		
		states.StopModeExitFlag=FALSE;// reset the flags only used for sleep		
		
		if(config.sleepModeSetting[3] == '1')// if Accelerometer motion wakeup Disabled
		{
			USART_SendData_s( DEBUG_COM, (unsigned char*)"Motion wakeup off\r\n");
			// Configure BMC_INT1 EXTI line 
			EXTI_InitStructure.EXTI_Line = BMA_INT_EXTI_LINE; 
			EXTI_InitStructure.EXTI_LineCmd = DISABLE;
			EXTI_Init(&EXTI_InitStructure);
		}
		while(1)
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*) "enter stop mode\r\n");
	
			
			IWDG_ReloadCounter(); 
			/* Enter Stop Mode */
			PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
			
			/* Configures system clock after wake-up from STOP: enable HSE, PLL and select 
       PLL as system clock source (HSE and PLL are disabled in STOP mode) */
			SYSCLKConfig_STOP();
			IWDG_ReloadCounter();
			
			USART_SendData_s( DEBUG_COM,(unsigned char*) "exit stop mode\r\n");
			
			DIO_StateMachine();
			
			//	if any event occurs or GSM goes off exit the sleep mode
			if(ioRead.anyIOChangeFlag == TRUE || ioRead.sosActive == TRUE ||  states.StopModeExitFlag==TRUE ||
				gsmComm.interruptCallFlag == TRUE || states.transmissionFlagIGOff == TRUE || 
				states.smsTrackFlag == TRUE || !GPIO_ReadInputDataBit(GSM_STATUS_PORT,GSM_STATUS_PIN))
			{	
				USART_SendData_s( DEBUG_COM,(unsigned char*) "exit stop mode 2\r\n");
				ioRead.anyIOChangeFlag = FALSE;
				gsmComm.interruptCallFlag = FALSE;
				states.StopModeExitFlag=FALSE;
				break;
			}
		}

		ADC_Channel_Init(ENABLE);////
		ADC_StartConversion();////
		Delay_ms(2);////
		
		states.sleepModeActiveFlag = FALSE; // sleep mode exit flag used in timer 1 ISR	
			
		states.sleepModeACKFlag = 2;// exit sleep mode								
		GPS_LOG.firstGpsGet = FALSE;// RESET GPS Flags				
		USART_SendData_s( DEBUG_COM, (unsigned char*)"Sleep mode End////////////////\r\n");
		SleepMode_Interrupt(DISABLE);	
		sleepModeEnterFlag = FALSE;	
		config.sleepModeCounter = 0;	
		GPS_RESET_OFF();		
		// Configure BMC_INT1 EXTI line 
		EXTI_InitStructure.EXTI_Line = BMA_INT_EXTI_LINE;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		GPS_LOG.TimerSelectFlag=FALSE;//increment 5sec GPS time update
	}	
}
/**
  * @brief  Capture Ignition State and increments the counter on every new ignition on, also generates the trip time 
					  and duration when ignition is turned off
  * @param  None
  * @retval None
  */
void Ignition_module(void)
{
	if(!GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN) && lastStateFlag == FALSE)// if ignition is ON
	{		
		states.ignitionOffTrueCounter=0;
		
		states.ignitionOnTrueCounter++;
		if(states.ignitionOnTrueCounter>=config.ignitionHysPollMinTrueAlert)
		{
			states.ignitionOnTrueCounter=0;
			
			USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nIgnition ON:\r\n");	////
			
			ioRead.ignitionEvenFlag = TRUE;
			
			states.IgnitionOnFlag = TRUE;
			GPS_LOG.IgnitionOffSaveFlag = FALSE;
			
			ioRead.ignitionOnOffState = TRUE;
			ioRead.initDistance = config.distanceTotal;					
			ioRead.initTime = config.totalTimeTravelled;		
			ioRead.totalTripTime = 0;				
			
			ioRead.tripId++;		
			if(ioRead.tripId>=999)
			{
				ioRead.tripId = 0;		
			}			
			eeprom_write_word(TRIP_ID_LSB,ioRead.tripId);										
			
			//ioRead.ignitionSMSFlag = config.tripReportSMS;
					
			lastStateFlag = TRUE;

			//////////////////////////
//			states.ReAuthenticationFlag=FALSE;
//			states.ReAuthenticationCounter=0;
		}
			
	}		
	else if(GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN) && lastStateFlag == TRUE)// if ignition is OFF
	{		
		states.ignitionOnTrueCounter=0;
		
		states.ignitionOffTrueCounter++;
		if(states.ignitionOffTrueCounter>=config.ignitionHysPollMinTrueAlert)
		{
			states.ignitionOffTrueCounter=0;
		
			USART_SendData_s( DEBUG_COM,(unsigned char *)"\r\nIgnition OFF //////////\r\n");	////
			
			ioRead.ignitionOnOffState = FALSE;	

			states.IgnitionOnFlag = FALSE;
			
			ioRead.FinalTime = config.totalTimeTravelled;	//IgnitionHys on & speed>3,Total travelled time since tracker install 	
			ioRead.totalTravelTime = ioRead.FinalTime - ioRead.initTime;//Total Trip time(speed>3) from ignition on to off state				
			ioRead.tripDistance = config.distanceTotal - ioRead.initDistance;//Total Trip distance(speed>3) from ignition on to off state		
	
			/*USART_SendData_s(DEBUG_COM,"\r\nconfig.totalTimeTravelled=");
			sprintf(( char*)G_sprintfBuffer,"%d",config.totalTimeTravelled);	
			USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
			USART_SendData_s(DEBUG_COM,"\r\n");
			
			USART_SendData_s(DEBUG_COM,"\r\nioRead.totalTravelTime=");
			sprintf(( char*)G_sprintfBuffer,"%d",ioRead.totalTravelTime);	
			USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
			USART_SendData_s(DEBUG_COM,"\r\n");
			
			USART_SendData_s(DEBUG_COM,"\r\nioRead.tripDistance=");
			sprintf(( char*)G_sprintfBuffer,"%d",ioRead.tripDistance);	
			USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
			USART_SendData_s(DEBUG_COM,"\r\n");*/
			
			if(ioRead.tripDistance>20)
			{
				
				sprintf((char*)ioRead.tripDistanceAasci,"%lu",ioRead.tripDistance);	
				sprintf((char*)ioRead.travelTimeAasci,"%lu",ioRead.totalTravelTime);				
				sprintf((char*)ioRead.totalTripTimeAasci,"%lu",ioRead.totalTripTime);		
				
				ioRead.tripCompEvent = TRUE;		
			}			
			else
			{
				arrayInit2Zero((unsigned char*)ioRead.tripDistanceAasci,tripDistanceAasciSize);
				arrayInit2Zero((unsigned char*)ioRead.travelTimeAasci,travelTimeAasciSize);
				arrayInit2Zero((unsigned char*)ioRead.totalTripTimeAasci,totalTripTimeAasciSize);		
			}
			
			
			ioRead.ignitionEvenFlag = 2;							
			ioRead.totalTripTime = 0;						
			lastStateFlag = FALSE;	
			
			
			////////////////////////////////
//			if(states.AuthenticationOkFlag==TRUE)
//			{
//				USART_SendData_s( DEBUG_COM,(unsigned char *)"\r\nReAuthenticationFlag TRUE///:\r\n");	////
//				states.ReAuthenticationFlag=TRUE;
//				states.ReAuthenticationCounter=0;
//			}
			
		}	
	}
	

}

/**
  * @brief  computes time
  * @param  None
  * @retval None
  */
void TimeComputeController(void)
{
	if(config.timeFlag > 0)// 
	{			
		if(GPS_LOG.speedFloat>3 && !GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN)) // speed >3 kmph and ignition is on then increment time this is real travel duration time
		{	
			config.totalTimeTravelled = config.totalTimeTravelled + (config.timeFlag*20);// 20 seconds increment						
			eeprom_write_dword(travelTimeStart,config.totalTimeTravelled);		
			ioRead.totalIdleTime = 0;
			ioRead.RepeatIdleTimeAlert=FALSE;
		}
		else if(GPS_LOG.speedFloat<3 && !GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN)) // speed is < 3kmph and igntition is on
		{		
			ioRead.totalIdleTime = ioRead.totalIdleTime+(config.timeFlag*20); // idle timer	20 seconds increment
			 			
			if(ioRead.totalIdleTime>=config.idleTimeSetting && config.idleTimeSetting != 0)
			{
				if(ioRead.RepeatIdleTimeAlert==FALSE)
				{
					ioRead.idleTimeAlert = TRUE;//Excess Idling alert
					ioRead.RepeatIdleTimeAlert=TRUE;
				}
				else if(ioRead.RepeatIdleTimeAlert==TRUE)
				{
					ioRead.idleTimeAlert = 2;//Repeat Excess Idling alert
				}
				ioRead.totalIdleTime = 0;
			}
			
			else if(config.idleTimeSetting == 0)// if setting is set to zero dont generate alerts
			{
				ioRead.idleTimeAlert = FALSE;
				ioRead.totalIdleTime = 0;				
			}
		}
		else
		{
			ioRead.totalIdleTime = 0;// idle time reset
		}
		
		ioRead.totalTripTime = ioRead.totalTripTime+(config.timeFlag*20);////Total Trip time(speed>3+Idel time) from ignition on to off state						
		sprintf((char*)config.totalTimeTravelledAASCI, "%lu",config.totalTimeTravelled);	
		config.timeFlag = 0;	
		
		/*USART_SendData_s(DEBUG_COM,"\r\nioRead.totalTripTime=");
		sprintf(( char*)G_sprintfBuffer,"%d",ioRead.totalTripTime);	
		USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
		USART_SendData_s(DEBUG_COM,"\r\n");*/
	}	
}
/**
  * @brief  Checks whether particualar alert transmission is enabled or disabled
  * @param  None
  * @retval None
  */
uint8_t IsAlertOn(uint8_t AlertHexvalue)
{
	uint8_t tempHex=0;
	uint8_t BitPosition=0;
	uint8_t AlertState=0;
	
	tempHex=ascii_to_hex(  config.AlertsOnOffEncode[(((AlertHexvalue-1)/8)*2)], config.AlertsOnOffEncode[(((AlertHexvalue-1)/8)*2)+1]);
	
	/*sprintf((char*)tempData,"%X",tempHex);
	USART_SendData_s( DEBUG_COM,"tempHex=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	BitPosition=AlertHexvalue-(((AlertHexvalue-1)/8)*8)-1;
	
	/*sprintf((char*)tempData,"%d",BitPosition);
	USART_SendData_s( DEBUG_COM,"BitPosition=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	AlertState=(tempHex>>(7-BitPosition)) & 0x01;
	
	/*sprintf((char*)tempData,"%d",AlertState);
	USART_SendData_s( DEBUG_COM,"AlertState=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	return AlertState;
}
/**
  * @brief  blinking routine of leds
  * @param  None
  * @retval None
  */
void Start_Blink(void)
{
	unsigned char blink=0;

	for ( blink=0;blink<6;blink++)	
	{		
		if(states.IgnitionOnFlag==TRUE)
		{
			STATE_MACHINE_LED_ON();
		}
//		Delay_ms(60);
		STATE_MACHINE_LED_OFF();
//		Delay_ms(100);
		if(states.IgnitionOnFlag==TRUE)
		{
			STATE_MACHINE_LED_ON();
		}
//		Delay_ms(60);
		STATE_MACHINE_LED_OFF();
//		Delay_ms(100);		
	}	
	
}
/**
  * @brief  This module controls the power on off of the tracker
  * @param  None
  * @retval None
  */
void Tracker_Power(void)
{
//	if(!GPIO_ReadInputDataBit(TRACKER_PWR_BTTN_PORT,TRACKER_PWR_BTTN_PIN) && ioRead.trackerOnFlag == FALSE)// if tracker was in off state last time and button is pressed
//	{
//		USART_SendData_s( DEBUG_COM,"\r\Tracker_Power ON////////////////:\r\n");	
//		ioRead.trackerOnFlag = TRUE;
//		ioRead.trackerFirstOn = TRUE;	
//		Timer_Init(Timer2,ENABLE);// starts the timer as tracker turned on		
//		Start_Blink();			
//		GPS_RESET_OFF();	
//	}
//	else if(!GPIO_ReadInputDataBit(TRACKER_PWR_BTTN_PORT,TRACKER_PWR_BTTN_PIN) && ioRead.trackerOnFlag == TRUE)// if tracker was in on state last time and button is pressed
//	{
//		USART_SendData_s( DEBUG_COM,"\r\Tracker_Power OFF////////////////:\r\n");	
//		ioRead.trackerOnFlag = FALSE;
//		ioRead.powerStateChange = TRUE;						
//		STATE_MACHINE_LED_OFF();
//		Timer_Init(Timer2,DISABLE);// stop the main state time for tracking etc
//		Start_Blink();	
//		GPS_RESET_ON();		
//		STATE_MACHINE_LED_OFF();
//	}
	
}

/**
  * @brief  receives the Input type Ignition , Door etc generates sms depending on the profile of the Input
  * @param  None
  * @retval None
  */
void IOMessageGenerator(unsigned char ioType,unsigned char hiLowType)
{
	switch (ioType)
	{
		case IGNITION:
		{
			if(config.ignitionConfigure[0] == '1' || config.ignitionConfigure[2] == '1' || config.ignitionConfigure[4] == '1')
			{				
				IOSMSBuffer[0] = 0x00;				
				strcat((char*)IOSMSBuffer,(const char*)", ");// con the reply to send							
				strcat((char*)IOSMSBuffer,(const char*)header.Ignition_Header);// con the reply to send											
				if(hiLowType == TRUE)
				{
					strcat((char*)IOSMSBuffer,(const char*)igOn);// con the reply to send								
					if(config.ignitionConfigure[0] == '1' && config.ignitionConfigure[1] == '1' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.ignitionConfigure[2] == '1' && config.ignitionConfigure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.ignitionConfigure[4] == '1' && config.ignitionConfigure[5] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				}
				else
				{
					strcat((char*)IOSMSBuffer,(const char*)igOff);// con the reply to send								
					if(config.ignitionConfigure[0] == '1' && config.ignitionConfigure[1] == '0' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.ignitionConfigure[2] == '1' && config.ignitionConfigure[3] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.ignitionConfigure[4] == '1' && config.ignitionConfigure[5] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				
				}					
				
			}				
			break;
		}	
		case DI_1:
		{
			if(config.IO1Configure[0] == '1' || config.IO1Configure[2] == '1' || config.IO1Configure[4] == '1')
			{				
				IOSMSBuffer[0] = 0x00;						
				strcat((char*)IOSMSBuffer,(const char*)", ");// con the reply to send							
				strcat((char*)IOSMSBuffer,(const char*)header.DI1Header);// con the reply to send							
				if(hiLowType == TRUE)
				{
					strcat((char*)IOSMSBuffer,(const char*)opened);// con the reply to send								
					if(config.IO1Configure[0] == '1' && config.IO1Configure[1] == '1' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.IO1Configure[2] == '1' && config.IO1Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.IO1Configure[4] == '1' && config.IO1Configure[5] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				}
				else
				{
					strcat((char*)IOSMSBuffer,(const char*)closed);// con the reply to send								
					if(config.IO1Configure[0] == '1' && config.IO1Configure[1] == '0' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.IO1Configure[2] == '1' && config.IO1Configure[3] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.IO1Configure[4] == '1' && config.IO1Configure[5] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				
				}					
				
			}				
			break;
		}	
		case DI_2:
		{
			if(config.IO2Configure[0] == '1' || config.IO2Configure[2] == '1' || config.IO2Configure[4] == '1')
			{				
				IOSMSBuffer[0] = 0x00;
				strcat((char*)IOSMSBuffer,(const char*)", ");// con the reply to send							
				strcat((char*)IOSMSBuffer,(const char*)header.DI2Header);// con the reply to send										
				if(hiLowType == TRUE)
				{					
					strcat((char*)IOSMSBuffer,(const char*)opened);// con the reply to send								
					if(config.IO2Configure[0] == '1' && config.IO2Configure[1] == '1' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.IO2Configure[2] == '1' && config.IO2Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.IO2Configure[4] == '1' && config.IO2Configure[5] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				}
				else
				{
					strcat((char*)IOSMSBuffer,(const char*)closed);// con the reply to send								
					if(config.IO2Configure[0] == '1' && config.IO2Configure[1] == '0' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.IO2Configure[2] == '1' && config.IO2Configure[3] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.IO2Configure[4] == '1' && config.IO2Configure[5] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				
				}					
				
			}				
			break;
		}	
		
		case DI_3:
		{
			if(config.IO3Configure[0] == '1' || config.IO3Configure[2] == '1' || config.IO3Configure[4] == '1')
			{				
				IOSMSBuffer[0] = 0x00;
				strcat((char*)IOSMSBuffer,(const char*)", ");// con the reply to send							
				strcat((char*)IOSMSBuffer,(const char*)header.DI3Header);// con the reply to send										
				if(hiLowType == TRUE)
				{					
					strcat((char*)IOSMSBuffer,(const char*)opened);// con the reply to send								
					if(config.IO3Configure[0] == '1' && config.IO3Configure[1] == '1' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.IO3Configure[2] == '1' && config.IO3Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.IO3Configure[4] == '1' && config.IO3Configure[5] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				}
				else
				{
					strcat((char*)IOSMSBuffer,(const char*)closed);// con the reply to send								
					if(config.IO3Configure[0] == '1' && config.IO3Configure[1] == '0' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.IO3Configure[2] == '1' && config.IO3Configure[3] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.IO3Configure[4] == '1' && config.IO3Configure[5] == '0')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				
				}					
				
			}				
			break;
		}
		
		case SOS:
		{
			if(config.sosConfigure[0] == '1' || config.sosConfigure[2] == '1' || config.sosConfigure[4] == '1')
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE)
				{				
					strcat((char*)IOSMSBuffer,(const char*)sosAler);// con the reply to send								
					if(config.sosConfigure[0] == '1' && config.sosConfigure[1] == '1' )
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,1);						
					}
					if(config.sosConfigure[2] == '1' && config.sosConfigure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,1);		
					}
					if(config.sosConfigure[4] == '1' && config.sosConfigure[5] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,1);		
					}				
				}			
			}				
			break;
		}
		case GEO1:
		{
			if(config.geo1Configure[1] == '1' || config.geo1Configure[2] == '1' || config.geo1Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo1Configure[0] == '1' || config.geo1Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo1AreaName);// con the reply to send								
					if(config.geo1Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo1Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo1Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo1Configure[0] == '2' || config.geo1Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo1AreaName);// con the reply to send												
					if(config.geo1Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo1Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo1Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}		
		case GEO2:
		{
			if(config.geo2Configure[1] == '1' || config.geo2Configure[2] == '1' || config.geo2Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo2Configure[0] == '1' || config.geo2Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo2AreaName);// con the reply to send								
					if(config.geo2Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo2Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo2Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo2Configure[0] == '2' || config.geo2Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo2AreaName);// con the reply to send												
					if(config.geo2Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo2Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo2Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO3:
		{
			if(config.geo3Configure[1] == '1' || config.geo3Configure[2] == '1' || config.geo3Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo3Configure[0] == '1' || config.geo3Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo3AreaName);// con the reply to send								
					if(config.geo3Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo3Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo3Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo3Configure[0] == '2' || config.geo3Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo3AreaName);// con the reply to send												
					if(config.geo3Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo3Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo3Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO4:
		{
			if(config.geo4Configure[1] == '1' || config.geo4Configure[2] == '1' || config.geo4Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo4Configure[0] == '1' || config.geo4Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo4AreaName);// con the reply to send								
					if(config.geo4Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo4Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo4Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo4Configure[0] == '2' || config.geo4Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo4AreaName);// con the reply to send												
					if(config.geo4Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo4Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo4Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO5:
		{
			if(config.geo5Configure[1] == '1' || config.geo5Configure[2] == '1' || config.geo5Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo5Configure[0] == '1' || config.geo5Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo5AreaName);// con the reply to send								
					if(config.geo5Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo5Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo5Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo5Configure[0] == '2' || config.geo5Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo5AreaName);// con the reply to send												
					if(config.geo5Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo5Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo5Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO6:
		{
			if(config.geo6Configure[1] == '1' || config.geo6Configure[2] == '1' || config.geo6Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo6Configure[0] == '1' || config.geo6Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo6AreaName);// con the reply to send								
					if(config.geo6Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo6Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo6Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo6Configure[0] == '2' || config.geo6Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo6AreaName);// con the reply to send												
					if(config.geo6Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo6Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo6Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		
		case GEO7:
		{
			if(config.geo7Configure[1] == '1' || config.geo7Configure[2] == '1' || config.geo7Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo7Configure[0] == '1' || config.geo7Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo7AreaName);// con the reply to send								
					if(config.geo7Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo7Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo7Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
		  			}
				}
				if(hiLowType == FALSE && (config.geo7Configure[0] == '2' || config.geo7Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo7AreaName);// con the reply to send												
					if(config.geo7Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo7Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo7Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO8:
		{
			if(config.geo8Configure[1] == '1' || config.geo8Configure[2] == '1' || config.geo8Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo8Configure[0] == '1' || config.geo8Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo8AreaName);// con the reply to send								
					if(config.geo8Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo8Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo8Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo8Configure[0] == '2' || config.geo8Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo8AreaName);// con the reply to send												
					if(config.geo8Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo8Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo8Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO9:
		{
			if(config.geo9Configure[1] == '1' || config.geo9Configure[2] == '1' || config.geo9Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo9Configure[0] == '1' || config.geo9Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo9AreaName);// con the reply to send								
					if(config.geo9Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo9Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo9Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo9Configure[0] == '2' || config.geo9Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo9AreaName);// con the reply to send												
					if(config.geo9Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo9Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo9Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}
		case GEO10:
		{
			if(config.geo10Configure[1] == '1' || config.geo10Configure[2] == '1' || config.geo10Configure[3] == '1')// if any number config is set
			{				
				IOSMSBuffer[0] = 0x00;								
				if(hiLowType == TRUE && (config.geo10Configure[0] == '1' || config.geo10Configure[0] == '3'))// if car has entered the Geo Region region and sms is on
				{
					strcat((char*)IOSMSBuffer,(const char*)isIn);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo10AreaName);// con the reply to send								
					if(config.geo10Configure[1] == '1')// send to number 1
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo10Configure[2] == '1')//send to number 2
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);							
					}
					if(config.geo10Configure[3] == '1')//send to number 3
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);							
					}				
				}
				if(hiLowType == FALSE && (config.geo10Configure[0] == '2' || config.geo10Configure[0] == '3'))// if car has left region and if sms in configured on region left
				{
					strcat((char*)IOSMSBuffer,(const char*)isOut);// con the reply to send								
					strcat((char*)IOSMSBuffer,(const char*)config.geo10AreaName);// con the reply to send												
					if(config.geo10Configure[1] == '1')
					{
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo1,2);												
					}
					if(config.geo10Configure[2] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo2,2);								
					}
					if(config.geo10Configure[3] == '1')
					{						
						SMS_GeneratorAlerts(IOSMSBuffer,config.ServerSMSCellNo3,2);								
					}				
				
				}							
			}				
			break;
		}		
	}
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void IgnitionHys(void)
{	
	//----------------------------------------------------------------------// 	iGNITION Monitoring 
	if(!GPIO_ReadInputDataBit(CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))// ignition on
	{
		states.ignition2OffTrueCounter=0;
		states.ignition2OnTrueCounter++;
		if(states.ignition2OnTrueCounter>=config.ignitionHysPollMinTrueAlert)
		{
			//USART_SendData_s( DEBUG_COM,"Ig 1 on\r\n");
			states.ignition2OnTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 | 0x01;		//SET BIT
			if(ioRead.ignitionStateClear == TRUE)
			{
				//USART_SendData_s( DEBUG_COM,"Ig 2 on\r\n");
				IOMessageGenerator(IGNITION,TRUE);	
				ioRead.ignitionStateClear = FALSE;
			}		
		}			
	}	
	else// ignition off
	{
		states.ignition2OnTrueCounter=0;
		states.ignition2OffTrueCounter++;
		if(states.ignition2OffTrueCounter>=config.ignitionHysPollMinTrueAlert)
		{
			//USART_SendData_s( DEBUG_COM,"Ig off 1\r\n");
			states.ignition2OffTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 & 0xFE;		//reset bit
			if(ioRead.ignitionStateClear == FALSE)
			{
				//USART_SendData_s( DEBUG_COM,"Ig off 2\r\n");
				IOMessageGenerator(IGNITION,FALSE);		
				ioRead.ignitionStateClear = TRUE;
			}	
		}
	}	
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void MainPowerHys(void)
{	
	//----------------------------------------------------------------------// 	Main Input Power Monitoring 
	if(GPIO_ReadInputDataBit(MAIN_POWER_DETECT_PORT,MAIN_POWER_DETECT_PIN))
	{
		states.mainPowerOffTrueCounter=0;
		states.mainPowerOnTrueCounter++;
		if(states.mainPowerOnTrueCounter>=config.MainPwrHysPollMinTrueAlert)
		{
			states.mainPowerOnTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 | 0x80;//set bit
			if(ioRead.mainPowerFlagStateClear == TRUE)
			{			
				USART_SendData_s( DEBUG_COM,(unsigned char*)"Main Power ON\r\n");
				ioRead.mainPowerFlag = TRUE;//high
				ioRead.mainPowerFlagStateClear = FALSE;	
			}		
		}			
	}	
	else
	{
		//USART_SendData_s( DEBUG_COM,"MP off\r\n");
		states.mainPowerOnTrueCounter=0;
		states.mainPowerOffTrueCounter++;
		if(states.mainPowerOffTrueCounter>=config.MainPwrHysPollMinTrueAlert)
		{
			states.mainPowerOffTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 & 0x7F; //reset bit
			if(ioRead.mainPowerFlagStateClear == FALSE)
			{			
				ioRead.mainPowerFlag = 2;//high		
				ioRead.mainPowerFlagStateClear = TRUE;
			}	
		}
	}	
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void SOSHys(void)
{	
	//----------------------------------------------------------------------// 	SOS SMS	
	if(ioRead.sosFlagSMS == TRUE)
	{		
		IOMessageGenerator(SOS,TRUE);	
		ioRead.sosFlagSMS = FALSE;	
	}
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void DIO1Hys(void)
{	
	//----------------------------------------------------------------------//	Digital Input 1
	if(!GPIO_ReadInputDataBit(D1_OPTO_PORT,D1_OPTO_PIN))//noramlly pin high(Input off,ground triggered)
	{
		states.DIO1OffTrueCounter=0;
		states.DIO1OnTrueCounter++;
		if(states.DIO1OnTrueCounter>=config.DIO1HysPollMinTrueAlert)
		{
			//USART_SendData_s( DEBUG_COM,"DIO1 1 on\r\n");
			states.DIO1OnTrueCounter=0;
			
			ioRead.inputIoByte1=ioRead.inputIoByte1 | 0x02;	//set bit
			USART_SendData_s( DEBUG_COM,(unsigned char*)"DIO 1 on\r\n");
			if(ioRead.DIO1StateClear == TRUE)
			{
				USART_SendData_s( DEBUG_COM,(unsigned char*)"DIO 1 on\r\n");
				IOMessageGenerator(DI_1,TRUE);						
				ioRead.D1eventFlag = TRUE;//high
				ioRead.DIO1StateClear = FALSE;
			}		
			DIO_StateMachine();
		}
	}	
	else
	{
		states.DIO1OnTrueCounter=0;
		states.DIO1OffTrueCounter++;
		if(states.DIO1OffTrueCounter>=config.DIO1HysPollMinTrueAlert)
		{
			//USART_SendData_s( DEBUG_COM,"DIO1 1 off\r\n");
			states.DIO1OffTrueCounter=0;
			
			ioRead.inputIoByte1=ioRead.inputIoByte1 & 0xFD;		//reset bit
			
			if(ioRead.DIO1StateClear == FALSE)
			{
			  //USART_SendData_s( DEBUG_COM,"DIO1 2 off\r\n");
				IOMessageGenerator(DI_1,FALSE);						
				ioRead.D1eventFlag = 2;// low
				ioRead.DIO1StateClear = TRUE;
			}	
			DIO_StateMachine();
		}
	}
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void DIO2Hys(void)
{	
	//----------------------------------------------------------------------// Digital Input 2
	if(!GPIO_ReadInputDataBit(D2_OPTO_PORT,D2_OPTO_PIN))//noramlly pin high(Input off,ground triggered)
	{
		states.DIO2OffTrueCounter=0;
		states.DIO2OnTrueCounter++;
		if(states.DIO2OnTrueCounter>=config.DIO2HysPollMinTrueAlert)
		{
			states.DIO2OnTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 | 0x04;//set bit
			USART_SendData_s( DEBUG_COM,(unsigned char*)"DIO 2 on\r\n");
			if(ioRead.DIO2StateClear == TRUE)
			{
				USART_SendData_s( DEBUG_COM,(unsigned char*)"DIO 2 on\r\n");
				IOMessageGenerator(DI_2,TRUE);						
				ioRead.D2eventFlag = TRUE;//high
				ioRead.DIO2StateClear = FALSE;	
			}	
			DIO_StateMachine();
		}
	}	
	else
	{
		states.DIO2OnTrueCounter=0;
		states.DIO2OffTrueCounter++;
		if(states.DIO2OffTrueCounter>=config.DIO2HysPollMinTrueAlert)
		{
			states.DIO2OffTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 & 0xFB;	//reset bit
			if(ioRead.DIO2StateClear == FALSE)
			{
				//USART_SendData_s( DEBUG_COM,"DIO2 off\r\n");
				IOMessageGenerator(DI_2,FALSE);						
				ioRead.D2eventFlag = 2;//high
				ioRead.DIO2StateClear = TRUE;
			}
			DIO_StateMachine();
		}
	}
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void DIO3Hys(void)
{	
	//----------------------------------------------------------------------// 	Digital Input 3
	if(!GPIO_ReadInputDataBit(D3_OPTO_PORT,D3_OPTO_PIN))//noramlly pin high(Input off,ground triggered)
	{
		states.DIO3OffTrueCounter=0;
		states.DIO3OnTrueCounter++;
		if(states.DIO3OnTrueCounter>=config.DIO3HysPollMinTrueAlert)
		{
			states.DIO3OnTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 | 0x08; //set bit
			USART_SendData_s( DEBUG_COM,(unsigned char*)"DIO 3 on\r\n");
			if(ioRead.DIO3StateClear == TRUE)
			{
				USART_SendData_s( DEBUG_COM,(unsigned char*)"DIO 3 on\r\n");
				IOMessageGenerator(DI_3,TRUE);						
				ioRead.D3eventFlag = TRUE;//high
				ioRead.DIO3StateClear = FALSE;
			}
			DIO_StateMachine();
		}
	}	
	else
	{
		states.DIO3OnTrueCounter=0;
		states.DIO3OffTrueCounter++;
		if(states.DIO3OffTrueCounter>=config.DIO3HysPollMinTrueAlert)
		{
			states.DIO3OffTrueCounter=0;
			ioRead.inputIoByte1=ioRead.inputIoByte1 & 0xF7; //reset bit
			if(ioRead.DIO3StateClear == FALSE)
			{
				IOMessageGenerator(DI_3,FALSE);	
				ioRead.D3eventFlag = 2;//high	
				ioRead.DIO3StateClear = TRUE;
			}	
			DIO_StateMachine();
			
		}
	}
}


/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void AIO1Hys(void)
{	
	int tempDiff = 0;
	//----------------------------------------------------------------------// Analog input 1
					
	ioRead.analog3 = (ioRead.ADCConvertedValue[IO1_ADC]*Vref*1000*divider_Add)/0xFFF;		
			
	ioRead.analog3Avg = ioRead.analog3Avg+ioRead.analog3; // average filter
	ioRead.analog3AvgCount++;
				
	if(ioRead.analog3AvgCount >= config.AIO1HysAvgSamples)
	{		
		ioRead.analog3AvgCount = 0;					
		ioRead.analog3Avg = ioRead.analog3Avg/config.AIO1HysAvgSamples;					
		tempDiff = ioRead.analog3AvgSend-ioRead.analog3Avg;										
		tempDiff = abs(tempDiff);
															
		ioRead.analog3AvgSend = ioRead.analog3Avg;
										
		if(ioRead.analog3Avg > config.AIO1HysMaxValue && ioRead.A3StateClear == FALSE && config.AIO1HysMaxValue != 0)
		{
			ioRead.A3Event = TRUE;	// Max Threshold crossing event code
			ioRead.A3StateClear = TRUE;			
		}
		else if(ioRead.analog3Avg<config.AIO1HysMinValue && ioRead.A3StateClear == FALSE && config.AIO1HysMinValue != 0)
		{
			ioRead.A3Event = 2;		// Min Threshold crossing event code
			ioRead.A3StateClear = TRUE;
		}					
		else if(tempDiff>config.AIO1HysDifference && ioRead.A3StateClear == FALSE && config.AIO1HysDifference != 0)	
		{
			ioRead.A3Event = 3;// Difference Threshold crossing event code
			ioRead.A3StateClear = TRUE;
		}
		else if(ioRead.analog3Avg>config.AIO1HysMinValue && ioRead.analog3Avg<config.AIO1HysMaxValue  && ioRead.A3StateClear==TRUE )
		{
			ioRead.A3Event = 4;
			ioRead.A3StateClear = FALSE;
		}
		
		ioRead.analog3Avg = 0;
		
		AIO_StateMachine();
					
	}		
	/*sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog3AvgSend);	
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");	*/

	/*sprintf((char*)tempData,"%d",ioRead.ADCConvertedValue[IO1_ADC]);
	USART_SendData_s( DEBUG_COM,"IO1_ADC ADC Value=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	sprintf((char*)tempData,"%.3f",ioRead.analog3);
	USART_SendData_s( DEBUG_COM,"IO1_ADC Voltage=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	sprintf((char*)tempData,"%.3f",ioRead.analog3AvgSend );
	USART_SendData_s( DEBUG_COM,"analog3AvgSend 1 Voltage=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
}
/**
  * @brief  eads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void AIO2Hys(void)
{	
	int tempDiff = 0;
	//----------------------------------------------------------------------// Analog input 2
					
	ioRead.analog4 = (ioRead.ADCConvertedValue[IO2_ADC]*Vref*1000*divider_Add)/0xFFF;		
			
	ioRead.analog4Avg = ioRead.analog4Avg+ioRead.analog4; // average filter
	ioRead.analog4AvgCount++;
				
	if(ioRead.analog4AvgCount >= config.AIO2HysAvgSamples)
	{		
		ioRead.analog4AvgCount = 0;					
		ioRead.analog4Avg = ioRead.analog4Avg/config.AIO2HysAvgSamples;					
		tempDiff = ioRead.analog4AvgSend-ioRead.analog4Avg;										
		tempDiff = abs(tempDiff);
															
		ioRead.analog4AvgSend = ioRead.analog4Avg;
										
		if(ioRead.analog4Avg > config.AIO2HysMaxValue && ioRead.A4StateClear == FALSE && config.AIO2HysMaxValue != 0)
		{
			ioRead.A4Event = TRUE;	// Max Threshold crossing event code
			ioRead.A4StateClear = TRUE;			
		}
		else if(ioRead.analog4Avg<config.AIO2HysMinValue && ioRead.A4StateClear == FALSE && config.AIO2HysMinValue != 0)
		{
			ioRead.A4Event = 2;		// Min Threshold crossing event code
			ioRead.A4StateClear = TRUE;
		}					
		else if(tempDiff>config.AIO2HysDifference && ioRead.A4StateClear == FALSE  && config.AIO2HysDifference != 0)	
		{
			ioRead.A4Event = 3;// Difference Threshold crossing event code
			ioRead.A4StateClear = TRUE;
		}
		else if(ioRead.analog4Avg>config.AIO2HysMinValue && ioRead.analog4Avg<config.AIO2HysMaxValue && ioRead.A4StateClear==TRUE )
		{
			ioRead.A4Event = 4;
			ioRead.A4StateClear = FALSE;
		}
	
		ioRead.analog4Avg = 0;
					
		AIO_StateMachine();
	}		
	/*sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog4AvgSend);	
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");	*/

	/*sprintf((char*)tempData,"%d",ioRead.ADCConvertedValue[IO2_ADC]);
	USART_SendData_s( DEBUG_COM,"IO2_ADC ADC Value=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	/*sprintf((char*)tempData,"%.3f",ioRead.analog4);
	USART_SendData_s( DEBUG_COM,"IO2_ADC Voltage=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
}

/**
  * @brief  Reads all the inputs and set flag according to the input state detected.
  * @param  None
  * @retval None
  */
void DIO_StateMachine(void)
{	
	unsigned char breakLoop=0;	
	

	arrayInit2Zero((unsigned char*)ioRead.digitalIoData,digitalIoDataSize);
		
	// Ignition,Main power,DIO1,DIO2,DIO3 are processed in TImer4 ISR
	//----------------------------------------------------------------------//	Read Relay 1 Status
	if(ioRead.relay1State == TRUE)
	{
		ioRead.inputIoByte2=ioRead.inputIoByte2 | 0x01;//bit_set	
	}	
	else
	{
		ioRead.inputIoByte2=ioRead.inputIoByte2 & 0xFE;//bit_clear
	}	
	//----------------------------------------------------------------------// Read Relay 2 Status
	if(ioRead.relay2State == TRUE)
	{
		ioRead.inputIoByte2=ioRead.inputIoByte2 | 0x02;//bit_set	
	}	
	else
	{
		ioRead.inputIoByte2=ioRead.inputIoByte2 & 0xFD;//bit_clear
	}	
	//----------------------------------------------------------------------// Read Fuel Relay  Status
	if(ioRead.FuelrelayState == TRUE)
	{
		ioRead.inputIoByte2=ioRead.inputIoByte2 | 0x04;//bit_set	
	}	
	else
	{
		ioRead.inputIoByte2=ioRead.inputIoByte2 & 0xFB;//bit_clear
	}	
	
	//----------------------------------------------------------------------// GeoFence Status and Relays Triggering

	if(ioRead.FuelKSTriggerEn == TRUE)// Fuel relay triggering module
	{
		if(ioRead.FuelKSTriggerEnter == 1 || ioRead.FuelKSTriggerExit == 1)
		{
			for( breakLoop=0;breakLoop<=config.FuelKSPulTimes;breakLoop++)
			{					
				GPIO_ToggleBits(RELAY1_PORT,RELAY1_PIN); 						
				Delay_ms(config.FuelKSPulDuration*1000);			
			}			
		}
		else if(ioRead.FuelKSTriggerEnter == 2 || ioRead.FuelKSTriggerExit == 2)
		{
			FUEL_KS_CUT;	
		
		}
		else if(ioRead.FuelKSTriggerEnter == 3 || ioRead.FuelKSTriggerExit == 3)
		{
			FUEL_KS_MAKE;	
		
		}
		ioRead.FuelKSTriggerEn =FALSE;
	}	
		
	if(ioRead.KS1TriggerEn == TRUE)// Relay1 triggering module
	{
		if(ioRead.KS1TriggerEnter == 1 || ioRead.KS1TriggerExit == 1)
		{
			for( breakLoop=0;breakLoop<=config.KS1PulTimes;breakLoop++)
			{					
				GPIO_ToggleBits(RELAY2_PORT,RELAY2_PIN); 						
				Delay_ms(config.KS1PulDuration*1000);			
			}			
		}
		else if(ioRead.KS1TriggerEnter == 2 || ioRead.KS1TriggerExit == 2)
		{
			RELAY1_CUT;	
		
		}
		else if(ioRead.KS1TriggerEnter == 3 || ioRead.KS1TriggerExit == 3)
		{
			RELAY1_MAKE;	
		
		}
		ioRead.KS1TriggerEn =FALSE;
	}
	

	//------// GeoFence Status and SMS Generation 
	if(ioRead.geo1State == TRUE && ioRead.geo1StateClear == FALSE)
	{		
		IOMessageGenerator(GEO1,TRUE);// enter state
		ioRead.geo1StateClear = TRUE;					
	}
	else if(ioRead.geo1State == FALSE && ioRead.geo1StateClear == TRUE)
	{
		ioRead.geo1StateClear = FALSE;
		IOMessageGenerator(GEO1,FALSE);// exit state
	}
	
	if(ioRead.geo2State == TRUE && ioRead.geo2StateClear == FALSE)
	{		
		IOMessageGenerator(GEO2,TRUE);// enter state
		ioRead.geo2StateClear = TRUE;
	}
	else if(ioRead.geo2State == FALSE && ioRead.geo2StateClear == TRUE)
	{
		ioRead.geo2StateClear = FALSE;
		IOMessageGenerator(GEO2,FALSE);// exit state
	}
	
	if(ioRead.geo3State == TRUE && ioRead.geo3StateClear == FALSE)
	{		
		IOMessageGenerator(GEO3,TRUE);// enter state
		ioRead.geo3StateClear = TRUE;			
	}
	else if(ioRead.geo3State == FALSE && ioRead.geo3StateClear == TRUE)
	{
		ioRead.geo3StateClear = FALSE;
		IOMessageGenerator(GEO3,FALSE);// exit state
	}	
	
	if(ioRead.geo4State == TRUE && ioRead.geo4StateClear == FALSE)
	{		
		IOMessageGenerator(GEO4,TRUE);// enter state
		ioRead.geo4StateClear = TRUE;			
	}
	else if(ioRead.geo4State == FALSE && ioRead.geo4StateClear == TRUE)
	{
		ioRead.geo4StateClear = FALSE;
		IOMessageGenerator(GEO4,FALSE);// exit state
	}
	
	if(ioRead.geo5State == TRUE && ioRead.geo5StateClear == FALSE)
	{		
		IOMessageGenerator(GEO5,TRUE);// enter state
		ioRead.geo5StateClear = TRUE;			
	}
	else if(ioRead.geo5State == FALSE && ioRead.geo5StateClear == TRUE)
	{
		ioRead.geo5StateClear = FALSE;
		IOMessageGenerator(GEO5,FALSE);// exit state
	}
	
	if(ioRead.geo6State == TRUE && ioRead.geo6StateClear == FALSE)
	{		
		IOMessageGenerator(GEO6,TRUE);// enter state
		ioRead.geo6StateClear = TRUE;			
	}
	else if(ioRead.geo6State == FALSE && ioRead.geo6StateClear == TRUE)
	{
		ioRead.geo6StateClear = FALSE;
		IOMessageGenerator(GEO6,FALSE);// exit state
	}
	
	if(ioRead.geo7State == TRUE && ioRead.geo7StateClear == FALSE)
	{		
		IOMessageGenerator(GEO7,TRUE);// enter state
		ioRead.geo7StateClear = TRUE;			
	}
	else if(ioRead.geo7State == FALSE && ioRead.geo7StateClear == TRUE)
	{
		ioRead.geo7StateClear = FALSE;
		IOMessageGenerator(GEO7,FALSE);// exit state
	}
	
	if(ioRead.geo8State == TRUE && ioRead.geo8StateClear == FALSE)
	{		
		IOMessageGenerator(GEO8,TRUE);// enter state
		ioRead.geo8StateClear = TRUE;			
	}
	else if(ioRead.geo8State == FALSE && ioRead.geo8StateClear == TRUE)
	{
		ioRead.geo8StateClear = FALSE;
		IOMessageGenerator(GEO8,FALSE);// exit state
	}
	
	if(ioRead.geo9State == TRUE && ioRead.geo9StateClear == FALSE)
	{		
		IOMessageGenerator(GEO9,TRUE);// enter state
		ioRead.geo9StateClear = TRUE;			
	}
	else if(ioRead.geo9State == FALSE && ioRead.geo9StateClear == TRUE)
	{
		ioRead.geo9StateClear = FALSE;
		IOMessageGenerator(GEO9,FALSE);// exit state
	}
	
	if(ioRead.geo10State == TRUE && ioRead.geo10StateClear == FALSE)
	{		
		IOMessageGenerator(GEO10,TRUE);// enter state
		ioRead.geo10StateClear = TRUE;			
	}
	else if(ioRead.geo10State == FALSE && ioRead.geo10StateClear == TRUE)
	{
		ioRead.geo10StateClear = FALSE;
		IOMessageGenerator(GEO10,FALSE);// exit state
	}
	
	if(ioRead.geo11State == TRUE && ioRead.geo11StateClear == FALSE)
	{		
		IOMessageGenerator(GEO11,TRUE);// enter state
		ioRead.geo11StateClear = TRUE;			
	}
	else if(ioRead.geo11State == FALSE && ioRead.geo11StateClear == TRUE)
	{
		ioRead.geo11StateClear = FALSE;
		IOMessageGenerator(GEO11,FALSE);// exit state
	}
	
	if(ioRead.geo12State == TRUE && ioRead.geo12StateClear == FALSE)
	{		
		IOMessageGenerator(GEO12,TRUE);// enter state
		ioRead.geo12StateClear = TRUE;			
	}
	else if(ioRead.geo12State == FALSE && ioRead.geo12StateClear == TRUE)
	{
		ioRead.geo12StateClear = FALSE;
		IOMessageGenerator(GEO12,FALSE);// exit state
	}
	
	if(ioRead.geo13State == TRUE && ioRead.geo13StateClear == FALSE)
	{		
		IOMessageGenerator(GEO13,TRUE);// enter state
		ioRead.geo13StateClear = TRUE;			
	}
	else if(ioRead.geo13State == FALSE && ioRead.geo13StateClear == TRUE)
	{
		ioRead.geo13StateClear = FALSE;
		IOMessageGenerator(GEO13,FALSE);// exit state
	}
	
	if(ioRead.geo14State == TRUE && ioRead.geo14StateClear == FALSE)
	{		
		IOMessageGenerator(GEO14,TRUE);// enter state
		ioRead.geo14StateClear = TRUE;			
	}
	else if(ioRead.geo14State == FALSE && ioRead.geo14StateClear == TRUE)
	{
		ioRead.geo14StateClear = FALSE;
		IOMessageGenerator(GEO14,FALSE);// exit state
	}
	
	if(ioRead.geo15State == TRUE && ioRead.geo15StateClear == FALSE)
	{		
		IOMessageGenerator(GEO15,TRUE);// enter state
		ioRead.geo15StateClear = TRUE;			
	}
	else if(ioRead.geo15State == FALSE && ioRead.geo15StateClear == TRUE)
	{
		ioRead.geo15StateClear = FALSE;
		IOMessageGenerator(GEO15,FALSE);// exit state
	}
	
	if(ioRead.geo16State == TRUE && ioRead.geo16StateClear == FALSE)
	{		
		IOMessageGenerator(GEO16,TRUE);// enter state
		ioRead.geo16StateClear = TRUE;			
	}
	else if(ioRead.geo16State == FALSE && ioRead.geo16StateClear == TRUE)
	{
		ioRead.geo16StateClear = FALSE;
		IOMessageGenerator(GEO16,FALSE);// exit state
	}
	
	if(ioRead.geo17State == TRUE && ioRead.geo17StateClear == FALSE)
	{		
		IOMessageGenerator(GEO17,TRUE);// enter state
		ioRead.geo17StateClear = TRUE;			
	}
	else if(ioRead.geo17State == FALSE && ioRead.geo17StateClear == TRUE)
	{
		ioRead.geo17StateClear = FALSE;
		IOMessageGenerator(GEO17,FALSE);// exit state
	}
	
	if(ioRead.geo18State == TRUE && ioRead.geo18StateClear == FALSE)
	{		
		IOMessageGenerator(GEO18,TRUE);// enter state
		ioRead.geo18StateClear = TRUE;			
	}
	else if(ioRead.geo18State == FALSE && ioRead.geo18StateClear == TRUE)
	{
		ioRead.geo18StateClear = FALSE;
		IOMessageGenerator(GEO18,FALSE);// exit state
	}
	
	if(ioRead.geo19State == TRUE && ioRead.geo19StateClear == FALSE)
	{		
		IOMessageGenerator(GEO19,TRUE);// enter state
		ioRead.geo19StateClear = TRUE;			
	}
	else if(ioRead.geo19State == FALSE && ioRead.geo19StateClear == TRUE)
	{
		ioRead.geo19StateClear = FALSE;
		IOMessageGenerator(GEO19,FALSE);// exit state
	}
	
	if(ioRead.geo20State == TRUE && ioRead.geo20StateClear == FALSE)
	{		
		IOMessageGenerator(GEO20,TRUE);// enter state
		ioRead.geo20StateClear = TRUE;			
	}
	else if(ioRead.geo20State == FALSE && ioRead.geo20StateClear == TRUE)
	{
		ioRead.geo20StateClear = FALSE;
		IOMessageGenerator(GEO20,FALSE);// exit state
	}
	
	
	//----------------------------------------------------------------------// If  any IO pin status is changed
	if(ioRead.inputIoByte1 != ioRead.inputIoByte1LastState || ioRead.inputIoByte2 != ioRead.inputIoByte2LastState)
	{
		ioRead.anyIOChangeFlag = TRUE;			
	}
	
	ioRead.inputIoByte1LastState = ioRead.inputIoByte1;
	ioRead.inputIoByte2LastState = ioRead.inputIoByte2;
	//----------------------------------------------------------------------//
	sprintf((char*)tempData, "%02X", ioRead.inputIoByte1);	
	strcat((char*)ioRead.digitalIoData,(const char*)tempData);

	/*USART_SendData_s( DEBUG_COM,"digitalIoData 1=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	sprintf((char*)tempData, "%02X", ioRead.inputIoByte2);	
	strcat((char*)ioRead.digitalIoData,(const char*)tempData);
	
	/*USART_SendData_s( DEBUG_COM,"digitalIoData 2=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
}	
/**
  * @brief  
  * @param  None
  * @retval None
  */
void AIO_StateMachine(void)
{	
	arrayInit2Zero((unsigned char*)ioRead.analogIoData,analogIoDataSize);
	//----------------------------------------------------------------------//	
	ADCbatVoltAvg = (ioRead.ADCConvertedValue[INT_BATT_ADC]*Vref*1000*Int_Bat_divider_Add)/0xFFF;	// VBAT Analog Tracker Internal Battery
	ioRead.analog1 = ioRead.analog1 + ADCbatVoltAvg;	
	ADCavgCounter++;	
	if(ADCavgCounter>=10)
	{	
		ADCavgCounter = 0;
				
		ioRead.analog1 = ioRead.analog1/10;
		
		ioRead.analog1AvgSend = ioRead.analog1;
		
		
		
		if(ioRead.analog1AvgSend<=Low_Batt && ioRead.lowBattFlagClear == FALSE /*&& (!GPIO_ReadInputDataBit(MAIN_POWER_DETECT_PORT,MAIN_POWER_DETECT_PIN))*/) // if main power is not conencted
		{
			ioRead.lowBattFlag = TRUE;
			ioRead.lowBattFlagClear = TRUE;
		}
		else if(ioRead.analog1>Low_Batt)
		{
			ioRead.lowBattFlagClear = FALSE;			
		}
		ioRead.analog1 = 0;		
	}	
	sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog1AvgSend/10);//3bytes ; 0x1C3=451=451/100=4.51V actual value
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");

	/*sprintf((char*)tempData,"%d",ioRead.ADCConvertedValue[INT_BATT_ADC]);
	USART_SendData_s( DEBUG_COM,"INT_BATT_CH ADC Value=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	sprintf((char*)tempData,"%.3f",ADCbatVoltAvg );
	USART_SendData_s( DEBUG_COM,"INT_BATT_CH Voltage=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	//----------------------------------------------------------------------// Main power 12V ADC
	ioRead.analog2 = (ioRead.ADCConvertedValue[MAIN_12V_ADC]*Vref*1000*Main_Bat_divider_Add)/0xFFF;	
	sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog2/100);//3bytes ; 0x6E=110=110/10=11.0V actual value
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");	
	
	/*sprintf((char*)tempData,"%d",ioRead.ADCConvertedValue[MAIN_12V_ADC]);
	USART_SendData_s( DEBUG_COM,"MAIN_12V_CH ADC Value=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	sprintf((char*)tempData,"%.3f",ioRead.analog2 );
	USART_SendData_s( DEBUG_COM,"MAIN_12V_CH Voltage=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/

	
	//----------------------------------------------------------------------// AIO1 ADC
	sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog3AvgSend);	
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");	
	
	
	/*USART_SendData_s( DEBUG_COM,"analog3AvgSend 2 Voltage=");
	USART_SendData_s( DEBUG_COM,tempData);
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	//----------------------------------------------------------------------// AIO2 ADC
	sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog4AvgSend);	
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");	
	//----------------------------------------------------------------------// AIO3 ADC
	sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog5AvgSend);	
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
	strcat((char*)ioRead.analogIoData,(const char*)",");	
	//----------------------------------------------------------------------// AIO4 ADC
	sprintf((char*)tempData,"%X",(unsigned int)ioRead.analog6AvgSend);	
	strcat((char*)ioRead.analogIoData,(const char*)tempData);	
}	





/**
  * @brief  generates the ping packet to be sent to server
  * @param  None
  * @retval None
  */
void pingPacketGen_SL_Protocol(void)
{
	unsigned char l_packetlength=0;
	unsigned char l_packetCheckSum=0;
	unsigned char l_sum_loop=0;
	

	/*l_stringCounter++;					
	
	if(l_stringCounter>=999)
	{
		l_stringCounter = 0;
	}		*/
	arrayInit2Zero(g_TransmissionPaket,g_TransmissionPaketSize);
	
	
	//sprintf((char*)l_BufferStringCounter,"%.0f",(double)l_stringCounter);		

	arrayInit2Zero(g_TransmissionPaket,g_TransmissionPaketSize);
	
	strcat((char*)g_TransmissionPaket,(const char*)SERVER_SEND_SL_HEADER);
	
	strcat((char*)g_TransmissionPaket,(const char*)",");
	

	strcat((char*)g_TransmissionPaket,(const char*)config.IMEI);

	strcat((char*)g_TransmissionPaket,(const char*)",");
	strcat((char*)g_TransmissionPaket,(const char*)"1");
	//strcat((char*)g_TransmissionPaket,(const char*)l_BufferStringCounter);	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	
	strcat((char*)g_TransmissionPaket,(const char*)LOGIN);	
	strcat((char*)g_TransmissionPaket,(const char*)",");
	strcat((char*)g_TransmissionPaket,(const char*)config.CompanyID);	
	strcat((char*)g_TransmissionPaket,(const char*)",");

	strcat((char*)g_TransmissionPaket,(const char*)"*");	
	l_packetlength = strnlen((const char*)g_TransmissionPaket,200);	
	for(  l_sum_loop=0;l_sum_loop<l_packetlength;l_sum_loop++)
	{
		l_packetCheckSum = g_TransmissionPaket[l_sum_loop] ^ l_packetCheckSum;
	}	
	sprintf((char*)l_tempCheckSum,"%02X",l_packetCheckSum);	
	strcat((char*)g_TransmissionPaket,(const char*)l_tempCheckSum);	
	strcat((char*)g_TransmissionPaket,(const char*)"\r\n");		
}
/**
  * @brief  generates the ping packet to be sent to server
  * @param  None
  * @retval None
  */
void pingPacketGen(void)
{
	unsigned char l_packetlength=0;
	unsigned char l_packetCheckSum=0;
	unsigned char l_sum_loop=0;
	
	l_stringCounter++;					
	
	if(l_stringCounter>=999)
	{
		l_stringCounter = 0;
	}	
	
	sprintf((char*)l_BufferStringCounter,"%.0f",(double)l_stringCounter);	
	
	arrayInit2Zero(g_TransmissionPaket,g_TransmissionPaketSize);
	strcat((char*)g_TransmissionPaket,(const char*)SERVER_SEND_HEADER);	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)g_TransmissionPaket,(const char*)config.IMEI);
	}

	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)l_BufferStringCounter);	
	strcat((char*)g_TransmissionPaket,(const char*)",");
	
	strcat((char*)g_TransmissionPaket,(const char*)HEART_B);	
	strcat((char*)g_TransmissionPaket,(const char*)",");				
	strcat((char*)g_TransmissionPaket,(const char*)"*");	
	l_packetlength = strnlen((const char*)g_TransmissionPaket,200);	
	for(  l_sum_loop=0;l_sum_loop<l_packetlength;l_sum_loop++)
	{
		l_packetCheckSum = g_TransmissionPaket[l_sum_loop] ^ l_packetCheckSum;
	}	
	sprintf((char*)l_tempCheckSum,"%02X",l_packetCheckSum);	
	strcat((char*)g_TransmissionPaket,(const char*)l_tempCheckSum);	
	strcat((char*)g_TransmissionPaket,(const char*)"\r\n");		
}
/**
  * @brief  Generated the tracking packet based on the even generated
  * @param  None
  * @retval None
  */
void Extension_Formatter(unsigned char* trackType)
{
	//arrayInit2Zero(g_ExtensionPaket,g_ExtensionPaketSize);
		
	if(strstr((const char*)trackType,(const char*)ENTER_GEO_FENCE) || strstr((const char*)trackType,(const char*)EXIT_GEO_FENCE))//Send GeoFence data 
	{
		strcat((char*)g_TransmissionPaket,(const char*)GEOFENCE_DATA_EX_ID);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");
		strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.geoFenceName);
		strcat((char*)g_TransmissionPaket,(const char*)"|");	
		strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.geoFencingUniqueID);		
		strcat((char*)g_TransmissionPaket,(const char*)",");
	}	
	
	if(strstr((const char*)trackType,(const char*)IG_OFF))//Send Trip data on each ignition off event
	{
		strcat((char*)g_TransmissionPaket,(const char*)TRIP_DATA_EX_ID);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");
		strcat((char*)g_TransmissionPaket,(const char*)l_BufferTripIdCounter);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");				
		strcat((char*)g_TransmissionPaket,(const char*)ioRead.tripDistanceAasci);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");				
		strcat((char*)g_TransmissionPaket,(const char*)ioRead.travelTimeAasci);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");					
		strcat((char*)g_TransmissionPaket,(const char*)ioRead.totalTripTimeAasci);	
		strcat((char*)g_TransmissionPaket,(const char*)",");
	}	
	

//	if(strstr((const char*)trackType,(const char*)DRIVER_UNAUTHORIZE))
//	{
//		//USART_SendData_s( DEBUG_COM,"DRIVER_UNAUTHORIZE////////////// \n\r");
//		strcat((char*)g_TransmissionPaket,(const char*)DRIVER_DATA_EX_ID);	
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)ioRead.DriverID);
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)",");
//	}
	
//	if(strstr((const char*)trackType,(const char*)DRIVER_AUTHORIZE) || states.AuthenticationOkFlag==TRUE)
//	{
//		//USART_SendData_s( DEBUG_COM,"DRIVER_AUTHORIZE////////////// \n\r");
//		strcat((char*)g_TransmissionPaket,(const char*)DRIVER_DATA_EX_ID);	
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)ioRead.DriverID);	
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)ioRead.AuthenticateMethod);	
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)ioRead.DriverFPID);	
//		strcat((char*)g_TransmissionPaket,(const char*)"|");
//		strcat((char*)g_TransmissionPaket,(const char*)ioRead.DriverRFID);	
//		strcat((char*)g_TransmissionPaket,(const char*)",");
//	}
	
	
	if(strstr((const char*)trackType,(const char*)IMPACT)
		|| strstr((const char*)trackType,(const char*)HARSH_BR)
			|| strstr((const char*)trackType,(const char*)HARSH_AC) )//Send Trip data on ACC event
	{
		strcat((char*)g_TransmissionPaket,(const char*)ADC_DATA_EX_ID);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");
		sprintf((char*)G_sprintfBuffer,"%06.3f",accMeter.SendValueX);
		strcat((char*)g_TransmissionPaket,(const char*)G_sprintfBuffer);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");
		sprintf((char*)G_sprintfBuffer,"%06.3f",accMeter.SendValueY);
		strcat((char*)g_TransmissionPaket,(const char*)G_sprintfBuffer);
		strcat((char*)g_TransmissionPaket,(const char*)"|");	
		sprintf((char*)G_sprintfBuffer,"%06.3f",accMeter.SendValueZ);
		strcat((char*)g_TransmissionPaket,(const char*)G_sprintfBuffer);
		strcat((char*)g_TransmissionPaket,(const char*)"|");	
		strcat((char*)g_TransmissionPaket,(const char*)",");
	}
	if(strstr((const char*)trackType,(const char*)LEFT_SHARP_TURN)
		|| strstr((const char*)trackType,(const char*)RIGHT_SHARP_TURN))//Send Trip data on ACC event
	{
		strcat((char*)g_TransmissionPaket,(const char*)ADC_DATA_EX_ID);	
		strcat((char*)g_TransmissionPaket,(const char*)"|");
		strcat((char*)g_TransmissionPaket,(const char*)"|");
		strcat((char*)g_TransmissionPaket,(const char*)"|");	
		strcat((char*)g_TransmissionPaket,(const char*)"|");	
		sprintf((char*)G_sprintfBuffer,"%d",accMeter.HeadingChange);
		strcat((char*)g_TransmissionPaket,(const char*)G_sprintfBuffer);	
		strcat((char*)g_TransmissionPaket,(const char*)",");
	}
}
/**
  * @brief  Generated the tracking packet based on the even generated
  * @param  None
  * @retval None
  */
void Packet_Formatter(unsigned char* trackType,unsigned char counterIncrement)
{
	unsigned char l_packetLength = 0;
	unsigned char l_packetCheckSum = 0;	
	unsigned char l_sum_loop=0;
	uint8_t TrackType[4]={0};
	
	strcpy((char*)TrackType,(const char*)trackType);

		
	if(counterIncrement == TRUE)
	{
		l_stringCounter++;					
	}
	if(l_stringCounter>=999)
	{
		l_stringCounter = 0;
	}		

	
	sprintf((char*)l_BufferStringCounter,"%.0f",(double)l_stringCounter);				
	sprintf((char*)l_BufferTripIdCounter,"%.0f",(double)ioRead.tripId);			
	l_simInUse[0] = config.simNumber + '0';
	l_simInUse[1] = 0x00;			
	
	arrayInit2Zero(g_TransmissionPaket,g_TransmissionPaketSize);
	strcat((char*)g_TransmissionPaket,(const char*)SERVER_SEND_HEADER);	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)g_TransmissionPaket,(const char*)config.IMEI);
	}
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)l_BufferStringCounter);	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)trackType);
	strcat((char*)g_TransmissionPaket,(const char*)",");

	if(GPS_LOG.gpsFix[0]=='A')//If GPS is fixed then send new lat,lon and date and save them
	{
		if(trackType == HEADING_C)
		{
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.Heading_latDecimalAASCI);		
			strcat((char*)g_TransmissionPaket,(const char*)",");			
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.Heading_lonDecimalAASCI);			
			strcat((char*)g_TransmissionPaket,(const char*)",");	
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.gpsTimeDateConverted);	
			
			multiByteContentWrite(GPS_LAST_VALID_LAT,GPS_LAST_VALID_LAT_LEN,GPS_LOG.Heading_latDecimalAASCI);
			multiByteContentWrite(GPS_LAST_VALID_LON,GPS_LAST_VALID_LON_LEN,GPS_LOG.Heading_lonDecimalAASCI);
			multiByteContentWrite(GPS_LAST_VALID_DATE,GPS_LAST_VALID_DATE_LEN,GPS_LOG.gpsTimeDateConverted);
		}
		else
		{
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.latDecimalAASCI);		
			strcat((char*)g_TransmissionPaket,(const char*)",");			
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.lonDecimalAASCI);			
			strcat((char*)g_TransmissionPaket,(const char*)",");	
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.gpsTimeDateConverted);	
			
			multiByteContentWrite(GPS_LAST_VALID_LAT,GPS_LAST_VALID_LAT_LEN,GPS_LOG.latDecimalAASCI);
			multiByteContentWrite(GPS_LAST_VALID_LON,GPS_LAST_VALID_LON_LEN,GPS_LOG.lonDecimalAASCI);
			multiByteContentWrite(GPS_LAST_VALID_DATE,GPS_LAST_VALID_DATE_LEN,GPS_LOG.gpsTimeDateConverted);
		}
	}
	else if(GPS_LOG.gpsFix[0]=='V')//If GPS is not fixed then send last saved lat,lon and date
	{		
		multiByteContentRead(GPS_LAST_VALID_LAT,GPS_LAST_VALID_LAT_LEN,GPS_LOG.latDecimalAASCI);
		multiByteContentRead(GPS_LAST_VALID_LON,GPS_LAST_VALID_LON_LEN,GPS_LOG.lonDecimalAASCI);
		//multiByteContentRead(GPS_LAST_VALID_DATE,GPS_LAST_VALID_DATE_LEN,GPS_LOG.gpsTimeDateConverted);
		
		strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.latDecimalAASCI);		
		strcat((char*)g_TransmissionPaket,(const char*)",");			
		strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.lonDecimalAASCI);			
		strcat((char*)g_TransmissionPaket,(const char*)",");	
		strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.gpsTimeDateConverted);	
	}
	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.gpsFix);		
	strcat((char*)g_TransmissionPaket,(const char*)",");		
	strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.numberOfSatellite);		
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)gsmComm.gsmRSSIAASCI);	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.speed);	
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.gpsHeading);	
	strcat((char*)g_TransmissionPaket,(const char*)",");
	
	strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.distanceAasci);	//distance travelled since tracker installed
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)config.totalTimeTravelledAASCI);	// ignition on,speed>3,since tracker installed
	strcat((char*)g_TransmissionPaket,(const char*)",");		

	strcat((char*)g_TransmissionPaket,(const char*)gsmComm.gsmMCCMNC);
	strcat((char*)g_TransmissionPaket,(const char*)",");	
	strcat((char*)g_TransmissionPaket,(const char*)ioRead.digitalIoData);
	strcat((char*)g_TransmissionPaket,(const char*)",");			
	strcat((char*)g_TransmissionPaket,(const char*)ioRead.analogIoData);
	strcat((char*)g_TransmissionPaket,(const char*)",");			
	strcat((char*)g_TransmissionPaket,(const char*)l_simInUse);			
	strcat((char*)g_TransmissionPaket,(const char*)",");
	
	Extension_Formatter(TrackType);
	//strcat((char*)g_TransmissionPaket,(const char*)g_ExtensionPaket);
	
	strcat((char*)g_TransmissionPaket,(const char*)"*");			
	l_packetLength = strnlen((const char*)g_TransmissionPaket,240);	
	for(  l_sum_loop=0;l_sum_loop<l_packetLength;l_sum_loop++)
	{
		l_packetCheckSum = g_TransmissionPaket[l_sum_loop] ^ l_packetCheckSum;
	}	
	sprintf((char*)l_tempCheckSum,"%02X",l_packetCheckSum);	
	strcat((char*)g_TransmissionPaket,(const char*)l_tempCheckSum);	
	strcat((char*)g_TransmissionPaket,(const char*)"\r\n");	
	
	
}


/**
  * @brief  data transmission controller check for the network connectivity, data transmission and data recovery from EEPROM
  * @param  None
  * @retval None
  */
void dataTransmissionController()
{	
	//unsigned char eepromReadSecess = 0;
	//unsigned char sendSucess = 0;
	//unsigned char l_readSend=0;
//	USART_SendData_s( DEBUG_COM,"TM-1\r\n");
	GPRStrackingModule();
//	USART_SendData_s( DEBUG_COM,"TM-2\r\n");
	if(config.eepromAlgoSet!=LIVE_ALGO )//if packet is to send live
	{
		DataRecoveryHandler();
	}
	
	if(states.GPRSStateOk == TRUE && states.FTPPUTConnectFlag==FALSE)
	{	
		if(DataSendStatus.gprsDemand == TRUE)
		{
			Packet_Formatter((unsigned char*)LOC_DEMAND,TRUE);	
			
			DataStringHandler();
			
			DataSendStatus.gprsDemand = FALSE;
		}
		if(config.replyReadyGprs == TRUE)// any ACK is pending reply from the tracker
		{			
			Command_Send_SEND_Get(config.replyToSend);		////////////////////////
			Delay_ms(1000);
			if(config.jmpCallFlag == TRUE)
			{		
				config.jmpCallFlag = FALSE;
				eeprom_write_byte(2,0xBB);// internal eeprom writing			
				Delay_ms(100);
				/* Go to the bootloader application  */
				USART_SendData_s( DEBUG_COM,"Jumping to Bootloader Application\r\n");
				//This function initiate a system reset request to reset the MCU.
				NVIC_SystemReset();				
			}
			config.replyReadyGprs = FALSE;
		}
											
	}
//	USART_SendData_s( DEBUG_COM,"TM-3\r\n");
	//***************************************************SMS CASE************************************
	
	if(states.networkInit == TRUE)
	{		
		if(states.smsTrackFlag == TRUE && config.transmissionTimeSMS != 0)
		{		
			Packet_Formatter((unsigned char*)TIME_TRACK,FALSE);			
//			SMS_Generator(g_TransmissionPaket,config.userCellNumber1);
			states.smsTrackFlag = FALSE;	
		}	
		if(DataSendStatus.smsDemand == TRUE)
		{		
			Packet_Formatter((unsigned char*)LOC_DEMAND,FALSE);
			SMS_Generator(g_TransmissionPaket,sms.smsReplyNumber);
			DataSendStatus.smsDemand = FALSE;	
		}	
		if(DataSendStatus.smsDemandGoogle == TRUE)
		{
			arrayInit2Zero(g_TransmissionPaket,g_TransmissionPaketSize);
			strcat((char*)g_TransmissionPaket,(const char*)GOOGLE_TOP);	
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.latDecimalAASCI);	
			strcat((char*)g_TransmissionPaket,(const char*)",");	
			strcat((char*)g_TransmissionPaket,(const char*)GPS_LOG.lonDecimalAASCI);	
			strcat((char*)g_TransmissionPaket,(const char*)GOOGLE_BOTTOM);				
			SMS_Generator(g_TransmissionPaket,sms.smsReplyNumber);
			DataSendStatus.smsDemandGoogle = FALSE;	
		}
		if(config.replyReadySms == TRUE)
		{
			SMS_Generator(config.replyToSend,sms.smsReplyNumber);
			config.replyReadySms = FALSE;
		}		
	}
//	USART_SendData_s( DEBUG_COM,"TM-4\r\n");
}
/**
  * @brief  this module extracts packet no from string to configure ack layer
  * @param  None
  * @retval None
  */
uint16_t PacketNoExtract(void)
{
	uint16_t l_packetNo;
	uint8_t l_loop;
	uint8_t l_tempIndex=0;
	uint8_t l_commaCount=0;
	uint8_t l_tempBuff[6]={0x00};
	for(l_loop=0;l_loop<30;l_loop++)
	{
		if(g_ReTransmissionPaket[l_loop]==',')
		{
			l_commaCount++;
			if(l_commaCount==2)
			{
				break;
			}
		}
	}
	l_loop++;
	
	for(;l_loop<30;l_loop++)
	{
		l_tempBuff[l_tempIndex++]=g_ReTransmissionPaket[l_loop];
		if(g_ReTransmissionPaket[l_loop+1]==',')
		{
			l_tempBuff[l_tempIndex]=0x00;
			break;
		}
	}
	
	l_packetNo= atoi((const char*)l_tempBuff);
//	USART_SendData_s(DEBUG_COM,"\r\nPacket No=");
//	sprintf(( char*)G_sprintfBuffer,"%d",l_packetNo);	
//	USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
//	USART_SendData_s(DEBUG_COM,"\r\n");
	
	return l_packetNo;
	
}
/**
  * @brief  this module check for different tracking and event flags and depending on flags transmit data to server
  * @param  None
  * @retval None
  */
void DataRecoveryHandler(void)
{
	unsigned char eepromReadSecess = FALSE;
	//unsigned char sendSucess = FALSE;
	//unsigned char l_readSend=FALSE;
	
	if(eeprom.eepromRecoveryEnable == TRUE  && config.eepromRecovSett == TRUE && states.JammingDetectFlag==FALSE && states.GPRSStateOk == TRUE && states.FTPPUTConnectFlag==FALSE)// perform data recovery from eeprom	
	{		
		if(config.AckLayerEnDs==TRUE)//If Ack layer is enabled
		{
			if(Ack.DataHandlerFlag==FALSE)
			{
				/////USART_SendData_s( DEBUG_COM,"Ack.DataHandlerFlag==FALSE\r\n");	
				arrayInit2Zero(g_ReTransmissionPaket,g_TransmissionPaketSize);
				eepromReadSecess=EEPROM_Read_Module(g_ReTransmissionPaket);
				
			}
			if(eepromReadSecess == TRUE)
			{
				l_SendPktCounter=PacketNoExtract();//Extrack Packet no of from Sent packet to configure next ack reply
				//USART_SendData_s( DEBUG_COM,g_ReTransmissionPaket);//Comment due to overload
				//USART_SendData_s( DEBUG_COM,"eepromReadSecess==TRUE\r\n");	//Comment due to overload
				Ack.DataHandlerFlag=TRUE;
				states.AckTimeOutCounter=0;
				TIM_SetCounter(TIMER_NO2,TIMER2_AUTORELOAD_VLAUE);
			
				if(states.GPRSStateOk == TRUE && states.FTPPUTConnectFlag==FALSE )
				{	
					//Delay_ms(1000*l_ackdelay);//Server next packet delay ;comment due to overload
					Command_Send_SEND_Get(g_ReTransmissionPaket);							
				}				
			}
		}
		else//If Ack layer is not enabled
		{			
			eepromReadSecess = EEPROM_Read_Module(g_TransmissionPaket);	
			if(eepromReadSecess == TRUE)
			{
				Command_Send_SEND_Get(g_TransmissionPaket);	
				//USART_SendData_s( DEBUG_COM,g_TransmissionPaket);//Comment due to overload
				Delay_ms(1000);
			}	
		}
	}

	
	if(states.AckPacketErrorCounter >=config.AckTry && config.AckLayerEnDs==TRUE)//if a packet is not acknowlegde after config.AckTry 
	{				
		USART_SendData_s( DEBUG_COM,"states.AckPacketErrorCounter/////////\n\r");
		states.AckPacketErrorCounter = 0;	
		states.AckGprsErrorCounter++;
		Ack.DataHandlerFlag=FALSE;//Transmit next packet if tries fail
	}					
	if(states.AckGprsErrorCounter >= 2 && config.AckLayerEnDs==TRUE)//if 2 packets are consectively not acknowlegde
	{	
		USART_SendData_s( DEBUG_COM,"states.AckGprsErrorCounter/////////\n\r");
		states.AckGprsErrorCounter=0;
		states.AckPacketErrorCounter = 0;	
		Ack.DataHandlerFlag=FALSE;
		states.gsmFirstInit = FALSE;		

		eeprom.eepromReadPointer -=2;//Resend previous 2 packets whose sending was failed 
	}			
	
	
}

/**
  * @brief  DataStringHandler
  * @param  None
  * @retval None
  */
void DataStringHandler(void)
{
	if(config.eepromAlgoSet==LIVE_ALGO && states.GPRSStateOk == TRUE && states.FTPPUTConnectFlag==FALSE)//if packet is to send live
	{		
		Command_Send_SEND_Get(g_TransmissionPaket);							
	}
	else if(config.eepromRecovSett==TRUE)//if recovery is on,save data and use DataRecoveryHandler()
	{
		EEPROM_Write_Module(g_TransmissionPaket);
	}
	else//if recovery is not on then send live data
	{		
		Command_Send_SEND_Get(g_TransmissionPaket);
	}
}
/**
  * @brief  this module check for different tracking and event flags and depending on flags transmit data to server
  * @param  None
  * @retval None
  */
void GPRStrackingModule(void)
{
	if(Ack.LoginPackFlag == TRUE && states.GPRSStateOk == TRUE)// login data to server
	{		
//	USART_SendData_s( DEBUG_COM,"Sending Login Packet/////////\r\n");
		Ack.LoginAckRetransmit=FALSE;

		pingPacketGen_SL_Protocol();
		strcpy((char*)g_ReTransmissionPaket,(const char*)g_TransmissionPaket);		
			
		l_SendPktCounter=PacketNoExtract();//Extrack Packet no of from Sent packet to configure next ack reply
	//USART_SendData_s( DEBUG_COM,g_ReTransmissionPaket);//Comment due to overload
		Ack.DataHandlerFlag=TRUE;
		states.AckTimeOutCounter=0;
		TIM_SetCounter(TIMER_NO2,TIMER2_AUTORELOAD_VLAUE);
				
		if(states.GPRSStateOk == TRUE && states.FTPPUTConnectFlag==FALSE )
		{	
			Command_Send_SEND_Get(g_ReTransmissionPaket);							
		}				
	
		
		Ack.LoginPackFlag = FALSE;	
	
	}	
	//*******************************************************************************EEPROM HANDLERS AND TRACKING MODULES***************************************************	
	if(states.JammingDetectFlag != FALSE)// Jamming Detection
	{
		if(states.JammingDetectFlag == TRUE && IsAlertOn(atoi(JAMMING_ON)))
		{
			Packet_Formatter((unsigned char*)JAMMING_ON,TRUE);				
		}
		else if(states.JammingDetectFlag == 2 && IsAlertOn(atoi(JAMMING_OFF)) )
		{
			Packet_Formatter((unsigned char*)JAMMING_OFF,TRUE);				
		}	
			
		DataStringHandler();
		
		states.JammingDetectFlag = FALSE;		
	}
	
	if(GPS_LOG.geoFencingInFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(GPS_LOG.geoFencingInFlag == TRUE && IsAlertOn(atoi(ENTER_GEO_FENCE)))
		{
			Packet_Formatter((unsigned char*)ENTER_GEO_FENCE,TRUE);				
		}
		else if(GPS_LOG.geoFencingInFlag == 2 && IsAlertOn(atoi(EXIT_GEO_FENCE)))
		{
			Packet_Formatter((unsigned char*)EXIT_GEO_FENCE,TRUE);				
			GPS_LOG.geoFenceName[0] = '0';
		}	
			
		DataStringHandler();
		
		GPS_LOG.geoFencingInFlag = FALSE;		
	}	
	
	
	
	if(states.transmissionFlagIGOn == TRUE && IsAlertOn(atoi(TIME_TRACK)) &&  config.transmissionTimeIGon != 0)// normal transmission TIME TRACK
	{		
		Packet_Formatter((unsigned char*)TIME_TRACK,TRUE);				
		
		DataStringHandler();
		
		states.transmissionFlagIGOn = FALSE;
		ioRead.motionDetect = FALSE;
		GPS_LOG.headingSendFlag = FALSE;	
		GPS_LOG.distanceTrackFlag = FALSE;
		
	}
	
	
	else if(states.transmissionFlagIGOff == TRUE && IsAlertOn(atoi(TIMETRACK_IG_OFF)) && config.transmissionTimeIGOff != 0)// normal transmission TIME TRACK
	{		
		Packet_Formatter((unsigned char*)TIMETRACK_IG_OFF,TRUE);				

		DataStringHandler();
		
		states.transmissionFlagIGOff = FALSE;
		ioRead.motionDetect = FALSE;
		GPS_LOG.headingSendFlag = FALSE;	
		GPS_LOG.distanceTrackFlag = FALSE;
		
	}
	
	if(config.MD_ON_OFF_FLAG  == FALSE)
	{
		ioRead.NextMotionAlertCount= 0;
	}
	if(config.noGPSMNOnOff == FALSE)
	{
		GPS_LOG.gpsNoFixCounterMD = 0;
	}
	
	
	
	if(ioRead.motionDetect == TRUE && IsAlertOn(atoi(MOTION_D)) && ((ioRead.NextMotionAlertCount > config.NextAnymotionAlertTimeOut) || GPS_LOG.gpsNoFixCounterMD > config.gpsNoFixMNTime)) // send packet on motion detection even if GPS is void 5 times check
	{		
		GPS_LOG.gpsNoFixCounterMD = 0;
		ioRead.NextMotionAlertCount=0;
		Packet_Formatter((unsigned char*)MOTION_D,TRUE);	
		
		DataStringHandler();
		
		states.transmissionFlagIGOn = FALSE;
		ioRead.motionDetect = FALSE;
		GPS_LOG.headingSendFlag = FALSE;	
		GPS_LOG.distanceTrackFlag = FALSE;
	}
	
	else if(accMeter.sharpTurnEvent != FALSE)// report Sharp turn
	{
		if(accMeter.sharpTurnEvent == 1 && IsAlertOn(atoi(RIGHT_SHARP_TURN)))
		{
			Packet_Formatter((unsigned char*)RIGHT_SHARP_TURN,TRUE);							
		}		
		else if(accMeter.sharpTurnEvent == 2 && IsAlertOn(atoi(LEFT_SHARP_TURN)))
		{
			Packet_Formatter((unsigned char*)LEFT_SHARP_TURN,TRUE);							
		}				
	
		DataStringHandler();
		
		accMeter.HeadingChange=0;
		accMeter.sharpTurnEvent = FALSE;	

		GPS_LOG.distanceTrackTemp = 0; // reset distance based tracking
		states.transmissionFlagIGOn = FALSE;
		ioRead.motionDetect = FALSE;
		GPS_LOG.headingSendFlag = FALSE;	
		GPS_LOG.distanceTrackFlag = FALSE;	
	}	
	
	else if(GPS_LOG.headingSendFlag == TRUE && IsAlertOn(atoi(HEADING_C)) && config.headingChange != 0)// report Heading Change
	{		
		Packet_Formatter((unsigned char*)HEADING_C,TRUE);				
		
		DataStringHandler();
		
		GPS_LOG.distanceTrackTemp = 0; // reset distance based tracking
		states.transmissionFlagIGOn = FALSE;
		ioRead.motionDetect = FALSE;
		GPS_LOG.headingSendFlag = FALSE;	
		GPS_LOG.distanceTrackFlag = FALSE;			
		
	}	
	
	else if(GPS_LOG.distanceTrackFlag == TRUE && IsAlertOn(atoi(DST_TRACK)) && config.distanceTracking != 0) // Heading Change TRACK
	{
		Packet_Formatter((unsigned char*)DST_TRACK,TRUE);				
			
		DataStringHandler();
		
		states.transmissionFlagIGOn = FALSE;
		ioRead.motionDetect = FALSE;
		GPS_LOG.headingSendFlag = FALSE;	
		GPS_LOG.distanceTrackFlag = FALSE;	
	}
	
	else if(states.resetTransmission>0) // GPRS Reset Transmission
	{		
		USART_SendData_s( DEBUG_COM, "resetTransmission////////////////\r\n");
		pingPacketGen();		
		if(states.resetTransmission>0)
		{
			states.resetTransmission--;
		}

		DataStringHandler();
	}
	
	else if(states.HeartBeatFlag == TRUE && config.heartBeatTime != 0)// heart beat data to server
	{		
		pingPacketGen();				
		
		DataStringHandler();
		
		states.HeartBeatFlag = FALSE;						
	}
	//***************************************************************************TRACKING END **********************************************************///	
	
	
	if(accMeter.harshBreakFlag == TRUE || accMeter.harshAcceFlag == TRUE || accMeter.impactFlag == TRUE)
	{		
		if(accMeter.harshAcceFlag == TRUE && IsAlertOn(atoi(HARSH_AC)))	
		{
			Packet_Formatter((unsigned char*)HARSH_AC,TRUE);		
			accMeter.harshAcceFlag = FALSE;					
		}
		else if(accMeter.harshBreakFlag && IsAlertOn(atoi(HARSH_BR)))
		{
			Packet_Formatter((unsigned char*)HARSH_BR,TRUE);				
			accMeter.harshBreakFlag = FALSE;					
		}					
		else if(accMeter.impactFlag == TRUE && IsAlertOn(atoi(IMPACT)))
		{
			Packet_Formatter((unsigned char*)IMPACT,TRUE);		
			accMeter.impactFlag = FALSE;	
			
		}

		DataStringHandler();
	}
	
	
	
	if(ioRead.ignitionEvenFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.ignitionEvenFlag == TRUE && IsAlertOn(atoi(IG_ON)) )
		{
			Packet_Formatter((unsigned char*)IG_ON,TRUE);				
		}
		else if(ioRead.ignitionEvenFlag  == 2 && IsAlertOn(atoi(IG_OFF)))
		{
			Packet_Formatter((unsigned char*)IG_OFF,TRUE);		
		}	
		
		DataStringHandler();
		
		ioRead.ignitionEvenFlag = FALSE;		
	}	
	
	if(ioRead.sosActive == TRUE && IsAlertOn(atoi(SOS_PRESS)))// reporting ignition change for trip IDS
	{
		Packet_Formatter((unsigned char*)SOS_PRESS,TRUE);	

		DataStringHandler();
		
		ioRead.sosActive = FALSE;		
	}
	
	if(ioRead.lowBattFlag == TRUE && IsAlertOn(atoi(LOW_BATT)))// reporting ignition change for trip IDS
	{
		Packet_Formatter((unsigned char*)LOW_BATT,TRUE);	

		DataStringHandler();
		
		ioRead.lowBattFlag = FALSE;		
	}		
	
	if(ioRead.mainPowerFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.mainPowerFlag == TRUE && IsAlertOn(atoi(MAIN_PWR_ON)))
		{
			Packet_Formatter((unsigned char*)MAIN_PWR_ON,TRUE);				
		}		
		else if(ioRead.mainPowerFlag == 2 && IsAlertOn(atoi(MAIN_PWR_OFF)))
		{
			Packet_Formatter((unsigned char*)MAIN_PWR_OFF,TRUE);				
		}
				
		DataStringHandler();
		
		ioRead.mainPowerFlag = FALSE;		
	}	
	
	if(ioRead.D1eventFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.D1eventFlag == TRUE && IsAlertOn(atoi(INPUT1_ACTIVE)))
		{
			Packet_Formatter((unsigned char*)INPUT1_ACTIVE,TRUE);				
		}		
		else if(ioRead.D1eventFlag == 2 && IsAlertOn(atoi(INPUT1_INACTIVE)))
		{
			Packet_Formatter((unsigned char*)INPUT1_INACTIVE,TRUE);				
		}
		
		DataStringHandler();
		
		ioRead.D1eventFlag = FALSE;		
	}	
	
	if(ioRead.D2eventFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.D2eventFlag == TRUE && IsAlertOn(atoi(INPUT2_ACTIVE)))
		{
			Packet_Formatter((unsigned char*)INPUT2_ACTIVE,TRUE);				
		}		
		else if(ioRead.D2eventFlag == 2 && IsAlertOn(atoi(INPUT2_INACTIVE)))
		{
			Packet_Formatter((unsigned char*)INPUT2_INACTIVE,TRUE);				
		}
		
		DataStringHandler();
		
		ioRead.D2eventFlag = FALSE;		
	}	
	
	if(ioRead.D3eventFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.D3eventFlag == TRUE && IsAlertOn(atoi(INPUT3_ACTIVE)))
		{
			Packet_Formatter((unsigned char*)INPUT3_ACTIVE,TRUE);				
		}		
		else if(ioRead.D3eventFlag == 2 && IsAlertOn(atoi(INPUT3_INACTIVE)))
		{
			Packet_Formatter((unsigned char*)INPUT3_INACTIVE,TRUE);				
		}
		
		DataStringHandler();
		
		ioRead.D3eventFlag = FALSE;		
	}	
	
	
	if(ioRead.A3Event != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.A3Event == TRUE && IsAlertOn(atoi(AI1_T_MAX)))
		{
			Packet_Formatter((unsigned char*)AI1_T_MAX,TRUE);				
		}		
		else if(ioRead.A3Event == 2 && IsAlertOn(atoi(AI1_T_MIN)))
		{
			Packet_Formatter((unsigned char*)AI1_T_MIN,TRUE);				
		}
		else if(ioRead.A3Event == 3 && IsAlertOn(atoi(AI1_DIFF)))
		{
			Packet_Formatter((unsigned char*)AI1_DIFF,TRUE);						
		}		
		else if(ioRead.A3Event == 4 && IsAlertOn(atoi(AI1_NORMAL)))
		{
			Packet_Formatter((unsigned char*)AI1_NORMAL,TRUE);						
		}	
			
		DataStringHandler();
		
		ioRead.A3Event = FALSE;		
	}	
	
	if(ioRead.A4Event != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.A4Event == TRUE && IsAlertOn(atoi(AI2_T_MAX)))
		{
			Packet_Formatter((unsigned char*)AI2_T_MAX,TRUE);				
		}		
		else if(ioRead.A4Event == 2 && IsAlertOn(atoi(AI2_T_MIN)))
		{
			Packet_Formatter((unsigned char*)AI2_T_MIN,TRUE);				
		}
		else if(ioRead.A4Event == 3 && IsAlertOn(atoi(AI2_DIFF)))
		{
			Packet_Formatter((unsigned char*)AI2_DIFF,TRUE);						
		}			
		else if(ioRead.A4Event == 4 && IsAlertOn(atoi(AI2_NORMAL)))
		{
			Packet_Formatter((unsigned char*)AI2_NORMAL,TRUE);						
		}	
				
		DataStringHandler();
		
		ioRead.A4Event = FALSE;		
	}	
	
	if(ioRead.A5Event != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.A5Event == TRUE && IsAlertOn(atoi(AI3_T_MAX)))
		{
			Packet_Formatter((unsigned char*)AI3_T_MAX,TRUE);				
		}		
		else if(ioRead.A5Event == 2 && IsAlertOn(atoi(AI3_T_MIN)))
		{
			Packet_Formatter((unsigned char*)AI3_T_MIN,TRUE);				
		}
		else if(ioRead.A5Event == 3 && IsAlertOn(atoi(AI3_DIFF)))
		{
			Packet_Formatter((unsigned char*)AI3_DIFF,TRUE);						
		}			
		else if(ioRead.A5Event == 4 && IsAlertOn(atoi(AI3_NORMAL)))
		{
			Packet_Formatter((unsigned char*)AI3_NORMAL,TRUE);						
		}	
		
		DataStringHandler();
		
		ioRead.A5Event = FALSE;		
	}	
	
	if(ioRead.A6Event != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.A6Event == TRUE && IsAlertOn(atoi(AI4_T_MAX)))
		{
			Packet_Formatter((unsigned char*)AI4_T_MAX,TRUE);				
		}		
		else if(ioRead.A6Event == 2 && IsAlertOn(atoi(AI4_T_MIN)))
		{
			Packet_Formatter((unsigned char*)AI4_T_MIN,TRUE);				
		}
		else if(ioRead.A6Event == 3 && IsAlertOn(atoi(AI4_DIFF)))
		{
			Packet_Formatter((unsigned char*)AI4_DIFF,TRUE);						
		}		
		else if(ioRead.A6Event == 4 && IsAlertOn(atoi(AI4_NORMAL)))
		{
			Packet_Formatter((unsigned char*)AI4_NORMAL,TRUE);						
		}	
				
		DataStringHandler();
		
		ioRead.A6Event = FALSE;		
	}	
	
	
	
	if(GPS_LOG.speedLimitFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(GPS_LOG.speedLimitFlag == TRUE && IsAlertOn(atoi(SPEEDING)))
		{
			Packet_Formatter((unsigned char*)SPEEDING,TRUE);		
		}		
		DataStringHandler();
		
		GPS_LOG.speedLimitFlag = FALSE;		
	}	
	
	if(GPS_LOG.overSpeedClearEvent == TRUE && IsAlertOn(atoi(OVERSPEED_CLR)))// reporting ignition change for trip IDS
	{
		Packet_Formatter((unsigned char*)OVERSPEED_CLR,TRUE);				
		
		DataStringHandler();
		GPS_LOG.overSpeedClearEvent = FALSE;		
	}
	
	
	if(ioRead.tripCompEvent == TRUE && IsAlertOn(atoi(TRIP_COMP)))// report Trip completion 
	{
		Packet_Formatter((unsigned char*)TRIP_COMP,TRUE);				
		DataStringHandler();
		ioRead.tripCompEvent = FALSE;		
	}
	
	if(GPS_LOG.gpsFixNoFix != FALSE)// reporting ignition change for trip IDS
	{
		if(GPS_LOG.gpsFixNoFix == TRUE && IsAlertOn(atoi(GPS_FIX)))
		{
			Packet_Formatter((unsigned char*)GPS_FIX,TRUE);				
		}		
		else if(GPS_LOG.gpsFixNoFix == 2 && IsAlertOn(atoi(GPS_NO_FIX)))
		{
			Packet_Formatter((unsigned char*)GPS_NO_FIX,TRUE);				
		}
		
		DataStringHandler();
				
		GPS_LOG.gpsFixNoFix = FALSE;		
	}	
	
	
	
	
	if(simChangeEvent == TRUE && IsAlertOn(atoi(SIM_CHG)))// reporting ignition change for trip IDS
	{
		Packet_Formatter((unsigned char*)SIM_CHG,TRUE);				
		
		DataStringHandler();
		
		simChangeEvent = FALSE;		
	}		
	
	if(config.mcuRebootFlag == TRUE && IsAlertOn(atoi(DEVICE_BOOT)))// reporting ignition change for trip IDS
	{
		Packet_Formatter((unsigned char*)DEVICE_BOOT,TRUE);				

		DataStringHandler();
		
		config.mcuRebootFlag = FALSE;		
	}			
	
	
	if(config.firmwareErrorFlag == TRUE && IsAlertOn(atoi(FIRM_ERROR_CODE)))// reporting ignition change for trip IDS
	{
		Packet_Formatter((unsigned char*)FIRM_ERROR_CODE,TRUE);				
	
		DataStringHandler();
		
		config.firmwareErrorFlag = FALSE;		
	}			
	
	if(config.IAPReadyFlag == TRUE && IsAlertOn(atoi(IAP_READY)))
	{
		USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nIAP_READY //////////////\r\n");
		Packet_Formatter((unsigned char*)IAP_READY,TRUE);				
	
		DataStringHandler();
		
		config.IAPReadyFlag = FALSE;		
	}
	
	if(config.IAPUpdatedFlag == TRUE && IsAlertOn(atoi(IAP_UPDATED)))
	{
		USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nIAP_UPDATED //////////////\r\n");
		Packet_Formatter((unsigned char*)IAP_UPDATED,TRUE);				

		DataStringHandler();
		
		config.IAPUpdatedFlag = FALSE;		
	}
	
	if(states.sleepModeACKFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(states.sleepModeACKFlag == TRUE && IsAlertOn(atoi(ENTER_SLEEP)))
		{
			Packet_Formatter((unsigned char*)ENTER_SLEEP,TRUE);				
		}		
		else if(states.sleepModeACKFlag == 2 && IsAlertOn(atoi(EXIT_SLEEP)))
		{
			Packet_Formatter((unsigned char*)EXIT_SLEEP,TRUE);				
		}
				
		DataStringHandler();
		
		states.sleepModeACKFlag = FALSE;		
	}	
	
	if(ioRead.FuelKSTriggerEnter != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.FuelKSTriggerEnter == 1 && IsAlertOn(atoi(FUEL_RELAY_PULSE_CODE)))
		{
			Packet_Formatter((unsigned char*)FUEL_RELAY_PULSE_CODE,TRUE);							
		}		
		else if(ioRead.FuelKSTriggerEnter == 2 && IsAlertOn(atoi(FUEL_RELAY_CUT_CODE)))
		{
			Packet_Formatter((unsigned char*)FUEL_RELAY_CUT_CODE,TRUE);							
		}				
		else if(ioRead.FuelKSTriggerEnter == 3 && IsAlertOn(atoi(FUEL_RELAY_MAKE_CODE)))
		{
			Packet_Formatter((unsigned char*)FUEL_RELAY_MAKE_CODE,TRUE);							
		}
		DataStringHandler();
		
		ioRead.FuelKSTriggerEnter = FALSE;		
	}	
	
	if(ioRead.KS1TriggerEnter != FALSE)// reporting ignition change for trip IDS
	{
		if(ioRead.KS1TriggerEnter == 1 && IsAlertOn(atoi(RELAY1_PULSE_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY1_PULSE_CODE,TRUE);							
		}		
		else if(ioRead.KS1TriggerEnter == 2 && IsAlertOn(atoi(RELAY1_CUT_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY1_CUT_CODE,TRUE);							
		}				
		else if(ioRead.KS1TriggerEnter == 3 && IsAlertOn(atoi(RELAY1_MAKE_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY1_MAKE_CODE,TRUE);							
		}
		DataStringHandler();
		
		ioRead.KS1TriggerEnter = FALSE;		
	}	
	
//	if(ioRead.KS2TriggerEnter != FALSE)// reporting ignition change for trip IDS
//	{
//		if(ioRead.KS2TriggerEnter == 1 && IsAlertOn(atoi(RELAY2_PULSE_CODE)))
//		{
//			Packet_Formatter((unsigned char*)RELAY2_PULSE_CODE,TRUE);							
//		}		
//		else if(ioRead.KS2TriggerEnter == 2 && IsAlertOn(atoi(RELAY2_CUT_CODE)))
//		{
//			Packet_Formatter((unsigned char*)RELAY2_CUT_CODE,TRUE);							
//		}				
//		else if(ioRead.KS2TriggerEnter == 3 && IsAlertOn(atoi(RELAY2_MAKE_CODE)))
//		{
//			Packet_Formatter((unsigned char*)RELAY2_MAKE_CODE,TRUE);							
//		}
//		DataStringHandler();
//		
//		ioRead.KS2TriggerEnter = FALSE;		
//	}	
	
	if(config.relay1CutFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(config.relay1CutFlag == TRUE && IsAlertOn(atoi(RELAY1_CUT_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY1_CUT_CODE,TRUE);							
		}		
		else if(config.relay1CutFlag == 2 && IsAlertOn(atoi(RELAY1_MAKE_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY1_MAKE_CODE,TRUE);							
		}				
	
		DataStringHandler();
		
		config.relay1CutFlag = FALSE;		
	}	
	
	if(config.relay2CutFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(config.relay2CutFlag == TRUE && IsAlertOn(atoi(RELAY2_CUT_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY2_CUT_CODE,TRUE);							
		}		
		else if(config.relay2CutFlag == 2 && IsAlertOn(atoi(RELAY2_MAKE_CODE)))
		{
			Packet_Formatter((unsigned char*)RELAY2_MAKE_CODE,TRUE);							
		}		

		DataStringHandler();

		config.relay2CutFlag = FALSE;		
	}

	if(config.FuelrelayCutFlag != FALSE)// reporting ignition change for trip IDS
	{
		if(config.FuelrelayCutFlag == TRUE && IsAlertOn(atoi(FUEL_RELAY_CUT_CODE)))
		{
			Packet_Formatter((unsigned char*)FUEL_RELAY_CUT_CODE,TRUE);							
		}		
		else if(config.FuelrelayCutFlag == 2 && IsAlertOn(atoi(FUEL_RELAY_MAKE_CODE)))
		{
			Packet_Formatter((unsigned char*)FUEL_RELAY_MAKE_CODE,TRUE);							
		}		

		DataStringHandler();

		config.FuelrelayCutFlag = FALSE;		
	}		
	
	

	if(ioRead.idleTimeAlert != FALSE)// // report Excess Idling
	{	
		if(ioRead.idleTimeAlert == 1 && IsAlertOn(atoi(EXCESS_IDLE)))
		{
			Packet_Formatter((unsigned char*)EXCESS_IDLE,TRUE);				
		}		
		else if(ioRead.idleTimeAlert == 2 && IsAlertOn(atoi(REPEAT_EXCESS_IDLE)))
		{
			Packet_Formatter((unsigned char*)REPEAT_EXCESS_IDLE,TRUE);				
		}
	
		DataStringHandler();
				
		ioRead.idleTimeAlert = FALSE;		
	}	
	
	if(config.pingReqFlag == TRUE)// heart beat data to server
	{		
		pingPacketGen();				
		
		DataStringHandler();
		
		config.pingReqFlag = FALSE;			
	}		
	
	// remaining Even data transmission to be consider here		
	//***********************************GPRS Transmission when its connected***********************	

	
}


/**
  * @brief  convert downloaded file to hex format,crosscheck the checksum of file records,extract Data to be programmed in FLASH 
  * @param  None
  * @retval indicates whether extraction is successful or not
	*    @arg 0: Download is successful
	*    @arg 1: Download is not successful
  */
uint8_t IAP_convert_file(void)
{
	#ifdef  FLASH_ENABLED

	uint32_t i=0;
	uint32_t j=0;
	uint32_t file_size=0;
	uint8_t checksum = 0;
	uint8_t temp_Buffer[17] = {0};
	uint8_t temp = 0;
	uint32_t array_index1 = 0;

	unsigned char l_tempPacket[30];

	IAP.File_Size=0;

	USART_SendData_s(DEBUG_COM,(unsigned char*)"\r\n Size of file = ");
	sprintf(( char*)l_tempPacket,"%d",size_of_file);	
	USART_SendData_s(DEBUG_COM,l_tempPacket);
	USART_SendData_s(DEBUG_COM,(unsigned char*)"\r\n");
	
/*	
	for( k=0 ; k<=size_of_file ; k=k+EEPROM_PAGE_SIZE)
	{
		sFLASH_ReadBuffer(IAP.USART_ISR_buffer_temp, k, EEPROM_PAGE_SIZE);
				
		for(i=0 ; i<EEPROM_PAGE_SIZE ; i++)
		{
			USART_Put(DEBUG_COM,IAP.USART_ISR_buffer_temp[i]);
		}
	}*/
	


	for(i=0 ; i<size_of_file-1 ; i=i+17)
	{
		if((i+17) > size_of_file)break;
		
		EEPROM_I2C_SQ_Read( i,temp_Buffer, 17);
		for(j=0 ; j<16 ;j++)
		{
			checksum = checksum + temp_Buffer[j];
		}

		if(checksum == temp_Buffer[j])
		{		
			EEPROM_I2C_SQ_Write(array_index1, temp_Buffer, 16);
			array_index1 = array_index1 + 16;
			file_size = file_size + 16;
			checksum = 0;
			USART_SendData_s( DEBUG_COM,(unsigned char*)"* ");
		}
		else
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"Checksum Failed\r\n");
			return 1;
		}
	}
	
	temp = size_of_file - i;
	if(temp > 0)
	{
		EEPROM_I2C_SQ_Read( i,temp_Buffer, temp);
		for(j=0 ; j<temp-1 ;j++)
		{
			checksum = checksum + temp_Buffer[j];
		}
		if(checksum == temp_Buffer[j])
		{		
			EEPROM_I2C_SQ_Write(array_index1, temp_Buffer, temp-1);
			file_size = file_size + (temp-1);
			checksum = 0;
			USART_SendData_s( DEBUG_COM,(unsigned char*)"+++ ");
		}
		else
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"Checksum Failed\r\n");
			return 1;
		}
	}

	USART_SendData_s(DEBUG_COM,(unsigned char*)"\r\n Binary file size = ");
	sprintf(( char*)l_tempPacket,"%d",file_size);	
	USART_SendData_s(DEBUG_COM,l_tempPacket);
	USART_SendData_s(DEBUG_COM,(unsigned char*)"\r\n");
	
	
	/*	
	for( k=0 ; k<file_size ; k++)
	{
		USART_Put(DEBUG_COM, eeprom_read_byte(k));
		Delay_ms(1);
	}*/
	
	
	IAP.File_Size = file_size;
	IAP.write_addess_2 = 0x00;
	
	config.IAPDownloadedFlag=0xFF;
	config.IAPFileSize=IAP.File_Size;
	config.IAPFileAddress=IAP.write_addess_2;
	eeprom_write_byte(IAP_DOWNLOAD,config.IAPDownloadedFlag);
	eeprom_write_dword(IAP_FILE_SIZE,config.IAPFileSize);
	eeprom_write_dword(IAP_FILE_ADDRESS,config.IAPFileAddress);
	
	
	config.FTPSetting = FALSE;		
	config.IAPSetting = FALSE;		
	eeprom_write_byte(IAP_SETTING,config.IAPSetting);		

	if(IAP.AfterIAPBootFlag==TRUE)//Automatically reboot after sucessfull IAP Download
		{
		// Go to the bootloader application  
		USART_SendData_s( DEBUG_COM,"Jumping to Bootloader Application\r\n");
		//This function initiate a system reset request to reset the MCU.
		NVIC_SystemReset();
		}
		else//Do not Automatically reboot after sucessfull IAP Download
		{
			config.IAPReadyFlag = TRUE;
		}	
	#endif
	
	return FALSE;
}
/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void RequestProcessController (void)
{
	// one more GPS poll here can be added		
	// read gprs buffer and process the requests
//	USART_SendData_s( DEBUG_COM,"RPC-1\r\n");
	gprsRequestProcess();	
//			USART_SendData_s( DEBUG_COM,"RPC-2\r\n");
	g_smsReadCounter++;		
	// read sms and gprs buffer after 6 cycles
	if(g_smsReadCounter >= 6)
	{			
		g_smsReadCounter = 0;
		// read sms and process requests	
		SMS_Read_module();		
	}	
//	USART_SendData_s( DEBUG_COM,"RPC-3\r\n");
}

/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void GSM_StateMachine(void)
{
	// If GSM is Off, Turn on the GSM Module	
	gsmPowerOn();	
	
	
	if(states.FTPPUTConnectFlag==FALSE)
	{
//		USART_SendData_s( DEBUG_COM,FM_Ver);		
		USART_SendData_s( DEBUG_COM,"\r\n\r\n");
		// If GSM in not initialized 
		if(states.gsmFirstInit == FALSE && IAP.Data_Start_Flag==0  )
		{
			states.gsmFirstInit = GSM_Init_Reset();	
			USART_SendData_s( DEBUG_COM,"gsmFirstInit\r\n");	
		}	

		// Reload IWDG counter 
		IWDG_ReloadCounter();
		
		getIMEI();	
		
		// Check for GSM network Strength and RSSI	
		states.networkInit = GSM_Network_Check();
		
		// Check if GSM network Strength is not enough or GPRS is not connected
		if(states.networkInit == FALSE || states.GPRSStateOk == FALSE)
		{		
			states.FTPPUTConnectFlag=FALSE;
		
			USART_SendData_s( DEBUG_COM,"NT\r\n");	
			states.noNetworkCounter++;
				
			if(states.noNetworkCounter >= config.GSMNoNetworkTry)
			{
				states.GPRSStateOk = FALSE;//////////////////////////////////////////////////////////////////
				
				states.noNetworkCounter = 0;
				if(config.simShiftEn == TRUE)
				{	
					USART_SendData_s( DEBUG_COM,"SIMShift++///////////\r\n");
					config.simNumber++;
					simChangeEvent = TRUE;
					if(config.simNumber == 3)
					{
						config.simNumber = 1;
					}
					
					Timer_Init(Timer4,DISABLE);//To resolve break issue due to magnetometer()
					Factory_Config_Read(FALSE,config.simNumber);
					Timer_Init(Timer4,ENABLE);
					
					GSM_Select_SIM(config.simNumber);
				}	
	
				USART_SendData_s( DEBUG_COM,"Network ERROR:Calling gsmFirstInit()////////\r\n");	
				
				states.gsmFirstInit = FALSE;
			}
		
//			if(states.IgnitionOnFlag==TRUE)
//			{
//				STATE_MACHINE_LED_ON();
//			}
//		
//			Delay_ms(40);	
//			STATE_MACHINE_LED_OFF();
//			Delay_ms(200);	
//			if(states.IgnitionOnFlag==TRUE)
//			{
//				STATE_MACHINE_LED_ON();
//			}
//			Delay_ms(40);	
//			STATE_MACHINE_LED_OFF();
			STATE_MACHINE_LED_TOGGLE();
			
			//Server side next try delay in sec
			Delay_ms(config.GSMTryDelay*1000);
		}
		else
		{
			STATE_MACHINE_LED_TOGGLE();
//			if(states.IgnitionOnFlag==TRUE)
//			{
//				STATE_MACHINE_LED_ON();
//			}
//			Delay_ms(20);
//			STATE_MACHINE_LED_OFF();

			states.noNetworkCounter = 0;
		}
	
	
		// check GPRS Current State
		states.GPRSStateOk = Command_Send_SATE_Get();	

		
		if(config.FTPSetting==TRUE)
		{
			// check FTP Current State
			states.FTPStateOk = Command_Send_SATE_FTP_Get();	
		}
	
		// If Network Strenth is OK and GPRS is not connected
		if(states.networkInit == TRUE && states.GPRSStateOk == FALSE)
		{		
			if(Ack.LoginAckRetransmit!=2 && Command_Send_SATE_Get()==FALSE)
			{
				Ack.LoginAckRetransmit=TRUE;
			}
			
			// Connect GPRS Connection with server	
			states.GPRSInit = GPRS_initialize();	
		
			if(states.GPRSInit == FALSE)
			{			
				states.severShiftCounter++;
				USART_SendData_s( DEBUG_COM,"GT\r\n");
				
				if(states.severShiftCounter >=config.GSMIPTry)
				{
					USART_SendData_s( DEBUG_COM,"severShiftCounter+////////////\r\n");
					states.severShiftCounter = 0;
					if(gsmComm.ipSelection == 0)
					{
						gsmComm.ipSelection = 1;
					}
					else
					{
						gsmComm.ipSelection = 0;
					}
					
					// Turn OFF the GSM Module
					if(config.GSMTryRebootEnDs==TRUE && states.CallOngoingFlag==FALSE)
					{
						USART_SendData_s( DEBUG_COM,"GPRS/IP ERROR:Calling gsmPowerOff()////////\r\n");	
						gsmPowerOff();//No need to turn off to shift SIM for M95EB,This  is called to only reboot on user option
					}
					//states.resetTransmission = 1;
				}
			}
			else
			{
				states.severShiftCounter = 0;
				//USART_SendData_s( DEBUG_COM,"severShiftCounter=0:\r\n");
			}		
		
			

			//Server side next try delay in sec
			Delay_ms(config.GSMTryDelay*1000);
		}

		if(Ack.LoginAckRetransmit==TRUE && Ack.LoginPackFlag==FALSE)
		{
//			USART_SendData_s( DEBUG_COM,"Ack.LoginAckRetransmit==TRUE\r\n");/comment due to overload
					
			Ack.LoginAckRetransmit=FALSE;
			Ack.LoginPackFlag=TRUE;
		}	
		
		//GSM_Get_SIM_Number();//Comment due to overload
	}
	else
	{
		USART_SendData_s( DEBUG_COM,"TIM4 DS\r\n");
		Timer_Init(Timer4,DISABLE);
		IAP.FTPConnectCounter=0;
		
		while(states.FTPPUTConnectFlag==TRUE && IAP.FTPConnectCounter<30)// wait 10 mint for IAP download
		{
			Delay_ms(1000);
		}
		states.FTPPUTConnectFlag=FALSE;
		IAP.FTPConnectCounter=0;
		Timer_Init(Timer4,ENABLE);
		USART_SendData_s( DEBUG_COM,"TIM4 EN\r\n");
	}
	
	
	if(states.networkInit == TRUE && states.GPRSStateOk == TRUE && states.FTPStateOk == FALSE && config.FTPSetting==TRUE)// If initialization and GSM network OK
	{		
		//USART_SendData_s( DEBUG_COM,"FTP Initializing:////////////////////////\r\n");
		states.FTPPUTConnectFlag=FALSE;
		// Initialize FTP
		states.FTPInit = FTP_initialize();
		IAP.RetryCount=0;
	}
	
	if( states.FTPPUTConnectFlag==FALSE && config.IAPSetting==TRUE && states.GPRSStateOk == TRUE && states.FTPStateOk == TRUE && config.FTPSetting==TRUE && GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))
	{		
		dataTransmissionController();	//Service any pending requests
		Delay_ms(1000);
		USART_SendData_s( DEBUG_COM,"Getting IAP File from FTP Server////////////////\r\n");
		
		IAP.RetryCount++;
		config.IAPReadyFlag = FALSE;
		
		if(IAP.RetryCount>config.IAPTry)
		{
			config.firmwareErrorFlag= TRUE;
			USART_SendData_s( DEBUG_COM,"IAP Error++\r\n");
			config.FTPSetting = FALSE;		
			config.IAPSetting = FALSE;		
			eeprom_write_byte(IAP_SETTING,config.IAPSetting);		
	
			IAP.RetryCount=0;
		}
		else
		{
			#ifdef  FLASH_ENABLED
			// Configure the DMA 
			sFLASH_DMA_Configuration(IAP.USART_ISR_buffer1,EEPROM_PAGE_SIZE);
//			sFLASH_Erase4KB_32KB_64KB_Chip(IAP_FLASH_PAGE_ERASE_ADDRESS,sFLASH_CMD_ER_CHIP);
			IAP.Data_Start_Flag=FALSE;
		
			IAP.File_download_status=FALSE;
			IAP.write_addess_1=0;
			IAP.write_addess_2=0;
			IAP.write_addess_3=0;
			Command_FTP_Get(config.IAPFileName);
			#endif
		}
	}
	
	
	if(IAP.File_download_status==TRUE )
	{
		IAP.FileCheckError=IAP_convert_file();
		if(IAP.FileCheckError==TRUE)
		{
			config.IAPSetting=TRUE;
			config.FTPSetting=TRUE;
		}
		
		///Timer_Init(Timer4,ENABLE);//////////////////
		
		IAP.File_download_status=FALSE;
	}
	
	
	if(config.FTPSetting == FALSE &&  states.FTPStateOk==TRUE )
	{
		// check FTP Current State
		states.FTPStateOk = Command_Send_SATE_FTP_Get();	
		USART_SendData_s( DEBUG_COM,"AT+QFTPCLOSE\r\n");
		USART_SendData_s( GSM_COM,(unsigned char*)FTP_CLOSE);	
		Delay_ms(1000);
	}
	
	
	if(states.CallOngoingFlag==TRUE)
	{
		CheckCallStatus();
	}
}


/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void GPS_StateMachine(void)
{
	if(states.FTPPUTConnectFlag==FALSE)
	{
		#ifdef GPS_ENABLED
		GPS_Controller();// Get the GPS Valid Data Second time	
		#endif
	}
}
/**
  * @brief  Main State Machine 
  * @param  None
  * @retval None
  */
void Main_StateMachine(void)
{	
	GPS_StateMachine();///////
//	USART_SendData_s( DEBUG_COM,"1-1\r\n");
	
	DIO_StateMachine();
//	USART_SendData_s( DEBUG_COM,"1-2\r\n");
	AIO_StateMachine();
//	USART_SendData_s( DEBUG_COM,"1-3\r\n");
	
	GPS_StateMachine();//////
//	USART_SendData_s( DEBUG_COM,"1-4\r\n");
	
	GSM_StateMachine();
//	USART_SendData_s( DEBUG_COM,"1-5\r\n");
	
	GPS_StateMachine();//////
//	USART_SendData_s( DEBUG_COM,"1-6\r\n");
	
	dataTransmissionController();
//	USART_SendData_s( DEBUG_COM,"1-7\r\n");
	RequestProcessController();
//	USART_SendData_s( DEBUG_COM,"1-8\r\n");
	
	SleepModeController();	
//	USART_SendData_s( DEBUG_COM,"1-9\r\n");
}
/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
