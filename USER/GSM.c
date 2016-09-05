/**
  ******************************************************************************
  * @file    GSM.c
  * @author  M.Uzair Afzal
  * @version V1.0.0
  * @date    30-May-2013
  * @brief   This file contains all the functions prototypes for the global variables and functions.
  ******************************************************************************
*/
/*
 GSM.cpp Includes
 
 1 -- UART 0 ISR vector data communications from SIMCOM Requests, Server GPRS Requests, SMS Receiving.
 2 -- Network Check module
 3 -- RSSI Check Module
 4 -- Base station ID Check module
 5 -- SMS Sending Module
 6 -- SMS Receiving Module
 7 -- GPRS Initialization Module
 8 -- GSM Power on off module
 9 -- Sever packet ACK check module
 10 -- APN Settings module 
 */
#include "Global_Defines.h"

#define nackedBytesSize   		10
#define gprsBufferContentSize 100
#define l_smsLocationSize   	3
#define smsContentTextSize 		200
#define l_cellNumberSize 			15

unsigned char nackedBytes[nackedBytesSize];		

unsigned char rxBufferLen;
unsigned char smsRead;
unsigned char gprsBufferContent[gprsBufferContentSize];	
unsigned char l_smsLocation[l_smsLocationSize];	
unsigned char smsContentText[smsContentTextSize];	

unsigned char l_tempByte = 0;
unsigned char l_tempByteIndex = 0;
unsigned char l_tempByteIndexNumber = 0;		
unsigned char l_tempCounter = 0;

unsigned char tempSearchIndex = 6;
unsigned char l_dataCopy = 0;

unsigned char g_replyReturn;
unsigned char l_commaCounter = 0;
unsigned char l_copyIndex = 0;


unsigned char l_stringLengthACK;
unsigned char l_commaCountACK = 0;	
unsigned char l_dataCopyIndexACK = 0;
unsigned char l_searchIndexTempACK = 0;	

static unsigned char Command_Send_MCC_MNC_Get(unsigned char*);
static unsigned char Command_Send_RSSI_Get(unsigned char*);
static void GSM_Request_Process(unsigned char*,unsigned char);
static unsigned char Command_Send_OK_Get(unsigned char*);

uint8_t l_cellNumber[l_cellNumberSize];
uint8_t l_counter=0;
uint8_t l_gsmServerVoiceCallFlag=FALSE;
//const char cellnumber[]  = "+923327358515";
extern const char cell1C[];

static void SMS_Processor(void);

const char dearSt[] = "\r\nVehicle # ";
const char infSt[]  = "\r\nwww.teresol.org";
const char region[] = " region.";


uint8_t 	FTPresume_point_string[6];
uint32_t 	FTPresume_point_int=0;

/**
  * @brief  process the requests received from GPRS by the sever
  * @param  None
  * @retval None
  */
void gprsRequestProcess(void)
{	
	uint16_t copyLoop=0;
	uint16_t l_loop=0;
	
	if(gprsRequestBuffer[gprsRequestBufferIndex] != '@')// check for the valid header
	{
		arrayInit2Zero(gprsRequestBuffer,gprsRequestBufferSize);
		gprsRequestBufferIndex = 0;
	}
	else
	{
		for( copyLoop=0;copyLoop<gprsRequestBufferSize;copyLoop++)
		{
			if(gprsRequestBuffer[gprsRequestBufferIndex] == '@' && gprsRequestBuffer[gprsRequestBufferIndex+1] == '@')// check for the valid header
			{
				gprsRequestBufferIndex = gprsRequestBufferIndex+2;
//				USART_SendData_s( DEBUG_COM,"All gprsRequests:\r\n");		//Comment due to overload
//				USART_SendData_s( DEBUG_COM,(unsigned char*)gprsRequestBuffer);	
//				USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
			}
			else if(gprsRequestBuffer[gprsRequestBufferIndex] == 0x00)//if buffer end is detected
			{
//				USART_SendData_s( DEBUG_COM,"gprs break :\r\n");	//Comment due to overload	
				break;
			}
			
			gprsBufferContent[l_loop] = gprsRequestBuffer[gprsRequestBufferIndex];
			l_loop++;
			if(gprsRequestBuffer[gprsRequestBufferIndex]  == '*')// copy the request to content buffer unless the footer * is found
			{
				gprsBufferContent[l_loop]=0x00;
				if(!strncmp((const char*)gprsBufferContent,(const char*)config.gprsPassword,4))// if GPRS is verified forward the request
				{					
//					USART_SendData_s( DEBUG_COM,"Processing gprsRequest:\r\n");	//Comment due to overload
//					USART_SendData_s( DEBUG_COM,(unsigned char*)gprsBufferContent);	
//					USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
					if(gprsBufferContent[5] == 'S')// if settings are requested
					{
						//USART_SendData_s( DEBUG_COM,"gprsRequest 3 :\r\n");		
						Config_Settings_Request_Process(gprsBufferContent,GPRS);	// calls the configuring engine		
					}
					else if(gprsBufferContent[5] == 'A')// if tracking is requested
					{
						GSM_Request_Process(gprsBufferContent,GPRS);	// calls the tracking request engine		
					}	
					else if(gprsBufferContent[5] == 'C')				
					{
						Config_Settings_Request_Process(gprsBufferContent,sConfig);	// calls the tracking request engine		
					}
				}	
				l_loop=0;
				//gprsRequestBufferIndex++;
				//break;
			}
			gprsRequestBufferIndex++;
		}
	}	
}


/**
  * @brief  This module process all the tracking requests received from SMS or GPRS
  * @param  a_Content
  * @param  a_requestType
  * @retval None
  */
void GSM_Request_Process(unsigned char *a_Content,unsigned char a_requestType)
{	
	unsigned char l_requestID = 0;	
	config.sleepModeCounter = 0;		
	l_requestID = (a_Content[6] - '0') * 10 + (a_Content[7] - '0');		
	switch (l_requestID)
	{
		case TRACK_ON_DEMAND_GPRS:// Track On Demand
		{				
			//USART_SendData_s( DEBUG_COM,(char*)"TRACK ON DEMAND\r\n");						
			if(a_requestType == SMS)
			{
				DataSendStatus.smsDemand = TRUE;
			}
			else
			{
				DataSendStatus.gprsDemand = TRUE;			
			}				
			break;
		}
		case GOOGLE_TRACK_DEMAND://Track On Demand GOOGLE Maps
		{			
			//USART_SendData_s( DEBUG_COM,(char*)"TRACK ON DEMAND GOOGLE\r\n");									
			if(a_requestType == SMS)
			{
				DataSendStatus.smsDemandGoogle = TRUE;
			}			
			break;
		}		
		default:
		{
			break;
		}		
	}	
}

/**
  * @brief  This module reads the sms depending on the index locations received from modem +CMTI
  * @param  None
  * @retval None
  */
unsigned char SMS_Read_module(void)
{	
	uint16_t l_count=0;	
	if(noOfSmsRec>0)// if any sms is received 
	{
		/*COPY THE SMS LOCATION FROM BUFFER-------------------------------------------------------------------------------------------------------------------*/												
		l_smsLocation[0] = smsRecBuffer[smsRecBufferReadIndex];		
		if(smsRecBuffer[++smsRecBufferReadIndex] != ',' )
		{				
			l_smsLocation[1] = smsRecBuffer[smsRecBufferReadIndex];				
			smsRecBufferReadIndex = smsRecBufferReadIndex+2;	
			l_smsLocation[2] = 0x00;				
		}
		else
		{
			l_smsLocation[1] = 0x00;
			l_smsLocation[2] = 0x00;	
			smsRecBufferReadIndex++;		
		}		
		/*READ SMS AND PROCESS IF RECEIVED FRIM VALID NUMBER-----------------------------------------------------------------------------------------------------------*/															
		Command_Send_OK_Get((unsigned char*)"AT+CMGR=");									
		USART_SendData_s( GSM_COM,(unsigned char*)l_smsLocation);								
		USART_SendData_s( GSM_COM,"\r\n");		
		Delay_ms(1000);// 
		
		SMS_Processor();// process the sms		
		
		if(!strncmp((const char*)smsContentText,(const char*)config.smsPassword,4))// if sms password is verified
		{		
			if(smsContentText[5] == 'S') // if settings change is requested
			{
				Config_Settings_Request_Process(smsContentText,SMS);// calls setting change module	
			}
			else if(smsContentText[5] == 'A')// if tracking is requested	
			{
				GSM_Request_Process(smsContentText,SMS);// calls tracking request module		
			}						
		}		
		//USART_SendData_s( DEBUG_COM,(char*)smsReadBuffer);				
		//USART_SendData_s( DEBUG_COM,(char*)"\r\n");							
		arrayInit2Zero(smsReadBuffer,smsReadBufferSize);// wash sms read buffer				
		if(noOfSmsRec>0)				
		{
			noOfSmsRec--;	// 												
		}		
		sms.messageReceivedFlag = FALSE;			
		sms.messageRead = TRUE;		
	}
	if(sms.messageReceivedFlag == 0 && noOfSmsRec == 0 && sms.messageRead == TRUE)// if no more message are received during reading process delete all the sms
	{				                                               
		sms.messageRead = FALSE;
		smsRecBufferIndex = 0;// reset receive buffer index
		
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
		USART_SendData_s( GSM_COM,(unsigned char*)DEL_ALL_SMS);	// delete all sms
	
		while(!strstr((const char*)gsmCommandsBuffer,(const char*)"OK") && l_count<200 )
		{
			Delay_ms(10);
			l_count++;
		}		
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);

		noOfSmsRec = 0;
		smsRecBufferReadIndex = 0;	
		arrayInit2Zero(smsReadBuffer,smsReadBufferSize);// wash sms read buffer									
		arrayInit2Zero(smsRecBuffer,smsRecBufferSize);// wash sms location buffer						
	}	
	return TRUE;	
}
/**
  * @brief  This module process the valid received sms
  * @param  None
  * @retval None
  */
void SMS_Processor(void)
{		
	unsigned char l_searchLoop=0;
	l_tempByte = 0;
	l_tempByteIndex = 0;
	l_tempByteIndexNumber = 0;		
	l_tempCounter = 0;	
	
	arrayInit2Zero(sms.smsReplyNumber,smsReplyNumberSize);	
	arrayInit2Zero(smsContentText,smsContentTextSize);		
	/*VALID SENDER NUMBER EXTRACTOR-------------------------------------------------------------------*/
	//USART_SendData_s( DEBUG_COM,"smsReadBuffer:");
	//USART_SendData_s( DEBUG_COM,(char*)smsReadBuffer);				
	//USART_SendData_s( DEBUG_COM,(char*)"\r\n");
	for(  l_searchLoop=0;l_searchLoop<100;l_searchLoop++)// check for first comma location
	{
		l_tempByte = smsReadBuffer[l_tempByteIndex];
		l_tempByteIndex++;	
		if(l_tempByte == ',')
		{
			break;
		}
	}	
	l_tempByteIndex++;	
	
	for(  l_searchLoop=0;l_searchLoop<30;l_searchLoop++)// copy the  number to buffer
	{ 
		sms.smsReplyNumber[l_tempByteIndexNumber] = smsReadBuffer[l_tempByteIndex];
		l_tempByteIndexNumber++;
		l_tempByteIndex++;
		if(smsReadBuffer[l_tempByteIndex] == '"')
		{			
			sms.smsReplyNumber[l_tempByteIndexNumber] = 0x00;
			break;			
		}
		
	}
	//USART_SendData_s( DEBUG_COM,(char*)"\r\n");
	//USART_SendData_s( DEBUG_COM,(char*)sms.smsReplyNumber);
	
	/*VALID SMS CONTENT EXTRACTOR------------------------------------------------------------------------------*/	
	if( ((!strcmp((const char*)sms.smsReplyNumber,(const char*)config.ServerSMSCellNo1)) || (!strcmp((const char*)sms.smsReplyNumber,(const char*)config.ServerSMSCellNo2)) || (!strcmp((const char*)sms.smsReplyNumber,(const char*)config.ServerSMSCellNo3)))
		|| (strstr((const char*)config.ServerCallCellNo1,(const char*)"0000000") && strstr((const char*)config.ServerCallCellNo2,(const char*)"0000000") && strstr((const char*)config.ServerCallCellNo3,(const char*)"0000000"))
	)// if valid number found process message
	{	
	
		for(  l_searchLoop=0;l_searchLoop<100;l_searchLoop++)// Extract SMS from the SMS BUFFER Reads up till valid header
		{
			l_tempByte = smsReadBuffer[l_tempByteIndex];
			l_tempByteIndex++;	
			if(l_tempByte == '"')
			{
				l_tempCounter++;
				if(l_tempCounter >=5)
				{
					l_tempCounter = 0;
					break;				
				}			
			}
		}	
		l_tempByteIndexNumber = 0;	
		for(  l_searchLoop=0;l_searchLoop<160;l_searchLoop++)//reads up to 200 characters reads up to end of SMS
		{
			smsContentText[l_tempByteIndexNumber] = smsReadBuffer[l_tempByteIndex];		
			l_tempByteIndexNumber++;
			l_tempByteIndex++;		
		}
		smsContentText[l_tempByteIndexNumber] = 0x00;	

		//USART_SendData_s( DEBUG_COM,"SMS Content:");
   	//USART_SendData_s( DEBUG_COM,(char*)smsContentText);				
		//USART_SendData_s( DEBUG_COM,(char*)"\r\n");
	
	}					
}

/**
  * @brief  This module generates the sms based on the parameters passed to it
  * @param  a_SMSContent
  * @param  a_SMSNumber
  * @retval None
  */
void SMS_Generator(unsigned char *a_SMSContent,unsigned char *a_SMSNumber)
{
	//unsigned char replyReturn;	
	USART_SendData_s( GSM_COM,(unsigned char*)SMS_SEND);			
	Delay_ms(300);			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if sms reqeust to GSM has error
	{
		//replyReturn = FALSE;
		//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
	}
	else// process ahead and send sms message
	{		
		USART_SendData_s( GSM_COM,(unsigned char*)a_SMSNumber);			
		USART_SendData_s( GSM_COM,(unsigned char*)"\"\r\n");						
		Delay_ms(1000);
		if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if GPRS is connected already
		{
			//replyReturn = FALSE;
			//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
		}	
		else
		{
			USART_SendData_s( GSM_COM,(unsigned char*)a_SMSContent);	
			Delay_ms(200);
			USART_Put( GSM_COM,CRTL_Z);		
			//replyReturn = TRUE;	
		}	
	}		
	Delay_ms(4000);
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);		
}
/**
  * @brief  This module generates sms for alerts , with message formatting
  * @param  a_SMSContent
  * @param  a_SMSNumber
  * @param  msgType
  * @retval None
  */
void SMS_GeneratorAlerts(unsigned char *a_SMSContent,unsigned char *a_SMSNumber,unsigned char msgType)
{
	//unsigned char replyReturn;	
	USART_SendData_s( GSM_COM,(unsigned char*)SMS_SEND);			
	Delay_ms(300);			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if sms reqeust to GSM has error
	{
		//replyReturn = FALSE;
		//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
	}
	else// process ahead and send sms message
	{		
		USART_SendData_s( GSM_COM,(unsigned char*)a_SMSNumber);			
		USART_SendData_s( GSM_COM,(unsigned char*)"\"\r\n");						
		Delay_ms(1000);
		if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if GPRS is connected already
		{
			//replyReturn = FALSE;
			//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
		}	
		else
		{		
			if(msgType == 1)// all io sms
			{
				USART_SendData_s( GSM_COM,(unsigned char*)dearSt);//const char string1[] PROGMEM = "Dear Customer, Your vehicle #"
				//USART_SendData_s( GSM_COM,(char*)config.carRegNo);//IDN-55112				
				USART_SendData_s( GSM_COM,(unsigned char*)a_SMSContent);// Door Open
				USART_SendData_s( GSM_COM,(unsigned char*)infSt);//for more information please visit www.teresol.org		
			}
			else// geofence sms
			{
				USART_SendData_s( GSM_COM,(unsigned char*)dearSt);//const char string1[] PROGMEM = "Dear Customer, Your vehicle #"
				//USART_SendData_s( GSM_COM,(char*)config.carRegNo);//IDN-55112				
				USART_SendData_s( GSM_COM,(unsigned char*)a_SMSContent);// is inside , outside , region name
				USART_SendData_s( GSM_COM,(unsigned char*)region);// is inside , outside , region name				
				USART_SendData_s( GSM_COM,(unsigned char*)infSt);//for more information please visit www.teresol.org					
			}								
			Delay_ms(200);
				
			USART_Put( GSM_COM,CRTL_Z);		
			//replyReturn = TRUE;	
		}	
	}		
	Delay_ms(4000);
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);		
}
/**
  * @brief  This module gets IMEI number of GSM module
  * @param  None
  * @retval None
  */
void getIMEI(void)
{
	unsigned char imeiCheck;				
	unsigned char l_copyLoop=0;
	unsigned char imeiLoop=0;
	
	if(states.gsmIMEIFixed==FALSE)
	{
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);	
		arrayInit2Zero(config.IMEI,IMEISize);			
		//USART_SendData_s( DEBUG_COM,"AT+GSN\r\n");
		//USART_SendData_s( DEBUG_COM,"\r\n");
		USART_SendData_s( GSM_COM,IMEI_GET);		
		Delay_ms(100);
		///USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
		///USART_SendData_s( DEBUG_COM,"\r\n");
						
		for (  l_copyLoop=0;l_copyLoop<50;l_copyLoop++)// copy the IMEI to config manager
		{
			if(gsmCommandsBuffer[l_copyLoop]!=0x0D && gsmCommandsBuffer[l_copyLoop]!=0x00 && gsmCommandsBuffer[l_copyLoop]!=0x0A && gsmCommandsBuffer[l_copyLoop] != 'O' && gsmCommandsBuffer[l_copyLoop] != 'K')
			{
				config.IMEI[config.IMEIIndex++] = gsmCommandsBuffer[l_copyLoop];
				if(config.IMEIIndex >= 15)
				{
					config.IMEIIndex = 0;				
					break;
				}
			}	
		}	
			
		for(  imeiLoop=0;imeiLoop<15;imeiLoop++)	
		{
			if(config.IMEI[imeiLoop] >= '0' && config.IMEI[imeiLoop] <= '9')// check if imei is valid
			{	
				imeiCheck = TRUE;
			}
			else
			{
				imeiCheck = FALSE;	
				break;
			}	
		}
			
		if(imeiCheck == TRUE)
		{
			USART_SendData_s( DEBUG_COM,"IMEI# Fixed: ");
			USART_SendData_s( DEBUG_COM,(unsigned char*)config.IMEI);///
			USART_SendData_s( DEBUG_COM,"\r\n");///
			multiByteContentWrite(IMEI_START,IMEI_LEN,config.IMEI);		
			
			states.gsmIMEIFixed=TRUE;
		}
		
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);	
	}
	else
	{
//		USART_SendData_s( DEBUG_COM,"IMEI#: ");////
//		USART_SendData_s( DEBUG_COM,(unsigned char*)config.IMEI);///
//		USART_SendData_s( DEBUG_COM,"\r\n");///
	}
}
/**
  * @brief  This module select SIM 
  * @param  None
  * @retval uint8_t 
  */
void CheckCallStatus(void)
{
	//unsigned char l_functionAck;
	//uint16_t count=0;
	
	USART_SendData_s( DEBUG_COM,"AT+CPAS\r\n");

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)"AT+CPAS\r\n");	
	Delay_ms(100);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);////
	USART_SendData_s( DEBUG_COM,"\r\n");
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"+CPAS: 0"))
	{
		states.CallOngoingFlag=FALSE;
	}
}
/**
  * @brief  This module select SIM 
  * @param  None
  * @retval uint8_t 
  */
void GSM_Get_SIM_Number(void)
{
	//unsigned char l_functionAck;
	//uint16_t count=0;
	
	//USART_SendData_s( DEBUG_COM,"AT+QDSIM?\r\n");

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)"AT+QDSIM?\r\n");	
	Delay_ms(100);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);////
	USART_SendData_s( DEBUG_COM,"\r\n");
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
}
/**
  * @brief  This module select SIM 
  * @param  None
  * @retval uint8_t 
  */
void GSM_Select_SIM(uint8_t SIMNumber)
{
	//unsigned char l_functionAck;
	uint16_t count=0;

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)"AT+QDSIM?\r\n");	
	Delay_ms(100);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);////
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	if((gsmCommandsBuffer[8]=='0' && SIMNumber!=1) || (gsmCommandsBuffer[8]=='1' && SIMNumber!=2))
	{
	
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
		
		if(SIMNumber==1)
		{
			USART_SendData_s( DEBUG_COM,"AT+QDSIM=0,1\r\n");
			USART_SendData_s( GSM_COM,(unsigned char*)SELECT_SIM1);	
		}
		else if(SIMNumber==2)
		{
			USART_SendData_s( DEBUG_COM,"AT+QDSIM=1,1\r\n");
			USART_SendData_s( GSM_COM,(unsigned char*)SELECT_SIM2);	
		}
		count=0;
		while(!strstr((const char*)gsmCommandsBuffer,(const char*)"OK") && count<700 )
		{
			Delay_ms(10);
			count++;
		}	
		USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);////
		USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");

		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
		
		// Reload IWDG counter 
		IWDG_ReloadCounter();
				
		gsmPowerOff();//Power Off to shift SIM
		
		// Reload IWDG counter 
		IWDG_ReloadCounter();
				
		gsmPowerOn();//Power On to shift SIM
		
		// Reload IWDG counter 
		IWDG_ReloadCounter();
	}
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
}
/**
  * @brief  This module initialize GSM when ever GSM power is reset
  * @param  None
  * @retval uint8_t 
  */
unsigned char GSM_Init_Reset(void)
{
	unsigned char l_functionAck;
	unsigned char l_copyLoop=0;
	uint16_t deact_count=0;
	
	USART_SendData_s( DEBUG_COM,(unsigned char*)"AT\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)AT_SEND);
	USART_SendData_s( DEBUG_COM,(unsigned char*)"AT\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)AT_SEND);
	USART_SendData_s( DEBUG_COM,(unsigned char*)"ATE0\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)ECHO_OFF);	
	
	GSM_Select_SIM(config.simNumber);//////////////////////////////////////

	USART_SendData_s( DEBUG_COM,(unsigned char*)"AT\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)AT_SEND);
	USART_SendData_s( DEBUG_COM,(unsigned char*)"AT\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)AT_SEND);
	USART_SendData_s( DEBUG_COM,(unsigned char*)"ATE0\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)ECHO_OFF);	
	
	USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QIDEACT\r\n");
	USART_SendData_s( GSM_COM,(unsigned char*)GPRS_DEACT);			
	while(!strstr((const char*)gsmCommandsBuffer,(const char*)"DEACT") && deact_count<100 )
	{
		Delay_ms(10);
		deact_count++;
	}
	USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
	USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");	
	
	if(l_functionAck == TRUE)// if echo off is set sucess
	{		
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QAUDCH=2\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)LOUDSPEAKER_AUDIO);
		
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+CLIP=1\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)ENABLE_CALL_URC);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+CRC=1\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)ENABLE_CALL_RING_URC);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+CSCS=GSM\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)SET_TE_CHSET_GSM);
		
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+CREG=1\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)CREG_SET);	
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QENG=1,0\r\n");//Enable to report cell information automatically		
		l_functionAck = Command_Send_OK_Get((unsigned char*)ENG_MODE_SET);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+CMGF=1\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)ENABLE_SMS_TEXT_MODE);		
		//USART_SendData_s( DEBUG_COM,"ATS0=3\r\n");
		//l_functionAck = Command_Send_OK_Get((unsigned char*)CALL_ANS);//Automatically answering call oafter 3 rings
		
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QIFGCNT\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)GPRS_ATTATCH);	
						
		/////////////////// Jamming Functions ///////////////////////////////////////
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QJDR=1\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)ENABLE_JAMMING_DETECT);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QJDCFG=\"mnl\",16\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)SET_JAMMING_RSSI);
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+QJDCFG=\"minch\",6\r\n");
		l_functionAck = Command_Send_OK_Get((unsigned char*)SET_JAMMING_CHANNELS);
		
					
		arrayInit2Zero(gsmComm.gsmMCCMNC,gsmMCCMNCSize);	
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);		
		
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AT+CIMI\r\n");
		USART_SendData_s( GSM_COM,IMSI_GET);		
		Delay_ms(100);
		USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
		USART_SendData_s( DEBUG_COM,"\r\n");
		for (  l_copyLoop=0;l_copyLoop<50;l_copyLoop++)// copy the IMEI to config manager
		{
			if(gsmCommandsBuffer[l_copyLoop]!=0x0D && gsmCommandsBuffer[l_copyLoop]!=0x00 && gsmCommandsBuffer[l_copyLoop]!=0x0A  && gsmCommandsBuffer[l_copyLoop] != 'O' && gsmCommandsBuffer[l_copyLoop] != 'K')
			{
				config.IMSI[config.IMSIIndex++] = gsmCommandsBuffer[l_copyLoop];
				if(config.IMSIIndex >= 15)
				{
					config.IMSIIndex = 0;
					break;
				}
			}	
		}	
		USART_SendData_s( DEBUG_COM,"IMSI# ");
		USART_SendData_s( DEBUG_COM,(unsigned char*)config.IMSI);
		USART_SendData_s( DEBUG_COM,"\r\n");
		
		

		states.FTPPUTConnectFlag=FALSE;
		
		GSMConenction.gsmInitOk = l_functionAck;
	}
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);		
	return l_functionAck;
}
/**
  * @brief  This module Set GPRS APN depending on the SIM Used
  * @param  simNumber
  * @retval uint8_t 
  */
unsigned char APNSettings(unsigned char simNumber)
{
	unsigned char l_functionAck;	
	USART_SendData_s( GSM_COM,APN_SET);
	USART_Put( GSM_COM,'"');
	USART_SendData_s( GSM_COM,(unsigned char*)config.sim1APN);	
	USART_Put( GSM_COM,'"');			
	USART_Put( GSM_COM,',');	
	switch (simNumber)
	{
		case SIM1:
		{						
			if(strlen((const char*)config.sim1APNUser)>1)// if APN user name is available		
			{
				USART_Put( GSM_COM,'"');
				USART_SendData_s( GSM_COM,(unsigned char*)config.sim1APNUser);	
				USART_Put( GSM_COM,'"');
			}				
			
			USART_Put( GSM_COM,',');			
			if(strlen((const char*)config.sim1APNPassword)>1)// if APN password is available
			{
				USART_Put( GSM_COM,'"');
				USART_SendData_s( GSM_COM,(unsigned char*)config.sim1APNPassword);	
				USART_Put( GSM_COM,'"');		
			}				
			break;	
		}
		case SIM2:
		{
			if(strlen((const char*)config.sim2APNUser)>1)// if APN user name is available		
			{
				USART_Put( GSM_COM,'"');
				USART_SendData_s( GSM_COM,(unsigned char*)config.sim2APNUser);	
				USART_Put( GSM_COM,'"');
			}				
			
			USART_Put( GSM_COM,',');			
			if(strlen((const char*)config.sim2APNPassword)>1)// if APN password is available
			{
				USART_Put( GSM_COM,'"');
				USART_SendData_s( GSM_COM,(unsigned char*)config.sim2APNPassword);	
				USART_Put( GSM_COM,'"');		
			}						
			break;	
		}

		default:
			break;
	}
	
	
	USART_SendData_s( DEBUG_COM,"AT+QICSGP(APN Set)\r\n");
	l_functionAck = Command_Send_OK_Get((unsigned char*)CARRIAGE_LINE_FEED);	
	return l_functionAck;
	
}

/**
  * @brief  This module initializes FTP service
  * @param  None
  * @retval uint8_t 
  */
unsigned char FTP_initialize(void)
{
	unsigned char l_functionAck=FALSE;	
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	
	switch (config.simNumber)
	{
		case SIM1:
		case SIM2:
		{						
			if(strlen((const char*)config.ftp1User)>1)// if APN user name is available		
			{
				USART_SendData_s( DEBUG_COM,"AT+QFTPUSER1\r\n");
				USART_SendData_s( GSM_COM,(unsigned char*)FTP_USERNAME);// establish FTP connection
				USART_SendData_s( GSM_COM,(unsigned char*)config.ftp1User);	
				USART_Put( GSM_COM,'"');
				l_functionAck = Command_Send_OK_Get((unsigned char*)CARRIAGE_LINE_FEED);
			}				
						
			if(strlen((const char*)config.ftp1Passsword)>1)// if APN password is available
			{
				USART_SendData_s( DEBUG_COM,"AT+QFTPPASS1\r\n");
				USART_SendData_s( GSM_COM,(unsigned char*)FTP_USERPASS);// establish FTP connection
				USART_SendData_s( GSM_COM,(unsigned char*)config.ftp1Passsword);	
				USART_Put( GSM_COM,'"');
				l_functionAck = Command_Send_OK_Get((unsigned char*)CARRIAGE_LINE_FEED);		
			}		

			if(strlen((const char*)config.ftp1Path)>1)// if APN password is available
			{
				USART_SendData_s( DEBUG_COM,"QFTP_PATH1=");
				USART_SendData_s( GSM_COM,(unsigned char*)FTP_PATH);
				USART_SendData_s( GSM_COM,(unsigned char*)config.ftp1Path);	
				USART_SendData_s( GSM_COM,"\"\r\n");
				Delay_ms(800);		
				USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);
				USART_SendData_s( DEBUG_COM,"\r\n");	
				if(strstr((const char*)gsmCommandsBuffer,(const char*)"+QFTPPATH:0"))// 
				{
					l_functionAck = TRUE;				
				}	
			}
			if(l_functionAck == TRUE)
			{		
				USART_SendData_s( GSM_COM,(unsigned char*)REUQEST_FTP_CONNECTION);// establish FTP connection
				USART_SendData_s( GSM_COM,(unsigned char*)config.ftpServerIP1);	
				USART_Put( GSM_COM,'"');
				USART_Put( GSM_COM,',');	

				USART_SendData_s( GSM_COM,(unsigned char*)config.ftpServerPort1);	
		
				l_functionAck = Command_Send_FTPConnect_Get((unsigned char*)CARRIAGE_LINE_FEED);			
			}
			
			break;	
		}
		
		
		default:
			break;
	}	
	return l_functionAck;
}	

/**
  * @brief  This module initialize the GPRS routines and connects to server 
  * @param  None
  * @retval uint8_t 
  */
unsigned char GPRS_initialize(void)
{
	unsigned char l_functionAck;			
	l_functionAck = APNSettings(config.simNumber);
	Delay_ms(100);
	if(l_functionAck == TRUE)
	{		
		if(config.SelectTcpUdp[0] == TCP)// if TCP connection is set
		{
			USART_SendData_s( GSM_COM,(unsigned char*)REUQEST_TCP_CONNECTION);// establish TCP connection
		}			
		else // connect to UDP
		{
			USART_SendData_s( GSM_COM,(unsigned char*)REUQEST_UDP_CONNECTION);//establish UDP connection
		}		
			
		if(gsmComm.ipSelection == 0)
		{
			//USART_SendData_s( DEBUG_COM,"Connecting GPRS 1::\r\n");	////
			USART_SendData_s( GSM_COM,(unsigned char*)config.serverIP1);	
			USART_Put( GSM_COM,'"');
			USART_Put( GSM_COM,',');	
			USART_Put( GSM_COM,'"');
			USART_SendData_s( GSM_COM,(unsigned char*)config.serverPort1);	
			USART_Put( GSM_COM,'"');
		}
		else
		{
			//USART_SendData_s( DEBUG_COM,"Connecting GPRS 2::\r\n");	////
			USART_SendData_s( GSM_COM,(unsigned char*)config.serverIP2);	
			USART_Put( GSM_COM,'"');
			USART_Put( GSM_COM,',');	
			USART_Put( GSM_COM,'"');
			USART_SendData_s( GSM_COM,(unsigned char*)config.serverPort2);	
			USART_Put( GSM_COM,'"');
				
		}				
			
		l_functionAck = Command_Send_Connect_Get((unsigned char*)CARRIAGE_LINE_FEED);			
	}		
	return l_functionAck;
}	

/**
  * @brief  This module check for network connectivity and valid RSSI and get MCC and MNC from modem
  * @param  None
  * @retval uint8_t 
  */
unsigned char GSM_Network_Check(void)
{
	unsigned char l_functionAck;	
	unsigned l_commaCheck;
	unsigned char l_tempSize;
	unsigned int loop=0;
	l_functionAck = Command_Send_Network_Get((unsigned char*)NETWORK_CHECK);	
	if(l_functionAck == TRUE)
	{
		l_functionAck = Command_Send_RSSI_Get((unsigned char*)NETWORK_SIGNAL_STRENGTH);				
		Command_Send_MCC_MNC_Get((unsigned char*)ENG_MODE);	
		
		//arrayInit2Zero(gsmComm.gsmMCCMNC);
		
		l_commaCheck = 0;	
		l_tempSize = strlen((const char*)gsmComm.gsmMCCMNC);
		for(  loop=0;loop<l_tempSize;loop++)
		{
			if(gsmComm.gsmMCCMNC[loop] == ',')
			{
				l_commaCheck++;						
			}				
		}
	
		if(l_commaCheck == 3)
		{
			//USART_SendData_s( DEBUG_COM,"Comma Complete\r\n");
		}	
		else
		{
			strcpy((char*)gsmComm.gsmMCCMNC,(const char*)",,,");					
			//USART_SendData_s( DEBUG_COM,"Comma Embedded\r\n");
		}
			
	}		
	else
	{
		USART_SendData_s( DEBUG_COM,"RSSI Fail\r\n");	
		strcpy((char*)gsmComm.gsmMCCMNC,(const char*)",,,");	
	}
	
	return l_functionAck;	
}	

/**
  * @brief  This module check the state of FTP connection with the server whether connected or not
  * @param  None
  * @retval uint8_t 
  */
unsigned char Command_Send_SATE_FTP_Get(void)
{
	unsigned char replyReturn=0;

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);	
	USART_SendData_s( GSM_COM,(unsigned char*)FTP_STATE_CHECK);	
	Delay_ms(100);		
	USART_SendData_s( DEBUG_COM,"QFTP_Current_STATE=");
	USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);
	USART_SendData_s( DEBUG_COM,"\r\n");	
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"OPENED"))// if GPRS is connected already
	{
		replyReturn = TRUE;
		//USART_SendData_s( DEBUG_COM,"GPRS_STATE_CHECK CONNECT O_K\r\n");				
	}	
	else
	{
		//USART_SendData_s( DEBUG_COM,"GPRS_STATE_CHECK CONNECT P_K\r\n");	
		replyReturn = FALSE;
	}	
	
	strcpy((char*)states.FTPCurrentState,(const char*)gsmCommandsBuffer);	
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);

	GSMConenction.FTPStateOK = replyReturn;
	return replyReturn;			
}

/**
  * @brief  This module check the state of GPRS connection with the server whether connected or not
  * @param  None
  * @retval uint8_t 
  */
unsigned char Command_Send_SATE_Get(void)
{
	unsigned char replyReturn=0;
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);	
	USART_SendData_s( GSM_COM,(unsigned char*)TCP_STATE_CHECK);	
	Delay_ms(100);		
	//USART_SendData_s( DEBUG_COM,"GPRS_Current_STATE=");
	//USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);
	//USART_SendData_s( DEBUG_COM,"\r\n");	
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"CONNECT OK"))// if GPRS is connected already
	{
		replyReturn = TRUE;
		USART_SendData_s( DEBUG_COM,"CONN\r\n");				
	}	
	else
	{
		USART_SendData_s( DEBUG_COM,"DISC\r\n");	
		replyReturn = FALSE;
	}	
	
	strcpy((char*)states.GPRSCurrentState,(const char*)gsmCommandsBuffer);	
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);

	GSMConenction.gprsStateOK = replyReturn;
	return replyReturn;			
}


/**
  * @brief  This module connects module to FTP server and sends specified file to server
  * @param  None
  * @retval uint8_t 
  */
uint8_t Command_SendFTP_SEND_Get(uint8_t* filename,uint16_t bytes2Send,uint16_t totalFileBytes,uint8_t* array2send )
{
	
	unsigned char replyReturn;		
	unsigned char l_tempPacket[20];		
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	
	FTP.bytes2Send=bytes2Send;
	FTP.FileSize=totalFileBytes;
	FTP.array2Send=array2send;
	
	USART_SendData_s( DEBUG_COM,"QFTPPUT CMD:\r\n");
	
	USART_SendData_s( GSM_COM,(unsigned char*)FTP_PUT);
	USART_SendData_s( GSM_COM,(unsigned char*)filename);	
	USART_Put( GSM_COM,'"');
	USART_Put( GSM_COM,',');

	sprintf(( char*)l_tempPacket,"%d",bytes2Send);	
	
	USART_SendData_s( GSM_COM,(unsigned char*)l_tempPacket);	
	USART_Put( GSM_COM,',');
	USART_SendData_s( GSM_COM,(unsigned char*)"20\r\n");
	
	Delay_ms(200);			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if GPRS is connected already
	{
		replyReturn = FALSE;
		//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
	}	
	else
	{
		USART_SendData_s( DEBUG_COM,"Setting QFTP Connect Flag:\r\n");
	
		states.FTPPUTConnectFlag=TRUE;
		
	}
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	return replyReturn;			
}
/**
  * @brief  This module connects module to FTP server and sends specified file to server using remuse capability
  * @param  None
  * @retval uint8_t 
  */
uint8_t Command_SendFTP_CONNECT_Get(void)
{
	unsigned char replyReturn=FALSE;
	uint16_t i=0,put_count=0;
	unsigned char l_tempPacket[20];
	
	FTP.PUTConnectFlag2=FALSE;
	FTP.PUTTryAgainFlag=FALSE;
	sprintf(( char*)l_tempPacket,"%d",FTP.bytes2Send);
	Delay_ms(1000);
	//USART_SendData_s( DEBUG_COM,"Inside Connect Funtion\r\n");
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"CONNECT"))
	{
		USART_SendData_s( DEBUG_COM,"Connect Detect\r\n");
		USART_SendData_s( DEBUG_COM,"\r\n");
		USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
		USART_SendData_s( DEBUG_COM,"\r\n");
				
		for(i=0;i<FTP.bytes2Send;i++)
		{
			USART_Put( GSM_COM,FTP.array2Send[i]);
			USART_Put( DEBUG_COM,FTP.array2Send[i]);
		}
		USART_SendData_s( GSM_COM,"\r\n");
		while(!strstr((const char*)gsmCommandsBuffer,(const char*)"+QFTPPUT") && put_count<2000)
		{
			Delay_ms(300);
			put_count++;
		}
		
		USART_SendData_s( DEBUG_COM,"\r\n");
		USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
		USART_SendData_s( DEBUG_COM,"\r\n");
				
		if(strstr((const char*)gsmCommandsBuffer,(const char*)l_tempPacket))
		{
			FTP.PUTTryAgainFlag=FALSE;
			replyReturn = TRUE;
		}
		else
		{
			USART_SendData_s( DEBUG_COM,"FTP Packet Send Partially failed:");
			FTP.PUTTryAgainFlag=TRUE;
			replyReturn = FALSE;
		}
		
		if(replyReturn ==TRUE)
		{
			FTPresume_point_int+=FTP.bytes2Send;
			USART_SendData_s( DEBUG_COM,"FTP Resume point:");
			sprintf((char*)FTPresume_point_string, "%d", FTPresume_point_int);
			USART_SendData_s( DEBUG_COM,FTPresume_point_string);
			USART_SendData_s( DEBUG_COM,"\n\r");
		}
		if(FTPresume_point_int<FTP.FileSize )
		{
			arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
			USART_SendData_s( GSM_COM,"AT+QFTPCFG=3,");
			USART_SendData_s( GSM_COM,FTPresume_point_string);
			USART_Put( GSM_COM,0x0D);
			USART_Put( GSM_COM,0x0A);
			Delay_ms(800);
			if(!strstr((const char*)gsmCommandsBuffer,(const char*)"+QFTPCFG:0"))
			{
				replyReturn = FALSE;
			}
		
			USART_SendData_s( DEBUG_COM,"AT+QFTPCFG=");
			USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
			USART_SendData_s( DEBUG_COM,"\r\n");
		
			arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
				
		}		
		else if(FTPresume_point_int>=FTP.FileSize )
		{
			USART_SendData_s( DEBUG_COM,"RESUME STATE=Finished/////////////////////\r\n");
			FTPresume_point_int=0;			
		}				
				
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
		FTP.PUTConnectFlag2=TRUE;
		replyReturn = TRUE;
				
	}
	else if(strstr((const char*)gsmCommandsBuffer,(const char*)"+QFTPPUT"))
	{
		USART_SendData_s( DEBUG_COM,"Connect Fail Detect\r\n");
		USART_SendData_s( DEBUG_COM,"\r\n");
		USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
		USART_SendData_s( DEBUG_COM,"\r\n");
			
		arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
		FTP.PUTConnectFlag2=TRUE;
		replyReturn = FALSE;
				
	}
	//USART_SendData_s( DEBUG_COM,"Outside Connect Function\r\n");
	return replyReturn;
}


/**
  * @brief  Gets specified file from FTP server
  * @param  filename: file name to be get
  * @retval None
  */
uint8_t Command_FTP_Get(uint8_t* filename)
{
	
	unsigned char replyReturn;		
	//unsigned char l_tempPacket[20];		
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	
	USART_SendData_s( DEBUG_COM,"AT+QFTPGET\r\n");
	
	USART_SendData_s( GSM_COM,(unsigned char*)FTP_GET);
	USART_SendData_s( GSM_COM,(unsigned char*)filename);	
	USART_Put( GSM_COM,'"');
	USART_SendData_s( GSM_COM,(unsigned char*)"\r\n");
	
	Delay_ms(200);	
	USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
	USART_SendData_s( DEBUG_COM,"\r\n");
	if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if GPRS is connected already
	{
		replyReturn = FALSE;
	}	
	else
	{
		//USART_SendData_s( DEBUG_COM,"Setting QFTP Connect Flag:\r\n");
		states.FTPPUTConnectFlag=TRUE;
	}
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	return replyReturn;			
}

/**
  * @brief  This module is used to send data on GPRS
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_SEND_Get(unsigned char *tempArray)
{
	unsigned char replyReturn;		
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)PACKET_TRANSMITTER);			
	Delay_viaCounter(DELAY_MILLI_SEC,100);			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if error
	{
		replyReturn = FALSE;
		//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
	}	
	else
	{
		USART_SendData_s( GSM_COM,(unsigned char*)tempArray);			
		Delay_viaCounter(DELAY_MILLI_SEC,20);	
		USART_Put( GSM_COM,CRTL_Z);		
			
	}		
	Delay_viaCounter(DELAY_MILLI_SEC,100);	
	if(strstr((const char*)gsmCommandsBuffer,(const char*)ERROR))// if Error
	{
		replyReturn = FALSE;
		//USART_SendData_s( DEBUG_COM,"O_K\r\n");					
	}	
	else
	{
		replyReturn = TRUE;	
	}	
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	return replyReturn;			
}

/**
  * @brief  This module is used to receive the Acknawledge of data packets send to server
  * @param  None
  * @retval uint8_t
  */
unsigned char Command_Send_ACK_Get(void)
{
	unsigned char replyReturn = 0;		
	unsigned int nackedBytesInt;
	static unsigned int nackedBytesPrevious;	
		
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)PACKET_ACK);		
	Delay_ms(300);		
	//USART_SendData_s( DEBUG_COM,'<');	
	//USART_SendData_s( DEBUG_COM,(char*)gsmCommandsBuffer);	
	//USART_SendData_s( DEBUG_COM,'>');			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"+QISACK"))// if valid QISACK Message is received
	{
		if(gsmCommandsBuffer[9] != '0')
		{
			ackContentsExtract(gsmCommandsBuffer,nackedBytes);		
			nackedBytesInt = atoi((const char*)nackedBytes); 		
			//USART_SendData_s( DEBUG_COM,nackedBytes);						
			if(nackedBytesInt == 0)
			{
				//USART_SendData_s( DEBUG_COM,"ACK OK 0\r\n");		
				nackedBytesPrevious = nackedBytesInt;
				replyReturn = TRUE;
			}			
			else if(nackedBytesInt<nackedBytesPrevious)
			{
				//USART_SendData_s( DEBUG_COM,"ACK BYTES LESS THEN PRE\r\n");		
				nackedBytesPrevious = nackedBytesInt;
				replyReturn = TRUE;
			}
			else
			{
				//USART_SendData_s( DEBUG_COM,"ACK SAME AS PREVIOUS\r\n");		
				nackedBytesPrevious = nackedBytesInt;
				replyReturn = FALSE;
			}
		}
		else
		{
			replyReturn = FALSE;
		}			
	}	
	else
	{
		replyReturn = FALSE;
	}	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	GSMConenction.packetAckOK = replyReturn;
	return replyReturn;		
}

/**
  * @brief  extract the comma seperated number ot sent and received bytes
  * @param  a_contentArray
  * @param  a_comma1
  * @retval uint8_t
  */
unsigned char ackContentsExtract(unsigned char* a_contentArray,unsigned char* a_comma1)
{
	unsigned char searchLoop = 0;
	l_commaCountACK = 0;	
	l_dataCopyIndexACK = 0;
	l_searchIndexTempACK = 0;	
	
			
	l_stringLengthACK =  strlen((const char*)a_contentArray);				
	for ( searchLoop = 0;searchLoop<l_stringLengthACK;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCountACK++;			
			switch (l_commaCountACK)
			{				
				case 2:
				{			
					l_searchIndexTempACK = searchLoop; 
					for (l_dataCopyIndexACK = 0;l_dataCopyIndexACK<l_stringLengthACK;l_dataCopyIndexACK++)				
					{
						l_searchIndexTempACK++;
						a_comma1[l_dataCopyIndexACK] = a_contentArray[l_searchIndexTempACK];										
						if(a_comma1[l_dataCopyIndexACK] == 'O')
						{
							break;
						}
					}										
					a_comma1[l_dataCopyIndexACK] = NULL;					
					break;				
				}				
			}				
		}				
	}	
	return TRUE;
}

/**
  * @brief  This module check the OK reponse of command from GSM modem
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_OK_Get(unsigned char *tempArray)
{
	unsigned char replyReturn;
	//USART_SendData_s( DEBUG_COM,"CMD:");////		
	//USART_SendData_s( DEBUG_COM,(unsigned char*)tempArray);////
	//USART_SendData_s( DEBUG_COM,(char*)gsmCommandsBuffer);////
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)tempArray);	
	Delay_ms(100);	
			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"OK"))// if GPRS is connected already
	{
		replyReturn = TRUE;
		USART_SendData_s( DEBUG_COM,"O_K\r\n");////					
	}	
	else
	{
		USART_SendData_s( DEBUG_COM,"P_K\r\n");	////
		replyReturn = FALSE;
	}	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	return replyReturn;			
}

/**
  * @brief  This module reads the RSSI of network
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_RSSI_Get(unsigned char *tempArray)
{
	unsigned char rssiLoop=0;
	tempSearchIndex = 6;
	l_dataCopy = 0;	
	g_replyReturn = 0;
		
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)tempArray);	
	Delay_ms(100);	
	arrayInit2Zero(gsmComm.gsmRSSIAASCI,gsmRSSIAASCISize);
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"+CSQ:"))// if valid CSQ is Received
	{				
		//USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);///////
		//USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");///////
		for (l_dataCopy = 0;l_dataCopy<3;l_dataCopy++)	// copy the RSSI values to RSSI buffer			
		{			
			gsmComm.gsmRSSIAASCI[l_dataCopy] = gsmCommandsBuffer[tempSearchIndex];	
			tempSearchIndex++;									
			if(gsmComm.gsmRSSIAASCI[l_dataCopy] == ',')
			{
				break;
			}
		}
		gsmComm.gsmRSSIAASCI[l_dataCopy] = NULL;	
		
		
		for(  rssiLoop=0;rssiLoop<3;rssiLoop++)	
		{
			if((gsmComm.gsmRSSIAASCI[rssiLoop] >= '0' && gsmComm.gsmRSSIAASCI[rssiLoop] <= '9') || gsmComm.gsmRSSIAASCI[rssiLoop] == 0x00)// check if imei is valid
			{
				g_replyReturn = TRUE;
			}
			else
			{
				g_replyReturn = FALSE;	
				arrayInit2Zero(gsmComm.gsmRSSIAASCI,gsmRSSIAASCISize);
				break;
			}	
		}		
					
		//gsmComm.gsmRSSI = (gsmComm.gsmRSSIAASCI[0] - '0') * 10 + (gsmComm.gsmRSSIAASCI[1] - '0');														
	}	
	else
	{
		g_replyReturn = FALSE;
	}
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);	
	GSMConenction.rssiOK = g_replyReturn;	
	return g_replyReturn;	
}

/**
  * @brief  This module copy the valid MCC and MNC of network 
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_MCC_MNC_Get(unsigned char *tempArray)
{
	unsigned char replyReturn;
	unsigned char l_commaCounter = 0;
	unsigned char l_copyIndex = 0;	
	unsigned char l_copyLoop = 0;
	//	unsigned int l_sizeCheck = 0;
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)tempArray);		
	Delay_ms(100);		
//	USART_SendData_s( DEBUG_COM,"QENG::");
//	USART_SendData_s( DEBUG_COM,gsmCommandsBuffer);
//	USART_SendData_s( DEBUG_COM,"\r\n");
	
	arrayInit2Zero(gsmComm.gsmMCCMNC,gsmMCCMNCSize);		
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"+QENG:"))// if valid +QENG is received
	{		
		//USART_SendData_s( DEBUG_COM,"QENG_OK\r\n");					
				
		for (l_copyLoop=0;l_copyLoop<40;l_copyLoop++)// extracts the valid MNC & MCC information
		{				
			if(gsmCommandsBuffer[l_copyLoop] == ',')
			{
				l_commaCounter++;
				if(l_commaCounter >= 2)
				{
					break;
				}
			}
		}
		l_copyLoop++;			
		l_commaCounter = 0;			
		for (;l_copyLoop<40;l_copyLoop++)
		{		
			gsmComm.gsmMCCMNC[l_copyIndex++] = gsmCommandsBuffer[l_copyLoop] ;											
			if(gsmCommandsBuffer[l_copyLoop] == ',')
			{				
				l_commaCounter++;
				if(l_commaCounter >= 4)
				{
					gsmComm.gsmMCCMNC[--l_copyIndex] = 0x00;								
					break;
				}
			}			
			
		}			
		
		//USART_SendData_s( DEBUG_COM,(char*)"IDS: ");			
		//USART_SendData_s( DEBUG_COM,(char*)gsmComm.gsmMCCMNC);
		//USART_SendData_s( DEBUG_COM,(char*)"\r\n");					
		replyReturn = TRUE;
		//USART_SendData_s( DEBUG_COM,"N_OK\r\n");			
	}	
	else
	{
		strcpy((char*)gsmComm.gsmMCCMNC,(const char*)",,,");	
		replyReturn = FALSE;
	}		
		
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	GSMConenction.networkConnection = replyReturn;
	return replyReturn;		
}

/**
  * @brief  This module is used to check the network state connected or not
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_Network_Get(unsigned char *tempArray)
{
	g_replyReturn = 0;
	l_commaCounter = 0;
	l_copyIndex = 0;	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);		
	USART_SendData_s( GSM_COM,(unsigned char*)tempArray);		
	Delay_ms(100);		
	//arrayWashGSM(gsmComm.gsmBaseID);		
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"+CREG:"))// if valid CREG is received
	{	
//		USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);///////
//		USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");///////
		//USART_SendData_s( DEBUG_COM,"CRE_OK\r\n");							
		if(gsmCommandsBuffer[9] == '1')// if network is connected
		{		
			
			/*for(unsigned char l_copyLoop=11;l_copyLoop<30;l_copyLoop++)
			{
				if(gsmCommandsBuffer[l_copyLoop]!='"')// copy the lic of network
				{
					gsmComm.gsmBaseID[l_copyIndex++] = gsmCommandsBuffer[l_copyLoop] ;					
				}				
				if(gsmCommandsBuffer[l_copyLoop] == '"')
				{
					l_commaCounter++;
					if(l_commaCounter >= 4)
					{
						gsmComm.gsmBaseID[l_copyIndex++] = 0x00;
						break;
					}
				}
			}			*/
			//USART_SendData_s( DEBUG_COM,(char*)gsmComm.gsmBaseID);
			//USART_SendData_s( DEBUG_COM,(char*)"\r\n");			
			g_replyReturn = TRUE;
			//USART_SendData_s( DEBUG_COM,"N_OK\r\n");					
		}	
		else
		{			
			//strcpy((char*)gsmComm.gsmBaseID,(const char*)",");	
			g_replyReturn = FALSE;			
		}
	}	
	else
	{		
		//strcpy((char*)gsmComm.gsmBaseID,(const char*)",");	
		g_replyReturn = FALSE;
	}	
	
	//arrayInit2Zero(gsmComm.gsmBaseID);
	
	/*	l_commaCounter = FALSE;	
	l_copyIndex = strlen((const char*)gsmComm.gsmBaseID);
	for(unsigned int loop=0;loop<l_copyIndex;loop++)
	{
		if(gsmComm.gsmBaseID[loop] == ',')
		{
			l_commaCounter = TRUE;			
			break;
		}				
	}	
		
	if(l_commaCounter == FALSE)
	{
		strcpy((char*)gsmComm.gsmBaseID,(const char*)",");					
	}
	*/

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	GSMConenction.networkConnection = g_replyReturn;
	return g_replyReturn;		
}

/**
  * @brief  This module is used to check the connectivity of FTP when connection request is sent
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_FTPConnect_Get(unsigned char *tempArray)
{
	GSMConenction.gprsConnection = FALSE;			

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	//USART_SendData_s( DEBUG_COM,"QFTPOPEN sending array:");	
	//USART_SendData_s( DEBUG_COM,(unsigned char*)tempArray);
	//USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( GSM_COM,(unsigned char*)tempArray);// send GPRS connection command			
	Delay_ms(800);	
	
	//**USART_SendData_s( DEBUG_COM,"QFTPOPEN Status:");	
	//**USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);
	//**USART_SendData_s( DEBUG_COM,"\r\n");	
			
	if(strstr((const char*)gsmCommandsBuffer,(const char*)"+QFTPOPEN:0"))// if GPRS is connected already		
	{			
		GSMConenction.gprsConnection = TRUE;					
	}		
	
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	return GSMConenction.gprsConnection;	
}

/**
  * @brief  This module is used to check the connectivity of GPRS when connection request is sent
  * @param  tempArray
  * @retval uint8_t
  */
unsigned char Command_Send_Connect_Get(unsigned char *tempArray)
{
	GSMConenction.gprsConnection = FALSE;			

	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	USART_SendData_s( GSM_COM,(unsigned char*)tempArray);// send GPRS connection command			
	Delay_ms(300);	
		
	//**USART_SendData_s( DEBUG_COM,"QIOPEN Status:");	
	//**USART_SendData_s( DEBUG_COM,(unsigned char*)gsmCommandsBuffer);
	//**USART_SendData_s( DEBUG_COM,"\r\n");	
			
  if(strstr((const char*)gsmCommandsBuffer,(const char*)"ALREADY CO"))// if GPRS is connected already		
	{		
		//USART_SendData_s( DEBUG_COM,"ALREADY_CON\r\n");	
		GSMConenction.gprsConnection = TRUE;					
	}		
	else if(strstr((const char*)gsmCommandsBuffer,(const char*)"CONNECT OK"))// if GPRS is connected		
	{		
		//USART_SendData_s( DEBUG_COM,"C_OK_FIRST\r\n");	
		GSMConenction.gprsConnection = TRUE;					
	}		
	else if(strstr((const char*)gsmCommandsBuffer,(const char*)"CONNECT F"))// if GPRS is failed
	{		
		//USART_SendData_s( DEBUG_COM,"C_FAIL_FIRST\r\n");	
		
		GSMConenction.gprsConnection = FALSE;					
	}		
	arrayInit2Zero(gsmCommandsBuffer,gsmCommandsBufferSize);
	return GSMConenction.gprsConnection;	
}

/**
  * @brief  This module turns the GSM on
  * @param  None
  * @retval uint8_t
  */
unsigned char gsmPowerOn(void)
{	
	if(!GPIO_ReadInputDataBit( GSM_STATUS_PORT , GSM_STATUS_PIN))// if GSM is off turn it on
	{	
		USART_SendData_s( DEBUG_COM,"GSM Power ON/////////////\r\n");
		
		
		//Sim_Selection(config.simNumber);
		GPIO_SetBits(GSM_PWRKEY_PORT , GSM_PWRKEY_PIN);// 2.5 secs pulse
		Delay_ms(2500);
		GPIO_ResetBits(GSM_PWRKEY_PORT , GSM_PWRKEY_PIN);
		Delay_ms(5000);
//		if(states.IgnitionOnFlag==TRUE)
//		{
//			GSM_PWR_STATUS_LED_ON();
//		}
//		Delay_ms(100);
//		GSM_PWR_STATUS_LED_OFF();
//		Delay_ms(100);
//		if(states.IgnitionOnFlag==TRUE)
//		{
//			GSM_PWR_STATUS_LED_ON();
//		}
//		Delay_ms(100);
//		GSM_PWR_STATUS_LED_OFF();
		
	}
	return TRUE;		
}

/**
  * @brief  This module turns the GSM off
  * @param  None
  * @retval uint8_t
  */
unsigned char gsmPowerOff(void)
{		
	if(GPIO_ReadInputDataBit( GSM_STATUS_PORT , GSM_STATUS_PIN))// if GSM is on turn it off
	{	
		
		USART_SendData_s( DEBUG_COM,"AT+QPOWD=1\r\n");
		Command_Send_OK_Get((unsigned char*)GSM_NORMAL_POWER_DOWN);
		
//		if(states.IgnitionOnFlag==TRUE)
//		{
//			GSM_PWR_STATUS_LED_ON();
//		}
//		states.gsmFirstInit = FALSE;
//		Delay_ms(8000);
//		
//		GSM_PWR_STATUS_LED_OFF();
		
		/*GPIO_SetBits(GSM_PWRKEY_PORT , GSM_PWRKEY_PIN);// 2.5 secs pulse
		Delay_ms(2500);
		GPIO_ResetBits(GSM_PWRKEY_PORT , GSM_PWRKEY_PIN);
		Delay_ms(1000);*/
		
		
	}
	return TRUE;		
}


/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
