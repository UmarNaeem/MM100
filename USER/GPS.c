/*
 * GPS.cpp
 *
 * Created: 3/7/2012 1:34:24 AM
 * Last Update : 05/July/2012
 * Author: Qasim
 * GPS.cpp Includes
 GPS String capturing module
 GPS GPRMC and GPGGA String data extracting and processing module
 Distance calculation module
 time date conversion module
 timer module in case of no GPS fix 
*/ 


#include "Global_Defines.h"

#ifdef  GPS_ENABLED





static unsigned char GPRMC_Extraction_Module(void);/* extract GPRMC from the GPS strings */;
static unsigned char GPGGA_Extraction_Module(void);
static unsigned char GPRMC_Paramters_Extract(void);
static unsigned char GPGGA_Paramters_Extract(void);	
static double distanceCalculator(double,double,double,double);		
	
static void GPSTimeProc (void);
static void NMEAParseTime(void);
static void NMEAParseDate(void);

static unsigned char checkSumCalculator(unsigned char*,unsigned char*);

static uint8_t IsLeapYear(uint16_t year);

static unsigned char functionCall = 0;

static double lat1 = 0;
static double lat2 = 0;
static double lon1 = 0;
static double lon2 = 0;	

static unsigned char gpggaCounter = 5;

static double distance;	
static double a;	
static double c;	
static double longitude;	
static double latitude;		
//static long l_distanceGeo = 0;	
static unsigned char l_tempArrayCheckSum[5];
static unsigned char l_originalStringCheckSum[3];	
static unsigned char l_originalStringCheckSumGPGGA[3];	

static unsigned char l_stringLength;
//static unsigned char l_commaCount = 0;	
//static unsigned char l_dataCopyIndex = 0;
//static unsigned char l_searchIndexTemp = 0;			
static unsigned char l_tempStrodFunction [ 10 ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array for strtod function		
//static unsigned char l_degree = 0;		
static signed char daysInMonth [ 13 ] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
//static unsigned char l_tempBufferDisplay[6];

/**
  * @brief  This module check whether the captured coordinates are inside or outside the 8 GEO fence regions
  * @param  None
  * @retval uint8_t
  */
unsigned char geoFencingEngine(void)
{	
	uint8_t FenceNoloop=0;
	uint8_t FenceNo=0;
	int result=0;
	uint8_t FenceNoBuff[3]={0x00};
	uint8_t FenceSettingBuff[11]={0x00};
	uint8_t FenceUIDBuff[4]={0x00};

	Point.x=GPS_LOG.latFloat;
	Point.y=GPS_LOG.lonFloat;
	
	/*sprintf(( char*)G_sprintfBuffer,"%.6f",Point.x);	
	USART_SendData_s( DEBUG_COM,"\n\rPoit.x=");
	USART_SendData_s( DEBUG_COM,G_sprintfBuffer);	
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	sprintf(( char*)G_sprintfBuffer,"%.6f",Point.y);	
	USART_SendData_s( DEBUG_COM,"\n\rPoit.y=");
	USART_SendData_s( DEBUG_COM,G_sprintfBuffer);	
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	for(FenceNoloop=0;FenceNoloop<config.PolyGeoFences;FenceNoloop++)
	{
		FenceNo=FenceNoloop+1;
		switch((FenceNo))//Fence starts from 1-20
		{
			case 1:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo1Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo1UID);

				/*USART_SendData_s( DEBUG_COM,"\n\rgeo1UID2:");
				USART_SendData_s( DEBUG_COM,config.geo1UID);	
				USART_SendData_s( DEBUG_COM,"\r\n");
					
				USART_SendData_s( DEBUG_COM,"\n\rgeo1Setting2:");
				USART_SendData_s( DEBUG_COM,config.geo1Setting);	
				USART_SendData_s( DEBUG_COM,"\r\n");*/
						
				break;
			}
			case 2:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo2Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo2UID);				
				break;
			}
			case 3:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo3Setting); 
				strcpy((char*)FenceUIDBuff,(const char*)config.geo3UID);
				break;
			}
			case 4:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo4Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo4UID);
				break;
			}
			case 5:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo5Setting); 
				strcpy((char*)FenceUIDBuff,(const char*)config.geo5UID);
				break;
			}
			case 6:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo6Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo6UID);
				break;
			}
			case 7:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo7Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo7UID);
				break;
			}
			case 8:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo8Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo8UID);
				break;
			}
			case 9:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo9Setting); 
				strcpy((char*)FenceUIDBuff,(const char*)config.geo9UID);
				break;
			}
			case 10:
			{
				strcpy((char*)FenceSettingBuff,(const char*)config.geo10Setting);
				strcpy((char*)FenceUIDBuff,(const char*)config.geo10UID);				
				break;
			}
			
			default:
				break;
		}
		
		if(FenceSettingBuff[0]=='1')//If This GeoFnce Enabled
		{
			//Then Run Algo
			result=wn_PnPoly(Point,config.PolyGeoFencePoints,FenceNoloop);
			
			/*sprintf(( char*)G_sprintfBuffer,"%d",result);			
			USART_SendData_s( DEBUG_COM,"\n\rresult:");
			USART_SendData_s( DEBUG_COM,G_sprintfBuffer);	
			USART_SendData_s( DEBUG_COM,"\r\n");*/
			
			sprintf(( char*)FenceNoBuff,"%d",FenceNo);			
			/*USART_SendData_s( DEBUG_COM,"\n\rFenceNo:");
			USART_SendData_s( DEBUG_COM,FenceNoBuff);	
			USART_SendData_s( DEBUG_COM,"\r\n");*/
			
			/*USART_SendData_s( DEBUG_COM,"\n\rGPS_LOG.geoFencingType:");
			USART_SendData_s( DEBUG_COM,GPS_LOG.geoFencingType);	
			USART_SendData_s( DEBUG_COM,"\r\n");*/
		
			if(result)// If tracker inside this region
			{	
				if(!strstr((const char*)GPS_LOG.geoFencingType,(const char*)FenceNoBuff))// if tracker was not in this region before
				{					
					strcpy((char*)GPS_LOG.geoFencingType,(char*)FenceNoBuff);
					strcpy((char*)GPS_LOG.geoFenceName,(char*)FenceNoBuff);
					strcpy((char*)GPS_LOG.geoFencingUniqueID,(char*)FenceUIDBuff);
	
					GPS_LOG.geoFencingInFlag = TRUE;	// set GEO fence 1 flag for Event code generation handling
						
					if(FenceNo==1)
					{
						ioRead.geo1State = TRUE;  // last geo fence state changed flag for SMS generation handling
					}
					else if(FenceNo==2)
					{
						ioRead.geo2State = TRUE; 
					}
					else if(FenceNo==3)
					{
						ioRead.geo3State = TRUE; 
					}
					else if(FenceNo==4)
					{
						ioRead.geo4State = TRUE; 
					}
					else if(FenceNo==5)
					{
						ioRead.geo5State = TRUE; 
					}
					else if(FenceNo==6)
					{
						ioRead.geo6State = TRUE; 
					}
					else if(FenceNo==7)
					{
						ioRead.geo7State = TRUE; 
					}
					else if(FenceNo==8)
					{
						ioRead.geo8State = TRUE; 
					}
					else if(FenceNo==9)
					{
						ioRead.geo9State = TRUE; 
					}
					else if(FenceNo==10)
					{
						ioRead.geo10State = TRUE; 
					}
					
					if(FenceSettingBuff[1]=='1')//If This GeoFnce Fuel KS Enabled
					{
						if(FenceSettingBuff[4]!='0')//If KS is unchanged on Fence Enter event
						{
							ioRead.FuelKSTriggerEn= TRUE;
						}
						if(FenceSettingBuff[4]=='1')//If KS is to be Pulse triggered on Fence Enter event
						{
							ioRead.FuelKSTriggerEnter= 1;
						}
						else if(FenceSettingBuff[4]=='2')//If KS is to be CUT on Fence Enter event
						{
							ioRead.FuelKSTriggerEnter= 2;
						}
						else if(FenceSettingBuff[4]=='3')//If KS is to be MAKE on Fence Enter event
						{
							ioRead.FuelKSTriggerEnter= 3;
						}
					}
					if(FenceSettingBuff[2]=='1')//If This GeoFnce Relay1 KS Enabled
					{
						if(FenceSettingBuff[6]!='0')//If KS is unchanged on Fence Enter event
						{
							ioRead.KS1TriggerEn= TRUE;
						}
						if(FenceSettingBuff[6]=='1')//If KS is to be Pulse triggered on Fence Enter event
						{
							ioRead.KS1TriggerEnter= 1;
						}
						else if(FenceSettingBuff[6]=='2')//If KS is to be CUT on Fence Enter event
						{
							ioRead.KS1TriggerEnter= 2;
						}
						else if(FenceSettingBuff[6]=='3')//If KS is to be MAKE on Fence Enter event
						{
							ioRead.KS1TriggerEnter= 3;
						}
					}
					if(FenceSettingBuff[3]=='1')//If This GeoFnce Realy2 KS Enabled
					{
						if(FenceSettingBuff[8]!='0')//If KS is unchanged on Fence Enter event
						{
							ioRead.KS2TriggerEn= TRUE;
						}
						if(FenceSettingBuff[8]=='1')//If KS is to be Pulse triggered on Fence Enter event
						{
							ioRead.KS2TriggerEnter= 1;
						}
						else if(FenceSettingBuff[8]=='2')//If KS is to be CUT on Fence Enter event
						{
							ioRead.KS2TriggerEnter= 2;
						}
						else if(FenceSettingBuff[8]=='3')//If KS is to be MAKE on Fence Enter event
						{
							ioRead.KS2TriggerEnter= 3;
						}
					}
				}		
			}
			else if(strstr((const char*)GPS_LOG.geoFencingType,(const char*)FenceNoBuff))// tracker outside this region
			{
				
				strcpy((char*)GPS_LOG.geoFencingType,(char*)"0");
				strcpy((char*)GPS_LOG.geoFenceName,(char*)"");
				strcpy((char*)GPS_LOG.geoFencingUniqueID,(char*)"");
				
				GPS_LOG.geoFencingInFlag = 2;
				
				if(FenceNo==1)
				{
					ioRead.geo1State = FALSE;  // last geo fence state changed flag for SMS generation handling
				}
				else if(FenceNo==2)
				{
					ioRead.geo2State = FALSE; 
				}
				else if(FenceNo==3)
				{
					ioRead.geo3State = FALSE; 
				}
				else if(FenceNo==4)
				{
					ioRead.geo4State = FALSE; 
				}
				else if(FenceNo==5)
				{
					ioRead.geo5State = FALSE; 
				}
				else if(FenceNo==6)
				{
					ioRead.geo6State = FALSE; 
				}
				else if(FenceNo==7)
				{
					ioRead.geo7State = FALSE; 
				}
				else if(FenceNo==8)
				{
					ioRead.geo8State = FALSE; 
				}
				else if(FenceNo==9)
				{
					ioRead.geo9State = FALSE; 
				}
				else if(FenceNo==10)
				{
					ioRead.geo10State = FALSE; 
				}
				
				if(FenceSettingBuff[1]=='1')//If This GeoFnce Fuel KS Enabled
				{
					if(FenceSettingBuff[5]!='0')//If KS is unchanged on Fence Enter event
					{
						ioRead.FuelKSTriggerEn= TRUE;
					}
				
					if(FenceSettingBuff[5]=='1')//If KS is to be Pulse triggered on Fence Exit event
					{
						ioRead.FuelKSTriggerExit= 1;
					}
					else if(FenceSettingBuff[5]=='2')//If KS is to be CUT on Fence Exit event
					{
						ioRead.FuelKSTriggerExit= 2;
					}
					else if(FenceSettingBuff[5]=='3')//If KS is to be MAKE on Fence Exit event
					{
						ioRead.FuelKSTriggerExit= 3;
					}
				}
				if(FenceSettingBuff[2]=='1')//If This GeoFnce Relay1 KS Enabled
				{
					if(FenceSettingBuff[7]!='0')//If KS is unchanged on Fence Enter event
					{
						ioRead.KS1TriggerEn= TRUE;
					}					
					if(FenceSettingBuff[7]=='1')//If KS is to be Pulse triggered on Fence Exit event
					{
						ioRead.KS1TriggerExit= 1;
					}
					else if(FenceSettingBuff[7]=='2')//If KS is to be CUT on Fence Exit event
					{
						ioRead.KS1TriggerExit= 2;
					}
					else if(FenceSettingBuff[7]=='3')//If KS is to be MAKE on Fence Exit event
					{
						ioRead.KS1TriggerExit= 3;
					}
				}
				if(FenceSettingBuff[3]=='1')//If This GeoFnce Realy2 KS Enabled
				{
					if(FenceSettingBuff[9]!='0')//If KS is unchanged on Fence Enter event
					{
						ioRead.KS2TriggerEn= TRUE;
					}								
					if(FenceSettingBuff[9]=='1')//If KS is to be Pulse triggered on Fence Exit event
					{
						ioRead.KS2TriggerExit= 1;
					}
					else if(FenceSettingBuff[9]=='2')//If KS is to be CUT on Fence Exit event
					{
						ioRead.KS2TriggerExit= 2;
					}
					else if(FenceSettingBuff[9]=='3')//If KS is to be MAKE on Fence Exit event
					{
						ioRead.KS2TriggerExit= 3;
					}
				}
						
			}
						
		}
		
	}
	
	return TRUE;
}

/**
  * @brief  sync the local clock with GPS time in case of no GPS fix
  * @param  None
  * @retval None
  */
void Init_TimeDate(void)
{	
	TD.ss=GPS_LOG.seconds;
	TD.mm=GPS_LOG.minutes;
	TD.hh=GPS_LOG.hour;
	TD.year=GPS_LOG.year;
	TD.mo=GPS_LOG.month;
	TD.dd=GPS_LOG.day;
}

/**
  * @brief  time update in case of no GPS fix
  * @param  None
  * @retval None
  */
void TimeDate_Update(void)
{
	if(GPS_LOG.TimerSelectFlag==FALSE)//If 5sec timer update is on
	{
		//USART_SendData_s( DEBUG_COM,(unsigned char*)"5sec///\r\n");
		TD.ss = TD.ss+5;// increments 5 secs
	}
	else//if 20sec timer update is on
	{
		//USART_SendData_s( DEBUG_COM,(unsigned char*)"20sec///\r\n");
		TD.ss = TD.ss+20;// increments 20 secs
	}
	
	if (TD.ss>=60)
	{
		TD.ss=0;
		if(++TD.mm>=60)
		{
			TD.mm=0;
			if(++TD.hh>=24)
			{
				TD.hh=0;
				if(IsLeapYear(TD.year)&&(TD.mo==2))
				{
					if(++TD.dd>daysInMonth[0])
					{
						TD.dd=1;
						if(++TD.mo>12)
						{
							TD.mo=1;
							TD.year++;
						}
					}
				}
				else if(++TD.dd>daysInMonth[TD.mo])
				{
					TD.dd=1;
					if(++TD.mo>12)
					{
						TD.mo=1;
						TD.year++;
					}
				}
			}
		}
	}
}

/**
  * @brief  check if the year is leap year
  * @param  year
  * @retval uint8_t
  */
uint8_t IsLeapYear(uint16_t year)
{
	if ((year % 4) == 0)
	{
		if ((year > 1582) && ((year % 100) == 0))
		{
			if ((year % 400) == 0)
			return 1;
			else return 0;
		}
		else return 1;
	}
	else return 0; 
}

/**
  * @brief  This module controls the GPS data extracting routines including GEO Fencing check and parameters extraction
  * @param  None
  * @retval uint8_t
  */
unsigned char GPS_Controller(void)
{	
	unsigned char l_gpsFix = 0;
	double distanceTemp;	

	gpggaCounter++;
	l_gpsFix = GPRMC_Extraction_Module();	// GPRMC Extraction Module
//	USART_SendData_s( DEBUG_COM,"GPS-1\r\n");
	
	GPGGA_Extraction_Module();// GPGGA Extraction for satellite numbers	
//	USART_SendData_s( DEBUG_COM,"GPS-2\r\n");
	
	if(l_gpsFix == TRUE)// if gps is valid
	{		
		GPS_LOG.TimerFlag=FALSE;
		
		GPS_LOG.firstGpsGet = TRUE;// flag for first gps time sync
		GPS_LOG.NoFixDetectCounter=0;
		GPS_LOG.gpsNoFixCounter = 0;	
		GPS_LOG.gpsNoFixCounterMD = 0; // counters reset when GPS fixed for MD				
		if(!GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN) && GPS_LOG.speedRaw > 3 /*&& ioRead.lastMotionDetDist == TRUE*/)// if car ignition is on only then increment distance
		{
			ioRead.lastMotionDetDist = FALSE;
			if(functionCall == FALSE)
			{
				lat1 = GPS_LOG.latFloat;			
				lon1 = GPS_LOG.lonFloat;
				functionCall = TRUE;				
			}
			else if(functionCall == TRUE)
			{
				lat2 = GPS_LOG.latFloat;			
				lon2 = GPS_LOG.lonFloat;
				
				distanceTemp = distanceCalculator(lat1,lon1,lat2,lon2);// calculate the distance between the 2 lat lon points	

			
				
				/*USART_SendData_s( DEBUG_COM,(char*)"Lat1:");
				sprintf(( char*)G_sprintfBuffer,"%.6f",lat1);	
				USART_SendData_s( DEBUG_COM,(char*)G_sprintfBuffer);
				USART_SendData_s( DEBUG_COM,(char*)"\r\n");
				
				USART_SendData_s( DEBUG_COM,(char*)"Lon1:");
				sprintf(( char*)G_sprintfBuffer,"%.6f",lon1);	
				USART_SendData_s( DEBUG_COM,(char*)G_sprintfBuffer);
				USART_SendData_s( DEBUG_COM,(char*)"\r\n");
				
				USART_SendData_s( DEBUG_COM,(char*)"Lat2:");
				sprintf(( char*)G_sprintfBuffer,"%.6f",lat2);	
				USART_SendData_s( DEBUG_COM,(char*)G_sprintfBuffer);
				USART_SendData_s( DEBUG_COM,(char*)"\r\n");
				
				USART_SendData_s( DEBUG_COM,(char*)"Lon2:");
				sprintf(( char*)G_sprintfBuffer,"%.6f",lon2);	
				USART_SendData_s( DEBUG_COM,(char*)G_sprintfBuffer);
				USART_SendData_s( DEBUG_COM,(char*)"\r\n");*/
				if(distanceTemp < 30000)//if gps jump to unknown location
				{
				
					arrayInit2Zero(GPS_LOG.distanceAasci,distanceAasciSize);	
					config.distanceTotal = config.distanceTotal+distanceTemp;// distance for odometer
					
					GPS_LOG.distanceTrackTemp = GPS_LOG.distanceTrackTemp+distanceTemp;// distance for distance track	
					
					GPS_LOG.geofenceDistCheck = GPS_LOG.geofenceDistCheck + distanceTemp;
					
					if(GPS_LOG.distanceTrackTemp >= config.distanceTracking)		
					{						
						GPS_LOG.distanceTrackTemp = 0;
						GPS_LOG.distanceTrackFlag = TRUE;				
					}							
					sprintf((char*)GPS_LOG.distanceAasci, "%lu",config.distanceTotal);

					//USART_SendData_s( DEBUG_COM,(unsigned char*)"DT:");
					//USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.distanceAasci);
					//USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
					
					eeprom_write_dword(travelDistanceStart,config.distanceTotal);		
					
					lat1 = lat2;
					lon1 = lon2;
				}
				else
				{
					GPS_LOG.headingSendFlag   = FALSE;
					GPS_LOG.speedLimitFlag    = FALSE;
					GPS_LOG.geoFencingInFlag = FALSE;
					accMeter.sharpTurnEvent = FALSE; 
					GPS_LOG.overSpeedClearEvent = FALSE;
					GPS_LOG.gpsFixNoFix = FALSE;
				}	
				
					
			}		
		}			
		sprintf((char*)GPS_LOG.distanceAasci, "%lu",config.distanceTotal);	
		if(gpggaCounter >= 5)
		{			
			GPGGA_Extraction_Module();// GPGGA Extraction for satellite numbers			
			gpggaCounter = 0;
		}
		
		if(GPS_LOG.gpsFixClear == TRUE)
		{
			GPS_LOG.gpsFixClear = FALSE;
			GPS_LOG.gpsFixNoFix = TRUE;
			GPS_LOG.headingSendFlag = FALSE;//to avoid any turn event after first gps fix
		}	
					
		if(GPS_LOG.geofenceMnCount>=30)//Calculate geofence
		{
			GPS_LOG.geofenceMnCount = 0;
			geoFencingEngine();					
		}
	}
	else if(l_gpsFix == FALSE)// if gps is invalid
	{
		GPS_LOG.gpsNoFixCounter++;
		GPS_LOG.gpsNoFixCounterMD++;
		if(GPS_LOG.gpsNoFixCounter >= 20)// if no GPS after 20 tries reset The GPS
		{
			//GPS_RESET_ON();
			//Delay_ms(1000);
			//GPS_RESET_OFF();
			GPS_LOG.gpsNoFixCounter = 0;
			
			if(GPS_LOG.gpsFixClear == FALSE)
			{
				GPS_LOG.gpsFixClear = TRUE;
				GPS_LOG.gpsFixNoFix = 2;			
			}				
		}
		
		
		GPS_LOG.NoFixDetectCounter++;
		if(GPS_LOG.NoFixDetectCounter >= 5)//If really gps is no fix
		{
			functionCall = FALSE;
			GPS_LOG.NoFixDetectCounter=0;
			
			if(GPS_LOG.firstGpsGet == TRUE)	
			{
				GPS_LOG.firstGpsGet = FALSE;
				Init_TimeDate();
			
				GPS_LOG.TimerFlag=TRUE;//Start GPS time increment if GPS is no fix after First fix			
			}			
			sprintf((char*)GPS_LOG.gpsTimeDateConverted,(const char*)"%02d%02d%02d%04d%02d%02d",TD.hh,TD.mm,TD.ss,TD.year,TD.mo,TD.dd);// generate buffer for date time		
	
		}
	}		
	
	Delay_ms(20);
	return TRUE;
}

/**
  * @brief  Calculates the check sum of the received GPS string for the authentication of data received
  * @param  a_originalString
  * @param  a_CompareChecksum
  * @retval uint8_t
  */
unsigned char checkSumCalculator(unsigned char* a_originalString,unsigned char* a_CompareChecksum)
{	
	unsigned char l_packetCheckSum = 0;	
	unsigned char lengthString;	
	unsigned char l_sum_loop=0;
	lengthString = strnlen((const char*)a_originalString,100);
	lengthString = lengthString - 4;			
	for(  l_sum_loop=1;l_sum_loop<lengthString;l_sum_loop++)
	{
		l_packetCheckSum ^= a_originalString[l_sum_loop];
		//USART_Put( DEBUG_COM,a_originalString[l_sum_loop]);
	}
	sprintf((char*)l_tempArrayCheckSum,"%02X",l_packetCheckSum);				
	
	//USART_SendData_s( DEBUG_COM,"\r\n");
	//USART_SendData_s( DEBUG_COM,l_tempArrayCheckSum);
	//USART_SendData_s( DEBUG_COM,"\r\n");
	
	if(l_tempArrayCheckSum[0] == a_CompareChecksum[0] && l_tempArrayCheckSum[1] == a_CompareChecksum[1])		
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  Captures the GPRMC string from GPS hardware module
  * @param  None
  * @retval uint8_t
  */
unsigned char GPRMC_Extraction_Module(void)// extract GPRMC from the GPS strings
{
	long a = 90000;
	unsigned char l_byteReceive = 0;	
	unsigned char l_gpsValidFlag = 0;
	unsigned char l_checkSumVerify = 0;
	unsigned char l_gprmcSearch=0;
	unsigned char wait=0;
	
//	USART_SendData_s( DEBUG_COM,"GPS-3\r\n");
	for ( l_gprmcSearch=0;l_gprmcSearch<GPS_TRY;l_gprmcSearch++)// try 10 times searching the GPRMC valid string, when found breaks the loop
	{	
		arrayInit2Zero(gprmsBuffer,gprmsBufferSize);		
		gprmsBufferIndex = 0;		
		for(  wait=0;wait<100;wait++)
		{				
			l_byteReceive = USART_Get(GPS_COM);				
			if(l_byteReceive == '$')
			{
				break;
			}
			
		}		
		a--;
		if(a==0)break;
		gprmsBuffer[gprmsBufferIndex] = l_byteReceive;// copy the NMEA to buffer	
		gprmsBufferIndex++;		
		for(  wait=0;wait<100;wait++)
		{			
			l_byteReceive = USART_Get(GPS_COM);	
			gprmsBuffer[gprmsBufferIndex] = l_byteReceive;// copy the NMEA contents	
			gprmsBufferIndex++;			
			if(l_byteReceive == 0x0D || gprmsBufferIndex>=95 )
			{
				break;
			}
		}	
		gprmsBuffer[gprmsBufferIndex] = NULL;// pad NULL at the end for reference 
		
		if(gprmsBuffer[4] == 'M' && gprmsBuffer[5] == 'C') // IF valid GPRMC is received
		{
			
			//USART_SendData_s( DEBUG_COM,(unsigned char*)gprmsBuffer);
			
			l_originalStringCheckSum[0] = gprmsBuffer[gprmsBufferIndex-3];// copy string original checksum in buffer
			l_originalStringCheckSum[1] = gprmsBuffer[gprmsBufferIndex-2];	
			l_checkSumVerify = checkSumCalculator(gprmsBuffer,l_originalStringCheckSum);// calculate checksum		
			
			/*USART_SendData_s( DEBUG_COM,"\r\nTeseting my GPRMC buffer:\r\n");
			USART_SendData_s( DEBUG_COM,gprmsBuffer);
			USART_SendData_s( DEBUG_COM,"\r\n");	*/	
			//USART_Put( DEBUG_COM,l_originalStringCheckSum[0]);
			//USART_Put( DEBUG_COM,l_originalStringCheckSum[1]);							
			if(l_checkSumVerify == TRUE)
			{		
				//USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nGPRMC_CHECK_OK::");
				//USART_SendData_s( DEBUG_COM,(unsigned char*)gprmsBuffer);
				//USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
				//USART_Put( DEBUG_COM,gprmsBuffer[17]);
				if(gprmsBuffer[17] == 'A')// if GPS is connected with satellite
				{				
					//USART_SendData_s( DEBUG_COM,"Valid Status OK\r\n");
					l_gpsValidFlag = TRUE;// if valid number of satellites are in view								
				}				
				break;	
			}			
		}	
		else
		{
			l_gpsValidFlag = FALSE;		
		}					
	}	
	
	//USART_SendData_s( DEBUG_COM,"GPS-4\r\n");
	if(l_gpsValidFlag == TRUE)
	{		
		GPS_FIX_LED_OFF();

		USART_SendData_s( DEBUG_COM,(unsigned char*)"Fix\r\n");
		
		GPRMC_Paramters_Extract();// extract parameters GPS.speed, GPS.lat GPS.long from string for further processing
		
		GPS_LOG.gpsFix[0] = 'A';
		GPS_LOG.gpsFix[1] = 0x00;		
		
		return TRUE;// if valid number of satellites are in view		
	}
	else if(l_gpsValidFlag == FALSE)
	{
		GPS_FIX_LED_ON();
		USART_SendData_s( DEBUG_COM,(unsigned char*)"V\r\n");
		GPS_LOG.gpsFix[0] = 'V';
		GPS_LOG.gpsFix[1] = 0x00;			
		
		GPS_LOG.numberOfSatellite[0] = '0';	
		GPS_LOG.numberOfSatellite[1] = '0';	
		
		return FALSE;// if valid number of satellites are not available
	}
	return FALSE;
}

/**
  * @brief  Captures the GPGGA string from GPS Hardware module
  * @param  None
  * @retval uint8_t
  */
unsigned char GPGGA_Extraction_Module(void)// extract GPRMC from the GPS strings
{
	unsigned char l_byteReceive = 0;	
	unsigned char l_checkSumVerifyGPGGA;
	unsigned char l_gprmcSearch=0;
	unsigned char wait=0;
//	USART_SendData_s( DEBUG_COM,"GPS-5\r\n");
	for (l_gprmcSearch=0;l_gprmcSearch<GPS_TRY;l_gprmcSearch++)// try 10 times searching the GPGGA valid string, when found breaks the loop
	{	
		arrayInit2Zero(gprmsBuffer,gprmsBufferSize);		
		gprmsBufferIndex = RESET;			
		for(  wait=0;wait<100;wait++)
		{						
			l_byteReceive =USART_Get(GPS_COM);			
			if(l_byteReceive == '$')
			{
				break;
			}
		}			
		gprmsBuffer[gprmsBufferIndex] = l_byteReceive;	
		gprmsBufferIndex++;				
		for(  wait=0;wait<100;wait++)
		{
			l_byteReceive =USART_Get(GPS_COM);	
			gprmsBuffer[gprmsBufferIndex] = l_byteReceive;		
			gprmsBufferIndex++;		
			if(l_byteReceive == 0x0D || gprmsBufferIndex>=95 )
			{
				break;
			}
		}			
		gprmsBuffer[gprmsBufferIndex] = NULL;// pad NULL at the end for reference 
		if(gprmsBuffer[4] == 'G' && gprmsBuffer[5] == 'A') // IF GPGGA is received
		{			
			//USART_SendData_s( DEBUG_COM,(unsigned char*)gprmsBuffer);
			
			l_originalStringCheckSumGPGGA[0] = gprmsBuffer[gprmsBufferIndex-3];// copy string original checksum in buffer
			l_originalStringCheckSumGPGGA[1] = gprmsBuffer[gprmsBufferIndex-2];	
			l_checkSumVerifyGPGGA = checkSumCalculator(gprmsBuffer,l_originalStringCheckSumGPGGA);// calculate checksum		
			//USART_SendData_s( DEBUG_COM,"\r\n");
			//USART_SendData_s( GSM_COM,gprmsBuffer);
			//USART_SendData_s( DEBUG_COM,"\r\n");			
			//USART_Put( DEBUG_COM,l_originalStringCheckSumGPGGA[0]);
			//USART_Put( DEBUG_COM,l_originalStringCheckSumGPGGA[1]);							
			if(l_checkSumVerifyGPGGA == TRUE)
			{
//				USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nGPGGA_CHECK_OK::");
//				USART_SendData_s( DEBUG_COM,(unsigned char*)gprmsBuffer);
//				USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
				//USART_SendData_s( DEBUG_COM,"GPGGA_CHECK_OK\r\n");
				GPGGA_Paramters_Extract();// extract the required parameters from GPGGS string					
				//USART_SendData_s( DEBUG_COM,"\r\n*-----------------------------------------------------------------------------------------------\r\n");			
				break;
			}				
			
		}					
	}			
//	USART_SendData_s( DEBUG_COM,"GPS-6\r\n");
	return TRUE;
}

/**
  * @brief  extract parameters from GPRMC string, including Latitude, Longitude, heading speed time date
  * @param  None
  * @retval uint8_t
  */
unsigned char GPRMC_Paramters_Extract(void)
{
	unsigned char searchLoop = 0;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;		
	unsigned char l_degree = 0;					
	l_stringLength =  strnlen((const char*)gprmsBuffer,100);	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(gprmsBuffer[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{	
				case TIME_LOC:
				{					
					//USART_SendData_s( GSM_COM,(char*)"\r\n*TIME_COMMAS:");
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<10;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						GPS_LOG.gpsTime[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
						if(GPS_LOG.gpsTime[l_dataCopyIndex] == '.')
						{
							break;
						}
					}										
					GPS_LOG.gpsTime[l_dataCopyIndex] = NULL;
					NMEAParseTime();// converts the time to standard time					
					//ltoa(GPS.hour,l_tempBufferDisplay,10);  // convert decimal to aasci					
					//USART_SendData_s( GSM_COM,(char*)l_tempBufferDisplay);	
					//USART_Put( GSM_COM,'|');												
					//ltoa(GPS.minutes,l_tempBufferDisplay,10);  // convert decimal to aasci					
					//USART_SendData_s( GSM_COM,(char*)l_tempBufferDisplay);	
					//USART_Put( GSM_COM,'|');												
					//ltoa(GPS.seconds,l_tempBufferDisplay,10);  // convert decimal to aasci					
					//USART_SendData_s( GSM_COM,(char*)l_tempBufferDisplay);	
					//USART_Put( GSM_COM,'|');											
					//USART_SendData_s( GSM_COM,(char*)GPS.gpsTime);					
					//USART_Put( GSM_COM,'|');					
					//timeConverter();
					//**USART_SendData_s( DEBUG_COM,"GPS_Time::");
					//**USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.gpsTime);
					//**USART_SendData_s( DEBUG_COM,"\r\n");
					break;				
				}
												
				case LATTITUDE_LOC://if comma count is 3 extract GPS.latitude from string
				{				
					//USART_SendData_s( GSM_COM,(char*)"\r\n*LAT_COMMAS:");
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<10;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						GPS_LOG.lat[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
						if(GPS_LOG.lat[l_dataCopyIndex] == ',')
						{
							break;
						}
					}
					GPS_LOG.latDirection[0] = gprmsBuffer[l_searchIndexTemp+1];// copy the GPS.lat direction in variable
					//USART_SendData_s( GSM_COM,GPS.latDirection[0]);					
					GPS_LOG.lat[l_dataCopyIndex] = NULL;					
					strcpy((char*)GPS_LOG.latOrig,(const char*)GPS_LOG.lat);													
					GPS_LOG.latOrig[14] = 0x00;
					//USART_SendData_s( DEBUG_COM,(char*)GPS.latOrig);					
					//USART_Put( DEBUG_COM,'|');					
					l_degree = (GPS_LOG.lat[0] - '0') * 10 + (GPS_LOG.lat[1] - '0');
					GPS_LOG.lat[0] = '0';
					GPS_LOG.lat[1] = '0';					
					GPS_LOG.latFloat = (strtod ((const char*)GPS_LOG.lat, (char**)&l_tempStrodFunction ) / (double)60.0) + (double)l_degree;					
					if(GPS_LOG.latDirection[0] == 'S')
					{
						GPS_LOG.latFloat *= (double)-1.0;
					}
					
					if(GPS_LOG.latFloat>=-90 && GPS_LOG.latFloat<=90)
					{ 
						if(GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))//If ignition is Off
						{
							if(GPS_LOG.IgnitionOffSaveFlag==FALSE || GPS_LOG.MDLatLongUpdateFlag == TRUE)//Save only one time to avoid clustring points
							{
								sprintf((char*)GPS_LOG.latDecimalAASCI,"%.6f",GPS_LOG.latFloat);		
							}
							
						}
						else if(!GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))//If ignition is On
						{						
							sprintf((char*)GPS_LOG.latDecimalAASCI,"%.6f",GPS_LOG.latFloat);	
						}							
					}	
					
					//**USART_SendData_s( DEBUG_COM,"GPS_Lat::");
					//**USART_Put( DEBUG_COM,GPS_LOG.latDirection[0]);
					//**USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.latOrig);
					//**USART_SendData_s( DEBUG_COM,"\r\n");
					
					break;
				}
				case LONGITUDE_LOC://if comma count is 5 extract GPS.longitude from string
				{
					//USART_Put( GSM_COM,l_commaCount);
					//USART_SendData_s( GSM_COM,(char*)"\r\n*LON_COMMAS:");
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<10;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						GPS_LOG.lon[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
						if(GPS_LOG.lon[l_dataCopyIndex] == ',')
						{
							break;
						}
					}
					GPS_LOG.lonDirection[0] = gprmsBuffer[l_searchIndexTemp+2];// copy the GPS.lon direction in variable
					//USART_Put( GSM_COM,GPS.lonDirection[0]);				
					GPS_LOG.lon[l_dataCopyIndex] = NULL;					
					strcpy((char*)GPS_LOG.lonOrig,(const char*)GPS_LOG.lon);																						
					GPS_LOG.lonOrig[14] = 0x00;
					//USART_SendData_s( GSM_COM,(char*)GPS.lonOrig);								
					//USART_Put( GSM_COM,'|');								
					l_degree = (GPS_LOG.lon[0] - '0') * 100 + (GPS_LOG.lon[1] - '0') * 10 + (GPS_LOG.lon[2] - '0');
					GPS_LOG.lon[0] = '0';
					GPS_LOG.lon[1] = '0';
					GPS_LOG.lon[2] = '0';										
					GPS_LOG.lonFloat = (strtod ((const char*)GPS_LOG.lon, (char**)&l_tempStrodFunction ) / (double)60.0) + (double)l_degree;					
					
					if(GPS_LOG.lonDirection[0] == 'W')
					{
						GPS_LOG.lonFloat *= (double)-1.0;
					}	
					
					if(GPS_LOG.lonFloat>=-180 && GPS_LOG.lonFloat<=180)				
					{
						if(GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))//If ignition is Off
						{
							if(GPS_LOG.IgnitionOffSaveFlag==FALSE || GPS_LOG.MDLatLongUpdateFlag == TRUE)//Save only one time to avoid clustring points
							{
								sprintf((char*)GPS_LOG.lonDecimalAASCI,"%.6f",GPS_LOG.lonFloat);		
								GPS_LOG.IgnitionOffSaveFlag=TRUE;
								GPS_LOG.MDLatLongUpdateFlag = FALSE;
							}
							
						}
						else if(!GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN))//If ignition is On
						{						
							sprintf((char*)GPS_LOG.lonDecimalAASCI,"%.6f",GPS_LOG.lonFloat);	
						}		
																										
					}	

					//**USART_SendData_s( DEBUG_COM,"GPS_Long::");
					//**USART_Put( DEBUG_COM,GPS_LOG.lonDirection[0]);
					//**USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.lonOrig);
					//**USART_SendData_s( DEBUG_COM,"\r\n");
					
					break;									
				}
				case SPEED_LOC:// extract GPS.speed from the string
				{
					//USART_Put( GSM_COM,l_commaCount);
					//USART_SendData_s( GSM_COM,(char*)"\r\n*SPEED_COMMAS:");
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<5;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						GPS_LOG.speed[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
						if(GPS_LOG.speed[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					GPS_LOG.speed[l_dataCopyIndex] = NULL;										
					
					GPS_LOG.speedFloat = (strtod((const char*)GPS_LOG.speed,(char**)&l_tempStrodFunction));																					
					
					GPS_LOG.speedRaw = GPS_LOG.speedFloat;		
									
					GPS_LOG.speedFloat = GPS_LOG.speedFloat*1.85;// knots to KMh	
										
					if(GPS_LOG.speedFloat>=config.speedLimit && GPS_LOG.speedLimitFlagClear == FALSE)
					{
						GPS_LOG.speedFloatReSend = GPS_LOG.speedFloat;									
						GPS_LOG.speedLimitFlag = TRUE;
						GPS_LOG.speedLimitFlagClear = TRUE;
					}
					else if(GPS_LOG.speedFloat<config.speedLimit && GPS_LOG.speedLimitFlagClear == TRUE)
					{
						GPS_LOG.speedLimitFlagClear = FALSE;
						GPS_LOG.overSpeedClearEvent = TRUE;
					}
					else if(GPS_LOG.speedFloat>=config.speedLimit && abs(GPS_LOG.speedFloatReSend-GPS_LOG.speedFloat)>config.overSpeedDiffA)
					{
						GPS_LOG.speedLimitFlagClear = FALSE;
					}	
							
																												
					if(GPS_LOG.speedFloat<=200)
					{
						sprintf((char*)GPS_LOG.speed,"%.0f",(double)GPS_LOG.speedFloat);	
					}						
															
										
					if((ioRead.FuelrelayStateChangeFlag == TRUE || ioRead.relay1StateChangeFlag == TRUE || ioRead.relay2StateChangeFlag == TRUE) && GPS_LOG.speedFloat <= ioRead.relaySwitchSpeedInteger)	// do changed gere
					{
						if(ioRead.FuelrelayState == TRUE)//Fuel relay cut
						{
							config.FuelrelayCutFlag = TRUE;
							FUEL_KS_CUT;
						}
						else if(ioRead.FuelrelayState == FALSE)//Fuel relay make
						{
							config.FuelrelayCutFlag = 2;
							FUEL_KS_MAKE;
						}
						
						if(ioRead.relay1State == TRUE)//relay1 cut
						{
							config.relay1CutFlag = TRUE;
							RELAY1_CUT;
						}
						else if(ioRead.relay1State == FALSE)//relay1 make
						{
							config.relay1CutFlag = 2;
							RELAY1_MAKE;
						}
						
						if(ioRead.relay2State == TRUE)//relay2 cut
						{
							config.relay2CutFlag = TRUE;
							RELAY2_CUT;
						}
						else if(ioRead.relay2State == FALSE)//relay2 make
						{
							config.relay2CutFlag = 2;
							RELAY2_MAKE;							
						}				
						
						ioRead.relay1StateChangeFlag = FALSE;													
						ioRead.relay2StateChangeFlag = FALSE;											
					}			
																		
					//USART_SendData_s( GSM_COM,(char*)GPS.speed);	
					//**USART_SendData_s( DEBUG_COM,"GPS_Speed::");
					//**USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.speed);
					//**USART_SendData_s( DEBUG_COM,"\r\n");
					break;									
				}
				case DATE_LOC:// extract GPS.speed from the string
				{
					//USART_Put( GSM_COM,l_commaCount);
					//USART_SendData_s( GSM_COM,(char*)"\r\n*Date_COMMAS:");
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<8;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						GPS_LOG.gpsDate[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
						if(GPS_LOG.gpsDate[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					GPS_LOG.gpsDate[l_dataCopyIndex] = NULL;
					
					NMEAParseDate();	
					
					GPSTimeProc();
					 
					//ltoa(GPS.day,l_tempBufferDisplay,10);  // convert decimal to aasci					
					//USART_SendData_s( GSM_COM,(char*)l_tempBufferDisplay);	
					//USART_Put( GSM_COM,'|');												
					//ltoa(GPS.month,l_tempBufferDisplay,10);  // convert decimal to aasci					
					//USART_SendData_s( GSM_COM,(char*)l_tempBufferDisplay);	
					//USART_Put( GSM_COM,'|');												
					//ltoa(GPS.year,l_tempBufferDisplay,10);  // convert decimal to aasci					
					//USART_SendData_s( GSM_COM,(char*)l_tempBufferDisplay);	
					//USART_Put( GSM_COM,'|');					
					
					
					//USART_SendData_s( GSM_COM,(char*)GPS.gpsDate);							
					//USART_Put( GSM_COM,'|');																			
					//USART_SendData_s( GSM_COM,(char*)"\r\n");
					break;									
				}
				case HEADING_LOC:// extract GPS.speed from the string
				{					
					//unsigned char headingDiff[6];					
					//USART_Put( GSM_COM,l_commaCount);
					//USART_SendData_s( GSM_COM,(char*)"\r\n*Heading_COMMAS:");
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<8;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						GPS_LOG.gpsHeading[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
						if(GPS_LOG.gpsHeading[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					GPS_LOG.gpsHeading[l_dataCopyIndex] = NULL;	

					//**USART_SendData_s( DEBUG_COM,"GPS_Heading::");
					//**USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.gpsHeading);
					//**USART_SendData_s( DEBUG_COM,"\r\n");					
					//USART_SendData_s( GSM_COM,(char*)GPS.gpsHeading);							
					//USART_Put( GSM_COM,'|');					
					
					//USART_SendData_s( DEBUG_COM,gprmsBuffer);
					
					//GPS.gpsHeadingFloatTemp = GPS.gpsHeadingFloat;						
							
					GPS_LOG.gpsHeadingFloat = (strtod((const char*)GPS_LOG.gpsHeading,(char**)&l_tempStrodFunction));							
					
					//sprintf((char*)headingDiff,"%.0f",(double)GPS.gpsHeadingFloatDifference);						
					if(GPS_LOG.gpsHeadingFloat<=360)
					{
						sprintf((char*)GPS_LOG.gpsHeading,"%.0f",(double)GPS_LOG.gpsHeadingFloat);						
					}	
					
					GPS_LOG.gpsHeadingFloatDifference = abs(GPS_LOG.gpsHeadingLastMaintain-GPS_LOG.gpsHeadingFloat);						
					if(GPS_LOG.gpsHeadingFloatDifference>=config.headingChange && GPS_LOG.gpsHeadingFloatDifference <= 330 &&
						 GPS_LOG.speedFloat>=SpeedLimitHeading && (ioRead.lastMotionDetected == TRUE || !GPIO_ReadInputDataBit( CAR_IGNITION_ON_PORT,CAR_IGNITION_ON_PIN )))
					{
						GPS_LOG.headingSendFlag = TRUE;
						ioRead.lastMotionDetected = FALSE;								
						GPS_LOG.gpsHeadingLastMaintain = GPS_LOG.gpsHeadingFloat;
						
						strcpy((char*)GPS_LOG.Heading_latDecimalAASCI,(char*)GPS_LOG.latDecimalAASCI);
						strcpy((char*)GPS_LOG.Heading_lonDecimalAASCI,(char*)GPS_LOG.lonDecimalAASCI);
					
						//USART_SendData_s( DEBUG_COM,"H SEND OK\r\n");
						
//						dataTransmissionController();
//						Delay_ms(500);
					}
								
										
					
					/*USART_SendData_s( DEBUG_COM,"Heading Float: ");
					USART_SendData_s( DEBUG_COM,(char*)GPS_LOG.gpsHeading);
					USART_SendData_s( DEBUG_COM,"\r\n");	*/																						
			
					break;									
				}
			}
		}			
	}	
	return TRUE;
}

/**
  * @brief  convert AASCI time into integer values
  * @param  None
  * @retval None
  */
void NMEAParseTime(void)
{
	GPS_LOG.hour = (GPS_LOG.gpsTime[0] - '0') * 10 + (GPS_LOG.gpsTime[1] - '0');
	GPS_LOG.minutes = (GPS_LOG.gpsTime[2] - '0') * 10 + (GPS_LOG.gpsTime[3] - '0');
	GPS_LOG.seconds = (GPS_LOG.gpsTime[4] - '0') * 10 + (GPS_LOG.gpsTime[5] - '0');
}

/**
  * @brief  convert AASCI Date into integer values
  * @param  None
  * @retval None
  */
void NMEAParseDate(void)
{
	GPS_LOG.day = (GPS_LOG.gpsDate[0] - '0') * 10 + (GPS_LOG.gpsDate[1] - '0');
	GPS_LOG.month = (GPS_LOG.gpsDate[2] - '0') * 10 + (GPS_LOG.gpsDate[3] - '0');
	GPS_LOG.year = (GPS_LOG.gpsDate[4] - '0') * 10 + (GPS_LOG.gpsDate[5] - '0');
}

/**
  * @brief  convert GPS time into PST time
  * @param  None
  * @retval None
  */
void GPSTimeProc(void)
{	
	signed char gDay = GPS_LOG.day;
	signed char gMonth = GPS_LOG.month;
	signed int gYear = GPS_LOG.year;
	signed char gHour = GPS_LOG.hour;
	signed char hHere;

	if ( GPS_LOG.year % 4 == 0 )
	{
		daysInMonth[2] = 29;
	}

	hHere = gHour + config.timeZoneSettings;

	if ( hHere < 0 )
	{
		gDay -= 1;
	}
	else if ( hHere >= 24 )
	{
		gDay += 1;
	}

	hHere += 24;
	hHere %= 24;

	if ( gDay > daysInMonth [ gMonth ] )
	{
		gDay = 1;
		gMonth += 1;
	}
	else if ( gDay == 0 )
	{
		gMonth -= 1;
		gDay = daysInMonth [ gMonth ];
	}

	if ( gMonth == 0 )
	{
		gMonth = 12;
		gYear -= 1;
	}
	else if ( gMonth == 13 )
	{
		gMonth = 1;
		gYear += 1;
	}

	GPS_LOG.day = gDay;
	GPS_LOG.month = gMonth;
	GPS_LOG.year = gYear + 2000;
	GPS_LOG.hour = hHere;	
	
	arrayInit2Zero(GPS_LOG.gpsTimeDateConverted,gpsTimeDateConvertedSize);	
	sprintf((char*)GPS_LOG.gpsTimeDateConverted,"%02d%02d%02d%04d%02d%02d",GPS_LOG.hour,GPS_LOG.minutes,GPS_LOG.seconds,GPS_LOG.year,GPS_LOG.month,GPS_LOG.day);// generate buffer for date time
	
	//USART_SendData_s( DEBUG_COM,(char*)GPS.gpsTimeDateConverted);	
}

/**
  * @brief  Extracts number of satellites connected
  * @param  None
  * @retval uint8_t
  */
unsigned char GPGGA_Paramters_Extract(void)
{	
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;		
	unsigned char validGPGGA = 0;		
	unsigned char searchLoop = 0;	
	
	l_stringLength =  strnlen((const char*)gprmsBuffer,100);	
	for (searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(gprmsBuffer[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{
				case 6://if comma count is 6 check for valid data
				{				
					l_searchIndexTemp = searchLoop; 
					l_searchIndexTemp++;
					validGPGGA = gprmsBuffer[l_searchIndexTemp];									
					break;
				}				
				case 7://if comma count is 7 extract satellite number from string
				{				
					if(validGPGGA != '0')
					{					
						l_searchIndexTemp = searchLoop; 
						for (l_dataCopyIndex = 0;l_dataCopyIndex<10;l_dataCopyIndex++)// copy the number of satellites to buffer				
						{
							l_searchIndexTemp++;
							GPS_LOG.numberOfSatellite[l_dataCopyIndex] = gprmsBuffer[l_searchIndexTemp];										
							if(GPS_LOG.numberOfSatellite[l_dataCopyIndex] == ',')
							{
								break;
							}
						}										
						GPS_LOG.numberOfSatellite[l_dataCopyIndex] = NULL;									
					}							
					break;
				}				
			}
		}			
	}	
	//**USART_SendData_s( DEBUG_COM,"GPS_NoOfSat::");
	//**USART_SendData_s( DEBUG_COM,(unsigned char*)GPS_LOG.numberOfSatellite);
	//**USART_SendData_s( DEBUG_COM,"\r\n");
	return TRUE;
}

/**
  * @brief  Calculates the distance between 2 latitude and longitude , using the haver sine formula
  * @param  lat1
  * @param  lon1
  * @param  lat2
  * @param  lon2
  * @retval double
  */
double distanceCalculator(double lat1, double lon1,double lat2, double lon2)
{	
	lat1 = lat1*(M_PI/180);// radian degrees conversion.
	lat2 = lat2*(M_PI/180);
	lon1 = lon1*(M_PI/180);
	lon2 = lon2*(M_PI/180);	

	//dGPS.lon = (GPS.lon2-GPS.lon1)*cos((GPS.lat1+GPS.lat2)/2);
	//dGPS.lat = (GPS.lat2-GPS.lat1);
	//distance = sqrt(dGPS.lon*dGPS.lon + dGPS.lat*dGPS.lat)*6371;	
	latitude = lat2-lat1;
	longitude = lon2-lon1;	
	a =  (sin(latitude/2))*(sin(latitude/2))+cos(lat1)*cos(lat2)*(sin(longitude/2))*(sin(longitude/2));
	c = 2*(atan2(sqrt(a),sqrt(1-a)));	
	distance = 1.039*(6371000*c);	// convert the distance in meters
		
	//USART_SendData_s( DEBUG_COM,(char*)GPS.distanceAasci);
	//USART_SendData_s( DEBUG_COM,(char*)"\r\n");		
	return distance;	
}
/**
  * @brief  polygon Geofences
  * @param  lat1
  * @param  lon1
  * @param  lat2
  * @param  lon2
  * @retval double
  */
#endif



// a Point is defined by its coordinates {int x, y;}
//===================================================================
// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"

double isLeft( GeoFencePoint P0, GeoFencePoint P1, GeoFencePoint P2 )
{
	double temp=0;
	//1000000 factor is used only for .6f conversion to int64
  temp= ((((P1.x*1000000) - (P0.x*1000000)) * ((P2.y*1000000) - (P0.y*1000000))) - 
	       (((P2.x*1000000) - (P0.x*1000000)) * ((P1.y*1000000) - (P0.y*1000000))));
	
	return temp;
}
//===================================================================
//===================================================================
// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)

int8_t wn_PnPoly( GeoFencePoint P, int8_t TotalFencePoints , int8_t FenceNo)
{
	//signed char buffer[30];
	int8_t  wn = 0,i=0;    // the  winding number counter

	
	/*sprintf(( char*)buffer,"%.0f",P.x);			
	USART_SendData_s( DEBUG_COM,"\n\rP.x:");
	USART_SendData_s( DEBUG_COM,buffer);	
	USART_SendData_s( DEBUG_COM,"\r\n");
	
	sprintf(( char*)buffer,"%.0f",P.y);			
	USART_SendData_s( DEBUG_COM,"\n\rP.y:");
	USART_SendData_s( DEBUG_COM,buffer);	
	USART_SendData_s( DEBUG_COM,"\r\n");*/
	
	Vertex[FenceNo][10].x=Vertex[FenceNo][0].x;//Copy Point 1 to point 11 also to complete Polygon
	Vertex[FenceNo][10].y=Vertex[FenceNo][0].y;
	
  // loop through all edges of the polygon
  for ( i=0; i<(TotalFencePoints); i++) 
	{
		
		/*sprintf(( char*)buffer,"%.6f",Vertex[FenceNo][i].x);			
		USART_SendData_s( DEBUG_COM,"\n\rVertex[i].x:");
		USART_SendData_s( DEBUG_COM,buffer);	
		USART_SendData_s( DEBUG_COM,"       ");
		
		sprintf(( char*)buffer,"%.6f",Vertex[FenceNo][i].y);			
		USART_SendData_s( DEBUG_COM,"\n\rVertex[i].y:");
		USART_SendData_s( DEBUG_COM,buffer);	
		USART_SendData_s( DEBUG_COM,"\r\n");*/
		
		// edge from Point[i] to  Point[i+1]; 1000000 factor is used only for .6f conversion to int64
    if ((Vertex[FenceNo][i].y*1000000) <= (P.y*1000000)) 
		{       
			// start y <= P.y
      if ((Vertex[FenceNo][i+1].y*1000000)  > (P.y*1000000))      // an upward crossing
			{
        if (isLeft( Vertex[FenceNo][i], Vertex[FenceNo][i+1], P) > 0)  // P left of  edge
				{
					++wn;            // have  a valid up intersect
				}
			}
    }
		
    else 
		{ 
			// start y > P.y (no test needed)
			if ((Vertex[FenceNo][i+1].y*1000000)  <= (P.y*1000000))     // a downward crossing
			{
        if (isLeft( Vertex[FenceNo][i], Vertex[FenceNo][i+1], P) < 0)  // P right of  edge
				{
					--wn;            // have  a valid down intersect
        }
			}
    }
	}
	
  return wn;
}
//===================================================================
/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
