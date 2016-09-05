/*
 * Configuration.c
 *
 * Created: 1/28/2012 3:06:57 PM
 * Author: Qasim
 Last Update : 05:July_2012
 
 GPS_Tracker.cpp Includes
 
 1 -- Factory settings module
 2 -- GPRS, SMS , Serial configurations module
 3 -- Firmware data packet receiving module
 4 -- factory settings EEPROM writing , Reading module
 5 -- Data content conversion and writing modules for EEPROM
 6 -- Main state Machine
 
 */
#include "Global_Defines.h"
void requestReplyGenUserData(unsigned char* a_content,uint8_t CmdStatus);
static void requestReplyGenOk(unsigned char*);
static void requestContentExtract(unsigned char*,unsigned char*);
static void dataExtractor3Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3);
void Page_Write_Module(unsigned char *Array, uint16_t PageNumber);
static void dataExtractor2Comma(unsigned char*,unsigned char*,unsigned char*);
static double aasciToFloatCopy(unsigned char*);
static void dataExtractor4Comma(unsigned char*,unsigned char*,unsigned char*,unsigned char*,unsigned char*);
void dataExtractor5Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5);
void dataExtractor6Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5,unsigned char* a_comma6);
void dataExtractor7Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5,unsigned char* a_comma6,unsigned char* a_comma7);
void dataExtractor8Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5,unsigned char* a_comma6,unsigned char* a_comma7,unsigned char* a_comma8);
static float eepromReadFloat(int address);
static void eepromWriteFloat(int address, float valuee);
static void requestReplyConfig(unsigned char* a_content);
static void firmVerGen(unsigned char* a_content);
//static void firmwarePageAck(unsigned char* a_content);
//static unsigned char checkSumCalculatorConfig(unsigned char* a_originalString,unsigned char* a_CompareChecksum);
void GeoDataAppendToBuffer(uint8_t  FenceNo);
static void requestReplyGenError(unsigned char* a_content);
static void requestReplyErrorConfig(unsigned char* a_content);
void AppendToBuffer(uint32_t  ConfigValue);
extern USART_TypeDef* COM_NO[COMn];

#define l_requestContentSize   700
#define l_tempBufferSize   60
#define l_firmwareContentSize   70
#define EEPROMTempPageSize   6
#define tempIONumberAAsciSize   2
#define tempIODataSize   7
#define latBufferSize   12
#define lonBufferSize   12

#define radiusBufferSize   12
#define headerTypeAAsciSize   2
#define speedAAsciSize   4
#define relaySwitchDataSize   4

#define tempHeaderBufferSize   11
#define headerTypeAAsciSize   2
#define numberTypeAAsciSize   2

unsigned int intEEPIndex = 700;// for firmware update

unsigned int l_pageCheck = 8;

unsigned char xorByte = 0;
	
unsigned int oldPageNumber = 0;

uint8_t number=0;

unsigned char l_tempBuffer[l_tempBufferSize];	
unsigned char l_requestContent[l_requestContentSize];			
unsigned char l_firmwareContent[l_firmwareContentSize];
unsigned char EEPROMTempPage[EEPROMTempPageSize];
unsigned char tempSim = 0;

unsigned char tempIONumberAAsci[tempIONumberAAsciSize];
unsigned char tempIOData[tempIODataSize];
unsigned char tempIONumber;		

unsigned char latBuffer[latBufferSize];
unsigned char lonBuffer[lonBufferSize];
unsigned char radiusBuffer[radiusBufferSize];
long radiusInteger;						
unsigned char headerTypeAAsci[headerTypeAAsciSize];			
unsigned char headerTypeInt;

unsigned char speedAAsci[speedAAsciSize];
unsigned char relaySwitchData[relaySwitchDataSize];	

unsigned char tempHeaderBuffer[tempHeaderBufferSize];
unsigned char headerTypeAAsci[headerTypeAAsciSize];

unsigned char numberTypeAAsci[numberTypeAAsciSize];							
unsigned char headerTypeInt;
unsigned char headerLength;	

const char cell1C[]  = "00000000";
//const char cell2C[]  = "+923425711282";

//const char ipAddressC[]  = "125.209.111.147";  // Primary IP   
//const char ipAddressC[]  = "203.99.58.185";  // NAYATEL Main Server IP
const char ipAddressC[]  ="203.175.74.148";/*"175.110.136.143";*/
const char portC[]  ="33370";/*"22334";*/
const char latFirst[]  = "31.552391";
const char lonFirst[]  = "74.330234";

const char ftpAddressC[]  = "teresol.org";
const char ftpPortC[]  = "21";
const char ftpUserC[]  = "qasim@teresol.org";
const char ftpPassC[]  = "Teresol123";
const char ftpPathC[]  = "/Tracker/";

const char apnC[]  = "internet";
const char userC[]  = "Telenor";
const char passC[]  = "Telenor";
const char ioCOnfigC[]  = "000000";

const char ignitionC[]  = "Ignition";

//const char doorC[]  = "Door";

const char SOSC[]  = "SOS";

//const char vIDC[]  = "Tere 123";

const char smsGPRSHEAD[]  = "0000";

const char killswitchC[]  = "000";
/**
  * @brief  setup the factory default settings for GPS tracker
  * @param  None
  * @retval None
  */
void Config_FactorySettings_SIM1_SIM2(void)
{
	config.PolyGeoFences=10;
	config.PolyGeoFencePoints=10;
	
	strcpy((char*)config.carRegNo,(const char*)"000");
	strcpy((char*)config.CompanyID,(const char*)"001");
	strcpy((char*)config.LotID,(const char*)"000");
	strcpy((char*)config.PID,(const char*)"AAA");
	strcpy((char*)config.CID,(const char*)"BBB");
	strcpy((char*)config.SIM1No,(const char*)"+923327358515");
	strcpy((char*)config.SIM2No,(const char*)"+923327358515");
	
	
	strcpy((char*)GPS_LOG.latDecimalAASCI,(const char*)latFirst);		
	strcpy((char*)GPS_LOG.lonDecimalAASCI,(const char*)lonFirst);

	strcpy((char*)config.ftpServerIP1,(const char*)ftpAddressC);	
	strcpy((char*)config.ftpServerPort1,(const char*)ftpPortC);
	strcpy((char*)config.ftp1User,(const char*)ftpUserC);	
	strcpy((char*)config.ftp1Passsword,(const char*)ftpPassC);	
	strcpy((char*)config.ftp1Path,(const char*)ftpPathC);		
	
	config.FPSetting = FALSE;//(FP Setting)
	config.RFIDSetting = FALSE;//(RFID Setting)
	config.CANSetting = FALSE;//(CAN OBD Setting)
	
	config.eepromRecovSett = TRUE;//(EEPROM RECOV ON OFF) eeprom recovery enabled by default
	config.SDRecovSett = FALSE;//(SD RECOV/Log ON OFF)
	
	config.simShiftEn = FALSE;//(SIM Shift On , OFF) SIM shifting disabled 
	config.simNumber = 1;// always connect with SIM 1 Initially
	
	
	config.TPMSSetting=FALSE;//(TPMS setting)
	config.FTPSetting = FALSE; // FTP state machine disabled by default
	config.IAPSetting = FALSE; //(IAP Setting ON OFF) IAP update disabled by default
	
	config.IAPResetFlag=0x0000;//(IAP Reset Flags)
	config.IAPDownloadedFlag=0x00;//(IAP Downloaded Flag)
	config.IAPFileSize=0x00000000;//(IAP File Size)
	config.IAPFileAddress=0x00000000;//(IAP Flash Address)
	
	config.AuthenticateFuelKSEnDs = TRUE;
	config.FuelKSPulDuration = 1; //(Fuel Kill Switch No.of Pulses) geo fence relay make break enabled    // max 2 seconds
	config.FuelKSPulTimes = 3; //(Fuel Kill Switch Pulses Duration) geo fence relay make break enabled  // max 5 pulses
	
	config.KS1PulDuration = 1; //(Kill Switch 1 No.of Pulses) geo fence relay make break enabled    // max 2 seconds
	config.KS1PulTimes = 3; //(Kill Switch 1 Pulses Duration) geo fence relay make break enabled  // max 5 pulses
	
	config.KS2PulDuration = 1; //(Kill Switch 2 No.of Pulses) geo fence relay make break enabled    // max 2 seconds
	config.KS2PulTimes = 3; //(Kill Switch 2 Pulses Duration) geo fence relay make break enabled  // max 5 pulses
	
	
	config.GSMNoNetworkTry=15;//(No Network tries# )
	config.GSMIPTry=15;//(IP/SIM connect tries# )
	config.GSMTryDelay=1;//(Delay b/w connect tries)
	config.GSMTryRebootEnDs=FALSE;//(GSM Reboot En/Ds after tries Fail)
	config.AuthenticateTimout=15;//(Re-Authentication timeout)
	
	config.AckLayerEnDs=TRUE;//(Ack layer En/Ds)
	config.AckTimeout=8;//8*8=1 min(Ack Timeout)
	config.AckTry=20;//(Ack tries)
	
	config.IAPTry=5;//(IAP tries)
	
	strcpy((char*)config.SelectTcpUdp,(const char*)"1");//(TCP/UDP) By default TCP Port
	config.IMEINoSett = TRUE;//(IMEI# En/Ds Setting) By default IMEI# is On
	config.eepromAlgoSet = FIFO_ALGO;//(EEPROM Algo type) FIFO by default
	config.BuzzerSett = 0;//(Buzzer En/Ds) By default Buzzer is Off
	
	strcpy((char*)config.ServerSMSCellNo1,(const char*)cell1C);//(Server Cell 1(SMS))	
	strcpy((char*)config.ServerSMSCellNo2,(const char*)cell1C);//(Server Cell 2(SMS))		
	strcpy((char*)config.ServerSMSCellNo3,(const char*)cell1C);//(Server Cell 3(SMS))	
	
	strcpy((char*)config.ServerCallCellNo1,(const char*)cell1C);//(Server Cell 1(voice call))	
	strcpy((char*)config.ServerCallCellNo2,(const char*)cell1C);//(Server Cell 2(voice call))		
	strcpy((char*)config.ServerCallCellNo3,(const char*)cell1C);//(Server Cell 3(voice call))	
	
	config.timeZoneSettings = 0; //(Time Zone Setting) time zone setting to PST standard
	
	strcpy((char*)config.sleepModeSetting,(const char*)"0000");//(Sleep Mode Setting)turned ON on imran request
	
	config.sleepModeTimeout = 15;////3mint on imran request(Sleep Mode Timeout) 5 mint timeout  15*20sec
	
	strcpy((char*)config.smsPassword,(const char*)smsGPRSHEAD);	//(SMS Password)
	strcpy((char*)config.gprsPassword,(const char*)smsGPRSHEAD);	//(GPRS Password)
	
	config.MainPwrHysPoll = 3;//(Main Power Poll Hysterisis)
	config.MainPwrHysPollDuration = 3;//(Main Power Poll Duration )
	config.MainPwrHysPollMinTrueAlert = 3;//(Main Power Min. True Alerts)
	
	config.ignitionHysPoll = 3;//(Ignition Poll Hysterisis)
	config.ignitionHysPollDuration = 3;//(Ignition Poll Duration  )
	config.ignitionHysPollMinTrueAlert = 3;//(Ignition Min. True Alerts)
	
	config.SOSHysPoll = 3;//(SOS Poll Hysterisis)
	config.SOSHysPollDuration = 3;//(SOS Poll Duration )
	config.SOSHysPollMinTrueAlert = 3;//(SOS Min. True Alerts)
	
	config.DIO1HysPoll = 4;//(DIO1 Poll Hysterisis)
	config.DIO1HysPollDuration = 3;//(DIO1 Poll Duration )
	config.DIO1HysPollMinTrueAlert = 3;//(DIO1 Min. True Alerts)
	
	config.DIO2HysPoll = 3;//(DIO1 Poll Hysterisis)
	config.DIO2HysPollDuration = 3;//(DIO1 Poll Duration )
	config.DIO2HysPollMinTrueAlert = 3;//(DIO1 Min. True Alerts)
	
	config.DIO3HysPoll = 3;//(DIO1 Poll Hysterisis)//if FP is used,this functionality Reserved
	config.DIO3HysPollDuration = 3;//(DIO1 Poll Duration )
	config.DIO3HysPollMinTrueAlert = 3;//(DIO1 Min. True Alerts)
	
	config.AIO1HysPoll = 4;//(AIO1 Poll Hysterisis)
	config.AIO1HysPollDuration = 3;//(AIO1 Poll Duration )
	config.AIO1HysAvgSamples = 5;//(AIO1 Samples for Avg)
	config.AIO1HysMinValue = 0;//(AIO1 Min Value)
	config.AIO1HysMaxValue = 10000;//(AIO1 Max Value)
	config.AIO1HysDifference = 500;//(AIO1 Difference value)
	
	config.AIO2HysPoll = 3;//(AIO2 Poll Hysterisis)
	config.AIO2HysPollDuration = 3;//(AIO2 Poll Duration )
	config.AIO2HysAvgSamples = 5;//(AIO2 Samples for Avg)
	config.AIO2HysMinValue = 0;//(AIO2 Min Value)
	config.AIO2HysMaxValue = 10000;//(AIO2 Max Value)
	config.AIO2HysDifference = 500;//(AIO2 Difference value)
	
	config.AIO3HysPoll = 3;//(AIO3 Poll Hysterisis)
	config.AIO3HysPollDuration = 3;//(AIO3 Poll Duration )
	config.AIO3HysAvgSamples = 5;//(AIO3 Samples for Avg)
	config.AIO3HysMinValue = 0;//(AIO3 Min Value)
	config.AIO3HysMaxValue = 10000;//(AIO3 Max Value)
	config.AIO3HysDifference = 500;//(AIO3 Difference value)
	
	config.AIO4HysPoll = 3;//(AIO4 Poll Hysterisis)  SPare,Not used in MX70
	config.AIO4HysPollDuration = 3;//(AIO4 Poll Duration )
	config.AIO4HysAvgSamples = 5;//(AIO4 Samples for Avg)
	config.AIO4HysMinValue = 0;//(AIO4 Min Value)
	config.AIO4HysMaxValue = 10000;//(AIO4 Max Value)
	config.AIO4HysDifference = 500;//(AIO4 Difference value)
	
	strcpy((char*)header.SOS_Header,(const char*)SOSC);	
	strcpy((char*)header.DI1Header,(const char*)"Door");		
	strcpy((char*)header.DI2Header,(const char*)"Seat Belt");		
	strcpy((char*)header.DI3Header,(const char*)"H-Break");
	strcpy((char*)header.Ignition_Header,(const char*)ignitionC);	
	
	config.SharpTurnHysPoll=4;
	config.SharpTurnThresh=45;
	
	strcpy((char*)config.geo1AreaName,(const char*)"GeoFence 1");	
	strcpy((char*)config.geo1Configure,(const char*)"1000");////SMS setting configure;first byte=SMS on enter(1),exit(2),both(3); 2,3,4 bytes=SMS numbers selection byte encoded 
	
	strcpy((char*)config.geo2AreaName,(const char*)"GeoFence 2");	
	strcpy((char*)config.geo2Configure,(const char*)"1000");
	
	strcpy((char*)config.geo3AreaName,(const char*)"GeoFence 3");	
	strcpy((char*)config.geo3Configure,(const char*)"1000");
	
	strcpy((char*)config.geo4AreaName,(const char*)"GeoFence 4");	
	strcpy((char*)config.geo4Configure,(const char*)"1000");
	
	strcpy((char*)config.geo5AreaName,(const char*)"GeoFence 5");	
	strcpy((char*)config.geo5Configure,(const char*)"1000");
	
	strcpy((char*)config.geo6AreaName,(const char*)"GeoFence 6");	
	strcpy((char*)config.geo6Configure,(const char*)"1000");
	
	strcpy((char*)config.geo7AreaName,(const char*)"GeoFence 7");	
	strcpy((char*)config.geo7Configure,(const char*)"1000");
	
	strcpy((char*)config.geo8AreaName,(const char*)"GeoFence 8");	
	strcpy((char*)config.geo8Configure,(const char*)"1000");
	
	strcpy((char*)config.geo9AreaName,(const char*)"GeoFence 9");	
	strcpy((char*)config.geo9Configure,(const char*)"1000");
	
	strcpy((char*)config.geo10AreaName,(const char*)"GeoFence 10");	
	strcpy((char*)config.geo10Configure,(const char*)"1000");
	
	strcpy((char*)config.geo1UID,(const char*)"001");	//Geofence unique ID no
	strcpy((char*)config.geo1Setting,(const char*)"1111000000");//Geofence setting: GeofenceEnable|FuelKSEn|Relay1En|Relay2En|FuelKsInFlag|FuelKsOutFlag|Relay1InFlag|Relay1OutFlag|Relay2InFlag|Relay2OutFlag
	
	strcpy((char*)config.geo2UID,(const char*)"002");	
	strcpy((char*)config.geo2Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo3UID,(const char*)"003");	
	strcpy((char*)config.geo3Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo4UID,(const char*)"004");	
	strcpy((char*)config.geo4Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo5UID,(const char*)"005");	
	strcpy((char*)config.geo5Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo6UID,(const char*)"006");	
	strcpy((char*)config.geo6Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo7UID,(const char*)"007");	
	strcpy((char*)config.geo7Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo8UID,(const char*)"008");	
	strcpy((char*)config.geo8Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo9UID,(const char*)"009");	
	strcpy((char*)config.geo9Setting,(const char*)"0000000000");
	
	strcpy((char*)config.geo10UID,(const char*)"010");	
	strcpy((char*)config.geo10Setting,(const char*)"0000000000");
	
}

/**
  * @brief  setup the factory default settings for GPS tracker
  * @param  None
  * @retval None
  */
void Config_FactorySettings_SIM1(void)
{
	USART_SendData_s( DEBUG_COM,(unsigned char*) "Config_FactorySettings_SIM1\r\n");
	Config_FactorySettings_SIM1_SIM2();
	strcpy((char*)config.serverIP1,(const char*)ipAddressC);	
	strcpy((char*)config.serverPort1,(const char*)portC);		
	
	strcpy((char*)config.serverIP2,(const char*)ipAddressC);	
	strcpy((char*)config.serverPort2,(const char*)portC);	
	
	strcpy((char*)config.sim1APN,(const char*)apnC);	
	strcpy((char*)config.sim1APNUser,(const char*)userC);	
	strcpy((char*)config.sim1APNPassword,(const char*)passC);	
	
	
	strcpy((char*)config.IO1Configure,(const char*)ioCOnfigC);	// setting IO1 Profile	
	strcpy((char*)config.IO2Configure,(const char*)ioCOnfigC);	// setting IO2 Profile		
	strcpy((char*)config.IO3Configure,(const char*)ioCOnfigC);	// setting IO3 Profile
	strcpy((char*)config.IO4Configure,(const char*)ioCOnfigC);	// setting IO4(Reserved) Profile
	strcpy((char*)config.ignitionConfigure,(const char*)ioCOnfigC);	// setting Ignition IO Profile			
	strcpy((char*)config.sosConfigure,(const char*)ioCOnfigC);	// setting SOS IO Profile		
	
	
	config.transmissionTimeIGon =30;//10mint on imran req //(Time Track)				/// 180 for 1 hour 180*20sec
	config.transmissionTimeSMS = 0; // (SMS Time Track)sms tracking off by default
	config.heartBeatTime = 90; //(Heart Beat Time)				/// 360 for 2 hour 180*20sec
	config.distanceTracking = 2000;//2000 on imran req	// (Distance Based Tracking)
	config.headingChange = 25; //(Heading Change Degrees)heading report on 45 degree difference
	
	config.noGPSMNOnOff = FALSE;//(No fix motion on off sett)
	config.MD_ON_OFF_FLAG = FALSE;//(Motion on off sett) motion detection off by default
	config.gpsNoFixMNTime = 5;//(No Fix MN Alerts Time Setting)counter for sending packet on motion detection even if GPS is void 5 times check
	config.transmissionTimeIGOff =30;//1080=6hours on imran req//(Time track ig off time setting)			/// 30 for 600sec/10 mint 30*20sec
	
	config.speedLimit = 100;//(Speed Limit)
	config.overSpeedDiffA = 10;//(Speed Diff check)
	config.idleTimeSetting =600 ;//(Idle Time Setting)				/// 600 for 10 minute
	
	
	strcpy((char*)config.AlertsOnOffEncode,(const char*)"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");//(Reason code/Alerts En/Ds encode)2bytes=1hex(00-FF) encoded alerts;16 hex(32 bytes)
	config.HarshBreakDetectTHR = 0.1;		//(ACC Harsh Break Thresh)
	config.AccelerationDetectTHR = 0.1;		//(ACC Acceleration Thresh)
	config.ImpactDetectTHR = 0.1;		//(ACC Impact Thresh)
	//config.AnymotionDetectTHR = 0x0F;		//(ACC Any-motion Threshold)
	config.NextAnymotionAlertTimeOut = 10;		//(ACC Any-motion Threshold) in sec
	
}

/**
  * @brief  setup the factory default settings for GPS tracker
  * @param  None
  * @retval None
  */
void Config_FactorySettings_SIM2(void)
{
	USART_SendData_s( DEBUG_COM,(unsigned char*) "Config_FactorySettings_SIM2\r\n");
	Config_FactorySettings_SIM1_SIM2();
	strcpy((char*)config.serverIP1,(const char*)ipAddressC);	
	strcpy((char*)config.serverPort1,(const char*)portC);		
	
	strcpy((char*)config.serverIP2,(const char*)ipAddressC);	
	strcpy((char*)config.serverPort2,(const char*)portC);	
	
	strcpy((char*)config.sim2APN,(const char*)apnC);	
	strcpy((char*)config.sim2APNUser,(const char*)userC);	
	strcpy((char*)config.sim2APNPassword,(const char*)passC);	
	
	
	strcpy((char*)config.IO1Configure,(const char*)ioCOnfigC);	// setting IO1 Profile	
	strcpy((char*)config.IO2Configure,(const char*)ioCOnfigC);	// setting IO2 Profile		
	strcpy((char*)config.IO3Configure,(const char*)ioCOnfigC);	// setting IO3 Profile
	strcpy((char*)config.IO4Configure,(const char*)ioCOnfigC);	// setting IO4(Reserved) Profile
	strcpy((char*)config.ignitionConfigure,(const char*)ioCOnfigC);	// setting Ignition IO Profile			
	strcpy((char*)config.sosConfigure,(const char*)ioCOnfigC);	// setting SOS IO Profile		
	
	
	config.transmissionTimeIGon =30;//10mint on imran req //(Time Track)				/// 180 for 1 hour 180*20sec
	config.transmissionTimeSMS = 0; // (SMS Time Track)sms tracking off by default
	config.heartBeatTime = 90; //(Heart Beat Time)				/// 360 for 2 hour 180*20sec
	config.distanceTracking = 2000;//2000 on imran req	// (Distance Based Tracking)
	config.headingChange = 25; //(Heading Change Degrees)heading report on 45 degree difference
	
	config.noGPSMNOnOff = FALSE;//(No fix motion on off sett)
	config.MD_ON_OFF_FLAG = FALSE;//(Motion on off sett) motion detection off by default
	config.gpsNoFixMNTime = 5;//(No Fix MN Alerts Time Setting)counter for sending packet on motion detection even if GPS is void 5 times check
	config.transmissionTimeIGOff =30;//6 hours on imran req//(Time track ig off time setting)			/// 30 for 600sec/10 mint 30*20sec
	
	config.speedLimit = 100;//(Speed Limit)
	config.overSpeedDiffA = 10;//(Speed Diff check)
	config.idleTimeSetting =600 ;//(Idle Time Setting)				/// 600 for 10 minute
	
	strcpy((char*)config.AlertsOnOffEncode,(const char*)"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");//(Reason code/Alerts En/Ds encode)2bytes=1hex(00-FF) encoded alerts;16 hex(32 bytes)
	config.HarshBreakDetectTHR = 0.1;		//(ACC Harsh Break Thresh)
	config.AccelerationDetectTHR = 0.1;		//(ACC Acceleration Thresh)
	config.ImpactDetectTHR = 0.1;		//(ACC Impact Thresh)
	//config.AnymotionDetectTHR = 0x0F;		//(ACC Any-motion Threshold)
	config.NextAnymotionAlertTimeOut = 10;		//(ACC Any-motion Threshold) in sec
}

/**
  * @brief  this module alert state machine that a Configuration request reply is ready to send via SMS,GPRS or Serial
  * @param  None
  * @retval None
  */
void ConfigReplyReady(unsigned char *a_receivedRequest,unsigned char a_requestType)
{
	if(a_requestType == SMS)
	{
		config.replyReadySms = TRUE;// set if request is send from SMS				
	}
	else if(a_requestType == GPRS)
	{
		config.replyReadyGprs = TRUE;// set if request is send from GPRS				
	}	
	else
	{
		requestReplyConfig(a_receivedRequest);
		USART_SendData_s( DEBUG_COM,(unsigned char*)config.replyToSend);// temp 			
	}	
}

/**
  * @brief  this module process the received configuration request from sever, sms or by serial.
  * @param  None
  * @retval None
  */
void Config_Settings_Request_Process(unsigned char *a_receivedRequest,unsigned char a_requestType)
{
	unsigned char l_tempPacket[20];	
	
	unsigned char l_requestID = 0;	
	int l_tempByte1 = 0;
	int l_tempByte2 = 0;
	int l_tempByte3 = 0;
	int l_tempByte4 = 0;
	int l_tempByte5 = 0;
	int l_tempByte6 = 0;
	//unsigned int l_pageNumber = 0;
	//unsigned char origCheckSum[4];
	//unsigned char checkSumCheck = 0;
	float tempFloat = 0;
	
	long odometer_reading = 0;
	
	
	unsigned char l_tempBuff1[30];
	unsigned char l_tempBuff2[40];
	unsigned char l_tempBuff3[30];
	unsigned char l_tempBuff4[30];
	unsigned char l_tempBuff5[30];
	unsigned char l_tempBuff6[30];
	
	uint16_t count_FP = 0;
	uint16_t template_index=0;
//	uint8_t XOR=0;
	uint8_t GeoFenceNo=0;
	uint8_t tempGeoFenceNo=0;
	uint16_t delay_count=0;	
	int8_t comma_count=0;
  //unsigned char temp_buffer_key[6]={0x00};
	//uint8_t tempdata[3];
	uint8_t PointNo=0;
	
	config.sleepModeCounter = 0;
			
	l_requestID = (a_receivedRequest[6] - '0') * 10 + (a_receivedRequest[7] - '0');	
	requestContentExtract(l_requestContent,a_receivedRequest);// extract message request
	
	switch (l_requestID)
	{		
		
		case POLY_GEOFENCE_CONFIG: // 90
		{			
			USART_SendData_s( DEBUG_COM,(unsigned char *)"PolyGeoFence Data\r\n");	
			
			//@@0000,S90,1,254,1111000000,2,3,10,4,5,7,2,3,0,0,0,0,0,0,0,0,0,0,0,0*\r\n
			
			dataExtractor4Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4);
	
			l_tempByte1 = atoi((const char*)l_tempBuff1);
			l_tempByte2=strnlen((const char *)l_tempBuff2,4);
			l_tempByte3=strnlen((const char *)l_tempBuff3,11);
			
			if(l_tempByte1<=10 && l_tempByte2<=3 && l_tempByte3==10)
			{
				tempGeoFenceNo=l_tempByte1;
				
				sprintf(( char*)G_sprintfBuffer,"%d",tempGeoFenceNo);			
				USART_SendData_s( DEBUG_COM,(unsigned char *)"\n\rGeoFenceNo:");
				USART_SendData_s( DEBUG_COM,G_sprintfBuffer);	
				USART_SendData_s( DEBUG_COM,(unsigned char *)"\r\n");
				
				GeoFenceNo=tempGeoFenceNo-1;
				/*USART_SendData_s( DEBUG_COM,"\n\rl_requestContent:");
				USART_SendData_s( DEBUG_COM,l_requestContent);	
				USART_SendData_s( DEBUG_COM,"\r\n");*/
				
				switch(tempGeoFenceNo)
				{
					case 1:
					{
						strcpy((char*)config.geo1UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo1Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO1IDStart,GEOIDLen,config.geo1UID); 	
						multiByteContentWrite(GEO1_SET,GEO_SET_LEN,config.geo1Setting); 
						
						/*USART_SendData_s( DEBUG_COM,"\n\rgeo1UID:");
						USART_SendData_s( DEBUG_COM,config.geo1UID);	
						USART_SendData_s( DEBUG_COM,"\r\n");
						
						USART_SendData_s( DEBUG_COM,"\n\rgeo1Setting:");
						USART_SendData_s( DEBUG_COM,config.geo1Setting);	
						USART_SendData_s( DEBUG_COM,"\r\n");*/
						
						break;
					}
					case 2:
					{
						strcpy((char*)config.geo2UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo2Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO2IDStart,GEOIDLen,config.geo2UID); 	
						multiByteContentWrite(GEO2_SET,GEO_SET_LEN,config.geo2Setting); 
						break;
					}
					case 3:
					{
						strcpy((char*)config.geo3UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo3Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO3IDStart,GEOIDLen,config.geo3UID); 	
						multiByteContentWrite(GEO3_SET,GEO_SET_LEN,config.geo3Setting); 
						break;
					}
					case 4:
					{
						strcpy((char*)config.geo4UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo4Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO4IDStart,GEOIDLen,config.geo4UID); 	
						multiByteContentWrite(GEO4_SET,GEO_SET_LEN,config.geo4Setting); 
						break;
					}
					case 5:
					{
						strcpy((char*)config.geo5UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo5Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO5IDStart,GEOIDLen,config.geo5UID); 	
						multiByteContentWrite(GEO5_SET,GEO_SET_LEN,config.geo5Setting); 
						break;
					}
					case 6:
					{
						strcpy((char*)config.geo6UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo6Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO6IDStart,GEOIDLen,config.geo6UID); 	
						multiByteContentWrite(GEO6_SET,GEO_SET_LEN,config.geo6Setting); 
						break;
					}
					case 7:
					{
						strcpy((char*)config.geo7UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo7Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO7IDStart,GEOIDLen,config.geo7UID); 	
						multiByteContentWrite(GEO7_SET,GEO_SET_LEN,config.geo7Setting); 
						break;
					}
					case 8:
					{
						strcpy((char*)config.geo8UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo8Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO8IDStart,GEOIDLen,config.geo8UID); 	
						multiByteContentWrite(GEO8_SET,GEO_SET_LEN,config.geo8Setting); 
						break;
					}
					case 9:
					{
						strcpy((char*)config.geo9UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo9Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO9IDStart,GEOIDLen,config.geo9UID); 	
						multiByteContentWrite(GEO9_SET,GEO_SET_LEN,config.geo9Setting); 
						break;
					}
					case 10:
					{
						strcpy((char*)config.geo10UID,(const char*)l_tempBuff2);	
						strcpy((char*)config.geo10Setting,(const char*)l_tempBuff3);
						
						multiByteContentWrite(GEO10IDStart,GEOIDLen,config.geo10UID); 	
						multiByteContentWrite(GEO10_SET,GEO_SET_LEN,config.geo10Setting); 
						break;
					}
					default:
						break;
				}
			
				for(l_tempByte1=0;comma_count<=2 && l_tempByte1<=40 ;l_tempByte1++)
				{
					l_tempBuff1[l_tempByte1]=l_requestContent[delay_count++];
					if(l_tempBuff1[l_tempByte1]==',')
					{
						comma_count++;
					}
				}
				
				comma_count=-1;
				
				for (;count_FP<l_requestContentSize;count_FP++)
				{
					latBuffer[template_index++]=l_requestContent[delay_count++];
		
					if (l_requestContent[delay_count]==',' || l_requestContent[delay_count]==0x00)
					{
						comma_count++;
						latBuffer[template_index]=0x00;
						if((comma_count%2)==0)
						{
							Vertex[GeoFenceNo][(comma_count/2)].x= aasciToFloatCopy(latBuffer);	
							
							sprintf(( char*)G_sprintfBuffer,"%.6f",Vertex[GeoFenceNo][(comma_count/2)].x);	

							eepromWriteDouble((POLY_GEOFENCE_Start+(GeoFenceNo*160)+((comma_count/2)*16)),Vertex[GeoFenceNo][(comma_count/2)].x);		
								
							/*USART_SendData_s( DEBUG_COM,"\n\rVertex[i][j].x:");
							USART_SendData_s( DEBUG_COM,latBuffer);	
							USART_SendData_s( DEBUG_COM,"       ");
							USART_SendData_s( DEBUG_COM,G_sprintfBuffer);	
							USART_SendData_s( DEBUG_COM,"       ");*/
						}
						else
						{
							Vertex[GeoFenceNo][(comma_count/2)].y= aasciToFloatCopy(latBuffer);	
							
							sprintf(( char*)G_sprintfBuffer,"%.6f",Vertex[GeoFenceNo][(comma_count/2)].y);	

							eepromWriteDouble((POLY_GEOFENCE_Start+(GeoFenceNo*160)+(((comma_count/2)*16)+8)),Vertex[GeoFenceNo][(comma_count/2)].y);		
							/*USART_SendData_s( DEBUG_COM,"\n\rVertex[i][j].y:");
							USART_SendData_s( DEBUG_COM,latBuffer);
							USART_SendData_s( DEBUG_COM,"       ");
							USART_SendData_s( DEBUG_COM,G_sprintfBuffer);	
							USART_SendData_s( DEBUG_COM,"\r\n");*/
						}
						
						if(l_requestContent[delay_count]==0x00)
						{
							break;
						}
						arrayInit2Zero(latBuffer,latBufferSize);
						template_index=0;
						delay_count++;
						
						
					}
				}
				requestReplyGenOk(a_receivedRequest); 							//generates the reply of request						
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}

		case TRACK_TIME_INTERVAL_SMS://0
		{				
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);
			l_tempByte1 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			if((tempSim==1 || tempSim==2 || tempSim==3) &&  l_tempByte1<= 65535)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.transmissionTimeSMS = l_tempByte1;// copy the content to configurations
				}
				if(tempSim == 1 || tempSim == 3)
				{
					eeprom_write_word(SMS_TIME_TRACK_LSB_1,l_tempByte1);	
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_word(SMS_TIME_TRACK_LSB_2,l_tempByte1);	
				}
			
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);					
			break;
		}
		
		case HEARTBEAT_INTERVAL://1
		{
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);
			l_tempByte1 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			if((tempSim==1 || tempSim==2 || tempSim==3) &&  l_tempByte1<= 65535)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.heartBeatTime = l_tempByte1;// copy the content to configurations
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					eeprom_write_word(HEART_BEAT_LSB_1,l_tempByte1);		
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_word(HEART_BEAT_LSB_2,l_tempByte1);	
				}
					
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);	
			break;
		}
		
	
		case TRACK_TIME_INTERVAL_GPRS://2
		{			
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor3Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3);
			
			l_tempByte1=atoi((const char*)l_tempBuff2);
			l_tempByte2=atoi((const char*)l_tempBuff3);
			
			if((tempSim==1 || tempSim==2 || tempSim==3) &&  l_tempByte1<= 65535 &&  l_tempByte2<= 65535)
			{
				
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.transmissionTimeIGon = atoi((const char*)l_tempBuff2);// convert AASCI to integer
					config.transmissionTimeIGOff = atoi((const char*)l_tempBuff3);// convert AASCI to integer
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					eeprom_write_word(TIME_TRACK_LSB_1,l_tempByte1);
					eeprom_write_word(TIME_TRK_IG_OFF_SET_1,l_tempByte2);
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_word(TIME_TRACK_LSB_2,l_tempByte1);
					eeprom_write_word(TIME_TRK_IG_OFF_SET_2,l_tempByte2);
				}
					
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);				
			break;
		}	

		case SET_HEADING_CHANGE://3
		{
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);
			l_tempByte1 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			if((tempSim==1 || tempSim==2 || tempSim==3) && l_tempByte1>=0 && l_tempByte1<=359)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.headingChange = l_tempByte1;// copy the content to configurations
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					eeprom_write_word(HEADING_LSB_1,l_tempByte1);			
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_word(HEADING_LSB_2,l_tempByte1);	
				}
				
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);	
			break;
		}	
		
		
		case TRACK_DISTANCE_INTERVAL://4
		{	
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);
			l_tempByte1 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			if((tempSim==1 || tempSim==2 || tempSim==3) && (l_tempByte1<=4294967295U))
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.distanceTracking = l_tempByte1;// copy the content to configurations
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					eeprom_write_dword(DIST_TRK_LSB1_1,l_tempByte1);
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_dword(DIST_TRK_LSB1_2,l_tempByte1);
				}
					
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);				
			
			break;
		}	
		
		
		case SET_GPRS://5
		{		
			USART_SendData_s( DEBUG_COM,"Previous GPRS IP ////////////:\r\n");		
			tempSim = (a_receivedRequest[9] - '0');								
			dataExtractor5Comma(l_requestContent,l_tempBuff1,tempIOData,l_tempBuff2,l_tempBuff3,l_tempBuff4);
			l_tempByte1 = atoi((const char*)tempIOData);
			if(tempSim==1 || tempSim==2 || tempSim==3)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					if((tempSim==1 && l_tempByte1==SERVERIP1) || (tempSim==2 && l_tempByte1==SERVERIP1))
					{
						strcpy((char*)config.serverIP1,(const char*)l_tempBuff3);	
						strcpy((char*)config.serverPort1,(const char*)l_tempBuff4);	
					}
					else if((tempSim==1 && l_tempByte1==SERVERIP2) || (tempSim==2 && l_tempByte1==SERVERIP2))
					{
						strcpy((char*)config.serverIP2,(const char*)l_tempBuff3);	
						strcpy((char*)config.serverPort2,(const char*)l_tempBuff4);	
					}
				}
				
				if(tempSim == 1 || tempSim==3)
				{
					switch (l_tempByte1)// 
					{
						case SERVERIP1:
						{	
							USART_SendData_s( DEBUG_COM,"Previous GPRS IP1////////////:\r\n");
							USART_SendData_s(DEBUG_COM,config.serverIP1);
							USART_SendData_s(DEBUG_COM,"\r\n");
							USART_SendData_s( DEBUG_COM,"New GPRS IP1////////////:\r\n");
			
							multiByteContentWrite(serverIP1Start_1,serverIP1ArrayLen_1,l_tempBuff3); // write server IP address		
							multiByteContentWrite(serverPort1Start_1,serverPort1ArrayLen_1,l_tempBuff4); // write server IP address					
							multiByteContentWrite(TCP_UDP_START,TCP_UDP_LEN,l_tempBuff2);

							USART_SendData_s(DEBUG_COM,config.serverIP1);
							USART_SendData_s(DEBUG_COM,"\r\n");
							USART_SendData_s(DEBUG_COM,config.serverPort1);
							USART_SendData_s(DEBUG_COM,"\r\n");
							break;
						}
						case SERVERIP2:
						{
							USART_SendData_s( DEBUG_COM,"New GPRS IP2///////////:\r\n");
							multiByteContentWrite(serverIP2Start_1,serverIP2ArrayLen_1,l_tempBuff3); // write server IP address		
							multiByteContentWrite(serverPort2Start_1,serverPort2ArrayLen_1,l_tempBuff4); // write server IP address					
							multiByteContentWrite(TCP_UDP_START,TCP_UDP_LEN,l_tempBuff2);	
							
							USART_SendData_s(DEBUG_COM,config.serverIP2);
							USART_SendData_s(DEBUG_COM,"\r\n");
							USART_SendData_s(DEBUG_COM,config.serverPort2);
							USART_SendData_s(DEBUG_COM,"\r\n");
							break;
						}
						default:
							break;
					}
				}
				if(tempSim == 2 || tempSim==3)
				{
					switch (l_tempByte1)// 
					{
						case SERVERIP1:
						{	
							USART_SendData_s( DEBUG_COM,"Previous GPRS IP1////////////:\r\n");
							USART_SendData_s(DEBUG_COM,config.serverIP1);
							USART_SendData_s(DEBUG_COM,"\r\n");
							USART_SendData_s( DEBUG_COM,"New GPRS IP1////////////:\r\n");
												
							multiByteContentWrite(serverIP1Start_2,serverIP1ArrayLen_2,l_tempBuff3); // write server IP address		
							multiByteContentWrite(serverPort1Start_2,serverPort1ArrayLen_2,l_tempBuff4); // write server IP address					
							multiByteContentWrite(TCP_UDP_START,TCP_UDP_LEN,l_tempBuff2);

							USART_SendData_s(DEBUG_COM,config.serverIP1);
							USART_SendData_s(DEBUG_COM,"\r\n");
							USART_SendData_s(DEBUG_COM,config.serverPort1);
							USART_SendData_s(DEBUG_COM,"\r\n");
							break;
						}
						case SERVERIP2:
						{
							USART_SendData_s( DEBUG_COM,"New GPRS IP2///////////:\r\n");

							multiByteContentWrite(serverIP2Start_2,serverIP2ArrayLen_2,l_tempBuff3); // write server IP address		
							multiByteContentWrite(serverPort2Start_2,serverPort2ArrayLen_2,l_tempBuff4); // write server IP address					
							multiByteContentWrite(TCP_UDP_START,TCP_UDP_LEN,l_tempBuff2);	
							
							USART_SendData_s(DEBUG_COM,config.serverIP2);
							USART_SendData_s(DEBUG_COM,"\r\n");
							USART_SendData_s(DEBUG_COM,config.serverPort2);
							USART_SendData_s(DEBUG_COM,"\r\n");
							break;
						}
						default:
							break;
					}
				}
								
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);	
			break;									
		}
		
		
		case SET_APN://6
		{				
			tempSim = (a_receivedRequest[9] - '0');								
			dataExtractor4Comma(l_requestContent,l_tempBuff1,tempIOData,l_tempBuff2,l_tempBuff3);
			l_tempByte1 = atoi((const char*)tempIOData);
			if(tempSim==1 || tempSim==2 || tempSim==3)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					if(tempSim==1 || tempSim==3)
					{
						strcpy((char*)config.sim1APN,(const char*)tempIOData);	
						strcpy((char*)config.sim1APNUser,(const char*)l_tempBuff2);	
						strcpy((char*)config.sim1APNPassword,(const char*)l_tempBuff3);	
					}
					if(tempSim==2 || tempSim==3)
					{
						strcpy((char*)config.sim2APN,(const char*)tempIOData);	
						strcpy((char*)config.sim2APNUser,(const char*)l_tempBuff2);	
						strcpy((char*)config.sim2APNPassword,(const char*)l_tempBuff3);	
					}
				}
				
				if(tempSim == 1 ||  tempSim == 3 )
				{
					
					multiByteContentWrite(APN1Start_1,APN1ArrayLen_1,tempIOData);
					multiByteContentWrite(APNUser1Start_1,APN1UserArrayLen_1,l_tempBuff2); 
					multiByteContentWrite(APN1PassStart_1,APN1PassArrayLen_1,l_tempBuff3); 
				}
				if(tempSim == 2 || tempSim == 3 )
				{
					multiByteContentWrite(APN1Start_2,APN1ArrayLen_2,tempIOData);
					multiByteContentWrite(APNUser1Start_2,APN1UserArrayLen_2,l_tempBuff2); 
					multiByteContentWrite(APN1PassStart_2,APN1PassArrayLen_2,l_tempBuff3); 
				}
					
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);	

			requestReplyGenOk(a_receivedRequest);//generates the reply of request						
			ConfigReplyReady(a_receivedRequest,a_requestType);							
			break;
		}	
		

		case SET_ACC_TH_DETECT://7
		{
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor4Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4);
			l_tempByte1 = atoi((const char*)l_tempBuff2);// Acceleration
			l_tempByte2 = atoi((const char*)l_tempBuff3);// impact
			l_tempByte3 = atoi((const char*)l_tempBuff4);// harsh break
	
			if((tempSim==1 || tempSim==2 || tempSim==3) && l_tempByte2<=5 && l_tempByte3<=5 && l_tempByte4<=5)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					tempFloat=l_tempByte1;
					config.AccelerationDetectTHR = tempFloat/10;
					tempFloat=l_tempByte2;
					config.HarshBreakDetectTHR = tempFloat/10;
					tempFloat=l_tempByte3;
					config.ImpactDetectTHR = tempFloat/10;
					
					/*USART_SendData_s( DEBUG_COM,(unsigned char*) "Config::");
					sprintf((char*)l_tempBuffer,"%.3f",config.AccelerationDetectTHR);	
					USART_SendData_s( DEBUG_COM,(unsigned char*) l_tempBuffer);
					USART_SendData_s( DEBUG_COM,(unsigned char*) "Config::\r\n");
					
					tempFloat=tempFloat/10;
					
					USART_SendData_s( DEBUG_COM,(unsigned char*) "Config2::");
					sprintf((char*)l_tempBuffer,"%.3f",tempFloat);	
					USART_SendData_s( DEBUG_COM,(unsigned char*) l_tempBuffer);
					USART_SendData_s( DEBUG_COM,(unsigned char*) "Config::\r\n");*/
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					tempFloat=l_tempByte1;
					eepromWriteFloat(ACC_ACCEL_TH_1,tempFloat/10);
					tempFloat=l_tempByte2;
					eepromWriteFloat(ACC_IMPACT_TH_1,tempFloat/10);
					tempFloat=l_tempByte3;
					eepromWriteFloat(ACC_Harsh_B_TH_1,tempFloat/10);	
				}
				if(tempSim == 2 || tempSim == 3)
				{
					tempFloat=l_tempByte1;
					eepromWriteFloat(ACC_ACCEL_TH_2,tempFloat/10);
					tempFloat=l_tempByte2;
					eepromWriteFloat(ACC_IMPACT_TH_2,tempFloat/10);
					tempFloat=l_tempByte3;
					eepromWriteFloat(ACC_Harsh_B_TH_2,tempFloat/10);	
				}
				
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);							
			break;
		}

		case SET_MOTION_ON_OFF://8
		{					
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor5Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4,l_tempBuff5);
			l_tempByte1 = atoi((const char*)l_tempBuff2);
			l_tempByte2 = atoi((const char*)l_tempBuff3);
			l_tempByte3 = atoi((const char*)l_tempBuff4);
			l_tempByte4 = atoi((const char*)l_tempBuff5);
			
			if((tempSim==1 || tempSim==2 || tempSim==3) && l_tempByte1<=2 && l_tempByte2<=2 && l_tempByte3<=255 && l_tempByte4<=255)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.MD_ON_OFF_FLAG = l_tempByte1;//Motion Detection based tracking on off				
					config.noGPSMNOnOff = l_tempByte2;//Motion Detection alerts in no fix state on off
					config.NextAnymotionAlertTimeOut = l_tempByte3;//counter for config.MD_ON_OFF_FLAG
					config.gpsNoFixMNTime = l_tempByte4;// counter for config.noGPSMNOnOff 
					
				}
				
				if(tempSim == 1 || tempSim == 3)
				{			
					eeprom_write_byte(MD_ON_OFF_1,l_tempByte1);
					eeprom_write_byte(NO_FX_MN_ON_OFF_1,l_tempByte2);
					eeprom_write_byte(ACC_ANYMOTION_TIME_1 ,l_tempByte3);	
					eeprom_write_byte(NO_FX_MN_TIME_1,l_tempByte4);	
					
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_byte(MD_ON_OFF_2,l_tempByte1);
					eeprom_write_byte(NO_FX_MN_ON_OFF_2,l_tempByte2);
					eeprom_write_byte(ACC_ANYMOTION_TIME_2 ,l_tempByte3);	
					eeprom_write_byte(NO_FX_MN_TIME_2,l_tempByte4);	
				}
				
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);				
			break;
		}
		
		case PING_REQ://9
		{	
			USART_SendData_s( DEBUG_COM,(unsigned char *)"PING_REQ////////////\r\n");
			config.pingReqFlag = TRUE;				
			break;		
		}

		case SET_SLEEP://10
		{				
			dataExtractor2Comma(l_requestContent,l_tempBuffer,l_tempBuff1);	
			l_tempByte1 = atoi((const char*)l_tempBuffer);// convert AASCI to integer	
			if(l_tempByte1<=65535)
			{
				config.sleepModeTimeout = l_tempByte1;
				strcpy((char*)config.sleepModeSetting,(char*)l_tempBuff1);
			
				multiByteContentWrite(SLEEP_SET_START,SLEEP_SET_LEN,config.sleepModeSetting);									
				eeprom_write_word(SLEEP_TIME_LSB,config.sleepModeTimeout);	
			
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}
		
		case TIME_ZONE_SETTING://20
		{						
			l_tempByte1 = atoi((const char*)l_requestContent);// convert AASCI to integer			
			if(l_tempByte1<21)
			{
				config.timeZoneSettings = l_tempByte1;// copy the content to configurations						
			
				eeprom_write_byte(TIME_ZONE_S,config.timeZoneSettings);// write settings to EEPROM	
			
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}				
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);					
			break;
		}
		
		case REBOOT_GPS://21
		{		
			
			GPS_RESET_ON();			
			Delay_ms(500);			
			GPS_RESET_OFF();						
			requestReplyGenOk(a_receivedRequest);//generates the reply of request						
			ConfigReplyReady(a_receivedRequest,a_requestType);									
			break;	
		}	
		
		case CLEAR_T_O://22
		{
			if((a_receivedRequest[9]-'0')  <=3)
			{
				if(a_receivedRequest[9]  == '1')// reset odometer
				{
					config.distanceTotal = 0;
					eeprom_write_dword(travelDistanceStart,config.distanceTotal);				
				}
				else if(a_receivedRequest[9]  == '2')
				{
					config.totalTimeTravelled = 0;
					eeprom_write_dword(travelTimeStart,config.totalTimeTravelled);				
				}	
				else if(a_receivedRequest[9]  == '3')
				{
					config.totalTimeTravelled = 0;
					config.distanceTotal = 0;
					eeprom_write_dword(travelDistanceStart,config.distanceTotal);	
					eeprom_write_dword(travelTimeStart,config.totalTimeTravelled);				
				}
				requestReplyGenOk(a_receivedRequest);//generates the reply of request	
			}			
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);												
			break;	
		}			
	
		case FCT_RST://23
		{
			USART_SendData_s( DEBUG_COM,(unsigned char *)"Factory Reset////////////\r\n");
			eeprom_write_byte((CONFIG_OFFSET+3),0xFF);// wash EEPROM locations
			eeprom_write_byte((CONFIG_OFFSET+4),0xFF);// wash EEPROM locations
			eeprom_write_byte((CONFIG_OFFSET+5),0xFF);// wash EEPROM locations	

			//This function initiate a system reset request to reset the MCU.
			NVIC_SystemReset();
		
			break;		
		}		
		
		case OUT_CONTROL://24
		{																
			dataExtractor2Comma(l_requestContent,speedAAsci,relaySwitchData);
			l_tempByte1=atoi((const char*)speedAAsci);// convert AASCI to integer	
			if(l_tempByte1<=255)
			{
				ioRead.relaySwitchSpeedInteger = l_tempByte1;				
				if(relaySwitchData[0] == '1')//Fuel relay cut
				{
					if(ioRead.relaySwitchSpeedInteger == 0)// switch relay not depending on any condition
					{
						config.FuelrelayCutFlag = TRUE;
						FUEL_KS_CUT;	
						eeprom_write_byte(FUEL_RELAY_STATE,TRUE);
					}					
					ioRead.FuelrelayState = TRUE;
					ioRead.FuelrelayStateChangeFlag = TRUE;
				}
				else if(relaySwitchData[0] == '0')//Fuel relay make
				{
					if(ioRead.relaySwitchSpeedInteger == 0)
					{					
						config.FuelrelayCutFlag = 2;
						FUEL_KS_MAKE;
						eeprom_write_byte(FUEL_RELAY_STATE,FALSE);
					}							
					ioRead.FuelrelayState = FALSE;
					ioRead.FuelrelayStateChangeFlag = TRUE;				
				}	
				//////////////////////////////////////////////////////////////	
				if(relaySwitchData[1] == '1')//Relay1 cut
				{
					if(ioRead.relaySwitchSpeedInteger == 0)
					{
						config.relay1CutFlag = TRUE;
						RELAY1_CUT;
						eeprom_write_byte(RELAY1_STATE,TRUE);
					}					
					ioRead.relay1State = TRUE;
					ioRead.relay1StateChangeFlag = TRUE;
				}
				else if(relaySwitchData[1] == '0')//Relay1 make
				{
					if(ioRead.relaySwitchSpeedInteger == 0)
					{
						config.relay1CutFlag = 2;
						RELAY1_MAKE;
						eeprom_write_byte(RELAY1_STATE,FALSE);
					}					
					ioRead.relay1State = FALSE;
					ioRead.relay1StateChangeFlag = TRUE;
				}			
				//////////////////////////////////////////////////////////////
				if(relaySwitchData[2] == '1')//Relay2 cut
				{
					if(ioRead.relaySwitchSpeedInteger == 0)
					{
						config.relay2CutFlag = TRUE;
						RELAY2_CUT;
						eeprom_write_byte(RELAY2_STATE,TRUE);
					}					
					ioRead.relay2State = TRUE;
					ioRead.relay2StateChangeFlag = TRUE;
				}
				else if(relaySwitchData[2] == '0')//Relay2 make
				{
					if(ioRead.relaySwitchSpeedInteger == 0)
					{
						config.relay2CutFlag = 2;
						RELAY2_MAKE;
						eeprom_write_byte(RELAY2_STATE,FALSE);
					}					
					ioRead.relay2State = FALSE;
					ioRead.relay2StateChangeFlag = TRUE;
				}	
				requestReplyGenOk(a_receivedRequest);//generates the reply of request	
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);	
			break;
		}		

		case GET_FM_V://25
		{						
			firmVerGen(a_receivedRequest);						
			ConfigReplyReady(a_receivedRequest,a_requestType);						
			break;	
		}	
		
		case REBOOT_GSM://26
		{	
			gsmPowerOff();			
			requestReplyGenOk(a_receivedRequest);//generates the reply of request						
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;	
			
		}	
	
		case SET_OVER_SPEED://27
		{		
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor3Comma(l_requestContent,l_tempBuff1,l_tempBuffer,tempIOData);			
			l_tempByte1 = atoi((const char*)l_tempBuffer);// convert AASCI to integer	
			l_tempByte2 = atoi((const char*)tempIOData);// convert AASCI to integer
			
			if((tempSim==1 || tempSim==2 || tempSim==3) && l_tempByte1>=0 && l_tempByte1<=255 && l_tempByte2<=255)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.speedLimit = l_tempByte1;// copy the content to configurations		
					config.overSpeedDiffA=l_tempByte2;
				}
				
				if(tempSim == 1 || tempSim == 3)
				{			
					eeprom_write_word(Speed_Limit_S_1,l_tempByte1);
					eeprom_write_byte(SPED_DIFF_SET_1,l_tempByte2);					
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_word(Speed_Limit_S_2,l_tempByte1);
					eeprom_write_byte(SPED_DIFF_SET_2,l_tempByte2);	
				}
				
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}	
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			
			break;
		}	
		
		case EEPROM_EN_DS://28
		{			
			USART_SendData_s( DEBUG_COM,(unsigned char *)"EEPROM_EN_DS////////////:\r\n");		
			dataExtractor2Comma(l_requestContent,l_tempBuffer,tempIOData);			
			l_tempByte1 = atoi((const char*)tempIOData);// convert AASCI to integer	
			
			tempSim = (a_receivedRequest[9] - '0');		
			if(tempSim<2 && l_tempByte1<=3)
			{
				config.eepromRecovSett = tempSim;		
				config.eepromAlgoSet = l_tempByte1;	
				eeprom_write_byte(EEPROM_RECOV,config.eepromRecovSett);			
				eeprom_write_byte(EEPROM_ALGO_SET,config.eepromAlgoSet);
						
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);							
			break;
		}	
		
		case SIM_SHFT_EN_DS://29
		{				
			USART_SendData_s( DEBUG_COM,(unsigned char *)"SIM_SHFT_EN_DS////////////:\r\n");		
			tempSim = (a_receivedRequest[9] - '0');		
			if(tempSim<2)
			{
				config.simShiftEn = tempSim;		
				eeprom_write_byte(SIM_SHFT_EEPRM,config.simShiftEn);			
						
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}				
			ConfigReplyReady(a_receivedRequest,a_requestType);						
			break;
		}	
		
		case ANALOG_INPUT_SETTING://30
		{								
			dataExtractor6Comma(l_requestContent,tempIOData,l_tempBuffer,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4);
			l_tempByte1 = atoi((const char*)l_tempBuffer);
			l_tempByte2 = atoi((const char*)l_tempBuff1);
			l_tempByte3 = atoi((const char*)l_tempBuff2);
			l_tempByte4 = atoi((const char*)l_tempBuff3);
			l_tempByte5 = atoi((const char*)l_tempBuff4);
			if((a_receivedRequest[9]-'0') <=4  && l_tempByte1<=10 && l_tempByte2<=5 && l_tempByte3<=28000 && l_tempByte4<=28000 && l_tempByte5<=28000)
			{
				if(a_receivedRequest[9] == '1')	
				{
					config.AIO1HysPoll = l_tempByte1;
					config.AIO1HysAvgSamples = l_tempByte2;
					config.AIO1HysMaxValue = l_tempByte3;
					config.AIO1HysMinValue = l_tempByte4;
					config.AIO1HysDifference = l_tempByte5;
					
					eeprom_write_byte(AIO1_POLL,config.AIO1HysPoll);	
					eeprom_write_byte(AIO1_AVG_SAMPLES,config.AIO1HysAvgSamples);
					eeprom_write_word(AIO1_MAX,config.AIO1HysMaxValue);	
					eeprom_write_word(AIO1_MIN,config.AIO1HysMinValue);
					eeprom_write_word(AIO1_DIFFERENCE,config.AIO1HysDifference);	
		
				}
				else if(a_receivedRequest[9] == '2')	
				{
					config.AIO2HysPoll = l_tempByte1;
					config.AIO2HysAvgSamples = l_tempByte2;
					config.AIO2HysMaxValue = l_tempByte3;
					config.AIO2HysMinValue = l_tempByte4;
					config.AIO2HysDifference = l_tempByte5;
					
					eeprom_write_byte(AIO2_POLL,config.AIO2HysPoll);	
					eeprom_write_byte(AIO2_AVG_SAMPLES,config.AIO2HysAvgSamples);
					eeprom_write_word(AIO2_MAX,config.AIO2HysMaxValue);	
					eeprom_write_word(AIO2_MIN,config.AIO2HysMinValue);
					eeprom_write_word(AIO2_DIFFERENCE,config.AIO2HysDifference);	
				}
				else if(a_receivedRequest[9] == '3')	
				{
					config.AIO3HysPoll = l_tempByte1;
					config.AIO3HysAvgSamples = l_tempByte2;
					config.AIO3HysMaxValue = l_tempByte3;
					config.AIO3HysMinValue = l_tempByte4;
					config.AIO3HysDifference = l_tempByte5;
					
					eeprom_write_byte(AIO3_POLL,config.AIO3HysPoll);	
					eeprom_write_byte(AIO3_AVG_SAMPLES,config.AIO3HysAvgSamples);
					eeprom_write_word(AIO3_MAX,config.AIO3HysMaxValue);	
					eeprom_write_word(AIO3_MIN,config.AIO3HysMinValue);
					eeprom_write_word(AIO3_DIFFERENCE,config.AIO3HysDifference);	
				}
				else if(a_receivedRequest[9] == '4')	
				{
					config.AIO4HysPoll = l_tempByte1;
					config.AIO4HysAvgSamples = l_tempByte2;
					config.AIO4HysMaxValue = l_tempByte3;
					config.AIO4HysMinValue = l_tempByte4;
					config.AIO4HysDifference = l_tempByte5;
					
					eeprom_write_byte(AIO4_POLL,config.AIO4HysPoll);	
					eeprom_write_byte(AIO4_AVG_SAMPLES,config.AIO4HysAvgSamples);
					eeprom_write_word(AIO4_MAX,config.AIO4HysMaxValue);	
					eeprom_write_word(AIO4_MIN,config.AIO4HysMinValue);
					eeprom_write_word(AIO4_DIFFERENCE,config.AIO4HysDifference);	
				}
			

				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}		
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;			
		}	
		
		case MCU_RESET://31
		{
			USART_SendData_s( DEBUG_COM,(unsigned char *)"MCU Reset////////////\r\n");
			//This function initiate a system reset request to reset the MCU.
			NVIC_SystemReset();
		
			break;
		}
		
		case IO_SMS_SETTINGS://33
		{												
			dataExtractor3Comma(l_requestContent,l_tempBuffer,tempIONumberAAsci,tempIOData);
			tempSim = atoi((const char*)l_tempBuffer);// convert AASCI to integer	
			tempIONumber = atoi((const char*)tempIONumberAAsci);// convert AASCI to integer	
			
			if((tempSim==1 || tempSim==2 || tempSim==3) && tempIONumber<=4)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					switch (tempIONumber)// check which io needs to be configured
					{
						case IGNITION:
						{
							strcpy((char*)config.ignitionConfigure,(const char*)tempIOData);	
							break;	
						}
						case SOS:
						{
							strcpy((char*)config.sosConfigure,(const char*)tempIOData);	
							break;	
						}
						case DI_1:
						{
							strcpy((char*)config.IO1Configure,(const char*)tempIOData);	
							break;	
						}
						case DI_2:
						{
							strcpy((char*)config.IO2Configure,(const char*)tempIOData);	
							break;	
						}
						case DI_3:
						{
							strcpy((char*)config.IO3Configure,(const char*)tempIOData);	
							break;	
						}				
						default:
							break;							
					}
		
				}
				
				if(tempSim == 1 || tempSim == 3)
				{			
					switch (tempIONumber)// check which io needs to be configured
					{
						case IGNITION:
						{
							multiByteContentWrite(IO_CONFIG_IG_START_1,IO_CONFIG_IG_ArrayLen_1,tempIOData);	
							break;	
						}
						case SOS:
						{
							multiByteContentWrite(IO_CONFIG_SS_START_1,IO_CONFIG_SS_ArrayLen_1,tempIOData);
							break;	
						}
						case DI_1:
						{
							multiByteContentWrite(IO_CONFIG_1_START_1,IO_CONFIG_1_ArrayLen_1,tempIOData);	
							break;	
						}
						case DI_2:
						{
							multiByteContentWrite(IO_CONFIG_2_START_1,IO_CONFIG_2_ArrayLen_1,tempIOData);		
							break;	
						}
						case DI_3:
						{
							multiByteContentWrite(IO_CONFIG_3_START_1,IO_CONFIG_3_ArrayLen_1,tempIOData);		
							break;	
						}
						default:
							break;
					}								
				}
				if(tempSim == 2 || tempSim == 3)
				{
					switch (tempIONumber)// check which io needs to be configured
					{
						case IGNITION:
						{
							multiByteContentWrite(IO_CONFIG_IG_START_2,IO_CONFIG_IG_ArrayLen_2,tempIOData);	
							break;	
						}
						case SOS:
						{
							multiByteContentWrite(IO_CONFIG_SS_START_2,IO_CONFIG_SS_ArrayLen_2,tempIOData);
							break;	
						}
						case DI_1:
						{
							multiByteContentWrite(IO_CONFIG_1_START_2,IO_CONFIG_1_ArrayLen_2,tempIOData);	
							break;	
						}
						case DI_2:
						{
							multiByteContentWrite(IO_CONFIG_2_START_2,IO_CONFIG_2_ArrayLen_2,tempIOData);		
							break;	
						}
						case DI_3:
						{
							multiByteContentWrite(IO_CONFIG_3_START_2,IO_CONFIG_3_ArrayLen_2,tempIOData);		
							break;	
						}
						default:
							break;
					}
				}
				
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}	
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the reply of request	
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);						
			break;
		}
		
		case SMS_NUMBER_CONFIGURE://34
		{													
			tempSim = (a_receivedRequest[9] - '0');	
			if(tempSim<=6)
			{
				if(tempSim == 1)
				{
					arrayInit2Zero(config.ServerCallCellNo1,ServerSMSCellNoSize);
					dataExtractor2Comma(l_requestContent,numberTypeAAsci,config.ServerCallCellNo1);				
					multiByteContentWrite(SERVER_VOICE_CELL1_START,SERVER_VOICE_CELL1_ArrayLen,config.ServerCallCellNo1);	
				}
				else if(tempSim == 2)
				{
					arrayInit2Zero(config.ServerCallCellNo2,ServerSMSCellNoSize);
					dataExtractor2Comma(l_requestContent,numberTypeAAsci,config.ServerCallCellNo2);				
					multiByteContentWrite(SERVER_VOICE_CELL2_START,SERVER_VOICE_CELL2_ArrayLen,config.ServerCallCellNo2);	
				}
				else if(tempSim == 3)
				{
					arrayInit2Zero(config.ServerCallCellNo3,ServerSMSCellNoSize);
					dataExtractor2Comma(l_requestContent,numberTypeAAsci,config.ServerCallCellNo3);				
					multiByteContentWrite(SERVER_VOICE_CELL3_START,SERVER_VOICE_CELL3_ArrayLen,config.ServerCallCellNo3);		
				}	
				else if(tempSim == 4)
				{
					arrayInit2Zero(config.ServerSMSCellNo1,ServerSMSCellNoSize);
					dataExtractor2Comma(l_requestContent,numberTypeAAsci,config.ServerSMSCellNo1);				
					multiByteContentWrite(SERVER_SMS_CELL1_START,SERVER_SMS_CELL1_ArrayLen,config.ServerSMSCellNo1);		
				}
				else if(tempSim == 5)
				{
					arrayInit2Zero(config.ServerSMSCellNo1,ServerSMSCellNoSize);
					dataExtractor2Comma(l_requestContent,numberTypeAAsci,config.ServerSMSCellNo1);				
					multiByteContentWrite(SERVER_SMS_CELL1_START,SERVER_SMS_CELL2_ArrayLen,config.ServerSMSCellNo1);		
				}
				else if(tempSim == 6)
				{
					arrayInit2Zero(config.ServerSMSCellNo1,ServerSMSCellNoSize);
					dataExtractor2Comma(l_requestContent,numberTypeAAsci,config.ServerSMSCellNo1);				
					multiByteContentWrite(SERVER_SMS_CELL1_START,SERVER_SMS_CELL3_ArrayLen,config.ServerSMSCellNo1);		
				}
				requestReplyGenOk(a_receivedRequest);//generates the reply of request		
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the reply of request		
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);							
			break;	
		}			
	
		case SMS_HEADER_CONFIGURE://35
		{	
			tempSim = (a_receivedRequest[9] - '0');	
			if(tempSim<=4)
			{			
				dataExtractor2Comma(l_requestContent,headerTypeAAsci,tempHeaderBuffer);			
				headerTypeInt = atoi((const char*)headerTypeAAsci);// convert AASCI to integer				
				headerLength =  strnlen((const char*)tempHeaderBuffer,11);				
				if(headerLength<10)
				{
					switch (headerTypeInt)
					{
						case IGNITION:
						{
							strcpy((char*)header.Ignition_Header,(const char*)tempHeaderBuffer);					
							break;
						}
						case SOS:
						{
							strcpy((char*)header.SOS_Header,(const char*)tempHeaderBuffer);					
							break;
						}
						case DI_1:
						{
							strcpy((char*)header.DI1Header,(const char*)tempHeaderBuffer);					
							multiByteContentWrite(DIO1_HEAD_ST,DIO1_HEAD_LN,header.DI1Header);
							break;
						}
						case DI_2:
						{
							strcpy((char*)header.DI2Header,(const char*)tempHeaderBuffer);					
							multiByteContentWrite(DIO2_HEAD_ST,DIO2_HEAD_LN,header.DI2Header);
							break;
						}
						case DI_3:
						{
							strcpy((char*)header.DI3Header,(const char*)tempHeaderBuffer);		
							multiByteContentWrite(DIO3_HEAD_ST,DIO3_HEAD_LN,header.DI3Header);			
							break;
						}
						default:
							break;
					}					
					requestReplyGenOk(a_receivedRequest);//generates the reply of request		
				}
				else
				{
					requestReplyGenError(a_receivedRequest);//generates the reply of request
				}
				ConfigReplyReady(a_receivedRequest,a_requestType);
			}								
			break;	
		}		
		
		case GET_ODOMETER_VALUE://36
		{
			arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array		
			strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI	
			strcat((char*)config.replyToSend,(const char*)",");// con tracker IMEI	
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}	
			strcat((char*)config.replyToSend,(const char*)",S36,");
			
			odometer_reading =  eeprom_read_dword(travelDistanceStart);
			sprintf(( char*)G_sprintfBuffer,"%d",odometer_reading);	
			strcat((char*)config.replyToSend,(const char*)G_sprintfBuffer);// con the reply to send				
			strcat((char*)config.replyToSend,(const char*)",OK\r\n");// con the reply to send	
			ConfigReplyReady(a_receivedRequest,a_requestType);						
			break;
		}
		case SET_ODOMETER_VALUE://37
		{				
			l_tempByte1= atol((const char*)l_requestContent);// convert AASCI to integer					
			if(l_tempByte1<=4294967295U)
			{
				config.distanceTotal = l_tempByte1;															
				eeprom_write_dword(travelDistanceStart,config.distanceTotal);		
				
				requestReplyGenOk(a_receivedRequest);//generates the reply of request			
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the reply of request	
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);								
			break;	
		}	
		
		case GEO_FENCE_CONFIG://38
		{								
			dataExtractor3Comma(l_requestContent,numberTypeAAsci,l_tempBuff1,l_tempBuff2);	
			l_tempByte1= atoi((const char*)numberTypeAAsci);// convert AASCI to integer	
			l_tempByte2=strnlen((const char *)l_tempBuff1,6);
			l_tempByte3=strnlen((const char *)l_tempBuff2,16);
			if(l_tempByte1<=20 && l_tempByte2==4 && l_tempByte3<=15)
			{		
				switch(l_tempByte1)
				{
					case 1:
					{
						strcpy((char*)config.geo1Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo1AreaName,(const char*)l_tempBuff2);	
						
						
						multiByteContentWrite(GEO1SMSStart,GEO1SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO1NameStart,GEO1NameLen,l_tempBuff2);	
						break;
					}
					case 2:
					{
						strcpy((char*)config.geo2Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo2AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO2SMSStart,GEO2SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO2NameStart,GEO2NameLen,l_tempBuff2);	
						break;
					}
					case 3:
					{
						strcpy((char*)config.geo3Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo3AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO3SMSStart,GEO3SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO3NameStart,GEO3NameLen,l_tempBuff2);	
						break;
					}
					case 4:
					{
						strcpy((char*)config.geo4Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo4AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO4SMSStart,GEO4SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO4NameStart,GEO4NameLen,l_tempBuff2);	
						break;
					}
					case 5:
					{
						strcpy((char*)config.geo5Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo5AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO5SMSStart,GEO5SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO5NameStart,GEO5NameLen,l_tempBuff2);	
						break;
					}
					case 6:
					{
						strcpy((char*)config.geo6Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo6AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO6SMSStart,GEO6SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO6NameStart,GEO6NameLen,l_tempBuff2);	
						break;
					}
					case 7:
					{
						strcpy((char*)config.geo7Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo7AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO7SMSStart,GEO7SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO7NameStart,GEO7NameLen,l_tempBuff2);	
						break;
					}
					case 8:
					{
						strcpy((char*)config.geo8Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo8AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO8SMSStart,GEO8SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO8NameStart,GEO8NameLen,l_tempBuff2);	
						break;
					}
					case 9:
					{
						strcpy((char*)config.geo9Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo9AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO9SMSStart,GEO9SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO9NameStart,GEO9NameLen,l_tempBuff2);	
						break;
					}
					case 10:
					{
						strcpy((char*)config.geo10Configure,(const char*)l_tempBuff1);//SMS setting configure
						strcpy((char*)config.geo10AreaName,(const char*)l_tempBuff2);	
						
						multiByteContentWrite(GEO10SMSStart,GEO10SMSLen,l_tempBuff1);	
						multiByteContentWrite(GEO10NameStart,GEO10NameLen,l_tempBuff2);	
						break;
					}					
				}
				requestReplyGenOk(a_receivedRequest);//generates the reply of request	
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the reply of request	
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);				
			break;	
		}	

		
		case QUEUE_DELETE://39
		{

			eeprom.eepromReadPointer = 0;
			eeprom.eepromWritePointer = 0;
			eeprom.eepromRecoveryEnable = FALSE ;
						
			requestReplyGenOk(a_receivedRequest);//generates the reply of request						
			ConfigReplyReady(a_receivedRequest,a_requestType);		
			break;
		}

		case ALERTS_ENCODE://40
		{		
			USART_SendData_s( DEBUG_COM,(unsigned char *)"ALERTS_ENCODE////////////:\r\n");	
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);
			l_tempByte1=strnlen((const char*)l_tempBuff2,35);
			if((tempSim==1 || tempSim==2 || tempSim==3) && l_tempByte1==32)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					strcpy((char*)config.AlertsOnOffEncode,(const char*)l_tempBuff2);//(Reason code/Alerts En/Ds encode)2bytes=1hex(00-FF) encoded alerts;16 hex(32 bytes)
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					multiByteContentWrite(REASON_CODE_SET_1,REASON_CODE_LEN_1,l_tempBuff2);	
				}
				if(tempSim == 2 || tempSim == 3)
				{
					multiByteContentWrite(REASON_CODE_SET_2,REASON_CODE_LEN_2,l_tempBuff2);	
				}
					
				USART_SendData_s(DEBUG_COM,config.AlertsOnOffEncode);
				USART_SendData_s(DEBUG_COM,"\r\n");
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}			
		
		
		case FIRMWARE_JUMP_CALL://42
		{			
			USART_SendData_s( DEBUG_COM,(unsigned char*)"FIRMWARE_JUMP_CALL\r\n");			
			requestReplyGenOk(a_receivedRequest);//generates the reply of request
			config.jmpCallFlag = TRUE;	
			config.replyReadyGprs = TRUE;// set if request is send from GPRS			
			break;
		}		
		
		case IDLE_TIME_SET://45
		{	
			tempSim = (a_receivedRequest[9] - '0');
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);
			l_tempByte1 = atol((const char*)l_tempBuff2);// convert AASCI to long	
			if((tempSim==1 || tempSim==2 || tempSim==3) && l_tempByte1<=65535)
			{
				if(config.simNumber==tempSim || tempSim==3)
				{
					config.idleTimeSetting=l_tempByte1;
				}
				
				if(tempSim == 1 || tempSim == 3)
				{
					eeprom_write_word(IDLE_TIME_SET_1,l_tempByte1);	
				}
				if(tempSim == 2 || tempSim == 3)
				{
					eeprom_write_word(IDLE_TIME_SET_2,l_tempByte1);
					
				}
					
				USART_SendData_s(DEBUG_COM,config.AlertsOnOffEncode);
				USART_SendData_s(DEBUG_COM,"\r\n");
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);			
			
			break;
		}
		
		
		case GEO_FENCE_TRIGGER_SET://46
		{																
			dataExtractor6Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4,l_tempBuff5,l_tempBuff6);
			l_tempByte1 = atoi((const char*)l_tempBuff1);
			l_tempByte2 = atoi((const char*)l_tempBuff2);
			l_tempByte3 = atoi((const char*)l_tempBuff3);
			l_tempByte4 = atoi((const char*)l_tempBuff4);
			l_tempByte5 = atoi((const char*)l_tempBuff5);
			l_tempByte6 = atoi((const char*)l_tempBuff6);
			if(l_tempByte1<=10 && l_tempByte2<=10 && l_tempByte3<=10 && l_tempByte4<=10 && l_tempByte5<=10 && l_tempByte6<=10)
			{
				config.FuelKSPulTimes = l_tempByte1;
				config.FuelKSPulDuration = l_tempByte2;
				config.KS1PulTimes = l_tempByte3;
				config.KS1PulDuration = l_tempByte4;
				config.KS2PulTimes = l_tempByte5;
				config.KS2PulDuration = l_tempByte6;
					
				eeprom_write_byte(FUEL_KS_PULSES,config.FuelKSPulTimes);	
				eeprom_write_byte(FUEL_KS_DURATION,config.FuelKSPulDuration);
				eeprom_write_byte(KS1_PULSES,config.KS1PulTimes);	
				eeprom_write_byte(KS1_DURATION,config.KS1PulDuration);
				eeprom_write_byte(KS2_PULSES,config.KS2PulTimes);	
				eeprom_write_byte(KS2_DURATION,config.KS2PulDuration);	
				
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}		
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}	
				
		case SHARP_TURN_SETTING://47
		{		
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);			
			l_tempByte1 = atoi((const char*)l_tempBuff1);// convert AASCI to integer	
			l_tempByte2 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			
			if(l_tempByte1>=1 && l_tempByte1<=10 && l_tempByte2>=1 && l_tempByte2<=180)
			{
				config.SharpTurnHysPoll=l_tempByte1;
				config.SharpTurnThresh=l_tempByte2;
				
				eeprom_write_byte(SHARP_TURN_DURATION,l_tempByte1);
				eeprom_write_byte(SHARP_TURN_THRESHOLD,l_tempByte2);					
				
				requestReplyGenOk(a_receivedRequest);//generates the OK reply of request
			}	
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			break;
		}	
		
		
		
		case DIGITAL_INPUT_SET://74
		{							
			dataExtractor3Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3);
			l_tempByte1 = atoi((const char*)l_tempBuff2);
			l_tempByte2 = atoi((const char*)l_tempBuff3);
			if((a_receivedRequest[9]-0x30) <=6  && l_tempByte1<=10 && l_tempByte2<=5)
			{
				if(a_receivedRequest[9] == '1')	
				{
					config.MainPwrHysPoll = l_tempByte1;
					config.MainPwrHysPollMinTrueAlert = l_tempByte2;
					
					eeprom_write_byte(MAIN_PWR_POLL,config.MainPwrHysPoll);	
					eeprom_write_byte(MAIN_PWR_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
				}
				else if(a_receivedRequest[9] == '2')	
				{
					config.ignitionHysPoll = l_tempByte1;
					config.ignitionHysPollMinTrueAlert = l_tempByte2;
					
					eeprom_write_byte(IGNITION_POLL,config.MainPwrHysPoll);	
					eeprom_write_byte(IGNITION_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
				}
				else if(a_receivedRequest[9] == '3')	
				{
					config.SOSHysPoll = l_tempByte1;
					config.SOSHysPollMinTrueAlert = l_tempByte2;
					
					eeprom_write_byte(SOS_POLL,config.MainPwrHysPoll);	
					eeprom_write_byte(SOS_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
				}
				else if(a_receivedRequest[9] == '4')	
				{
					config.DIO1HysPoll = l_tempByte1;
					config.DIO1HysPollMinTrueAlert = l_tempByte2;
					
					eeprom_write_byte(DIO1_POLL,config.MainPwrHysPoll);	
					eeprom_write_byte(DIO1_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
				}
				else if(a_receivedRequest[9] == '5')	
				{
					config.DIO2HysPoll = l_tempByte1;
					config.DIO2HysPollMinTrueAlert = l_tempByte2;
					
					eeprom_write_byte(DIO2_POLL,config.MainPwrHysPoll);	
					eeprom_write_byte(DIO2_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
				}
				else if(a_receivedRequest[9] == '6')	
				{
					config.DIO3HysPoll = l_tempByte1;
					config.DIO3HysPollMinTrueAlert = l_tempByte2;
					
					eeprom_write_byte(DIO3_POLL,config.MainPwrHysPoll);	
					eeprom_write_byte(DIO3_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
				}

				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}		
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;									
		}	
		
		case IAP_EN_DS://76
		{		
			USART_SendData_s( DEBUG_COM,"IAP_EN_DS////////////:\r\n");	
			
			dataExtractor8Comma(l_requestContent,l_tempBuffer,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4,l_tempBuff5,l_tempBuff6,tempIOData);	
			tempSim = (a_receivedRequest[9] - '0');	
			l_tempByte1 = atoi((const char*)tempIOData);
			l_tempByte2=strnlen((const char*)l_tempBuff1,21);//IP
			l_tempByte3=strnlen((const char*)l_tempBuff2,6);//Port
			l_tempByte4=strnlen((const char*)l_tempBuff3,21);//Path
			l_tempByte5=strnlen((const char*)l_tempBuff4,26);//UserName
			l_tempByte6=strnlen((const char*)l_tempBuff5,16);//Password
			PointNo=strnlen((const char*)l_tempBuff6,26);//FileName
			
			if(tempSim<3 && l_tempByte1<=255 && l_tempByte2<=20 && l_tempByte3<=5 && l_tempByte4<=20 && l_tempByte5<=25 && l_tempByte6<=15 && PointNo<=25)
			{
				if(tempSim==1 || tempSim==2)
				{
					config.IAPSetting = TRUE;
					config.FTPSetting = TRUE;	
					if(tempSim==2)
					{
						IAP.AfterIAPBootFlag=TRUE;//Automatically reboot after sucessfull IAP Download
					}
					else
					{
						IAP.AfterIAPBootFlag=FALSE;//Do not Automatically reboot after sucessfull IAP Download
					}
				}	
				else
				{
					config.IAPSetting = FALSE;
					config.FTPSetting = FALSE;	
				}			
				eeprom_write_byte(IAP_SETTING,config.IAPSetting);		

				IAP.RetryCount=0;				
				
				strcpy((char*)config.ftpServerIP1,(const char*)l_tempBuff1);	
				strcpy((char*)config.ftpServerPort1,(const char*)l_tempBuff2);	
				strcpy((char*)config.ftp1Path,(const char*)l_tempBuff3);	
				strcpy((char*)config.ftp1User,(const char*)l_tempBuff4);	
				strcpy((char*)config.ftp1Passsword,(const char*)l_tempBuff5);	
				strcpy((char*)config.IAPFileName,(const char*)l_tempBuff6);	
				config.IAPTry=l_tempByte1;
				eeprom_write_byte(IAP_RETRIES,config.IAPTry);
							
				USART_SendData_s(DEBUG_COM,config.ftpServerIP1);
				USART_SendData_s(DEBUG_COM,"\r\n");
				USART_SendData_s(DEBUG_COM,config.ftpServerPort1);
				USART_SendData_s(DEBUG_COM,"\r\n");
				USART_SendData_s(DEBUG_COM,config.ftp1Path);
				USART_SendData_s(DEBUG_COM,"\r\n");
				USART_SendData_s(DEBUG_COM,config.ftp1User);
				USART_SendData_s(DEBUG_COM,"\r\n");
				USART_SendData_s(DEBUG_COM,config.ftp1Passsword);
				USART_SendData_s(DEBUG_COM,"\r\n");
				USART_SendData_s( DEBUG_COM,config.IAPFileName);	
				USART_SendData_s( DEBUG_COM,"\r\n");
				USART_SendData_s(DEBUG_COM,"IAPTry=");
				sprintf(( char*)l_tempPacket,"%d",config.IAPTry);	
				USART_SendData_s(DEBUG_COM,l_tempPacket);
				USART_SendData_s(DEBUG_COM,"\r\n");
				requestReplyGenOk(a_receivedRequest);//generates the reply of request		
			}
			else
			{
				requestReplyGenError(a_receivedRequest);
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}			
		case TRACKER_ID_DATA://83
		{		
			USART_SendData_s( DEBUG_COM,"TRACKER_ID_DATA////////////:\r\n");

			dataExtractor7Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4,l_tempBuff5,l_tempBuff6,l_tempBuffer);	
			l_tempByte1=strnlen((const char*)l_tempBuff1,16);//CarReg
			l_tempByte2=strnlen((const char*)l_tempBuff2,4);//CompanyID
			l_tempByte3=strnlen((const char*)l_tempBuff3,9);//LotNo
			l_tempByte4=strnlen((const char*)l_tempBuff4,11);//PIDNo
			l_tempByte5=strnlen((const char*)l_tempBuff5,11);//CIDNo
			l_tempByte6=strnlen((const char*)l_tempBuff6,21);//SIM1No
			PointNo=strnlen((const char*)l_tempBuffer,21);//SIM2No
			
			if(l_tempByte1<=15 && l_tempByte2<=3 && l_tempByte3<=8 && l_tempByte4<=10 && l_tempByte5<=10 && l_tempByte6<=20 && PointNo<=20)
			{
				strcpy((char*)config.carRegNo,(const char*)l_tempBuff1);	
				strcpy((char*)config.CompanyID,(const char*)l_tempBuff2);	
				strcpy((char*)config.LotID,(const char*)l_tempBuff3);	
				strcpy((char*)config.PID,(const char*)l_tempBuff4);	
				strcpy((char*)config.CID,(const char*)l_tempBuff5);	
				strcpy((char*)config.SIM1No,(const char*)l_tempBuff6);	
				strcpy((char*)config.SIM2No,(const char*)l_tempBuffer);

				multiByteContentWrite(CARRegStart,CARRegArrayLen,config.carRegNo);			
				multiByteContentWrite(COMPANY_ID_START,COMAPANY_ID_LEN,config.CompanyID);			
				multiByteContentWrite(LotNoStart,LotNoArrayLen,config.LotID);			
				multiByteContentWrite(PIDNoStart,PIDNoArrayLen,config.PID);			
				multiByteContentWrite(CIDNoStart,CIDNoArrayLen,config.CID);			
				multiByteContentWrite(SIM1NoStart,SIM1NoArrayLen,config.SIM1No);			
				multiByteContentWrite(SIM2NoStart,SIM2NoArrayLen,config.SIM2No);
					
				requestReplyGenOk(a_receivedRequest);//generates the reply of request		
			}
			else
			{
				requestReplyGenError(a_receivedRequest);
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}			

			
		case SD_EN_DS://84
		{			
			USART_SendData_s( DEBUG_COM,"SD_EN_DS////////////:\r\n");		
			
			tempSim = (a_receivedRequest[9] - '0');		
			if(tempSim<2)
			{
				config.SDRecovSett = tempSim;		
				eeprom_write_byte(SD_RECOV,config.SDRecovSett);
						
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);							
			break;
		}	

		case GSM_NEWTEORK_SET://85
		{			
			dataExtractor4Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3,l_tempBuff4);	
			l_tempByte1 = atoi((const char*)l_tempBuff1);// convert AASCI to integer	
			l_tempByte2 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			l_tempByte3 = atoi((const char*)l_tempBuff3);// convert AASCI to integer
			l_tempByte4 = atoi((const char*)l_tempBuff4);// convert AASCI to integer
			
			if(l_tempByte1<=255 && l_tempByte2<=255 && l_tempByte3<=255 && l_tempByte4<=1 )
			{
				config.GSMNoNetworkTry = l_tempByte1;
				config.GSMIPTry = l_tempByte2;
				config.GSMTryDelay = l_tempByte3;
				config.GSMTryRebootEnDs = l_tempByte4;
			
				eeprom_write_byte(NETWORK_TRIES_SET,config.GSMNoNetworkTry);	
				eeprom_write_byte(IP_SIM_TRIES_SET,config.GSMIPTry);	
				eeprom_write_byte(NETWORK_TRIES_DELAY,config.GSMTryDelay);	
				eeprom_write_byte(GSM_REBOOT_SET,config.GSMTryRebootEnDs);	
			
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}
		

		
		case IMEI_PAD_EN_DS://87
		{			
			USART_SendData_s( DEBUG_COM,"IMEI_PAD_EN_DS////////////:\r\n");		
			
			tempSim = (a_receivedRequest[9] - '0');		
			if(tempSim<2)
			{
				config.IMEINoSett = tempSim;		
				eeprom_write_byte(IMEI_SETTING,config.IMEINoSett);
						
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);							
			break;
		}
		
		
		case ACK_SET://88
		{			
			dataExtractor3Comma(l_requestContent,l_tempBuff1,l_tempBuff2,l_tempBuff3);	
			l_tempByte1 = atoi((const char*)l_tempBuff1);// convert AASCI to integer	
			l_tempByte2 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			l_tempByte3 = atoi((const char*)l_tempBuff3);// convert AASCI to integer
			
			if(l_tempByte1<=1 && l_tempByte2<=255 && l_tempByte3<=255)
			{
				config.AckLayerEnDs = l_tempByte1;
				config.AckTimeout = l_tempByte2;
				config.AckTry = l_tempByte3;
			
				eeprom_write_byte(ACK_LAYER_SET,config.AckLayerEnDs);	
				eeprom_write_byte(ACK_TIMEOUT,config.AckTimeout);	
				eeprom_write_byte(ACK_TRIES,config.AckTry);	
			
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}
		
		
		case AUTHENTICATE_SET://89
		{				
			dataExtractor2Comma(l_requestContent,l_tempBuff1,l_tempBuff2);	
			l_tempByte1 = atoi((const char*)l_tempBuff1);// convert AASCI to integer	
			l_tempByte2 = atoi((const char*)l_tempBuff2);// convert AASCI to integer
			
			if(l_tempByte1<=1 && l_tempByte2<=255)
			{
				config.AuthenticateFuelKSEnDs = l_tempByte1;
				config.AuthenticateTimout = l_tempByte2;
			
				eeprom_write_byte(AUTHENTICATE_FUEL_KS_SET,config.AuthenticateFuelKSEnDs);	
				eeprom_write_byte(REAUTHENTICATION_TIMEOUT,config.AuthenticateTimout);	
			
				requestReplyGenOk(a_receivedRequest);//generates the reply of request
			}
			else
			{
				requestReplyGenError(a_receivedRequest);//generates the ERROR reply of request
			}
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}
				

									
		case READ_CONFIG_0://S50  General Settings
		{			
			arrayInit2Zero(config.replyToSend,replyToSendSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);			
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}	
			strcat((char*)config.replyToSend,(const char*)",S50,");	
			strcat((char*)config.replyToSend,(const char*)"1,");
			strcat((char*)config.replyToSend,(const char*)config.CompanyID);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.carRegNo);	
			strcat((char*)config.replyToSend,(const char*)",");					
			strcat((char*)config.replyToSend,(const char*)config.LotID);	
			strcat((char*)config.replyToSend,(const char*)",");									
			strcat((char*)config.replyToSend,(const char*)config.PID);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.CID);	
			strcat((char*)config.replyToSend,(const char*)",");				
			strcat((char*)config.replyToSend,(const char*)config.SIM1No);	
			strcat((char*)config.replyToSend,(const char*)",");		
			strcat((char*)config.replyToSend,(const char*)config.SIM2No);	
			strcat((char*)config.replyToSend,(const char*)",");	
			AppendToBuffer(config.FPSetting);
			AppendToBuffer(config.RFIDSetting);
			AppendToBuffer(config.CANSetting);
			AppendToBuffer(config.eepromRecovSett);
			AppendToBuffer(config.SDRecovSett);
			AppendToBuffer(config.simShiftEn);
			AppendToBuffer(config.IAPSetting);
			AppendToBuffer(config.AuthenticateFuelKSEnDs);
			AppendToBuffer(config.KS1PulTimes);
			AppendToBuffer(config.KS1PulDuration);
			AppendToBuffer(config.KS2PulTimes);
			AppendToBuffer(config.KS2PulDuration);
			AppendToBuffer(config.FuelKSPulTimes);
			AppendToBuffer(config.FuelKSPulDuration);
			AppendToBuffer(config.SharpTurnHysPoll);
			AppendToBuffer(config.SharpTurnThresh);
			AppendToBuffer(config.GSMNoNetworkTry);
			AppendToBuffer(config.GSMIPTry);
			AppendToBuffer(config.GSMTryDelay);
			AppendToBuffer(config.GSMTryRebootEnDs);
			AppendToBuffer(config.AuthenticateTimout);
			AppendToBuffer(config.AckLayerEnDs);
			AppendToBuffer(config.AckTimeout);
			AppendToBuffer(config.AckTry);
			AppendToBuffer(config.IAPTry);	
			strcat((char*)config.replyToSend,(const char*)config.SelectTcpUdp);	
			strcat((char*)config.replyToSend,(const char*)",");		
			AppendToBuffer(config.IMEINoSett);
			AppendToBuffer(config.eepromAlgoSet);
			AppendToBuffer(config.BuzzerSett);
			strcat((char*)config.replyToSend,(const char*)configFoot);			
			ConfigReplyReady(a_receivedRequest,a_requestType);
			dataTransmissionController();
			
			
			arrayInit2Zero(config.replyToSend,replyToSendSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);			
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S50,");	
			strcat((char*)config.replyToSend,(const char*)"2,");
			strcat((char*)config.replyToSend,(const char*)config.ServerSMSCellNo1);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.ServerSMSCellNo2);	
			strcat((char*)config.replyToSend,(const char*)",");					
			strcat((char*)config.replyToSend,(const char*)config.ServerSMSCellNo3);	
			strcat((char*)config.replyToSend,(const char*)",");									
			strcat((char*)config.replyToSend,(const char*)config.ServerCallCellNo1);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.ServerCallCellNo2);	
			strcat((char*)config.replyToSend,(const char*)",");				
			strcat((char*)config.replyToSend,(const char*)config.ServerCallCellNo3);	
			strcat((char*)config.replyToSend,(const char*)",");		
			AppendToBuffer(config.timeZoneSettings);
			strcat((char*)config.replyToSend,(const char*)config.sleepModeSetting);	
			strcat((char*)config.replyToSend,(const char*)",");		
			AppendToBuffer(config.sleepModeTimeout);
			strcat((char*)config.replyToSend,(const char*)config.smsPassword);	
			strcat((char*)config.replyToSend,(const char*)",");		
			strcat((char*)config.replyToSend,(const char*)config.gprsPassword);			
			strcat((char*)config.replyToSend,(const char*)configFoot);			
			ConfigReplyReady(a_receivedRequest,a_requestType);
			dataTransmissionController();
			
			arrayInit2Zero(config.replyToSend,replyToSendSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);			
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S50,");	
			strcat((char*)config.replyToSend,(const char*)"3,");
			AppendToBuffer(config.MainPwrHysPoll);
			AppendToBuffer(config.MainPwrHysPollMinTrueAlert);
			AppendToBuffer(config.ignitionHysPoll);
			AppendToBuffer(config.ignitionHysPollMinTrueAlert);
			AppendToBuffer(config.SOSHysPoll);
			AppendToBuffer(config.SOSHysPollMinTrueAlert);
			AppendToBuffer(config.DIO1HysPoll);
			AppendToBuffer(config.DIO1HysPollMinTrueAlert);
			AppendToBuffer(config.DIO2HysPoll);
			AppendToBuffer(config.DIO2HysPollMinTrueAlert);
			AppendToBuffer(config.DIO3HysPoll);
			AppendToBuffer(config.DIO3HysPollMinTrueAlert);
			AppendToBuffer(config.AIO1HysPoll);
			AppendToBuffer(config.AIO1HysAvgSamples);
			AppendToBuffer(config.AIO1HysMinValue);
			AppendToBuffer(config.AIO1HysMaxValue);
			AppendToBuffer(config.AIO1HysDifference);
			AppendToBuffer(config.AIO2HysPoll);
			AppendToBuffer(config.AIO2HysAvgSamples);
			AppendToBuffer(config.AIO2HysMinValue);
			AppendToBuffer(config.AIO2HysMaxValue);
			AppendToBuffer(config.AIO2HysDifference);
			AppendToBuffer(config.AIO3HysPoll);
			AppendToBuffer(config.AIO3HysAvgSamples);
			AppendToBuffer(config.AIO3HysMinValue);
			AppendToBuffer(config.AIO3HysMaxValue);
			AppendToBuffer(config.AIO3HysDifference);
			AppendToBuffer(config.AIO4HysPoll);
			AppendToBuffer(config.AIO4HysAvgSamples);
			AppendToBuffer(config.AIO4HysMinValue);
			AppendToBuffer(config.AIO4HysMaxValue);
			AppendToBuffer(config.AIO4HysDifference);
			strcat((char*)config.replyToSend,(const char*)header.Ignition_Header);
			strcat((char*)config.replyToSend,(const char*)",");		
			strcat((char*)config.replyToSend,(const char*)header.SOS_Header);			
			strcat((char*)config.replyToSend,(const char*)",");		
			strcat((char*)config.replyToSend,(const char*)header.DI1Header);	
			strcat((char*)config.replyToSend,(const char*)",");		
			strcat((char*)config.replyToSend,(const char*)header.DI2Header);	
			strcat((char*)config.replyToSend,(const char*)",");		
			strcat((char*)config.replyToSend,(const char*)header.DI3Header);	
							
			strcat((char*)config.replyToSend,(const char*)configFoot);			
			ConfigReplyReady(a_receivedRequest,a_requestType);
			dataTransmissionController();
			break;
		}		
	
		
			
		case READ_CONFIG_1://S51  SIM1 Settings
		{		
			Timer_Init(Timer4,DISABLE);// Due to EEPROM break problem as magnetometer is triggering after every 1sec
			Factory_Config_Read(FALSE,SIM1);
			
			arrayInit2Zero(l_requestContent,l_requestContentSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S51,");	
			strcat((char*)config.replyToSend,(const char*)config.serverIP1);			
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.serverPort1);			
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.serverIP2);						
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.serverPort2);		
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sim1APN);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sim1APNUser);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sim1APNPassword);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.IO1Configure);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.IO2Configure);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.IO3Configure);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.ignitionConfigure);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sosConfigure);
			strcat((char*)config.replyToSend,(const char*)",");	
			AppendToBuffer(config.transmissionTimeIGon);
			AppendToBuffer(config.heartBeatTime);
			AppendToBuffer(config.transmissionTimeSMS);
			AppendToBuffer(config.distanceTracking);
			AppendToBuffer(config.headingChange);
			AppendToBuffer(config.MD_ON_OFF_FLAG);
			AppendToBuffer(config.noGPSMNOnOff);
			AppendToBuffer(config.NextAnymotionAlertTimeOut);
			AppendToBuffer(config.gpsNoFixMNTime);
			AppendToBuffer(config.transmissionTimeIGOff);
			AppendToBuffer(config.speedLimit);
			AppendToBuffer(config.overSpeedDiffA);
			AppendToBuffer(config.idleTimeSetting);
			tempFloat = config.HarshBreakDetectTHR;			
			tempFloat = tempFloat*10;							
			sprintf((char*)l_tempBuffer,"%.0f",tempFloat);	
			strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
			strcat((char*)config.replyToSend,(const char*)",");	
			tempFloat = config.AccelerationDetectTHR;			
			tempFloat = tempFloat*10;							
			sprintf((char*)l_tempBuffer,"%.0f",tempFloat);	
			strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
			strcat((char*)config.replyToSend,(const char*)",");	
			tempFloat = config.ImpactDetectTHR;			
			tempFloat = tempFloat*10;							
			sprintf((char*)l_tempBuffer,"%.0f",tempFloat);	
			strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.AlertsOnOffEncode);								
			strcat((char*)config.replyToSend,(const char*)configFoot);	
			ConfigReplyReady(a_receivedRequest,a_requestType);
			
			Factory_Config_Read(FALSE,config.simNumber);
			Timer_Init(Timer4,ENABLE);
			break;
		}			
	
		
		case READ_CONFIG_2://C52  SIM2 Settings
		{			
			Timer_Init(Timer4,DISABLE);
			Factory_Config_Read(FALSE,SIM2);
			
			arrayInit2Zero(l_requestContent,l_requestContentSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S52,");	
			strcat((char*)config.replyToSend,(const char*)config.serverIP1);			
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.serverPort1);			
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.serverIP2);						
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.serverPort2);		
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sim1APN);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sim1APNUser);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sim1APNPassword);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.IO1Configure);	
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.IO2Configure);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.IO3Configure);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.ignitionConfigure);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.sosConfigure);
			strcat((char*)config.replyToSend,(const char*)",");	
			AppendToBuffer(config.transmissionTimeIGon);
			AppendToBuffer(config.heartBeatTime);
			AppendToBuffer(config.transmissionTimeSMS);
			AppendToBuffer(config.distanceTracking);
			AppendToBuffer(config.headingChange);
			AppendToBuffer(config.MD_ON_OFF_FLAG);
			AppendToBuffer(config.noGPSMNOnOff);
			AppendToBuffer(config.NextAnymotionAlertTimeOut);
			AppendToBuffer(config.gpsNoFixMNTime);
			AppendToBuffer(config.transmissionTimeIGOff);
			AppendToBuffer(config.speedLimit);
			AppendToBuffer(config.overSpeedDiffA);
			AppendToBuffer(config.idleTimeSetting);
			tempFloat = config.HarshBreakDetectTHR;				
			tempFloat = tempFloat*10;							
			sprintf((char*)l_tempBuffer,"%.0f",tempFloat);	
			strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
			strcat((char*)config.replyToSend,(const char*)",");	
			tempFloat = config.AccelerationDetectTHR;			
			tempFloat = tempFloat*10;							
			sprintf((char*)l_tempBuffer,"%.0f",tempFloat);	
			strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
			strcat((char*)config.replyToSend,(const char*)",");	
			tempFloat = config.ImpactDetectTHR;			
			tempFloat = tempFloat*10;							
			sprintf((char*)l_tempBuffer,"%.0f",tempFloat);	
			strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
			strcat((char*)config.replyToSend,(const char*)",");	
			strcat((char*)config.replyToSend,(const char*)config.AlertsOnOffEncode);								
			strcat((char*)config.replyToSend,(const char*)configFoot);	
			ConfigReplyReady(a_receivedRequest,a_requestType);
			
			Factory_Config_Read(FALSE,config.simNumber);
			Timer_Init(Timer4,ENABLE);
		
			break;
		}
			
		case READ_CONFIG_3://C53 PolyGeoFence Settings
		{			
			arrayInit2Zero(l_requestContent,l_requestContentSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S53,");	
			strcat((char*)config.replyToSend,(const char*)"1,");
			strcat((char*)config.replyToSend,(const char*)config.geo1Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo1AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo2Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo2AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo3Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo3AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo4Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo4AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo5Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo5AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo6Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo6AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo7Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo7AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo8Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo8AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo9Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo9AreaName);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo10Configure);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo10AreaName);		
			strcat((char*)config.replyToSend,(const char*)configFoot);	
			ConfigReplyReady(a_receivedRequest,a_requestType);
			
			dataTransmissionController();
			
			arrayInit2Zero(config.replyToSend,replyToSendSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);			
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S53,");	
			strcat((char*)config.replyToSend,(const char*)"3,");
			strcat((char*)config.replyToSend,(const char*)config.geo1UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo2UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo3UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo4UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo5UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo6UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo7UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo8UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo9UID);			
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo10UID);						
			strcat((char*)config.replyToSend,(const char*)configFoot);	
			ConfigReplyReady(a_receivedRequest,a_requestType);
			
			dataTransmissionController();
			
			arrayInit2Zero(config.replyToSend,replyToSendSize);
			strcpy((char*)config.replyToSend,(const char*)configHeader);			
			if(config.IMEINoSett==TRUE)
			{
				strcat((char*)config.replyToSend,(const char*)config.IMEI);
			}
			strcat((char*)config.replyToSend,(const char*)",S53,");	
			strcat((char*)config.replyToSend,(const char*)"4,");
			strcat((char*)config.replyToSend,(const char*)config.geo1Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo2Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo3Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo4Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo5Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo6Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo7Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo8Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo9Setting);	
			strcat((char*)config.replyToSend,(const char*)",");
			strcat((char*)config.replyToSend,(const char*)config.geo10Setting);	
			strcat((char*)config.replyToSend,(const char*)configFoot);	
			ConfigReplyReady(a_receivedRequest,a_requestType);
			break;
		}		
		
	
		case READ_CONFIG_4://54 PolyGeoFence points data 
		{
			
			for(l_tempByte1=0;l_tempByte1<10;l_tempByte1++)
			{
				GeoDataAppendToBuffer(l_tempByte1);
				ConfigReplyReady(a_receivedRequest,a_requestType);
				dataTransmissionController();
			}
			//GeoDataAppendToBuffer(19);
			//ConfigReplyReady(a_receivedRequest,a_requestType);			
			break;
		}		
		
		
		default:
		{		
			requestReplyGenError(a_receivedRequest);//generates the reply of request						
			if(a_requestType == SMS)
			{
				config.replyReadySms = TRUE;// set if request is send from SMS				
			}
			else if(a_requestType == GPRS)
			{
				config.replyReadyGprs = TRUE;// set if request is send from GPRS				
			}	
			else
			{
				requestReplyErrorConfig(a_receivedRequest);
				USART_SendData_s( DEBUG_COM,(unsigned char*)config.replyToSend);// temp 		
				///USART_SendData_s( TPMS_COM,(unsigned char*)config.replyToSend);// temp 		
			}
			break;
		}
	}
	dataTransmissionController();
}


/**
  * @brief  generates reply packet for the serial configuration manager, without IMEI
  * @param  None
  * @retval None
  */
void requestReplyErrorConfig(unsigned char* a_content)
{	
	arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array	
	arrayInit2Zero(l_tempBuffer,l_tempBufferSize);
	l_tempBuffer[0] =  ',';
	l_tempBuffer[1] =  'C';
	l_tempBuffer[2] =  a_content[6];
	l_tempBuffer[3] =  a_content[7];
	l_tempBuffer[4] =  ',';

	l_tempBuffer[5] =  'E';
	l_tempBuffer[6] =  'R';

	l_tempBuffer[7] =  '\r';
	l_tempBuffer[8] =  '\n';		
	l_tempBuffer[9] =  0x00;		
	strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI		
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);// con the reply to send				
}

/**
  * @brief  generates reply string of the respective request.
  * @param  None
  * @retval None
  */
void requestReplyGenError(unsigned char* a_content)
{
	arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array	
	arrayInit2Zero(l_tempBuffer,l_tempBufferSize);
	l_tempBuffer[0] =  ',';
	l_tempBuffer[1] =  'S';
	l_tempBuffer[2] =  a_content[6];
	l_tempBuffer[3] =  a_content[7];
	l_tempBuffer[4] =  ',';
	l_tempBuffer[5] =  'E';
	l_tempBuffer[6] =  'R';
	l_tempBuffer[7] =  '\r';
	l_tempBuffer[8] =  '\n';		
	l_tempBuffer[9] =  0x00;		
	strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI	
	strcat((char*)config.replyToSend,(const char*)",");// con tracker IMEI	
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)config.replyToSend,(const char*)config.IMEI);
	}
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);// con the reply to send				
}

/**
  * @brief  this module converts aasci values into float
  * @param  None
  * @retval None
  */
double aasciToFloatCopy(unsigned char* bufferPointer)
{
	unsigned char l_tempStrodFunction [ 10 ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array for strtod function		
	double valueReturnFloat;		
	valueReturnFloat = (strtod((const char*)bufferPointer,(char**)&l_tempStrodFunction));		
	return valueReturnFloat;
}

/**
  * @brief  generates reply string of the respective request.
  * @param  None
  * @retval None
  */
void requestReplyGenOk(unsigned char* a_content)
{
	arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array	
	arrayInit2Zero(l_tempBuffer,l_tempBufferSize);
	l_tempBuffer[0] =  ',';
	l_tempBuffer[1] =  'S';
	l_tempBuffer[2] =  a_content[6];
	l_tempBuffer[3] =  a_content[7];
	l_tempBuffer[4] =  ',';
	l_tempBuffer[5] =  'O';
	l_tempBuffer[6] =  'K';
	l_tempBuffer[7] =  '\r';
	l_tempBuffer[8] =  '\n';		
	l_tempBuffer[9] =  0x00;		
	strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI	
	strcat((char*)config.replyToSend,(const char*)",");// con tracker IMEI	
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)config.replyToSend,(const char*)config.IMEI);
	}
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);// con the reply to send				
}

/**
  * @brief  generates reply string of the respective request.
  * @param  None
  * @retval None
  */
void requestReplyGenUserData(unsigned char* a_content,uint8_t CmdStatus)
{
	//uint8_t tempcount=0;
	uint8_t tempIndex=0;
	arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array	
	arrayInit2Zero(l_tempBuffer,l_tempBufferSize);
	l_tempBuffer[0] =  ',';
	l_tempBuffer[1] =  'S';
	l_tempBuffer[2] =  a_content[6];
	l_tempBuffer[3] =  a_content[7];
	l_tempBuffer[4] =  ',';
	l_tempBuffer[5] =  'O';
	l_tempBuffer[6] =  'K';
	l_tempBuffer[7] =  ',';

	if(CmdStatus==TRUE)//User data is found
	{
		l_tempBuffer[8] =  'O';
		l_tempBuffer[9] =  'K';
		tempIndex=10;
		
		if(a_content[6]=='9' && a_content[7]=='7' )
		{
			l_tempBuffer[tempIndex++] =  ',';
//			for(tempcount=0;tempcount<USER_BLOCK_LENGTH;tempcount++)
//			{
//				if(DBBuffer[tempcount]==0x00)
//				{
//					break;
//				}
//				l_tempBuffer[tempIndex++]=DBBuffer[tempcount] ;
//				
//			}
		}
	}
	else
	{
		l_tempBuffer[8] =  'E';
		l_tempBuffer[9] =  'R';
		tempIndex=10;
	}

	l_tempBuffer[tempIndex++] =  '\r';
	l_tempBuffer[tempIndex++] =  '\n';		
	l_tempBuffer[tempIndex++] =  0x00;
	
	strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI	
	strcat((char*)config.replyToSend,(const char*)",");// con tracker IMEI	
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)config.replyToSend,(const char*)config.IMEI);
	}
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);// con the reply to send				
}

/**
  * @brief  generates the firmware version number packet for server, when server request for firmware version number
  * @param  None
  * @retval None
  */
void firmVerGen(unsigned char* a_content)
{		
	arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array	
	arrayInit2Zero(l_tempBuffer,l_tempBufferSize);// clear array	
	l_tempBuffer[0] =  ',';
	l_tempBuffer[1] =  'S';
	l_tempBuffer[2] =  a_content[6];
	l_tempBuffer[3] =  a_content[7];
	l_tempBuffer[4] =  ',';
			
	strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI	
	strcat((char*)config.replyToSend,(const char*)",");// con tracker IMEI	
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)config.replyToSend,(const char*)config.IMEI);
	}	
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);// con the reply to send				
	strcat((char*)config.replyToSend,(const char*)FM_Ver);// con the reply to send
	strcat((char*)config.replyToSend,(const char*)",OK\r\n");// con the reply to send					
	
}
/**
  * @brief  generates reply packet for the serial configuration manager, without IMEI
  * @param  None
  * @retval None
  */
void requestReplyConfig(unsigned char* a_content)
{	
	arrayInit2Zero(config.replyToSend,replyToSendSize);// clear array	
	arrayInit2Zero(l_tempBuffer,l_tempBufferSize);
	l_tempBuffer[0] =  ',';
	l_tempBuffer[1] =  'C';
	l_tempBuffer[2] =  a_content[6];
	l_tempBuffer[3] =  a_content[7];
	l_tempBuffer[4] =  ',';
	l_tempBuffer[5] =  'O';
	l_tempBuffer[6] =  'K';
	l_tempBuffer[7] =  '\r';
	l_tempBuffer[8] =  '\n';		
	l_tempBuffer[9] =  0x00;		
	strcat((char*)config.replyToSend,(const char*)SERVER_SEND_HEADER);// con tracker IMEI		
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);// con the reply to send				
}

/**
  * @brief  extract the requested data message from the data packet received from the server
  * @param  None
  * @retval None
  */
void requestContentExtract(unsigned char* a_requestContent,unsigned char* a_receivedRequest)// extracts the data content from message untill * is received at end
{	
	uint16_t l_requestIndex = 9;// start copying after the request ID
	uint16_t l_copyLoop=0;
	for (  l_copyLoop=0;l_copyLoop<gprsRequestBufferSize;l_copyLoop++)
	{				
		a_requestContent[l_copyLoop] = a_receivedRequest[l_requestIndex];
		l_requestIndex++;
		if(a_receivedRequest[l_requestIndex] == '*')
		{
			a_requestContent[l_copyLoop+1] = 0x00;
			break;
		}				
	}	
}

/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor2Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;
  unsigned char searchLoop = 0;	
	l_stringLength =  strnlen((const char*)a_contentArray,100);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == 0x00)
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				default:
				{
			
					break;
				}
			}				
		}				
	}		
}
/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor3Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;	
	unsigned char searchLoop = 0;
	l_stringLength =  strnlen((const char*)a_contentArray,100);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 2:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma3[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma3[l_dataCopyIndex] == 0x00)
						{
							break;
						}
					}										
					a_comma3[l_dataCopyIndex] = NULL_END;					
					break;	
				}				
				default:
				{
			
					break;
				}
			}				
		}				
	}
}
/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor4Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;	
	unsigned char searchLoop = 0;
	l_stringLength =  strnlen((const char*)a_contentArray,100);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}			
				case 2:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma3[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma3[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma3[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				case 3:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma4[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma4[l_dataCopyIndex] == 0x00 || a_comma4[l_dataCopyIndex] ==',')
						{
							break;
						}
					}										
					a_comma4[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				default:
				{
			
					break;
				}
			}				
		}				
	}	
}

/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor5Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;	
	unsigned char searchLoop = 0;
	l_stringLength =  strnlen((const char*)a_contentArray,100);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}			
				case 2:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma3[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma3[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma3[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				case 3:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma4[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma4[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma4[l_dataCopyIndex] = NULL_END;					
					break;				
				}	

				case 4:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma5[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma5[l_dataCopyIndex] == 0x00 || a_comma5[l_dataCopyIndex]==',')
						{
							break;
						}
					}										
					a_comma5[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				default:
				{
			
					break;
				}
			}				
		}				
	}	
}


/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor6Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5,unsigned char* a_comma6)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;	
	unsigned char searchLoop = 0;
	l_stringLength =  strnlen((const char*)a_contentArray,100);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}			
				case 2:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma3[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma3[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma3[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				case 3:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma4[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma4[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma4[l_dataCopyIndex] = NULL_END;					
					break;				
				}	

				case 4:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma5[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma5[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma5[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 5:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma6[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma6[l_dataCopyIndex] == 0x00 || a_comma6[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma6[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				
				default:
				{
			
					break;
				}
			}				
		}				
	}	
}

/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor7Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5,unsigned char* a_comma6,unsigned char* a_comma7)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;	
	unsigned char searchLoop = 0;
	l_stringLength =  strnlen((const char*)a_contentArray,200);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}			
				case 2:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma3[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma3[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma3[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				case 3:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma4[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma4[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma4[l_dataCopyIndex] = NULL_END;					
					break;				
				}	

				case 4:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma5[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma5[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma5[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 5:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma6[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma6[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma6[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 6:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma7[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma7[l_dataCopyIndex] == 0x00 || a_comma7[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma7[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				
				default:
				{
			
					break;
				}
			}				
		}				
	}	
}

/**
  * @brief  separates the comma separated data fields from the data content received.
  * @param  None
  * @retval None
  */
void dataExtractor8Comma(unsigned char* a_contentArray,unsigned char* a_comma1,unsigned char* a_comma2,unsigned char* a_comma3,unsigned char* a_comma4,unsigned char* a_comma5,unsigned char* a_comma6,unsigned char* a_comma7,unsigned char* a_comma8)
{
	unsigned char l_stringLength;
	unsigned char l_commaCount = 0;	
	unsigned char l_dataCopyIndex = 0;
	unsigned char l_searchIndexTemp = 0;	
	unsigned char searchLoop = 0;
	l_stringLength =  strnlen((const char*)a_contentArray,200);			
	for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
	{		
		a_comma1[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
		l_searchIndexTemp++;
		if(a_comma1[l_dataCopyIndex] == ',')
		{
			break;
		}
	}										
	a_comma1[l_dataCopyIndex] = NULL_END;	
	for ( searchLoop = 0;searchLoop<l_stringLength;searchLoop++)
	{
		if(a_contentArray[searchLoop] == ',')
		{
			l_commaCount++;			
			switch (l_commaCount)
			{				
				case 1:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma2[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma2[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma2[l_dataCopyIndex] = NULL_END;					
					break;				
				}			
				case 2:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma3[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma3[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma3[l_dataCopyIndex] = NULL_END;					
					break;				
				}				
				case 3:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma4[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma4[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma4[l_dataCopyIndex] = NULL_END;					
					break;				
				}	

				case 4:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma5[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma5[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma5[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 5:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma6[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma6[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma6[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 6:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma7[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma7[l_dataCopyIndex] == ',')
						{
							break;
						}
					}										
					a_comma7[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				case 7:
				{			
					l_searchIndexTemp = searchLoop; 
					for (l_dataCopyIndex = 0;l_dataCopyIndex<l_stringLength;l_dataCopyIndex++)				
					{
						l_searchIndexTemp++;
						a_comma8[l_dataCopyIndex] = a_contentArray[l_searchIndexTemp];										
						if(a_comma8[l_dataCopyIndex] == 0x00 || a_comma8[l_dataCopyIndex] ==',')
						{
							break;
						}
					}										
					a_comma8[l_dataCopyIndex] = NULL_END;					
					break;				
				}
				default:
				{
			
					break;
				}
			}				
		}				
	}	
}

/**
  * @brief  this module writes all the factory settings in internal EEPROM
  * @param  None
  * @retval None
  */
void Factory_Config_Write()// Writes Factory default settings to EEPROM on first run of programmed firmware
{
	//uint8_t tempByte=0;
	unsigned char l_programCheck[5];
	unsigned char l_parameterUpdateFlag = 0;
	uint32_t l_readLoop=3;
	unsigned int l_writeLoop=0;
	uint8_t l_readCounter=0;
	//uint8_t GeoFenceNo=0;
	//uint8_t PointNo=0;
	
	if(eeprom_read_byte(IAP_DOWNLOAD)==0xFF)
	{
		USART_SendData_s( DEBUG_COM,"IAP MCU_Reset Flag Detected\r\n");
		eeprom_write_byte(IAP_DOWNLOAD,0x00);
		config.IAPUpdatedFlag = TRUE;
	}
	
	for (  l_readLoop=(CONFIG_OFFSET+3);l_readLoop<=(CONFIG_OFFSET+5);l_readLoop++) 
	{	
		l_programCheck[l_readCounter] = eeprom_read_byte(l_readLoop);	// read 3,4,5 location of eeprom to check whether the MCU is programmed via ISP
	
		///USART_Put( DEBUG_COM,l_programCheck[l_readCounter]);	
		l_readCounter++;
	}	
		
	if(l_programCheck[0] == 0xFF && l_programCheck[1] == 0xFF /*&& l_programCheck[2] == 0xFF*/)// if MCU is programmed for the first time wash eeprom locations with 00
	{
		for (  l_writeLoop=(CONFIG_OFFSET+0);l_writeLoop<(CONFIG_OFFSET+900);l_writeLoop++)
		{					
			eeprom_write_byte(l_writeLoop,0x00);// wash EEPROM locations	
			///tempByte = eeprom_read_byte(l_writeLoop);
			///USART_Put( DEBUG_COM,tempByte);			
		}
		l_parameterUpdateFlag = TRUE;			
	}
	
	if(l_parameterUpdateFlag == TRUE)
	{	
		USART_SendData_s( DEBUG_COM,"Resetting EEPROM Config Section://///////\r\n");
		
		Config_FactorySettings_SIM1();
		////////////////////////////////////////////////////////////////////////////////
		eeprom_write_byte(FUEL_RELAY_STATE,FALSE);//make Kill relay by default
		eeprom_write_byte(RELAY1_STATE,FALSE);//make relay1 by default
		eeprom_write_byte(RELAY2_STATE,FALSE);//make relay2 by default
		
		
		multiByteContentWrite(CARRegStart,CARRegArrayLen,config.carRegNo);			
		multiByteContentWrite(COMPANY_ID_START,COMAPANY_ID_LEN,config.CompanyID);			
		multiByteContentWrite(LotNoStart,LotNoArrayLen,config.LotID);			
		multiByteContentWrite(PIDNoStart,PIDNoArrayLen,config.PID);			
		multiByteContentWrite(CIDNoStart,CIDNoArrayLen,config.CID);			
		multiByteContentWrite(SIM1NoStart,SIM1NoArrayLen,config.SIM1No);			
		multiByteContentWrite(SIM2NoStart,SIM2NoArrayLen,config.SIM2No);
		
		eeprom_write_byte(FP_SETTING,config.FPSetting);
		eeprom_write_byte(RFID_SETTING,config.RFIDSetting);
		eeprom_write_byte(CAN_SETTING,config.CANSetting);
		eeprom_write_byte(EEPROM_RECOV,config.eepromRecovSett);
		eeprom_write_byte(SD_RECOV,config.SDRecovSett);
		
		eeprom_write_byte(SIM_SHFT_EEPRM,config.simShiftEn);
		eeprom_write_byte(TPMS_SETTING,config.TPMSSetting);


		eeprom_write_byte(IAP_SETTING,config.IAPSetting);
		eeprom_write_word(IAP_RESET,(uint16_t)config.IAPResetFlag);
		eeprom_write_byte(IAP_DOWNLOAD,config.IAPDownloadedFlag);
		eeprom_write_dword(IAP_FILE_SIZE,(uint32_t)config.IAPFileSize);
		eeprom_write_dword(IAP_FILE_ADDRESS,(uint32_t)config.IAPFileAddress);
		
		eeprom_write_byte(AUTHENTICATE_FUEL_KS_SET,config.AuthenticateFuelKSEnDs);
		eeprom_write_byte(FUEL_KS_DURATION,config.FuelKSPulDuration);
		eeprom_write_byte(FUEL_KS_PULSES,config.FuelKSPulTimes);

		eeprom_write_byte(KS1_DURATION,config.KS1PulDuration);
		eeprom_write_byte(KS1_PULSES,config.KS1PulTimes);
		
		eeprom_write_byte(KS2_DURATION,config.KS2PulDuration);
		eeprom_write_byte(KS2_PULSES,config.KS2PulTimes);
				
		eeprom_write_byte(SHARP_TURN_DURATION,config.SharpTurnHysPoll);
		eeprom_write_byte(SHARP_TURN_THRESHOLD,config.SharpTurnThresh);

		eeprom_write_byte(NETWORK_TRIES_SET,config.GSMNoNetworkTry);
		eeprom_write_byte(IP_SIM_TRIES_SET,config.GSMIPTry);
		eeprom_write_byte(NETWORK_TRIES_DELAY,config.GSMTryDelay);
		eeprom_write_byte(GSM_REBOOT_SET,config.GSMTryRebootEnDs);
		eeprom_write_word(REAUTHENTICATION_TIMEOUT,config.AuthenticateTimout);
		
		eeprom_write_byte(ACK_LAYER_SET,config.AckLayerEnDs);
		eeprom_write_byte(ACK_TIMEOUT,config.AckTimeout);
		eeprom_write_byte(ACK_TRIES,config.AckTry);
		eeprom_write_byte(IAP_RETRIES,config.IAPTry);
		
		multiByteContentWrite(TCP_UDP_START,TCP_UDP_LEN,config.SelectTcpUdp);	
		eeprom_write_byte(IMEI_SETTING,config.IMEINoSett);
		eeprom_write_byte(EEPROM_ALGO_SET,config.eepromAlgoSet);
		eeprom_write_byte(BUZZER_SETTTING,config.BuzzerSett);
		
		multiByteContentWrite(SERVER_SMS_CELL1_START,SERVER_SMS_CELL1_ArrayLen,config.ServerSMSCellNo1);	
		multiByteContentWrite(SERVER_SMS_CELL2_START,SERVER_SMS_CELL2_ArrayLen,config.ServerSMSCellNo2);	
		multiByteContentWrite(SERVER_SMS_CELL3_START,SERVER_SMS_CELL3_ArrayLen,config.ServerSMSCellNo3);
		
		multiByteContentWrite(SERVER_VOICE_CELL1_START,SERVER_VOICE_CELL1_ArrayLen,config.ServerCallCellNo1);	
		multiByteContentWrite(SERVER_VOICE_CELL2_START,SERVER_VOICE_CELL2_ArrayLen,config.ServerCallCellNo2);	
		multiByteContentWrite(SERVER_VOICE_CELL3_START,SERVER_VOICE_CELL3_ArrayLen,config.ServerCallCellNo3);
		
		eeprom_write_word(TRIP_ID_LSB,ioRead.tripId);	
		eeprom_write_byte(TIME_ZONE_S,config.timeZoneSettings);
		multiByteContentWrite(SLEEP_SET_START,SLEEP_SET_LEN,config.sleepModeSetting);	
		eeprom_write_word(SLEEP_TIME_LSB,config.sleepModeTimeout);	
		
		multiByteContentWrite(GPRS_PASS_START,GPRS_PASS_ArrayLen,config.gprsPassword);		
		multiByteContentWrite(SMS_PASS_START,SMS_PASS_ArrayLen,config.smsPassword);	
		
		
		eeprom_write_byte(MAIN_PWR_POLL,config.MainPwrHysPoll);
		eeprom_write_byte(MAIN_PWR_POLL_DURATION,config.MainPwrHysPollDuration);
		eeprom_write_byte(MAIN_PWR_MIN_TRUE,config.MainPwrHysPollMinTrueAlert);
		
		eeprom_write_byte(IGNITION_POLL,config.ignitionHysPoll);
		eeprom_write_byte(IGNITION_POLL_DURATION,config.ignitionHysPollDuration);
		eeprom_write_byte(IGNITION_MIN_TRUE,config.ignitionHysPollMinTrueAlert);
			
		eeprom_write_byte(SOS_POLL,config.SOSHysPoll);
		eeprom_write_byte(SOS_POLL_DURATION,config.SOSHysPollDuration);
		eeprom_write_byte(SOS_MIN_TRUE,config.SOSHysPollMinTrueAlert);

		eeprom_write_byte(DIO1_POLL,config.DIO1HysPoll);
		eeprom_write_byte(DIO1_POLL_DURATION,config.DIO1HysPollDuration);
		eeprom_write_byte(DIO1_MIN_TRUE,config.DIO1HysPollMinTrueAlert);
		
		eeprom_write_byte(DIO2_POLL,config.DIO2HysPoll);
		eeprom_write_byte(DIO2_POLL_DURATION,config.DIO2HysPollDuration);
		eeprom_write_byte(DIO2_MIN_TRUE,config.DIO2HysPollMinTrueAlert);
		
		eeprom_write_byte(DIO3_POLL,config.DIO3HysPoll);
		eeprom_write_byte(DIO3_POLL_DURATION,config.DIO3HysPollDuration);
		eeprom_write_byte(DIO3_MIN_TRUE,config.DIO3HysPollMinTrueAlert);
		
		eeprom_write_byte(AIO1_POLL,config.AIO1HysPoll);
		eeprom_write_byte(AIO1_POLL_DURATION,config.AIO1HysPollDuration);
		eeprom_write_byte(AIO1_AVG_SAMPLES,config.AIO1HysAvgSamples);
		eeprom_write_word(AIO1_MIN,config.AIO1HysMinValue);	
		eeprom_write_word(AIO1_MAX,config.AIO1HysMaxValue);
		eeprom_write_word(AIO1_DIFFERENCE,config.AIO1HysDifference);
		
		eeprom_write_byte(AIO2_POLL,config.AIO2HysPoll);
		eeprom_write_byte(AIO2_POLL_DURATION,config.AIO2HysPollDuration);
		eeprom_write_byte(AIO2_AVG_SAMPLES,config.AIO2HysAvgSamples);
		eeprom_write_word(AIO2_MIN,config.AIO2HysMinValue);	
		eeprom_write_word(AIO2_MAX,config.AIO2HysMaxValue);
		eeprom_write_word(AIO2_DIFFERENCE,config.AIO2HysDifference);
		
		eeprom_write_byte(AIO3_POLL,config.AIO3HysPoll);
		eeprom_write_byte(AIO3_POLL_DURATION,config.AIO3HysPollDuration);
		eeprom_write_byte(AIO3_AVG_SAMPLES,config.AIO3HysAvgSamples);
		eeprom_write_word(AIO3_MIN,config.AIO3HysMinValue);	
		eeprom_write_word(AIO3_MAX,config.AIO3HysMaxValue);
		eeprom_write_word(AIO3_DIFFERENCE,config.AIO3HysDifference);
		
		eeprom_write_byte(AIO4_POLL,config.AIO4HysPoll);
		eeprom_write_byte(AIO4_POLL_DURATION,config.AIO4HysPollDuration);
		eeprom_write_byte(AIO4_AVG_SAMPLES,config.AIO4HysAvgSamples);
		eeprom_write_word(AIO4_MIN,config.AIO4HysMinValue);	
		eeprom_write_word(AIO4_MAX,config.AIO4HysMaxValue);
		eeprom_write_word(AIO4_DIFFERENCE,config.AIO4HysDifference);
		
		
		multiByteContentWrite(DIO1_HEAD_ST,DIO1_HEAD_LN,header.DI1Header);	
		multiByteContentWrite(DIO2_HEAD_ST,DIO2_HEAD_LN,header.DI2Header);	
		multiByteContentWrite(DIO3_HEAD_ST,DIO3_HEAD_LN,header.DI3Header);	
		multiByteContentWrite(SOS_HEAD_ST,SOS_HEAD_LN,header.SOS_Header);
		multiByteContentWrite(IGNITION_HEAD_ST,IGNITION_HEAD_LN,header.Ignition_Header);
		
		multiByteContentWrite(GEO1SMSStart,GEO1SMSLen,config.geo1Configure);	
		multiByteContentWrite(GEO1NameStart,GEO1NameLen,config.geo1AreaName);	
		
		multiByteContentWrite(GEO2SMSStart,GEO2SMSLen,config.geo2Configure);	
		multiByteContentWrite(GEO2NameStart,GEO2NameLen,config.geo2AreaName);
		
		multiByteContentWrite(GEO3SMSStart,GEO3SMSLen,config.geo3Configure);	
		multiByteContentWrite(GEO3NameStart,GEO3NameLen,config.geo3AreaName);
		
		multiByteContentWrite(GEO4SMSStart,GEO4SMSLen,config.geo4Configure);	
		multiByteContentWrite(GEO4NameStart,GEO4NameLen,config.geo4AreaName);
		
		multiByteContentWrite(GEO5SMSStart,GEO5SMSLen,config.geo5Configure);	
		multiByteContentWrite(GEO5NameStart,GEO5NameLen,config.geo5AreaName);
		
		multiByteContentWrite(GEO6SMSStart,GEO6SMSLen,config.geo6Configure);	
		multiByteContentWrite(GEO6NameStart,GEO6NameLen,config.geo6AreaName);
		
		multiByteContentWrite(GEO7SMSStart,GEO7SMSLen,config.geo7Configure);	
		multiByteContentWrite(GEO7NameStart,GEO7NameLen,config.geo7AreaName);
		
		multiByteContentWrite(GEO8SMSStart,GEO8SMSLen,config.geo8Configure);	
		multiByteContentWrite(GEO8NameStart,GEO8NameLen,config.geo8AreaName);
		
		multiByteContentWrite(GEO9SMSStart,GEO9SMSLen,config.geo9Configure);	
		multiByteContentWrite(GEO9NameStart,GEO9NameLen,config.geo9AreaName);
		
		multiByteContentWrite(GEO10SMSStart,GEO10SMSLen,config.geo10Configure);	
		multiByteContentWrite(GEO10NameStart,GEO10NameLen,config.geo10AreaName);
		
		multiByteContentWrite(GEO1IDStart,GEOIDLen,config.geo1UID);	
		multiByteContentWrite(GEO1_SET,GEO_SET_LEN,config.geo1Setting);
		
		multiByteContentWrite(GEO2IDStart,GEOIDLen,config.geo2UID);	
		multiByteContentWrite(GEO2_SET,GEO_SET_LEN,config.geo2Setting);
		
		multiByteContentWrite(GEO3IDStart,GEOIDLen,config.geo3UID);	
		multiByteContentWrite(GEO3_SET,GEO_SET_LEN,config.geo3Setting);
		
		multiByteContentWrite(GEO4IDStart,GEOIDLen,config.geo4UID);	
		multiByteContentWrite(GEO4_SET,GEO_SET_LEN,config.geo4Setting);
		
		multiByteContentWrite(GEO5IDStart,GEOIDLen,config.geo5UID);	
		multiByteContentWrite(GEO5_SET,GEO_SET_LEN,config.geo5Setting);
		
		multiByteContentWrite(GEO6IDStart,GEOIDLen,config.geo6UID);	
		multiByteContentWrite(GEO6_SET,GEO_SET_LEN,config.geo6Setting);
		
		multiByteContentWrite(GEO7IDStart,GEOIDLen,config.geo7UID);	
		multiByteContentWrite(GEO7_SET,GEO_SET_LEN,config.geo7Setting);
		
		multiByteContentWrite(GEO8IDStart,GEOIDLen,config.geo8UID);	
		multiByteContentWrite(GEO8_SET,GEO_SET_LEN,config.geo8Setting);
		
		multiByteContentWrite(GEO9IDStart,GEOIDLen,config.geo9UID);	
		multiByteContentWrite(GEO9_SET,GEO_SET_LEN,config.geo9Setting);
		
		multiByteContentWrite(GEO10IDStart,GEOIDLen,config.geo10UID);	
		multiByteContentWrite(GEO10_SET,GEO_SET_LEN,config.geo10Setting);
		
		////////////////////////////////////  SIM 1 //////////////////////////////////////////		
		
		multiByteContentWrite(serverIP1Start_1,serverIP1ArrayLen_1,config.serverIP1); 	
		multiByteContentWrite(serverPort1Start_1,serverPort1ArrayLen_1,config.serverPort1); 	

		multiByteContentWrite(serverIP2Start_1,serverIP2ArrayLen_1,config.serverIP2); 	
		multiByteContentWrite(serverPort2Start_1,serverPort2ArrayLen_1,config.serverPort2);
		
		multiByteContentWrite(APN1Start_1,APN1ArrayLen_1,config.sim1APN);
		multiByteContentWrite(APNUser1Start_1,APN1UserArrayLen_1,config.sim1APNUser); 
		multiByteContentWrite(APN1PassStart_1,APN1PassArrayLen_1,config.sim1APNPassword);

		multiByteContentWrite(IO_CONFIG_1_START_1,IO_CONFIG_1_ArrayLen_1,config.IO1Configure);			
		multiByteContentWrite(IO_CONFIG_2_START_1,IO_CONFIG_2_ArrayLen_1,config.IO2Configure);			
		multiByteContentWrite(IO_CONFIG_3_START_1,IO_CONFIG_3_ArrayLen_1,config.IO3Configure);			
		//multiByteContentWrite(IO_CONFIG_4_START_1,IO_CONFIG_4_ArrayLen_1,config.IO4Configure);			
		multiByteContentWrite(IO_CONFIG_IG_START_1,IO_CONFIG_IG_ArrayLen_1,config.ignitionConfigure);	
		multiByteContentWrite(IO_CONFIG_SS_START_1,IO_CONFIG_SS_ArrayLen_1,config.sosConfigure);	
		
		eeprom_write_word(TIME_TRACK_LSB_1,config.transmissionTimeIGon);	
		eeprom_write_word(SMS_TIME_TRACK_LSB_1,config.transmissionTimeSMS);	
		eeprom_write_word(HEART_BEAT_LSB_1,config.heartBeatTime);
		eeprom_write_dword(DIST_TRK_LSB1_1,config.distanceTracking);
		eeprom_write_word(HEADING_LSB_1,config.headingChange);	
		
		eeprom_write_byte(NO_FX_MN_ON_OFF_1,config.noGPSMNOnOff);
		eeprom_write_byte(MD_ON_OFF_1,config.MD_ON_OFF_FLAG);
		eeprom_write_byte(NO_FX_MN_TIME_1,config.gpsNoFixMNTime);
		eeprom_write_word(TIME_TRK_IG_OFF_SET_1,config.transmissionTimeIGOff);
		
		eeprom_write_word(Speed_Limit_S_1,config.speedLimit);
		eeprom_write_byte(SPED_DIFF_SET_1,config.overSpeedDiffA);
		eeprom_write_word(IDLE_TIME_SET_1,config.idleTimeSetting);


		multiByteContentWrite(REASON_CODE_SET_1,REASON_CODE_LEN_1,config.AlertsOnOffEncode);
						
		eepromWriteFloat(ACC_Harsh_B_TH_1,config.HarshBreakDetectTHR);
		eepromWriteFloat(ACC_ACCEL_TH_1,config.AccelerationDetectTHR);
		eepromWriteFloat(ACC_IMPACT_TH_1,config.ImpactDetectTHR);
		//eeprom_write_byte(ACC_ANYMOTION_TH_1,config.AnymotionDetectTHR);
		eeprom_write_byte(ACC_ANYMOTION_TIME_1,config.NextAnymotionAlertTimeOut);

		////////////////////////////////////  SIM 2 //////////////////////////////////////////	
		Config_FactorySettings_SIM2();
		
		multiByteContentWrite(serverIP1Start_2,serverIP1ArrayLen_2,config.serverIP1); 	
		multiByteContentWrite(serverPort1Start_2,serverPort1ArrayLen_2,config.serverPort1); 	

		multiByteContentWrite(serverIP2Start_2,serverIP2ArrayLen_2,config.serverIP2); 	
		multiByteContentWrite(serverPort2Start_2,serverPort2ArrayLen_2,config.serverPort2);
		
		multiByteContentWrite(APN1Start_2,APN1ArrayLen_2,config.sim1APN);
		multiByteContentWrite(APNUser1Start_2,APN1UserArrayLen_2,config.sim1APNUser); 
		multiByteContentWrite(APN1PassStart_2,APN1PassArrayLen_2,config.sim1APNPassword);

		multiByteContentWrite(IO_CONFIG_1_START_2,IO_CONFIG_1_ArrayLen_2,config.IO1Configure);			
		multiByteContentWrite(IO_CONFIG_2_START_2,IO_CONFIG_2_ArrayLen_2,config.IO2Configure);			
		multiByteContentWrite(IO_CONFIG_3_START_2,IO_CONFIG_3_ArrayLen_2,config.IO3Configure);			
		//multiByteContentWrite(IO_CONFIG_4_START_2,IO_CONFIG_4_ArrayLen_2,config.IO4Configure);			
		multiByteContentWrite(IO_CONFIG_IG_START_2,IO_CONFIG_IG_ArrayLen_2,config.ignitionConfigure);	
		multiByteContentWrite(IO_CONFIG_SS_START_2,IO_CONFIG_SS_ArrayLen_2,config.sosConfigure);	
		
		eeprom_write_word(TIME_TRACK_LSB_2,config.transmissionTimeIGon);	
		eeprom_write_word(SMS_TIME_TRACK_LSB_2,config.transmissionTimeSMS);	
		eeprom_write_word(HEART_BEAT_LSB_2,config.heartBeatTime);
		eeprom_write_dword(DIST_TRK_LSB1_2,config.distanceTracking);
		eeprom_write_word(HEADING_LSB_2,config.headingChange);	
		
		eeprom_write_byte(NO_FX_MN_ON_OFF_2,config.noGPSMNOnOff);
		eeprom_write_byte(MD_ON_OFF_2,config.MD_ON_OFF_FLAG);
		eeprom_write_byte(NO_FX_MN_TIME_2,config.gpsNoFixMNTime);
		eeprom_write_word(TIME_TRK_IG_OFF_SET_2,config.transmissionTimeIGOff);
		
		eeprom_write_word(Speed_Limit_S_2,config.speedLimit);
		eeprom_write_byte(SPED_DIFF_SET_2,config.overSpeedDiffA);
		eeprom_write_word(IDLE_TIME_SET_2,config.idleTimeSetting);


		multiByteContentWrite(REASON_CODE_SET_2,REASON_CODE_LEN_2,config.AlertsOnOffEncode);
						
		eepromWriteFloat(ACC_Harsh_B_TH_2,config.HarshBreakDetectTHR);
		eepromWriteFloat(ACC_ACCEL_TH_2,config.AccelerationDetectTHR);
		eepromWriteFloat(ACC_IMPACT_TH_2,config.ImpactDetectTHR);
		//eeprom_write_byte(ACC_ANYMOTION_TH_2,config.AnymotionDetectTHR);
		eeprom_write_byte(ACC_ANYMOTION_TIME_2,config.NextAnymotionAlertTimeOut);
		
		
		//////////////////////////////////////////////////////////////////////////////////
	}
}	

/**
  * @brief  this module reads all the factory settings from internal EEPROM
  * @param  None
  * @retval None
  */
void Factory_Config_Read(uint8_t ReadGeneralConfig,uint8_t SIM_No)
{
	uint8_t GeoFenceNo=0;
	uint8_t PointNo=0;
	//uint8_t temp=0;
	
	USART_SendData_s( DEBUG_COM,"Reading EEPROM Config Section://///////\r\n");
	
	if(ReadGeneralConfig==TRUE)
	{
		
//		temp=eeprom_read_byte(FUEL_RELAY_STATE);//read Kill relay last state
//		if(temp)
//		{
//			FUEL_KS_CUT;
//		}
//		else
//		{
//			FUEL_KS_MAKE;
//		}
//		temp=eeprom_read_byte(RELAY1_STATE);//read relay1 last state
//		if(temp)
//		{
//			RELAY1_CUT;
//		}
//		else
//		{
//			RELAY1_MAKE;
//		}
//		temp=eeprom_read_byte(RELAY2_STATE);//read relay2 last state
//		if(temp)
//		{
//			RELAY2_CUT;
//		}
//		else
//		{
//			RELAY2_MAKE;
//		}
		
		config.totalTimeTravelled =  eeprom_read_dword(travelTimeStart);
		config.distanceTotal =  eeprom_read_dword(travelDistanceStart);
		
		
		sprintf((char*)GPS_LOG.distanceAasci, "%lu",config.distanceTotal);//convert to ascii before sending to server
		
		USART_SendData_s(DEBUG_COM,(unsigned char*)"\r\nOdometer=");
		sprintf(( char*)G_sprintfBuffer,"%d",config.distanceTotal);	
		USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
		USART_SendData_s(DEBUG_COM,(unsigned char*)"\r\n");
		
		for(GeoFenceNo=0;GeoFenceNo<POLY_GEOFENCE;GeoFenceNo++)
		{
			for(PointNo=0;PointNo<POLY_GEOFENCEPOINT;PointNo++)
			{
				Vertex[GeoFenceNo][PointNo].x=eepromReadDouble(POLY_GEOFENCE_Start+(GeoFenceNo*160)+(PointNo*16));		
				Vertex[GeoFenceNo][PointNo].y=eepromReadDouble(POLY_GEOFENCE_Start+(GeoFenceNo*160)+((PointNo*16)+8));
			}			
		}
		
		multiByteContentRead(CARRegStart,CARRegArrayLen,config.carRegNo);			
		multiByteContentRead(COMPANY_ID_START,COMAPANY_ID_LEN,config.CompanyID);			
		multiByteContentRead(LotNoStart,LotNoArrayLen,config.LotID);			
		multiByteContentRead(PIDNoStart,PIDNoArrayLen,config.PID);			
		multiByteContentRead(CIDNoStart,CIDNoArrayLen,config.CID);			
		multiByteContentRead(SIM1NoStart,SIM1NoArrayLen,config.SIM1No);			
		multiByteContentRead(SIM2NoStart,SIM2NoArrayLen,config.SIM2No);
			
		config.FPSetting=eeprom_read_byte(FP_SETTING);
		config.RFIDSetting=	eeprom_read_byte(RFID_SETTING);
		config.CANSetting=	eeprom_read_byte(CAN_SETTING);
		config.eepromRecovSett=	eeprom_read_byte(EEPROM_RECOV);
		config.SDRecovSett=	eeprom_read_byte(SD_RECOV);
		
		config.simShiftEn=	eeprom_read_byte(SIM_SHFT_EEPRM);
		config.TPMSSetting=	eeprom_read_byte(TPMS_SETTING);
			
		config.IAPSetting=	eeprom_read_byte(IAP_SETTING);
		config.IAPResetFlag=	eeprom_read_word(IAP_RESET);
		config.IAPDownloadedFlag=	eeprom_read_byte(IAP_DOWNLOAD);
		config.IAPFileSize=	eeprom_read_dword(IAP_FILE_SIZE);
		config.IAPFileAddress=	eeprom_read_dword(IAP_FILE_ADDRESS);

		config.AuthenticateFuelKSEnDs=	eeprom_read_byte(AUTHENTICATE_FUEL_KS_SET);
		config.FuelKSPulDuration=	eeprom_read_byte(FUEL_KS_DURATION);
		config.FuelKSPulTimes=	eeprom_read_byte(FUEL_KS_PULSES);
			
		config.KS1PulDuration=	eeprom_read_byte(KS1_DURATION);
		config.KS1PulTimes=	eeprom_read_byte(KS1_PULSES);
		
		config.KS2PulDuration=	eeprom_read_byte(KS2_DURATION);
		config.KS2PulTimes=	eeprom_read_byte(KS2_PULSES);
		
		config.SharpTurnHysPoll=	eeprom_read_byte(SHARP_TURN_DURATION);
		config.SharpTurnThresh=	eeprom_read_byte(SHARP_TURN_THRESHOLD);

		config.GSMNoNetworkTry=	eeprom_read_byte(NETWORK_TRIES_SET);
		config.GSMIPTry=	eeprom_read_byte(IP_SIM_TRIES_SET);
		config.GSMTryDelay=	eeprom_read_byte(NETWORK_TRIES_DELAY);
		config.GSMTryRebootEnDs=	eeprom_read_byte(GSM_REBOOT_SET);
		config.AuthenticateTimout=	eeprom_read_word(REAUTHENTICATION_TIMEOUT);
		
		config.AckLayerEnDs=	eeprom_read_byte(ACK_LAYER_SET);
		config.AckTimeout=	eeprom_read_byte(ACK_TIMEOUT);
		config.AckTry=	eeprom_read_byte(ACK_TRIES);
		config.IAPTry=	eeprom_read_byte(IAP_RETRIES);

		multiByteContentRead(TCP_UDP_START,TCP_UDP_LEN,config.SelectTcpUdp);

		config.IMEINoSett=	eeprom_read_byte(IMEI_SETTING);
		config.eepromAlgoSet=	eeprom_read_byte(EEPROM_ALGO_SET);
		config.BuzzerSett=	eeprom_read_byte(BUZZER_SETTTING);
		
		multiByteContentRead(SERVER_SMS_CELL1_START,SERVER_SMS_CELL1_ArrayLen,config.ServerSMSCellNo1);	
		multiByteContentRead(SERVER_SMS_CELL2_START,SERVER_SMS_CELL2_ArrayLen,config.ServerSMSCellNo2);	
		multiByteContentRead(SERVER_SMS_CELL3_START,SERVER_SMS_CELL3_ArrayLen,config.ServerSMSCellNo3);
			
		multiByteContentRead(SERVER_VOICE_CELL1_START,SERVER_VOICE_CELL1_ArrayLen,config.ServerCallCellNo1);	
		multiByteContentRead(SERVER_VOICE_CELL2_START,SERVER_VOICE_CELL2_ArrayLen,config.ServerCallCellNo2);	
		multiByteContentRead(SERVER_VOICE_CELL3_START,SERVER_VOICE_CELL3_ArrayLen,config.ServerCallCellNo3);
			
		ioRead.tripId = eeprom_read_word(TRIP_ID_LSB);
		config.timeZoneSettings=	eeprom_read_byte(TIME_ZONE_S);
			
		multiByteContentRead(SLEEP_SET_START,SLEEP_SET_LEN,config.sleepModeSetting);	
		config.sleepModeTimeout = eeprom_read_word(SLEEP_TIME_LSB);
			
		multiByteContentRead(GPRS_PASS_START,GPRS_PASS_ArrayLen,config.gprsPassword);		
		multiByteContentRead(SMS_PASS_START,SMS_PASS_ArrayLen,config.smsPassword);	
		
		
		config.MainPwrHysPoll=	eeprom_read_byte(MAIN_PWR_POLL);
		config.MainPwrHysPollDuration=	eeprom_read_byte(MAIN_PWR_POLL_DURATION);
		config.MainPwrHysPollMinTrueAlert=	eeprom_read_byte(MAIN_PWR_MIN_TRUE);
		
		config.ignitionHysPoll=	eeprom_read_byte(IGNITION_POLL);
		config.ignitionHysPollDuration=	eeprom_read_byte(IGNITION_POLL_DURATION);
		config.ignitionHysPollMinTrueAlert=	eeprom_read_byte(IGNITION_MIN_TRUE);
		
		config.SOSHysPoll=	eeprom_read_byte(SOS_POLL);
		config.SOSHysPollDuration=	eeprom_read_byte(SOS_POLL_DURATION);
		config.SOSHysPollMinTrueAlert=	eeprom_read_byte(SOS_MIN_TRUE);
		
		config.DIO1HysPoll=	eeprom_read_byte(DIO1_POLL);
		config.DIO1HysPollDuration=	eeprom_read_byte(DIO1_POLL_DURATION);
		config.DIO1HysPollMinTrueAlert=	eeprom_read_byte(DIO1_MIN_TRUE);
		
		config.DIO2HysPoll=	eeprom_read_byte(DIO2_POLL);
		config.DIO2HysPollDuration=	eeprom_read_byte(DIO2_POLL_DURATION);
		config.DIO2HysPollMinTrueAlert=	eeprom_read_byte(DIO2_MIN_TRUE);

		config.DIO3HysPoll=	eeprom_read_byte(DIO3_POLL);
		config.DIO3HysPollDuration=	eeprom_read_byte(DIO3_POLL_DURATION);
		config.DIO3HysPollMinTrueAlert=	eeprom_read_byte(DIO3_MIN_TRUE);

		config.AIO1HysPoll=	eeprom_read_byte(AIO1_POLL);
		config.AIO1HysPollDuration=	eeprom_read_byte(AIO1_POLL_DURATION);
		config.AIO1HysAvgSamples=	eeprom_read_byte(AIO1_AVG_SAMPLES);
		config.AIO1HysMinValue=	eeprom_read_word(AIO1_MIN);
		config.AIO1HysMaxValue=	eeprom_read_word(AIO1_MAX);
		config.AIO1HysDifference=	eeprom_read_word(AIO1_DIFFERENCE);
			
		config.AIO2HysPoll=	eeprom_read_byte(AIO2_POLL);
		config.AIO2HysPollDuration=	eeprom_read_byte(AIO2_POLL_DURATION);
		config.AIO2HysAvgSamples=	eeprom_read_byte(AIO2_AVG_SAMPLES);
		config.AIO2HysMinValue=	eeprom_read_word(AIO2_MIN);
		config.AIO2HysMaxValue=	eeprom_read_word(AIO2_MAX);
		config.AIO2HysDifference=	eeprom_read_word(AIO2_DIFFERENCE);
		
		config.AIO3HysPoll=	eeprom_read_byte(AIO3_POLL);
		config.AIO3HysPollDuration=	eeprom_read_byte(AIO3_POLL_DURATION);
		config.AIO3HysAvgSamples=	eeprom_read_byte(AIO3_AVG_SAMPLES);
		config.AIO3HysMinValue=	eeprom_read_word(AIO3_MIN);
		config.AIO3HysMaxValue=	eeprom_read_word(AIO3_MAX);
		config.AIO3HysDifference=	eeprom_read_word(AIO3_DIFFERENCE);
		
		config.AIO4HysPoll=	eeprom_read_byte(AIO4_POLL);
		config.AIO4HysPollDuration=	eeprom_read_byte(AIO4_POLL_DURATION);
		config.AIO4HysAvgSamples=	eeprom_read_byte(AIO4_AVG_SAMPLES);
		config.AIO4HysMinValue=	eeprom_read_word(AIO4_MIN);
		config.AIO4HysMaxValue=	eeprom_read_word(AIO4_MAX);
		config.AIO4HysDifference=	eeprom_read_word(AIO4_DIFFERENCE);
			
		multiByteContentRead(DIO1_HEAD_ST,DIO1_HEAD_LN,header.DI1Header);	
		multiByteContentRead(DIO2_HEAD_ST,DIO2_HEAD_LN,header.DI2Header);	
		multiByteContentRead(DIO3_HEAD_ST,DIO3_HEAD_LN,header.DI3Header);	
		multiByteContentRead(SOS_HEAD_ST,SOS_HEAD_LN,header.SOS_Header);
		multiByteContentRead(IGNITION_HEAD_ST,IGNITION_HEAD_LN,header.Ignition_Header);
			
		multiByteContentRead(GEO1SMSStart,GEO1SMSLen,config.geo1Configure);	
		multiByteContentRead(GEO1NameStart,GEO1NameLen,config.geo1AreaName);	
			
		multiByteContentRead(GEO2SMSStart,GEO2SMSLen,config.geo2Configure);	
		multiByteContentRead(GEO2NameStart,GEO2NameLen,config.geo2AreaName);
			
		multiByteContentRead(GEO3SMSStart,GEO3SMSLen,config.geo3Configure);	
		multiByteContentRead(GEO3NameStart,GEO3NameLen,config.geo3AreaName);
			
		multiByteContentRead(GEO4SMSStart,GEO4SMSLen,config.geo4Configure);	
		multiByteContentRead(GEO4NameStart,GEO4NameLen,config.geo4AreaName);
			
		multiByteContentRead(GEO5SMSStart,GEO5SMSLen,config.geo5Configure);	
		multiByteContentRead(GEO5NameStart,GEO5NameLen,config.geo5AreaName);
			
		multiByteContentRead(GEO6SMSStart,GEO6SMSLen,config.geo6Configure);	
		multiByteContentRead(GEO6NameStart,GEO6NameLen,config.geo6AreaName);
			
		multiByteContentRead(GEO7SMSStart,GEO7SMSLen,config.geo7Configure);	
		multiByteContentRead(GEO7NameStart,GEO7NameLen,config.geo7AreaName);
			
		multiByteContentRead(GEO8SMSStart,GEO8SMSLen,config.geo8Configure);	
		multiByteContentRead(GEO8NameStart,GEO8NameLen,config.geo8AreaName);
			
		multiByteContentRead(GEO9SMSStart,GEO9SMSLen,config.geo9Configure);	
		multiByteContentRead(GEO9NameStart,GEO9NameLen,config.geo9AreaName);
			
		multiByteContentRead(GEO10SMSStart,GEO10SMSLen,config.geo10Configure);	
		multiByteContentRead(GEO10NameStart,GEO10NameLen,config.geo10AreaName);
			
		multiByteContentRead(GEO1IDStart,GEOIDLen,config.geo1UID);	
		multiByteContentRead(GEO1_SET,GEO_SET_LEN,config.geo1Setting);
			
		multiByteContentRead(GEO2IDStart,GEOIDLen,config.geo2UID);	
		multiByteContentRead(GEO2_SET,GEO_SET_LEN,config.geo2Setting);
			
		multiByteContentRead(GEO3IDStart,GEOIDLen,config.geo3UID);	
		multiByteContentRead(GEO3_SET,GEO_SET_LEN,config.geo3Setting);
			
		multiByteContentRead(GEO4IDStart,GEOIDLen,config.geo4UID);	
		multiByteContentRead(GEO4_SET,GEO_SET_LEN,config.geo4Setting);
			
		multiByteContentRead(GEO5IDStart,GEOIDLen,config.geo5UID);	
		multiByteContentRead(GEO5_SET,GEO_SET_LEN,config.geo5Setting);
			
		multiByteContentRead(GEO6IDStart,GEOIDLen,config.geo6UID);	
		multiByteContentRead(GEO6_SET,GEO_SET_LEN,config.geo6Setting);
			
		multiByteContentRead(GEO7IDStart,GEOIDLen,config.geo7UID);	
		multiByteContentRead(GEO7_SET,GEO_SET_LEN,config.geo7Setting);
			
		multiByteContentRead(GEO8IDStart,GEOIDLen,config.geo8UID);	
		multiByteContentRead(GEO8_SET,GEO_SET_LEN,config.geo8Setting);
			
		multiByteContentRead(GEO9IDStart,GEOIDLen,config.geo9UID);	
		multiByteContentRead(GEO9_SET,GEO_SET_LEN,config.geo9Setting);
			
		multiByteContentRead(GEO10IDStart,GEOIDLen,config.geo10UID);	
		multiByteContentRead(GEO10_SET,GEO_SET_LEN,config.geo10Setting);
			
	}
	////////////////////////////////////  SIM 1 //////////////////////////////////////////		
		
	if(SIM_No==1)
	{
		USART_SendData_s( DEBUG_COM,"EEPROM SIM 1 Reading///////////\r\n");
		multiByteContentRead(serverIP1Start_1,serverIP1ArrayLen_1,config.serverIP1); 	
		multiByteContentRead(serverPort1Start_1,serverPort1ArrayLen_1,config.serverPort1); 	

		multiByteContentRead(serverIP2Start_1,serverIP2ArrayLen_1,config.serverIP2); 	
		multiByteContentRead(serverPort2Start_1,serverPort2ArrayLen_1,config.serverPort2);
			
		multiByteContentRead(APN1Start_1,APN1ArrayLen_1,config.sim1APN);
		multiByteContentRead(APNUser1Start_1,APN1UserArrayLen_1,config.sim1APNUser); 
		multiByteContentRead(APN1PassStart_1,APN1PassArrayLen_1,config.sim1APNPassword);

		multiByteContentRead(IO_CONFIG_1_START_1,IO_CONFIG_1_ArrayLen_1,config.IO1Configure);			
		multiByteContentRead(IO_CONFIG_2_START_1,IO_CONFIG_2_ArrayLen_1,config.IO2Configure);			
		multiByteContentRead(IO_CONFIG_3_START_1,IO_CONFIG_3_ArrayLen_1,config.IO3Configure);			
		//multiByteContentRead(IO_CONFIG_4_START_1,IO_CONFIG_4_ArrayLen_1,config.IO4Configure);			
		multiByteContentRead(IO_CONFIG_IG_START_1,IO_CONFIG_IG_ArrayLen_1,config.ignitionConfigure);	
		multiByteContentRead(IO_CONFIG_SS_START_1,IO_CONFIG_SS_ArrayLen_1,config.sosConfigure);	
		
		config.transmissionTimeIGon=	eeprom_read_word(TIME_TRACK_LSB_1);
		config.transmissionTimeSMS=	eeprom_read_word(SMS_TIME_TRACK_LSB_1);
		config.heartBeatTime=	eeprom_read_word(HEART_BEAT_LSB_1);
		config.distanceTracking=	eeprom_read_dword(DIST_TRK_LSB1_1);
		config.headingChange=	eeprom_read_word(HEADING_LSB_1);
		
		config.noGPSMNOnOff=	eeprom_read_byte(NO_FX_MN_ON_OFF_1);
		config.MD_ON_OFF_FLAG=	eeprom_read_byte(MD_ON_OFF_1);
		config.gpsNoFixMNTime=	eeprom_read_byte(NO_FX_MN_TIME_1);
		config.transmissionTimeIGOff=	eeprom_read_word(TIME_TRK_IG_OFF_SET_1);

		config.speedLimit=	eeprom_read_word(Speed_Limit_S_1);
		config.overSpeedDiffA=	eeprom_read_byte(SPED_DIFF_SET_1);
		config.idleTimeSetting=	eeprom_read_word(IDLE_TIME_SET_1);

		multiByteContentRead(REASON_CODE_SET_1,REASON_CODE_LEN_1,config.AlertsOnOffEncode);
		
		
		config.HarshBreakDetectTHR = eepromReadFloat(ACC_Harsh_B_TH_1);	
		config.AccelerationDetectTHR = eepromReadFloat(ACC_ACCEL_TH_1);	
		config.ImpactDetectTHR = eepromReadFloat(ACC_IMPACT_TH_1);	
		//config.AnymotionDetectTHR=	eeprom_read_byte(ACC_ANYMOTION_TH_1);			
		config.NextAnymotionAlertTimeOut=	eeprom_read_byte(ACC_ANYMOTION_TIME_1);
	}
	////////////////////////////////////  SIM 2 //////////////////////////////////////////	
	else if(SIM_No==2)
	{
		USART_SendData_s( DEBUG_COM,"EEPROM SIM 2 Reading///////////\r\n");
		multiByteContentRead(serverIP1Start_2,serverIP1ArrayLen_2,config.serverIP1); 	
		multiByteContentRead(serverPort1Start_2,serverPort1ArrayLen_2,config.serverPort1); 	

		multiByteContentRead(serverIP2Start_2,serverIP2ArrayLen_2,config.serverIP2); 	
		multiByteContentRead(serverPort2Start_2,serverPort2ArrayLen_2,config.serverPort2);
			
		multiByteContentRead(APN1Start_2,APN1ArrayLen_2,config.sim1APN);
		multiByteContentRead(APNUser1Start_2,APN1UserArrayLen_2,config.sim1APNUser); 
		multiByteContentRead(APN1PassStart_2,APN1PassArrayLen_2,config.sim1APNPassword);

		multiByteContentRead(IO_CONFIG_1_START_2,IO_CONFIG_1_ArrayLen_2,config.IO1Configure);			
		multiByteContentRead(IO_CONFIG_2_START_2,IO_CONFIG_2_ArrayLen_2,config.IO2Configure);			
		multiByteContentRead(IO_CONFIG_3_START_2,IO_CONFIG_3_ArrayLen_2,config.IO3Configure);			
		//multiByteContentRead(IO_CONFIG_4_START_2,IO_CONFIG_4_ArrayLen_2,config.IO4Configure);			
		multiByteContentRead(IO_CONFIG_IG_START_2,IO_CONFIG_IG_ArrayLen_2,config.ignitionConfigure);	
		multiByteContentRead(IO_CONFIG_SS_START_2,IO_CONFIG_SS_ArrayLen_2,config.sosConfigure);	
	
		config.transmissionTimeIGon=	eeprom_read_word(TIME_TRACK_LSB_2);
		config.transmissionTimeSMS=	eeprom_read_word(SMS_TIME_TRACK_LSB_2);
		config.heartBeatTime=	eeprom_read_word(HEART_BEAT_LSB_2);
		config.distanceTracking=	eeprom_read_dword(DIST_TRK_LSB1_2);
		config.headingChange=	eeprom_read_word(HEADING_LSB_2);
		
		config.noGPSMNOnOff=	eeprom_read_byte(NO_FX_MN_ON_OFF_2);
		config.MD_ON_OFF_FLAG=	eeprom_read_byte(MD_ON_OFF_2);
		config.gpsNoFixMNTime=	eeprom_read_byte(NO_FX_MN_TIME_2);
		config.transmissionTimeIGOff=	eeprom_read_word(TIME_TRK_IG_OFF_SET_2);

		config.speedLimit=	eeprom_read_word(Speed_Limit_S_2);
		config.overSpeedDiffA=	eeprom_read_byte(SPED_DIFF_SET_2);
		config.idleTimeSetting=	eeprom_read_word(IDLE_TIME_SET_2);

		multiByteContentRead(REASON_CODE_SET_2,REASON_CODE_LEN_2,config.AlertsOnOffEncode);
		
		
		config.HarshBreakDetectTHR = eepromReadFloat(ACC_Harsh_B_TH_2);	
		config.AccelerationDetectTHR = eepromReadFloat(ACC_ACCEL_TH_2);	
		config.ImpactDetectTHR = eepromReadFloat(ACC_IMPACT_TH_2);	
		//config.AnymotionDetectTHR=	eeprom_read_byte(ACC_ANYMOTION_TH_2);			
		config.NextAnymotionAlertTimeOut=	eeprom_read_byte(ACC_ANYMOTION_TIME_2);
	}
}

/**
  * @brief  writes multi byte data content settings to internal EEPROM
  * @param  None
  * @retval None
  */
void multiByteContentWrite(unsigned int startLocation,unsigned int blocklength, unsigned char* receiveBufferPOinter)
{	
	unsigned char l_readLoop=0;

	for(  l_readLoop=0;l_readLoop<blocklength;l_readLoop++)
	{
		eeprom_write_byte(startLocation,receiveBufferPOinter[l_readLoop]);// wash EEPROM locations				
		startLocation++;
	}	
}

/**
  * @brief  read multi byte data contents from internal EEPROM and stores them to respective arrays
  * @param  None
  * @retval None
  */
void multiByteContentRead(unsigned int startLocation,unsigned int blocklength, unsigned char* receiveBufferPOinter)
{	
	unsigned char l_readLoop=0;
		
	for(  l_readLoop=0;l_readLoop<blocklength;l_readLoop++)
	{
		receiveBufferPOinter[l_readLoop] = eeprom_read_byte(startLocation);// wash EEPROM locations				
		startLocation++;	
	}
	Delay_ms(10);
}

/**
  * @brief  read float value from internal EEPROM
  * @param  None
  * @retval None
  */
float eepromReadFloat(int address)
{
	union u_tag 
	{
		char b[4];
    float fval;
  } u;   
  u.b[0] = eeprom_read_byte(address);
  u.b[1] = eeprom_read_byte(address+1);
  u.b[2] = eeprom_read_byte(address+2);
  u.b[3] = eeprom_read_byte(address+3);
  return u.fval;
}

/**
  * @brief  writes float values to internal EEPROM
  * @param  None
  * @retval None
  */
void eepromWriteFloat(int address, float value)
{
  union u_tag 
	{
    char b[4];
    float fval;
  } u;
  u.fval=value;
 
  eeprom_write_byte(address  , u.b[0]);
  eeprom_write_byte(address+1, u.b[1]);
  eeprom_write_byte(address+2, u.b[2]);
  eeprom_write_byte(address+3, u.b[3]);
}

/**
  * @brief  eepromWriteDouble
  * @param  None
  * @retval None
  */
void eepromWriteDouble(uint32_t address, double value)
{
  union u_tag 
	{
    char b[sizeof(double)];
    double fval;
  } u;
  u.fval=value;
 
  eeprom_write_byte(address  , u.b[0]);
  eeprom_write_byte(address+1, u.b[1]);
  eeprom_write_byte(address+2, u.b[2]);
  eeprom_write_byte(address+3, u.b[3]);
	eeprom_write_byte(address+4, u.b[4]);
	eeprom_write_byte(address+5, u.b[5]);
	eeprom_write_byte(address+6, u.b[6]);
	eeprom_write_byte(address+7, u.b[7]);
	
}
/**
  * @brief  eepromReadDouble
  * @param  None
  * @retval None
  */
double eepromReadDouble(uint32_t  address)
{
	union u_tag 
	{
		char b[sizeof(double)];
    double fval;
  } u; 

  u.b[0] = eeprom_read_byte(address);
  u.b[1] = eeprom_read_byte(address+1);
  u.b[2] = eeprom_read_byte(address+2);
  u.b[3] = eeprom_read_byte(address+3);
	u.b[4] = eeprom_read_byte(address+4);
	u.b[5] = eeprom_read_byte(address+5);
	u.b[6] = eeprom_read_byte(address+6);
	u.b[7] = eeprom_read_byte(address+7);
  return u.fval;
}
/**
  * @brief  eepromReadDouble
  * @param  None
  * @retval None
  */
void AppendToBuffer(uint32_t  ConfigValue)
{
  sprintf((char*)l_tempBuffer, "%d", ConfigValue);
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);
	strcat((char*)config.replyToSend,(const char*)",");
}
/**
  * @brief  eepromReadDouble
  * @param  None
  * @retval None
  */
void GeoDataAppendToBuffer(uint8_t  FenceNo)
{
	uint8_t PointNo=0;
  arrayInit2Zero(config.replyToSend,replyToSendSize);
	strcpy((char*)config.replyToSend,(const char*)configHeader);			
	if(config.IMEINoSett==TRUE)
	{
		strcat((char*)config.replyToSend,(const char*)config.IMEI);
	}	
	strcat((char*)config.replyToSend,(const char*)",S54,");
	
	sprintf((char*)l_tempBuffer,"%d",FenceNo+1);				
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);			
	strcat((char*)config.replyToSend,(const char*)",");	
	
	for(PointNo=0;PointNo<9;PointNo++)
	{
		sprintf((char*)l_tempBuffer,"%.6f",Vertex[FenceNo][PointNo].x);				
		strcat((char*)config.replyToSend,(const char*)l_tempBuffer);			
		strcat((char*)config.replyToSend,(const char*)",");	
				
		sprintf((char*)l_tempBuffer,"%.6f",Vertex[FenceNo][PointNo].y);				
		strcat((char*)config.replyToSend,(const char*)l_tempBuffer);			
		strcat((char*)config.replyToSend,(const char*)",");	
							
	}
	sprintf((char*)l_tempBuffer,"%.6f",Vertex[FenceNo][9].x);				
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);			
	strcat((char*)config.replyToSend,(const char*)",");	
				
	sprintf((char*)l_tempBuffer,"%.6f",Vertex[FenceNo][9].y);				
	strcat((char*)config.replyToSend,(const char*)l_tempBuffer);			
	strcat((char*)config.replyToSend,(const char*)configFoot);	
}

			

/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
