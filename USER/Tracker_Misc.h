/*
 * GPS_Tracker.h
 *
 * Created: 3/7/2012 1:29:59 AM
 *  Author: Qasim
 */ 




/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TRACKER_MISC_H
#define __TRACKER_MISC_H

#include <stdint.h>
/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

#define IAP_WRITE_DELAY  							 5
#define IAP_FLASH_PAGE_ERASE_ADDRESS   0x000000

#define BOOTLOADER_ADDRESS    				 0x8000000
#define APPLICATION_ADDRESS    				 0x8004000

#define MCU_FLASH_PAGE_SIZE            (0x800)    /* 2 Kbytes */
#define MCU_FLASH_SIZE                 (0x40000) /* 1 MByte */



#define watchDogLimit 30 // 4 minutes watch dog


#define INPUT1_ACTIVE 						"01"
#define INPUT1_INACTIVE 					"04"

#define INPUT2_ACTIVE 						"02"
#define INPUT2_INACTIVE 					"05"

#define INPUT3_ACTIVE 						"03"
#define INPUT3_INACTIVE 					"06"

#define LOW_BATT 									"07"

#define SPEEDING 									"08"

#define ENTER_GEO_FENCE 					"09"
#define EXIT_GEO_FENCE 						"10"

#define MAIN_PWR_ON 							"11"
#define MAIN_PWR_OFF				  		"12"

#define GPS_NO_FIX 								"13"
#define GPS_FIX 									"14"

#define ENTER_SLEEP 							"15"
#define EXIT_SLEEP 								"16"

#define DEVICE_BOOT 							"17"

#define IMPACT 										"18"

#define HEART_B 									"19"

#define HEADING_C 								"20"

#define DST_TRACK 								"21"

#define LOC_DEMAND 								"22"
	
#define MOTION_D 									"23"

#define ANS_CALL 									"24"

#define IG_ON								  		"25"

#define IG_OFF 										"26"

#define HARSH_BR 									"27"

#define HARSH_AC						 		 	"28"
	
#define SOS_PRESS 								"29"

#define SIM_CHG 									"31"

#define AI1_T_MAX						  		"32"
#define AI1_T_MIN 								"33"

#define AI2_T_MAX 								"34"
#define AI2_T_MIN 								"35"

#define TIME_TRACK 								"36"

#define RELAY1_CUT_CODE 					"37"
#define RELAY1_MAKE_CODE 					"38"

#define RELAY2_CUT_CODE 					"39"
#define RELAY2_MAKE_CODE 					"40"

#define FIRM_ERROR_CODE 					"41"

#define EXCESS_IDLE 							"42"

#define AI1_DIFF 									"43"
#define AI2_DIFF 									"44"

#define TIMETRACK_IG_OFF 					"45"
#define TRIP_COMP 								"46"

#define OVERSPEED_CLR 						"47"

#define NO_SECURITY_AUTHENTICATE 	"49"
#define FPID_AUTHENTICATE 				"50"
#define RFID_AUTHENTICATE 				"51"
#define FP_RFID_AUTHENTICATE 			"52"
#define DRIVER_UNREGISTER 				"53"
#define DRIVER_UNAUTHORIZE 				"54"
#define DRIVER_AUTHORIZE 					"55"

#define ONE_TOUCH_DIAL 			  		"70"

#define IAP_UPDATED			  				"71"
#define IAP_READY			  					"72"


#define LOGIN 										"73"
#define JAMMING_ON 								"74"
#define JAMMING_OFF 							"75"

#define AI3_T_MAX 								"76"
#define AI3_T_MIN 								"77"
#define AI3_DIFF 									"78"

#define AI4_T_MAX 								"79"
#define AI4_T_MIN 								"80"
#define AI4_DIFF 									"81"

#define AI1_NORMAL 								"82"
#define AI2_NORMAL 								"83"
#define AI3_NORMAL 								"84"
#define AI4_NORMAL 								"85"

#define FUEL_RELAY_CUT_CODE 			"86"
#define FUEL_RELAY_MAKE_CODE 			"87"
#define FUEL_RELAY_PULSE_CODE 		"88"
#define RELAY1_PULSE_CODE 				"89"
#define RELAY2_PULSE_CODE 				"90"

#define LEFT_SHARP_TURN 					"91"
#define RIGHT_SHARP_TURN 					"92"
#define REPEAT_EXCESS_IDLE 				"93"




//////////////////////////////////////////////////////////

#define g_ExtensionPaketSize   		100
#define g_TransmissionPaketSize   300


#define TRIP_DATA_EX_ID						"01"
#define DRIVER_DATA_EX_ID					"02"
#define ACC_DATA_EX_ID						"03"
#define OBD_DATA_EX_ID						"04"
#define GEOFENCE_DATA_EX_ID				"05"
#define ADC_DATA_EX_ID						"06"

_DECL unsigned char g_oneTouchDialEventFlag;

//_DECL unsigned char	g_ExtensionPaket[g_ExtensionPaketSize];
_DECL unsigned char	g_TransmissionPaket[g_TransmissionPaketSize];
_DECL unsigned char	g_ReTransmissionPaket[g_TransmissionPaketSize];

_DECL unsigned char sleepModeEnterFlag;
_DECL unsigned char sleepModeEnterClr;
_DECL unsigned char gsmErrorCounter;
//_DECL unsigned char g_GPRMCString[80];
_DECL unsigned char g_smsReadCounter;
_DECL unsigned int  g_packetSendCounter;
_DECL unsigned int  g_packetSendCounterIGOff;
_DECL unsigned int  g_HeartbeatSendCounter;
_DECL unsigned int  g_SmsTrackSendCounter;
_DECL unsigned int  g_watchDogCounter ;
_DECL unsigned char simChangeEvent;

struct _states
{
	unsigned char gpsBreakFlag;
	unsigned char gsmIMEIFixed;
	
	unsigned char gsmFirstInit;
	unsigned char networkInit;
	unsigned char GPRSInit;
	unsigned char FTPInit;
	unsigned char GPRSStateOk;
	
	unsigned char FTPStateOk;
	unsigned char FTPPUTConnectFlag;
	
	unsigned char GPSValid;
	unsigned char SendValid;
	unsigned char packetAck;
	unsigned char transmissionFlagIGOn;
	unsigned char transmissionFlagIGOff;
	
	unsigned char HeartBeatFlag;
	unsigned char smsTrackFlag;	
	
	unsigned char ATCommandsErrorCounter;
	unsigned char ATNetworkErrorCounter;
	
	unsigned char AckTimeOutCounter;
	unsigned char AckPacketErrorCounter;
	unsigned char AckGprsErrorCounter;
	
	unsigned char resetTransmission;
	
	unsigned char noNetworkCounter;
	
	unsigned char sleepModeACKFlag;
	
	unsigned char sleepModeActiveFlag;
	
	
	unsigned char severShiftCounter;
	
	unsigned char ignitionPollCounter;
	unsigned char  ignitionOnTrueCounter;
	unsigned char  ignitionOffTrueCounter;
	
	unsigned char ignition2PollCounter;
	unsigned char  ignition2OnTrueCounter;
	unsigned char  ignition2OffTrueCounter;
	
	unsigned char mainPowerPollCounter;
	unsigned char  mainPowerOnTrueCounter;
	unsigned char  mainPowerOffTrueCounter;
	
	unsigned char DIO1PollCounter;
	unsigned char  DIO1OnTrueCounter;
	unsigned char  DIO1OffTrueCounter;
	
	unsigned char DIO2PollCounter;
	unsigned char  DIO2OnTrueCounter;
	unsigned char  DIO2OffTrueCounter;
	
	unsigned char DIO3PollCounter;
	unsigned char  DIO3OnTrueCounter;
	unsigned char  DIO3OffTrueCounter;
	
	unsigned char AIO1PollCounter;
	unsigned char AIO2PollCounter;
	unsigned char AIO3PollCounter;
	unsigned char AIO4PollCounter;
	
	unsigned char SharpTurnPollCounter;

	unsigned char GPRSCurrentState[30];
	
	unsigned char FTPCurrentState[30];
	
	uint8_t StopModeExitFlag;
	
	uint8_t JammingDetectFlag;
	uint8_t CallOngoingFlag;
	
	uint8_t AuthenticationOkFlag;
	uint8_t ReAuthenticationFlag;
	uint16_t ReAuthenticationCounter;
	
	uint8_t  CarKillFlag;
	uint8_t  IgnitionOnFlag;

};
_DECL  struct _states states;

void SleepModeController(void);
void Tracker_Power(void);
void DIO_StateMachine(void);
void AIO_StateMachine(void);
uint8_t IsAlertOn(uint8_t AlertHexvalue);
void Start_Blink(void);
void timeCompute(void);
void Main_StateMachine(void);
void dataTransmissionController(void);
void Packet_Formatter(unsigned char*,unsigned char);
void GPRStrackingModule(void);
void pingPacketGen(void);
void TimeComputeController(void);
void pingPacketGen_SL_Protocol(void);
uint8_t mainbPktExtractor (uint8_t* mainbCMDArray);
void fpStateMachine(void);
uint8_t RFIDPktExtractor (void);


uint8_t IAP_convert_file(void);

void SleepModeAlgo(void);
void SYSCLKConfig_STOP(void);
void Ignition_module(void);

void GSM_StateMachine(void);
void GPS_StateMachine(void);
void RequestProcessController (void);

void IgnitionHys(void);
void MainPowerHys(void);
void SOSHys(void);
void DIO1Hys(void);
void DIO2Hys(void);
void DIO3Hys(void);
void AIO1Hys(void);
void AIO2Hys(void);
//void AIO3Hys(void);
//void AIO4Hys(void);
#endif /* __TRACKER_MISC_H */
