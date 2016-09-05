/*
 * GSM.h
 *
 * Created: 3/14/2012 1:30:47 PM
 *  Author: Qasim
 */ 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GSM_H
#define __GSM_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

//_DECL int var_a _INIT(100);   Declare & initialize var_a variable with 100 i.e; var_a=100;
#include <stdint.h>

#define CARRIAGE_LINE_FEED "\r\n"// enter

#define sConfig 2
#define GPRS 1
#define SMS 0

#define ERROR "ERROR"

#define SIM1 1
#define SIM2 2
#define SIM3 3

/*--------------------  General Settings-------------------------------------------------------*/
#define GSM_NORMAL_POWER_DOWN 	"AT+QPOWD=1\r\n"
#define IMEI_GET 								"AT+GSN\r\n"
#define IMSI_GET 								"AT+CIMI\r\n"
#define ECHO_OFF 								"ATE0\r\n"// turn ECHO off
#define AT_SEND 								"AT\r\n"// at command send
#define DISCONNECT_CALL 				"ATH\r\n"
#define ATTEND_CALL 						"ATA\r\n"
#define LOUDSPEAKER_AUDIO 			"AT+QAUDCH=2\r\n"
#define ENABLE_CALL_URC 				"AT+CLIP=1\r\n"
#define ENABLE_CALL_RING_URC 		"AT+CRC=1\r\n"
#define ENABLE_SMS_TEXT_MODE 		"AT+CMGF=1\r\n"
#define SET_TE_CHSET_GSM 				"AT+CSCS=\"GSM\"\r\n"
#define CREG_SET 								"AT+CREG=1\r\n"
#define ENG_MODE_SET 						"AT+QENG=1,0\r\n"
#define CALL_ANS 								"ATS0=3\r\n"
#define PACKET_ACK 							"AT+QISACK\r\n"
#define PACKET_TRANSMITTER 			"AT+QISEND\r\n"

#define SERVER_PKT_HEADER 			"@@"
#define GSM_INIT 								0
#define RSSI_LIMIT 							10
#define CRTL_Z 									0x1A// ending command for SMS send
/*--------------------  Query Commands-------------------------------------------------------*/
#define NETWORK_CHECK 					"AT+CREG?\r\n"// command for Network Status Check
#define NETWORK_SIGNAL_STRENGTH "AT+CSQ\r\n"// Command for checking signal strength
#define ENG_MODE 								"AT+QENG?\r\n"
/*--------------------  SMS Setting-------------------------------------------------------*/
#define SMS_READ 								"AT+CMGR="// Command for SMS reading
#define SMS_SEND 								"AT+CMGS=\""// command for sending SMS
#define DEL_ALL_SMS 						"AT+CMGD=1,4\r\n"//command for deleting all SMS
#define SMS_LOC_HEADER 					"+CMT"
#define SMS_MESSAGE_HEADER 			"+CMGR"
/*---------------------------GPRS----------------------------------------------------------------*/
#define QIREGAPP 								"AT+QIREGAPP\r\n"
#define QIACT 									"AT+QIACT\r\n"
#define GPRS_DEACT 							"AT+QIDEACT\r\n"
#define GPRS_ATTATCH 						"AT+QIFGCNT=0\r\n"// GPRS Attachment
#define APN_SET 								"AT+QICSGP=1,"

#define TCP_STATE_CHECK 				"AT+QISTAT\r\n"
#define REUQEST_TCP_CONNECTION 	"AT+QIOPEN=\"TCP\",\""
#define REUQEST_UDP_CONNECTION 	"AT+QIOPEN=\"UDP\",\""

#define FTP_USERNAME 						"AT+QFTPUSER=\""
#define FTP_USERPASS 						"AT+QFTPPASS=\""
#define FTP_PATH 								"AT+QFTPPATH=\""
#define REUQEST_FTP_CONNECTION 	"AT+QFTPOPEN=\""
#define FTP_PUT 								"AT+QFTPPUT=\""
#define FTP_GET 								"AT+QFTPGET=\""
#define FTP_STATE_CHECK 				"AT+QFTPSTAT\r\n"
#define FTP_CLOSE 							"AT+QFTPCLOSE\r\n"


#define SELECT_SIM1 						"AT+QDSIM=0,1\r\n"// select SIM 1
#define SELECT_SIM2 						"AT+QDSIM=1,1\r\n"// select SIM 2

#define MIN_FUNCTION 						"AT+CFUN=0\r\n" 
#define FULL_FUNCTION 					"AT+CFUN=1\r\n"

#define ENABLE_JAMMING_DETECT 	"AT+QJDR=1\r\n"
#define SET_JAMMING_RSSI 				"AT+QJDCFG=\"mnl\",16\r\n"
#define SET_JAMMING_CHANNELS 		"AT+QJDCFG=\"minch\",6\r\n"
//#define FTP_USERNAME "AT+QFTPUSER=\"qasim@teresol.org\"\r\n"
//#define FTP_USERPASS "AT+QFTPPASS=\"Teresol123\"\r\n"
//#define FTP_PATH "AT+QFTPPATH=\"/Tracker/\"\r\n"
//#define FTP_OPEN "AT+QFTPOPEN=\"teresol.org\",21\r\n"
//#define FTP_PUT "AT+QFTPPUT=\"FP.txt\",10219,20\r\n"
/*---------------------------------------------------------------SETTINGS----------------------------------------------------------------*/
#define smsReadBufferSize    		 200
#define rxBufferSize    		 		 700 //170
#define gsmCommandsBufferSize    700 //100
#define gprsRequestBufferSize    700 //150
#define smsRecBufferSize    		 150
#define gsmRSSIAASCISize    		 3
#define gsmMCCMNCSize    		 		 20
#define smsReplyNumberSize    	 15
#define ServerAckNoBuffSize 		 5
#define ServerAckLenBuffSize 		 3
 

_DECL unsigned char rxBuffer[rxBufferSize];
_DECL uint16_t rxBufferIndex;	

_DECL unsigned char smsRecBuffer[smsRecBufferSize];	
_DECL uint16_t smsRecBufferIndex;
_DECL uint16_t smsRecBufferReadIndex;

_DECL unsigned char smsFooterCounter;	
_DECL unsigned char smsReadBuffer[smsReadBufferSize];// this buffer can be reduced to 200 bytes
_DECL uint16_t smsReadBufferIndex;
_DECL unsigned char gsmCommandsBuffer[gsmCommandsBufferSize];// was 200 bytes before
_DECL unsigned char noOfSmsRec;

_DECL unsigned char gprsRequestBuffer[gprsRequestBufferSize];
_DECL uint16_t gprsRequestBufferIndex;
//unsigned char gprsRequestCounter;


struct _gsmCommunication
{		
	//char gsmRSSI;
	unsigned char gsmRSSIAASCI[gsmRSSIAASCISize];		
	//unsigned char gsmBaseID[10];	
	unsigned char gsmMCCMNC[gsmMCCMNCSize];	
	unsigned char smsHeaderCounter;
	unsigned char gsmCommandsBufferWriteEnable;	
	
	unsigned char interruptCallFlag;
	
	unsigned char ipSelection;
};

_DECL struct _gsmCommunication gsmComm;

struct _SMS
{	
	unsigned char searchLoop;	
	unsigned char messageReceivedFlag;		
	unsigned char messageRead;
	unsigned char gsmSMSReadFlag;
	unsigned char smsReplyNumber[15];	
	
};

_DECL struct _SMS sms;

struct _GSMConnections
{
	unsigned char gsmInitOk;
	unsigned char networkConnection;
	unsigned char gprsConnection;	
	unsigned char sendOK;		
	unsigned char rssiOK;
	unsigned char gprsStateOK;
	
	unsigned char FTPStateOK;
	
	unsigned char packetAckOK;			
	unsigned char packetAckReset;
	
};

_DECL struct _GSMConnections GSMConenction;

struct _DataSendStatus
{	
	unsigned char smsDemand;
	unsigned char smsDemandGoogle;	
	unsigned char gprsDemand;		
	
};

_DECL struct _DataSendStatus DataSendStatus;


struct _ftp
{
	uint8_t  *array2Send;
	uint16_t bytes2Send;
	uint16_t FileSize;
	
	uint8_t  PUTConnectStatus;
	uint8_t  PUTConnectFlag2;
	uint8_t  PUTTryAgainFlag;

};
_DECL  struct _ftp  FTP;

#define IAP_BUFFER_SIZE 		0x100		//!< Page Size of Flash(256 bytes)
struct _iap
{
	uint8_t USART_ISR_buffer1[IAP_BUFFER_SIZE];
	uint8_t USART_ISR_buffer2[IAP_BUFFER_SIZE];
	
	uint32_t sFLASH_ADD_Counter;
	volatile uint8_t Data_Start_Flag;
	uint32_t USART_ISR_Count;
	uint8_t *Array_Track;
	uint8_t *Array;
	volatile uint8_t BufferNo;
	uint32_t 	write_addess_1;
	uint32_t  write_addess_2;
	uint32_t  write_addess_3;
	uint8_t 	File_download_status;  
	uint32_t  File_Size;
	
	uint8_t  RetryCount;
	
	uint8_t  FileCheckError;
	
	uint8_t USART_ISR_buffer_temp[300];
	uint8_t  AfterIAPBootFlag;
	
	uint32_t FTPConnectCounter;
};
_DECL  struct _iap  IAP;


struct _ack
{
	unsigned char LoginPackFlag;
	//unsigned char message_pack_flag;
	unsigned char AckRecFlag;
	unsigned char ServerAckNoBuff[ServerAckNoBuffSize];
	unsigned char ServerAckLenBuff[ServerAckLenBuffSize];
	uint32_t  PacketSendNo;
	uint8_t  DataHandlerFlag;
	
	uint8_t  LoginAckRetransmit;
	uint8_t  LoginAckRetransmitSend;
};
_DECL  struct _ack  Ack;



unsigned char Command_Send_Network_Get(unsigned char*);
void arrayWashGSM(unsigned char*);
unsigned char GPRS_initialize(void);
unsigned char Command_Send_Connect_Get(unsigned char*);
unsigned char SMS_Read_module(void);
void SMS_Generator(unsigned char*,unsigned char*);
unsigned char Command_Send_SEND_Get(unsigned char*);
unsigned char GSM_Init_Reset(void);
unsigned char GSM_Network_Check(void);
unsigned char Command_Send_SATE_Get(void);
unsigned char Command_Send_ACK_Get(void);
unsigned char gsmPowerOn(void);
void gprsRequestProcess(void);
unsigned char ackContentsExtract(unsigned char*,unsigned char*);
unsigned char gsmPowerOff(void);
void SMS_GeneratorAlerts(unsigned char*,unsigned char*,unsigned char);
void getIMEI(void);

void gsmGPIOInit(void);


unsigned char Command_Send_SATE_FTP_Get(void);
unsigned char FTP_initialize(void);
unsigned char Command_Send_FTPConnect_Get(unsigned char *tempArray);
uint8_t Command_SendFTP_SEND_Get(uint8_t* filename,uint16_t bytes2Send,uint16_t totalFileBytes,uint8_t* array2send);
uint8_t Command_SendFTP_CONNECT_Get(void);
uint8_t Command_FTP_Get(uint8_t* filename);
void GSM_Get_SIM_Number(void);
void GSM_Select_SIM(uint8_t SIMNumber);
void CheckCallStatus(void);
#endif /* __GSM_H */
