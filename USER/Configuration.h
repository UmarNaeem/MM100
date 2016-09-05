/**
  ******************************************************************************
  * @file    Configuration.h
  * @author  M.Uzair Afzal
  * @version V1.0.0
  * @date    30-May-2013
  * @brief   This file contains all the functions prototypes for the global variables and functions.
  ******************************************************************************
*/
//Global_Defines.h

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIGURATION_H
#define __CONFIGURATION_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

#include <stdint.h>
#define FM_Ver "FW2.1"
// Sim Shifting config added shifting algo working, sim shifting counter range extended rssi check in gsm section removed RSSI Buffer wash
// time date fixed in case of no gps


#define bit_set(p,m) ((p) |= (m)) 
#define BIT(x)	(0x01 << (x)) 
#define bit_clear(p,m) ((p) &= ~(m)) 
#define bit_get(p,m) ((p) & (m)) 

#define configFoot ",*\r\n"
#define configHeader "$C,"


/*

#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))
*/

#define Kph 0
#define mph 1
#define TRUE 1
#define FALSE 0

#define SMS 0
#define GPRS 1

#define TCP '1'
#define UDP '2'



#define SpeedLimitHeading 4 // Speed Limit For Heading change Algorithm activation

#define CARRIAGE_RETURN 0x0D
#define LINE_FEED 0x0A

#define TRACK_ON_DEMAND_GPRS 	1
#define GOOGLE_TRACK_DEMAND 	2

#define TRACK_TIME_INTERVAL_SMS				  0
#define HEARTBEAT_INTERVAL 							1
#define TRACK_TIME_INTERVAL_GPRS 				2
#define SET_HEADING_CHANGE 							3
#define TRACK_DISTANCE_INTERVAL 				4
#define SET_GPRS 												5
#define SET_APN 												6
#define SET_ACC_TH_DETECT 							7
#define SET_MOTION_ON_OFF 							8
#define PING_REQ 												9
#define SET_SLEEP 											10

#define TIME_ZONE_SETTING 							20
#define REBOOT_GPS 											21
#define CLEAR_T_O 											22
#define FCT_RST 												23
#define OUT_CONTROL 										24
#define GET_FM_V 												25
#define REBOOT_GSM 											26
#define SET_OVER_SPEED 									27
#define EEPROM_EN_DS 										28
#define SIM_SHFT_EN_DS 									29
#define ANALOG_INPUT_SETTING 						30
#define MCU_RESET 											31
#define TRACKER_PWR_SETTING							32
#define IO_SMS_SETTINGS 								33

#define SMS_NUMBER_CONFIGURE 						34
#define SMS_HEADER_CONFIGURE 						35
//#define GEO_FENCE_REGION_CHANGE 				36

#define GET_ODOMETER_VALUE							36
#define SET_ODOMETER_VALUE 							37

#define GEO_FENCE_CONFIG 								38

#define QUEUE_DELETE 										39
#define ALERTS_ENCODE 									40
#define FIRMWARE_JUMP_CALL 							42
#define IDLE_TIME_SET 									45
#define GEO_FENCE_TRIGGER_SET 					46
#define SHARP_TURN_SETTING 							47
//#define HYS_SETT 												48

#define READ_CONFIG_0 									50
#define READ_CONFIG_1 									51
#define READ_CONFIG_2 									52
#define READ_CONFIG_3 									53
#define READ_CONFIG_4 									54
#define READ_CONFIG_5 									55
#define READ_CONFIG_6 									56
#define READ_CONFIG_7 									57
#define READ_CONFIG_8 									58
#define READ_CONFIG_9 									59
#define READ_CONFIG_10								  60
#define READ_CONFIG_11 									61
#define READ_CONFIG_12 									62
#define READ_CONFIG_13 									63
#define READ_CONFIG_14 									64
#define READ_CONFIG_15 									65
#define READ_CONFIG_16 									66
#define READ_CONFIG_17 									67
#define READ_CONFIG_18 									68
#define READ_CONFIG_19 									69


#define DIGITAL_INPUT_SET 							74
#define IAP_EN_DS 											76

//#define CAN_EN_DS 											79
//#define FP_EN_DS 												80
//#define RFID_EN_DS 											81
//#define TPMS_EN_DS 											82
#define TRACKER_ID_DATA 								83
#define SD_EN_DS 												84
#define GSM_NEWTEORK_SET 								85
//#define BUZZER_EN_DS 										86
#define IMEI_PAD_EN_DS 									87
#define ACK_SET 												88

#define AUTHENTICATE_SET  							89
#define POLY_GEOFENCE_CONFIG  					90
//#define DB_CHANGE_FIELD 								91

//#define DB_GET_INFO 										97
//#define DB_ENROLL_PKT 									98
//#define DB_DELETE_ENTRY_PKT 						99

#define SERVER_SEND_HEADER 		"$$"
#define SERVER_SEND_SL_HEADER "$$"
// Config Define EEPROM------------------------------------------------------


#define IMEI_START 									CONFIG_OFFSET+6	
#define IMEI_LEN 										20

#define CARRegStart 								CONFIG_OFFSET+26
#define CARRegArrayLen 							15

#define LotNoStart 									CONFIG_OFFSET+41
#define LotNoArrayLen 							8

#define PIDNoStart 									CONFIG_OFFSET+49
#define PIDNoArrayLen 							10

#define CIDNoStart 									CONFIG_OFFSET+59
#define CIDNoArrayLen 							10

#define SIM1NoStart 								CONFIG_OFFSET+69
#define SIM1NoArrayLen 							20

#define SIM2NoStart 								CONFIG_OFFSET+89
#define SIM2NoArrayLen 							20

#define FP_SETTING 									CONFIG_OFFSET+109
#define RFID_SETTING 								CONFIG_OFFSET+110
#define CAN_SETTING 								CONFIG_OFFSET+111
#define EEPROM_RECOV 								CONFIG_OFFSET+112
#define SD_RECOV 										CONFIG_OFFSET+114
#define SIM_SHFT_EEPRM 							CONFIG_OFFSET+117
#define TPMS_SETTING 								CONFIG_OFFSET+118
#define IAP_SETTING 								CONFIG_OFFSET+119
#define IAP_RESET 									CONFIG_OFFSET+120
#define IAP_DOWNLOAD 								CONFIG_OFFSET+122
#define IAP_FILE_SIZE 							CONFIG_OFFSET+123
#define IAP_FILE_ADDRESS 						CONFIG_OFFSET+127

#define AUTHENTICATE_FUEL_KS_SET 		CONFIG_OFFSET+131

#define KS1_PULSES 									CONFIG_OFFSET+132
#define KS1_DURATION 								CONFIG_OFFSET+133

#define KS2_PULSES 									CONFIG_OFFSET+134
#define KS2_DURATION 								CONFIG_OFFSET+135

#define FUEL_KS_PULSES 							CONFIG_OFFSET+136
#define FUEL_KS_DURATION 						CONFIG_OFFSET+137

#define SHARP_TURN_DURATION 				CONFIG_OFFSET+138
#define SHARP_TURN_THRESHOLD 				CONFIG_OFFSET+139


#define NETWORK_TRIES_SET 					CONFIG_OFFSET+140
#define IP_SIM_TRIES_SET 						CONFIG_OFFSET+141
#define NETWORK_TRIES_DELAY 				CONFIG_OFFSET+142
#define GSM_REBOOT_SET 							CONFIG_OFFSET+143
#define REAUTHENTICATION_TIMEOUT		CONFIG_OFFSET+144//2bytes
#define ACK_LAYER_SET 							CONFIG_OFFSET+146
#define ACK_TIMEOUT		 							CONFIG_OFFSET+147
#define ACK_TRIES		 								CONFIG_OFFSET+148
#define TRACKER_POWER_STATE 				CONFIG_OFFSET+149
#define IAP_RETRIES		 							CONFIG_OFFSET+150

#define TCP_UDP_START 							CONFIG_OFFSET+151
#define TCP_UDP_LEN 								2

#define IMEI_SETTING	 							CONFIG_OFFSET+153
#define EEPROM_ALGO_SET		 					CONFIG_OFFSET+154
#define BUZZER_SETTTING							CONFIG_OFFSET+155

#define SERVER_SMS_CELL1_START 			CONFIG_OFFSET+156
#define SERVER_SMS_CELL1_ArrayLen 	20

#define SERVER_SMS_CELL2_START 			CONFIG_OFFSET+176
#define SERVER_SMS_CELL2_ArrayLen 	20

#define SERVER_SMS_CELL3_START 			CONFIG_OFFSET+196
#define SERVER_SMS_CELL3_ArrayLen 	20

#define SERVER_VOICE_CELL1_START 		CONFIG_OFFSET+216
#define SERVER_VOICE_CELL1_ArrayLen 20

#define SERVER_VOICE_CELL2_START 		CONFIG_OFFSET+236
#define SERVER_VOICE_CELL2_ArrayLen 20

#define SERVER_VOICE_CELL3_START 		CONFIG_OFFSET+256
#define SERVER_VOICE_CELL3_ArrayLen 20

#define travelTimeStart 						CONFIG_OFFSET+276
#define travelTimeAAsciLen 					12

#define travelDistanceStart 				CONFIG_OFFSET+296
#define travelDistanAAsciLen 				12

#define TRIP_ID_LSB 								CONFIG_OFFSET+316//2 bytes
//#define TRIP_ID_MSB 								CONFIG_OFFSET+317
#define TIME_ZONE_S 								CONFIG_OFFSET+320

#define SLEEP_SET_START 						CONFIG_OFFSET+325
#define SLEEP_SET_LEN 							5

#define SLEEP_TIME_LSB 							CONFIG_OFFSET+330 //2 bytes

#define SMS_PASS_START 							CONFIG_OFFSET+332
#define SMS_PASS_ArrayLen 					5

#define GPRS_PASS_START 						CONFIG_OFFSET+340
#define GPRS_PASS_ArrayLen 					5

#define FUEL_RELAY_STATE 						CONFIG_OFFSET+345
#define RELAY1_STATE								CONFIG_OFFSET+346
#define RELAY2_STATE								CONFIG_OFFSET+347



#define MAIN_PWR_POLL 							CONFIG_OFFSET+348
#define MAIN_PWR_POLL_DURATION			CONFIG_OFFSET+349
#define MAIN_PWR_MIN_TRUE						CONFIG_OFFSET+350

#define IGNITION_POLL 							CONFIG_OFFSET+351
#define IGNITION_POLL_DURATION			CONFIG_OFFSET+352
#define IGNITION_MIN_TRUE						CONFIG_OFFSET+353

#define SOS_POLL 										CONFIG_OFFSET+354
#define SOS_POLL_DURATION						CONFIG_OFFSET+355
#define SOS_MIN_TRUE								CONFIG_OFFSET+356

#define DIO1_POLL 									CONFIG_OFFSET+357
#define DIO1_POLL_DURATION					CONFIG_OFFSET+358
#define DIO1_MIN_TRUE								CONFIG_OFFSET+359

#define DIO2_POLL 									CONFIG_OFFSET+360
#define DIO2_POLL_DURATION					CONFIG_OFFSET+361
#define DIO2_MIN_TRUE								CONFIG_OFFSET+362

#define DIO3_POLL 									CONFIG_OFFSET+363
#define DIO3_POLL_DURATION					CONFIG_OFFSET+364
#define DIO3_MIN_TRUE								CONFIG_OFFSET+365

#define AIO1_POLL 									CONFIG_OFFSET+366
#define AIO1_POLL_DURATION					CONFIG_OFFSET+367
#define AIO1_AVG_SAMPLES						CONFIG_OFFSET+368
#define AIO1_MIN										CONFIG_OFFSET+369//2 bytes
#define AIO1_MAX										CONFIG_OFFSET+371//2 bytes
#define AIO1_DIFFERENCE							CONFIG_OFFSET+373//2 bytes

#define AIO2_POLL 									CONFIG_OFFSET+375
#define AIO2_POLL_DURATION					CONFIG_OFFSET+376
#define AIO2_AVG_SAMPLES						CONFIG_OFFSET+377
#define AIO2_MIN										CONFIG_OFFSET+378//2 bytes
#define AIO2_MAX										CONFIG_OFFSET+380//2 bytes
#define AIO2_DIFFERENCE							CONFIG_OFFSET+382//2 bytes

#define AIO3_POLL 									CONFIG_OFFSET+384
#define AIO3_POLL_DURATION					CONFIG_OFFSET+385
#define AIO3_AVG_SAMPLES						CONFIG_OFFSET+386
#define AIO3_MIN										CONFIG_OFFSET+387//2 bytes
#define AIO3_MAX										CONFIG_OFFSET+389//2 bytes
#define AIO3_DIFFERENCE							CONFIG_OFFSET+391//2 bytes

#define AIO4_POLL 									CONFIG_OFFSET+393
#define AIO4_POLL_DURATION					CONFIG_OFFSET+394
#define AIO4_AVG_SAMPLES						CONFIG_OFFSET+395
#define AIO4_MIN										CONFIG_OFFSET+396//2 bytes
#define AIO4_MAX										CONFIG_OFFSET+398//2 bytes
#define AIO4_DIFFERENCE							CONFIG_OFFSET+400//2 bytes

#define SOS_HEAD_ST 								CONFIG_OFFSET+402
#define SOS_HEAD_LN 								10


#define DIO1_HEAD_ST 								CONFIG_OFFSET+412
#define DIO1_HEAD_LN 								10

#define DIO2_HEAD_ST 								CONFIG_OFFSET+422
#define DIO2_HEAD_LN 								10

#define DIO3_HEAD_ST 								CONFIG_OFFSET+432
#define DIO3_HEAD_LN 								10

#define IGNITION_HEAD_ST 						CONFIG_OFFSET+442
#define IGNITION_HEAD_LN 						10

#define COMPANY_ID_START 						CONFIG_OFFSET+452
#define COMAPANY_ID_LEN 						4


#define GPS_LAST_VALID_LAT 					CONFIG_OFFSET+456
#define GPS_LAST_VALID_LAT_LEN 			15

#define GPS_LAST_VALID_LON 					CONFIG_OFFSET+471
#define GPS_LAST_VALID_LON_LEN 			15

#define GPS_LAST_VALID_DATE 				CONFIG_OFFSET+486
#define GPS_LAST_VALID_DATE_LEN 		15
//////////////   SIM 1 Section //////////////////////////////

#define serverIP1Start_1 						CONFIG_OFFSET+512
#define serverIP1ArrayLen_1 				16

#define serverPort1Start_1 					CONFIG_OFFSET+528
#define serverPort1ArrayLen_1 			6

#define serverIP2Start_1 						CONFIG_OFFSET+534
#define serverIP2ArrayLen_1 				16

#define serverPort2Start_1 					CONFIG_OFFSET+550
#define serverPort2ArrayLen_1 			6

#define APN1Start_1 								CONFIG_OFFSET+556
#define APN1ArrayLen_1 							15

#define APNUser1Start_1 						CONFIG_OFFSET+571
#define APN1UserArrayLen_1 					10

#define APN1PassStart_1 						CONFIG_OFFSET+581
#define APN1PassArrayLen_1 					10

//#define APN2Start_1 								CONFIG_OFFSET+591
//#define APN2ArrayLen_1 							15

//#define APNUser2Start_1 						CONFIG_OFFSET+606
//#define APN2UserArrayLen_1 					10

//#define APN2PassStart_1 						CONFIG_OFFSET+616
//#define APN2PassArrayLen_1 					10

#define IO_CONFIG_1_START_1 				CONFIG_OFFSET+626
#define IO_CONFIG_1_ArrayLen_1 			7

#define IO_CONFIG_2_START_1 				CONFIG_OFFSET+633
#define IO_CONFIG_2_ArrayLen_1 			7

#define IO_CONFIG_3_START_1 				CONFIG_OFFSET+640
#define IO_CONFIG_3_ArrayLen_1 			7

#define IO_CONFIG_IG_START_1 				CONFIG_OFFSET+654
#define IO_CONFIG_IG_ArrayLen_1			7

#define IO_CONFIG_SS_START_1 				CONFIG_OFFSET+661
#define IO_CONFIG_SS_ArrayLen_1 		7

#define TIME_TRACK_LSB_1 						CONFIG_OFFSET+668

#define HEART_BEAT_LSB_1 						CONFIG_OFFSET+673//2B

#define SMS_TIME_TRACK_LSB_1 				CONFIG_OFFSET+679//2B


#define DIST_TRK_LSB1_1 						CONFIG_OFFSET+684
#define DIST_TRK_LSB2_1 						CONFIG_OFFSET+685
#define DIST_TRK_MSB1_1 						CONFIG_OFFSET+686
#define DIST_TRK_MSB2_1 						CONFIG_OFFSET+687

#define HEADING_LSB_1 							CONFIG_OFFSET+690

#define NO_FX_MN_ON_OFF_1 					CONFIG_OFFSET+696

#define MD_ON_OFF_1 								CONFIG_OFFSET+697

#define NO_FX_MN_TIME_1 						CONFIG_OFFSET+698

#define TIME_TRK_IG_OFF_SET_1 			CONFIG_OFFSET+699

#define Speed_Limit_S_1 						CONFIG_OFFSET+701
#define SPED_DIFF_SET_1 						CONFIG_OFFSET+704

#define IDLE_TIME_SET_1 						CONFIG_OFFSET+705

#define ACC_Harsh_B_TH_1 						CONFIG_OFFSET+718
#define ACC_ACCEL_TH_1 							CONFIG_OFFSET+722
#define ACC_IMPACT_TH_1 						CONFIG_OFFSET+726
#define ACC_ANYMOTION_TH_1 					CONFIG_OFFSET+730
#define ACC_ANYMOTION_TIME_1 				CONFIG_OFFSET+731

#define REASON_CODE_SET_1 					CONFIG_OFFSET+732
#define REASON_CODE_LEN_1 					32


//////////////   SIM 2 Section //////////////////////////////
#define SIM2_OFFSET 								256


#define serverIP1Start_2 						CONFIG_OFFSET+512+SIM2_OFFSET
#define serverIP1ArrayLen_2 				16

#define serverPort1Start_2 					CONFIG_OFFSET+528+SIM2_OFFSET
#define serverPort1ArrayLen_2 			6

#define serverIP2Start_2 						CONFIG_OFFSET+534+SIM2_OFFSET
#define serverIP2ArrayLen_2 				16

#define serverPort2Start_2 					CONFIG_OFFSET+550+SIM2_OFFSET
#define serverPort2ArrayLen_2 			6

#define APN1Start_2 								CONFIG_OFFSET+556+SIM2_OFFSET
#define APN1ArrayLen_2 							15

#define APNUser1Start_2 						CONFIG_OFFSET+571+SIM2_OFFSET
#define APN1UserArrayLen_2 					10

#define APN1PassStart_2 						CONFIG_OFFSET+581+SIM2_OFFSET
#define APN1PassArrayLen_2 					10

//#define APN2Start_2 								CONFIG_OFFSET+591+SIM2_OFFSET
//#define APN2ArrayLen_2 							15

//#define APNUser2Start_2 						CONFIG_OFFSET+606+SIM2_OFFSET
//#define APN2UserArrayLen_2 					10

//#define APN2PassStart_2 						CONFIG_OFFSET+616+SIM2_OFFSET
//#define APN2PassArrayLen_2 					10

#define IO_CONFIG_1_START_2 				CONFIG_OFFSET+626+SIM2_OFFSET
#define IO_CONFIG_1_ArrayLen_2 			7

#define IO_CONFIG_2_START_2 				CONFIG_OFFSET+633+SIM2_OFFSET
#define IO_CONFIG_2_ArrayLen_2 			7

#define IO_CONFIG_3_START_2 				CONFIG_OFFSET+640+SIM2_OFFSET
#define IO_CONFIG_3_ArrayLen_2 			7

#define IO_CONFIG_IG_START_2 				CONFIG_OFFSET+654+SIM2_OFFSET
#define IO_CONFIG_IG_ArrayLen_2			7

#define IO_CONFIG_SS_START_2 				CONFIG_OFFSET+661+SIM2_OFFSET
#define IO_CONFIG_SS_ArrayLen_2 		7

#define TIME_TRACK_LSB_2 						CONFIG_OFFSET+668+SIM2_OFFSET

#define HEART_BEAT_LSB_2 						CONFIG_OFFSET+673+SIM2_OFFSET

#define SMS_TIME_TRACK_LSB_2 				CONFIG_OFFSET+679+SIM2_OFFSET


#define DIST_TRK_LSB1_2 						CONFIG_OFFSET+684+SIM2_OFFSET
#define DIST_TRK_LSB2_2 						CONFIG_OFFSET+685+SIM2_OFFSET
#define DIST_TRK_MSB1_2 						CONFIG_OFFSET+686+SIM2_OFFSET
#define DIST_TRK_MSB2_2 						CONFIG_OFFSET+687+SIM2_OFFSET

#define HEADING_LSB_2 							CONFIG_OFFSET+690+SIM2_OFFSET

#define NO_FX_MN_ON_OFF_2 					CONFIG_OFFSET+696+SIM2_OFFSET

#define MD_ON_OFF_2 								CONFIG_OFFSET+697+SIM2_OFFSET

#define NO_FX_MN_TIME_2 						CONFIG_OFFSET+698+SIM2_OFFSET

#define TIME_TRK_IG_OFF_SET_2 			CONFIG_OFFSET+699+SIM2_OFFSET

#define Speed_Limit_S_2 						CONFIG_OFFSET+701+SIM2_OFFSET
#define SPED_DIFF_SET_2							CONFIG_OFFSET+704+SIM2_OFFSET

#define IDLE_TIME_SET_2 						CONFIG_OFFSET+705+SIM2_OFFSET


#define ACC_Harsh_B_TH_2 						CONFIG_OFFSET+718+SIM2_OFFSET
#define ACC_ACCEL_TH_2 							CONFIG_OFFSET+722+SIM2_OFFSET
#define ACC_IMPACT_TH_2 						CONFIG_OFFSET+726+SIM2_OFFSET
#define ACC_ANYMOTION_TH_2 					CONFIG_OFFSET+730+SIM2_OFFSET
#define ACC_ANYMOTION_TIME_2 				CONFIG_OFFSET+731+SIM2_OFFSET

#define REASON_CODE_SET_2 					CONFIG_OFFSET+732+SIM2_OFFSET
#define REASON_CODE_LEN_2 					32


//////////////   Ply-Geo Fence Section //////////////////////////////
#define GEOFENCE_POINTS_OFFSET 			1024

#define POLY_GEOFENCE_Start 			  CONFIG_OFFSET+GEOFENCE_POINTS_OFFSET
#define POLY_GEOFENCE_ArrayLen 			3200

#define GEO1SMSStart 								CONFIG_OFFSET+4352
#define GEO1SMSLen 									5

#define GEO1NameStart 							CONFIG_OFFSET+4357
#define GEO1NameLen 								15

#define GEO2SMSStart 								CONFIG_OFFSET+4372
#define GEO2SMSLen 									5

#define GEO2NameStart 							CONFIG_OFFSET+4377
#define GEO2NameLen 								15

#define GEO3SMSStart 								CONFIG_OFFSET+4392
#define GEO3SMSLen 									5

#define GEO3NameStart 							CONFIG_OFFSET+4397
#define GEO3NameLen 								15

#define GEO4SMSStart 								CONFIG_OFFSET+4412
#define GEO4SMSLen 									5

#define GEO4NameStart 							CONFIG_OFFSET+4417
#define GEO4NameLen 								15

#define GEO5SMSStart 								CONFIG_OFFSET+4432
#define GEO5SMSLen 									5

#define GEO5NameStart 							CONFIG_OFFSET+4437
#define GEO5NameLen 								15

#define GEO6SMSStart 								CONFIG_OFFSET+4452
#define GEO6SMSLen 									5

#define GEO6NameStart 							CONFIG_OFFSET+4457
#define GEO6NameLen 								15

#define GEO7SMSStart 								CONFIG_OFFSET+4472
#define GEO7SMSLen 									5

#define GEO7NameStart 							CONFIG_OFFSET+4477
#define GEO7NameLen 								15

#define GEO8SMSStart 								CONFIG_OFFSET+4492
#define GEO8SMSLen 									5

#define GEO8NameStart 							CONFIG_OFFSET+4497
#define GEO8NameLen 								15

#define GEO9SMSStart 								CONFIG_OFFSET+4512
#define GEO9SMSLen 									5

#define GEO9NameStart 							CONFIG_OFFSET+4517
#define GEO9NameLen 								15

#define GEO10SMSStart 							CONFIG_OFFSET+4532
#define GEO10SMSLen 								5

#define GEO10NameStart 							CONFIG_OFFSET+4537
#define GEO10NameLen 								15

#define GEO11SMSStart 							CONFIG_OFFSET+4552
#define GEO11SMSLen 								5

#define GEO11NameStart 							CONFIG_OFFSET+4557
#define GEO11NameLen 								15

#define GEO12SMSStart 							CONFIG_OFFSET+4572
#define GEO12SMSLen 								5

#define GEO12NameStart 							CONFIG_OFFSET+4577
#define GEO12NameLen 								15

#define GEO13SMSStart 							CONFIG_OFFSET+4592
#define GEO13SMSLen 								5

#define GEO13NameStart 							CONFIG_OFFSET+4597
#define GEO13NameLen 								15

#define GEO14SMSStart 							CONFIG_OFFSET+4612
#define GEO14SMSLen 								5

#define GEO14NameStart 							CONFIG_OFFSET+4617
#define GEO14NameLen 								15

#define GEO15SMSStart 							CONFIG_OFFSET+4632
#define GEO15SMSLen 								5

#define GEO15NameStart 							CONFIG_OFFSET+4637
#define GEO15NameLen 								15

#define GEO16SMSStart 							CONFIG_OFFSET+4652
#define GEO16SMSLen 								5

#define GEO16NameStart 							CONFIG_OFFSET+4657
#define GEO16NameLen 								15

#define GEO17SMSStart 							CONFIG_OFFSET+4672
#define GEO17SMSLen 								5

#define GEO17NameStart 							CONFIG_OFFSET+4677
#define GEO17NameLen 								15

#define GEO18SMSStart 							CONFIG_OFFSET+4692
#define GEO18SMSLen 								5

#define GEO18NameStart 							CONFIG_OFFSET+4697
#define GEO18NameLen 								15

#define GEO19SMSStart 							CONFIG_OFFSET+4712
#define GEO19SMSLen 								5

#define GEO19NameStart 							CONFIG_OFFSET+4717
#define GEO19NameLen 								15

#define GEO20SMSStart 							CONFIG_OFFSET+4732
#define GEO20SMSLen 								5

#define GEO20NameStart 							CONFIG_OFFSET+4737
#define GEO20NameLen 								15


#define GEOIDLen 										3

#define GEO1IDStart 								CONFIG_OFFSET+4752
#define GEO2IDStart 								CONFIG_OFFSET+4755
#define GEO3IDStart 								CONFIG_OFFSET+4758
#define GEO4IDStart 								CONFIG_OFFSET+4761
#define GEO5IDStart 								CONFIG_OFFSET+4764
#define GEO6IDStart 								CONFIG_OFFSET+4767
#define GEO7IDStart 								CONFIG_OFFSET+4770
#define GEO8IDStart 								CONFIG_OFFSET+4773
#define GEO9IDStart 								CONFIG_OFFSET+4776
#define GEO10IDStart 								CONFIG_OFFSET+4779
#define GEO11IDStart 								CONFIG_OFFSET+4782
#define GEO12IDStart 								CONFIG_OFFSET+4785
#define GEO13IDStart 								CONFIG_OFFSET+4788
#define GEO14IDStart 								CONFIG_OFFSET+4791
#define GEO15IDStart 								CONFIG_OFFSET+4794
#define GEO16IDStart 								CONFIG_OFFSET+4797
#define GEO17IDStart 								CONFIG_OFFSET+4800
#define GEO18IDStart 								CONFIG_OFFSET+4803
#define GEO19IDStart 								CONFIG_OFFSET+4806
#define GEO20IDStart 								CONFIG_OFFSET+4809

#define GEO_SET_LEN 								10
#define GEO1_SET 										CONFIG_OFFSET+4812
#define GEO2_SET 										CONFIG_OFFSET+4822
#define GEO3_SET 										CONFIG_OFFSET+4832
#define GEO4_SET 										CONFIG_OFFSET+4842
#define GEO5_SET 										CONFIG_OFFSET+4852
#define GEO6_SET 										CONFIG_OFFSET+4862
#define GEO7_SET 										CONFIG_OFFSET+4872
#define GEO8_SET 										CONFIG_OFFSET+4882
#define GEO9_SET 										CONFIG_OFFSET+4892
#define GEO10_SET 									CONFIG_OFFSET+4902
#define GEO11_SET 									CONFIG_OFFSET+4912
#define GEO12_SET 									CONFIG_OFFSET+4922
#define GEO13_SET 									CONFIG_OFFSET+4932
#define GEO14_SET 									CONFIG_OFFSET+4942
#define GEO15_SET 									CONFIG_OFFSET+4952
#define GEO16_SET 									CONFIG_OFFSET+4962
#define GEO17_SET 									CONFIG_OFFSET+4972
#define GEO18_SET 									CONFIG_OFFSET+4982
#define GEO19_SET 									CONFIG_OFFSET+4992
#define GEO20_SET 									CONFIG_OFFSET+5002
/////////////////////////////////////////////////////////////////////////
//Define Arrays Sizes
#define ftpServerIP1Size    						20
#define ftpServerPort1Size    					6
#define ftp1UserSize    								25
#define ftp1PasswordSize    						15
#define ftp1PathSize    								20

#define ftpServerIP2Size    						20
#define ftpServerPort2Size    					6
#define ftp2UserSize    								25
#define ftp2PasswordSize    						15
#define ftp2PathSize    								20

#define serverIP1Size    								16
#define serverPort1Size    							6
#define serverIP2Size    								16
#define serverPort2Size    							6
#define IMEISize    										20
#define IMSISize    										20
#define ServerSMSCellNoSize    					20
#define ServerSMSCellNoIOGenerateSize   10

#define smsPasswordSize    							5
#define gprsPasswordSize    						5
#define sim1APNSize    									15
#define sim1APNUserSize    							10
#define sim1APNPasswordSize    					10
#define sim2APNSize    									15
#define sim2APNUserSize    							10
#define sim2APNPasswordSize    					10
#define sim3APNSize    									15
#define sim3APNUserSize    							10
#define sim3APNPasswordSize    					10
#define sleepModeSettingSize    				5
#define replyToSendSize    							700
#define TCP_UDPSize    									1
#define IOConfigureSize    						7


#define ignitionConfigureSize    				7
#define sosConfigureSize    						7
#define geoConfigureSize    						5
#define geoAreaNameSize    							15
#define geoUIDSize    									4
#define geoSettingSize    							11


#define DIHeaderSize    								10
#define Ignition_HeaderSize    					10
#define SOS_HeaderSize    							10

#define IAP_FileNameSize    						20

#define AlertsOnOffEncodeSize    				33
#define SelectTcpUdpSize    						2

#define CompanyIDSize    								5
#define LotIDSize    										9
#define PIDSize    											11
#define CIDSize    											11
#define SIM1NoSize    									21
#define SIM2NoSize    									21

struct _Config 
{	
	unsigned char mcuRebootFlag;	
	
	unsigned char pingReqFlag;
	unsigned char FuelrelayCutFlag;
	unsigned char relay1CutFlag;
	unsigned char relay2CutFlag;
	unsigned char firmwareInitializ;
	unsigned char firmwareErrorFlag;
	unsigned char noGPSMNOnOff;
	unsigned char overSpeedDiffA;
	
	unsigned char ftpServerIP1[ftpServerIP1Size];
	unsigned char ftpServerPort1[ftpServerPort1Size];
	unsigned char ftp1User[ftp1UserSize];
	unsigned char ftp1Passsword[ftp1PasswordSize];
	unsigned char ftp1Path[ftp1PathSize];
	
	unsigned char serverIP1[serverIP1Size];
	unsigned char serverPort1[serverPort1Size];	
	unsigned char serverIP2[serverIP2Size];
	unsigned char serverPort2[serverPort2Size];
	unsigned char IMEI[IMEISize];
	unsigned char IMEIIndex;	
	unsigned char IMSI[IMSISize];
	unsigned char IMSIIndex;	
	
	unsigned char ServerSMSCellNo1[ServerSMSCellNoSize];
	unsigned char ServerSMSCellNo1IOGenerate[ServerSMSCellNoIOGenerateSize];
	unsigned char ServerSMSCellNo2[ServerSMSCellNoSize];
	unsigned char ServerSMSCellNo2IOGenerate[ServerSMSCellNoIOGenerateSize];
	unsigned char ServerSMSCellNo3[ServerSMSCellNoSize];
	unsigned char ServerSMSCellNo3IOGenerate[ServerSMSCellNoIOGenerateSize];	
	
	unsigned char ServerCallCellNo1[ServerSMSCellNoSize];
	unsigned char ServerCallCellNo1IOGenerate[ServerSMSCellNoIOGenerateSize];
	unsigned char ServerCallCellNo2[ServerSMSCellNoSize];
	unsigned char ServerCallCellNo2IOGenerate[ServerSMSCellNoIOGenerateSize];
	unsigned char ServerCallCellNo3[ServerSMSCellNoSize];
	unsigned char ServerCallCellNo3IOGenerate[ServerSMSCellNoIOGenerateSize];	
	
	unsigned char smsPassword[smsPasswordSize];	
	unsigned char gprsPassword[gprsPasswordSize];		
	unsigned char sim1APN[sim1APNSize];	
	unsigned char sim1APNUser[sim1APNUserSize];
	unsigned char sim1APNPassword[sim1APNPasswordSize];
	unsigned char sim2APN[sim2APNSize];	
	unsigned char sim2APNUser[sim2APNUserSize];
	unsigned char sim2APNPassword[sim2APNPasswordSize];
	unsigned char sim3APN[sim3APNSize];	
	unsigned char sim3APNUser[sim3APNUserSize];
	unsigned char sim3APNPassword[sim3APNPasswordSize];
	unsigned char carRegNo[15];
	unsigned char sleepModeSetting[sleepModeSettingSize];// user config
	unsigned int sleepModeTimeout;// 65535*8 secs maximum timeout for sleep mode user config
	long totalTimeTravelled;
	int idleTimeSetting;
	long distanceTotal;	// total distance traveled by car	
	unsigned char simNumber;
	unsigned char timeFlag;
	unsigned char totalTimeTravelledAASCI[travelTimeAAsciLen];
	unsigned char simShiftEn;
	unsigned int transmissionTimeIGon;	
	unsigned int transmissionTimeIGOff;	
	unsigned int transmissionTimeSMS;		
	unsigned int heartBeatTime;
	unsigned char MD_ON_OFF_FLAG;
	unsigned char gpsNoFixMNTime;
	unsigned int speedLimit;	
	long distanceTracking;	
	unsigned char replyToSend[replyToSendSize];	
	unsigned char replyReadyGprs;
	unsigned char replyReadySms;
	unsigned char SelectTcpUdp[SelectTcpUdpSize];
	unsigned char analogIoReadEnable;	
	char timeZoneSettings;	
	unsigned int headingChange;	
	//unsigned char fenceChangeAck;	
	double geo1Lat;
	double geo1Lon;
	long radius1;
	double geo2Lat;
	double geo2Lon;
	long radius2;
	double geo3Lat;
	double geo3Lon;
	long radius3;
	double geo4Lat;
	double geo4Lon;
	long radius4;	
	double geo5Lat;
	double geo5Lon;
	long radius5;
	double geo6Lat;
	double geo6Lon;
	long radius6;
	double geo7Lat;
	double geo7Lon;
	long radius7;
	double geo8Lat;
	double geo8Lon;
	long radius8;
	unsigned int analog5Min;
	unsigned int analog5Max;
	unsigned int analog5Diff;
	unsigned int analog6Min;
	unsigned int analog6Max;
	unsigned int analog6Diff;
	//unsigned char tripReportSMS;
	unsigned char eepromRecovSett;	
	unsigned char SDRecovSett;	
	
	unsigned char jmpCallFlag;	
	unsigned char IO1Configure[IOConfigureSize];
	unsigned char IO2Configure[IOConfigureSize];
	unsigned char IO3Configure[IOConfigureSize];
	unsigned char IO4Configure[IOConfigureSize];
	//unsigned char IO3Configure[7];
	unsigned char ignitionConfigure[ignitionConfigureSize];
	unsigned char sosConfigure[sosConfigureSize];	
	unsigned char geo1Configure[geoConfigureSize];	
	unsigned char geo1AreaName[geoAreaNameSize];		
	unsigned char geo2Configure[geoConfigureSize];	
	unsigned char geo2AreaName[geoAreaNameSize];	
	unsigned char geo3Configure[geoConfigureSize];	
	unsigned char geo3AreaName[geoAreaNameSize];
	unsigned char geo4Configure[geoConfigureSize];	
	unsigned char geo4AreaName[geoAreaNameSize];
	unsigned char geo5Configure[geoConfigureSize];	
	unsigned char geo5AreaName[geoAreaNameSize];
	unsigned char geo6Configure[geoConfigureSize];	
	unsigned char geo6AreaName[geoAreaNameSize];
	unsigned char geo7Configure[geoConfigureSize];	
	unsigned char geo7AreaName[geoAreaNameSize];
	unsigned char geo8Configure[geoConfigureSize];	
	unsigned char geo8AreaName[geoAreaNameSize];
	unsigned char geo9Configure[geoConfigureSize];	
	unsigned char geo9AreaName[geoAreaNameSize];
	unsigned char geo10Configure[geoConfigureSize];	
	unsigned char geo10AreaName[geoAreaNameSize];

	
	
	unsigned char geo1UID[geoUIDSize];
	unsigned char geo1Setting[geoSettingSize];
	unsigned char geo2UID[geoUIDSize];
	unsigned char geo2Setting[geoSettingSize];
	unsigned char geo3UID[geoUIDSize];
	unsigned char geo3Setting[geoSettingSize];
	unsigned char geo4UID[geoUIDSize];
	unsigned char geo4Setting[geoSettingSize];
	unsigned char geo5UID[geoUIDSize];
	unsigned char geo5Setting[geoSettingSize];
	unsigned char geo6UID[geoUIDSize];
	unsigned char geo6Setting[geoSettingSize];
	unsigned char geo7UID[geoUIDSize];
	unsigned char geo7Setting[geoSettingSize];
	unsigned char geo8UID[geoUIDSize];
	unsigned char geo8Setting[geoSettingSize];
	unsigned char geo9UID[geoUIDSize];
	unsigned char geo9Setting[geoSettingSize];
	unsigned char geo10UID[geoUIDSize];
	unsigned char geo10Setting[geoSettingSize];
	
	unsigned char sleepModeCounter;
	
	
	
	unsigned char AuthenticateFuelKSEnDs;
	
	unsigned char FuelKSPulDuration;
	unsigned char FuelKSPulTimes;
	
	//unsigned char KS1TriggerEnDs;
	unsigned char KS1PulDuration;
	unsigned char KS1PulTimes;
	
	//unsigned char KS2TriggerEnDs;
	unsigned char KS2PulDuration;
	unsigned char KS2PulTimes;
	
	

	unsigned char inputsHysPoll;
	
	
	
	unsigned char MainPwrHysPoll;
	unsigned char MainPwrHysPollDuration;
	unsigned char MainPwrHysPollMinTrueAlert;
	
	unsigned char ignitionHysPoll;
	unsigned char ignitionHysPollDuration;
	unsigned char ignitionHysPollMinTrueAlert;
	
	unsigned char SOSHysPoll;
	unsigned char SOSHysPollDuration;
	unsigned char SOSHysPollMinTrueAlert;
	
	unsigned char DIO1HysPoll;
	unsigned char DIO1HysPollDuration;
	unsigned char DIO1HysPollMinTrueAlert;
	
	unsigned char DIO2HysPoll;
	unsigned char DIO2HysPollDuration;
	unsigned char DIO2HysPollMinTrueAlert;
	
	unsigned char DIO3HysPoll;
	unsigned char DIO3HysPollDuration;
	unsigned char DIO3HysPollMinTrueAlert;
	
	
	unsigned char AIO1HysPoll ;
	unsigned char AIO1HysPollDuration;
	unsigned char AIO1HysAvgSamples ;
	unsigned int AIO1HysMinValue ;
	unsigned int AIO1HysMaxValue ;
	unsigned int AIO1HysDifference;
	
	unsigned char AIO2HysPoll ;
	unsigned char AIO2HysPollDuration;
	unsigned char AIO2HysAvgSamples ;
	unsigned int AIO2HysMinValue ;
	unsigned int AIO2HysMaxValue ;
	unsigned int AIO2HysDifference;
	
	unsigned char AIO3HysPoll ;
	unsigned char AIO3HysPollDuration;
	unsigned char AIO3HysAvgSamples ;
	unsigned int AIO3HysMinValue ;
	unsigned int AIO3HysMaxValue ;
	unsigned int AIO3HysDifference;
	
	unsigned char AIO4HysPoll ;
	unsigned char AIO4HysPollDuration;
	unsigned char AIO4HysAvgSamples ;
	unsigned int AIO4HysMinValue ;
	unsigned int AIO4HysMaxValue ;
	unsigned int AIO4HysDifference;
	
	uint8_t SharpTurnHysPoll;
	uint16_t SharpTurnThresh;

	unsigned char TPMSSetting;
	unsigned char FTPSetting;
	unsigned char IAPSetting;
	uint16_t IAPResetFlag;
	uint8_t IAPDownloadedFlag;
	uint32_t IAPFileSize;
	uint8_t IAPFileName[IAP_FileNameSize];
	uint32_t IAPFileAddress;
	
	uint8_t IAPUpdatedFlag;
	uint8_t IAPReadyFlag;
	
	uint8_t CANSetting ;
	uint8_t FPSetting;
	uint8_t RFIDSetting;
	
	uint8_t IMEINoSett;
	uint8_t eepromAlgoSet;
	uint8_t BuzzerSett;
	
	uint8_t PolyGeoFences;
	uint8_t PolyGeoFencePoints;
	
	uint8_t GeoFenceAlgo;
	
	float HarshBreakDetectTHR;
	float AccelerationDetectTHR;
	float ImpactDetectTHR;
	float AnymotionDetectTHR;
	
	unsigned char GSMNoNetworkTry;
	unsigned char GSMIPTry;
	unsigned char GSMTryDelay;
	unsigned char GSMTryRebootEnDs;
	uint16_t AuthenticateTimout;
	
	unsigned char AckLayerEnDs;
	unsigned char AckTimeout;
	unsigned char AckTry;
	unsigned char IAPTry;
	
	unsigned char AlertsOnOffEncode[AlertsOnOffEncodeSize];
	unsigned char NextAnymotionAlertTimeOut;
	
	unsigned char CompanyID[CompanyIDSize];
	unsigned char LotID[LotIDSize];
	unsigned char PID[PIDSize];
	unsigned char CID[CIDSize];
	unsigned char SIM1No[SIM1NoSize];
	unsigned char SIM2No[SIM2NoSize];
};

_DECL struct _Config config;

struct _Header 
{	
	unsigned char SOS_Header[SOS_HeaderSize];
	unsigned char DI1Header[DIHeaderSize];
	unsigned char DI2Header[DIHeaderSize];
	unsigned char DI3Header[DIHeaderSize];

	unsigned char Ignition_Header[Ignition_HeaderSize];	
	
		
};

_DECL struct _Header header;

/* Global function prototypes -----------------------------------------------*/
void Config_FactorySettings_SIM1(void);
void Config_FactorySettings_SIM2(void);
void Config_FactorySettings_SIM1_SIM2(void);
void Config_Settings_Request_Process(unsigned char*,unsigned char);
void Factory_Config_Read(uint8_t ReadGeneralConfig,uint8_t SIM_No);
void Factory_Config_Write(void)/* Writes Factory default settings to EEPROM on first run of programmed firmware */;
//void integerWrite(unsigned int firstByteAddress,unsigned int secondByteAddress, unsigned int writeContent);

void multiByteContentWrite(unsigned int,unsigned int, unsigned char*);
void multiByteContentRead(unsigned int,unsigned int, unsigned char*);


void eepromWriteDouble(uint32_t address, double value);
double eepromReadDouble(uint32_t address);
#endif /* __CONFIGURATION_H */


/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
