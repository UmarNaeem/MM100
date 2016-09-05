/*
 * GPS.h
 *
 * Created: 3/7/2012 1:34:32 AM
 *  Author: Qasim
 */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_H
#define __GPS_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif
//#include <arm_math.h>
#include <stdint.h>

#define TIME_LOC 						1
#define DATE_LOC 						9
#define HEADING_LOC				  8
#define LATTITUDE_LOC 			3
#define LONGITUDE_LOC 			5
#define SPEED_LOC 					7
#define GPS_TRY 						30		
#define M_PI	 							3.1415926535897932384626433832795



#define gprmsBufferSize   						100
#define geoFenceNameSize 							3
#define numberOfSatelliteSize 				5
#define gpsTimeDateConvertedSize 			15
#define gpsTimeSize 									7
#define gpsDateSize 									7
#define gpsHeadingSize 								6
#define latSize   										15
#define latOrigSize   								15
#define latDecimalAASCISize   				15
#define latDirectionSize   						2
#define lonSize   										15
#define lonOrigSize   								15
#define lonDecimalAASCISize   				15
#define lonDirectionSize   						2
#define speedSize   									5
#define distanceAasciSize   					12
#define geoFencingTypeSize   					3
#define geoFencingUniqueIDSize    		4		
		
_DECL unsigned char gprmsBuffer[gprmsBufferSize];
_DECL unsigned char gprmsBufferIndex;		


#define gpsTempBufferSize       100

_DECL unsigned char gpsTempBuffer[gpsTempBufferSize];
_DECL unsigned char gpsTempBufferIndex;

struct _GPS
{	
	unsigned char gpsFix[4];	
	unsigned char gpsNoFixCounter;	
	
	unsigned char gpsNoFixCounterMD;	
	
	unsigned char gpsFixNoFix;
	unsigned char gpsFixClear;
	
	unsigned char firstGpsGet;
	
	unsigned char geoFenceName[geoFenceNameSize];
	
	unsigned char numberOfSatellite[numberOfSatelliteSize];		
	unsigned char gpsTimeDateConverted[gpsTimeDateConvertedSize];	
	unsigned char gpsTime[gpsTimeSize];
	float totalTime;	
	unsigned char hour;
	unsigned char minutes;
	unsigned char seconds;		
	unsigned char gpsDate[gpsDateSize];		
	unsigned char day;
	unsigned char month;
	unsigned int year;
	unsigned char gpsHeading[gpsHeadingSize];		
	float gpsHeadingFloat;		
	float gpsHeadingFloatTemp;	
	float gpsHeadingFloatDifference;	
	
	float gpsHeadingLastMaintain;
	
	unsigned char headingSendFlag;	
	unsigned char harshAccFlag;
	unsigned char harshBreakFlag;
		
	unsigned char speedLimitFlag;
	unsigned char overSpeedClearEvent;
	
	unsigned char speedLimitFlagClear;
	unsigned char distanceTrackFlag;	
	unsigned char lat[latSize];
	unsigned char latOrig[latOrigSize];
	double latFloat;			
	unsigned char latDecimalAASCI[latDecimalAASCISize];	
	unsigned char latDirection[latDirectionSize];	
	unsigned char lon[lonSize];
	unsigned char lonOrig[lonOrigSize];
	double lonFloat;			
	unsigned char lonDecimalAASCI[lonDecimalAASCISize];	
	unsigned char lonDirection[lonDirectionSize];		
	unsigned char speed[speedSize];	
	float speedFloat;	
	float speedRaw;	
	float speedFloatReSend;	
	
	unsigned char Heading_latDecimalAASCI[latDecimalAASCISize];
	unsigned char Heading_lonDecimalAASCI[lonDecimalAASCISize];	
		
	unsigned char distanceAasci[distanceAasciSize];		
	
	unsigned char geoFencingInFlag;	
	unsigned char geoFencingType[geoFencingTypeSize];
	
	unsigned char geoFencingUniqueID[geoFencingUniqueIDSize];
	
	unsigned char MD_Last_State;
	
	unsigned char geofenceMnCount;
	
	long distanceTrackTemp;
	
	long geofenceDistCheck;
	
	uint8_t FirstFixFlag;
	uint8_t TimerFlag;
	uint8_t TimerSelectFlag;
	uint8_t TimerPollCounter;
	
	
	uint8_t NoFixDetectCounter;
	
	uint8_t DataReceiveFlag;
	uint8_t GetDataState;
	uint8_t IgnitionOffSaveFlag;
	uint8_t MDLatLongUpdateFlag;
};	

_DECL struct _GPS GPS_LOG;

struct _timedate
{
	uint16_t year;
	uint8_t mo;
	uint8_t dd;
	uint8_t hh;
	uint8_t mm;
	uint8_t ss;
};

_DECL struct _timedate TD;


#define POLYGEOFENCE_ALGO      1

#define POLY_GEOFENCE     		 20
#define POLY_GEOFENCEPOINT     10

typedef struct
{
	double x;
	double y;
}GeoFencePoint;

_DECL GeoFencePoint Vertex[POLY_GEOFENCE][POLY_GEOFENCEPOINT+1];//11 point is used by algo to complete Polyon
_DECL GeoFencePoint Point;

double isLeft( GeoFencePoint P0, GeoFencePoint P1, GeoFencePoint P2 );
int8_t wn_PnPoly( GeoFencePoint P, int8_t n , int8_t FenceNo);


/* Global function prototypes -----------------------------------------------*/
void Init_TimeDate(void);
unsigned char GPS_Controller(void);	
//void gpsGPIOInit(void);
void TimeDate_Update(void);
unsigned char geoFencingEngine(void);
#endif /* __GPS_H */
