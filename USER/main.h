/*
 * GPS_Tracker.h
 *
 * Created: 3/7/2012 1:29:59 AM
 *  Author: Qasim
 */ 




/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif


#define  MB_COMM_ENABLED


#define  ECOMPASS_ENABLED
#define  FLASH_ENABLED

#define  FP_ENABLED
#define  RFID_ENABLED
#define  TPMS_ENABLED

#define  CAN_ENABLED

#define  GPS_ENABLED
#define  DEBUG_ENABLED



#endif /* __MAIN_H */

/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
