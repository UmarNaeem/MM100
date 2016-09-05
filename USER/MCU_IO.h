/**
  ******************************************************************************
  * @file    MCU_IO.h
  * @author  M.Uzair Afzal
  * @version V1.0.0
  * @date    24-July-2013
  * @brief   This file contains all the initilization functions for USART,CAN,I2C,ADC,DMA,DAC,NVIC and SPI.
  ******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_IO_H
#define __MCU_IO_H

#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

/* Includes ------------------------------------------------------------------*/


#include "stm32f10x.h"

/* Defines ------------------------------------------------------------------*/
typedef enum 
{
  COM1 = 0,
	COM2 = 1,
	COM3 = 2,
  COM4 = 3,
	COM5 = 4,
	COM6 = 5
} COM_TypeDef;

typedef enum 
{
  I2CBus1 = 0,
	I2CBus2 = 1
} I2CBus_TypeDef;

typedef enum 
{
  SPIBus1 = 0,
	SPIBus2 = 1,
	SPIBus3 = 2
} SPIBus_TypeDef;

typedef enum 
{
  CANBus1 = 0,
  CANBus2 = 1
} CANBus_TypeDef;

typedef enum 
{
  Timer2 = 0,
	Timer3 = 1,
	Timer4 = 2,
} TIMER_TypeDef;




#define I2C_BREAK_TIMOUT			DELAY_VALUE_1us*10

/**
 * @brief Defines for GPIOs
 */

/*----------------------------------------------------------------------------------------------------*/
										/*ANALOG INPUTS*/
/*----------------------------------------------------------------------------------------------------*/

/* Analog1 (ADC1-IN10) input pin----------------------------------------------------------------------*/
#define ADC1_LN10_PIN          					GPIO_Pin_0
#define ADC1_LN10_PORT         					GPIOC
#define ADC1_LN10_CLK          					RCC_APB2Periph_GPIOC
/* Analog2 (ADC1-IN11) input pin----------------------------------------------------------------------*/
#define ADC1_LN11_PIN          					GPIO_Pin_1
#define ADC1_LN11_PORT         					GPIOC
#define ADC1_LN11_CLK          					RCC_APB2Periph_GPIOC
/* Analog3 (ADC1-IN12) input pin----------------------------------------------------------------------*/
#define INT_BATT_ADC_PIN          			GPIO_Pin_2
#define INT_BATT_ADC_PORT         			GPIOC
#define INT_BATT_ADC_CLK          			RCC_APB2Periph_GPIOC
/* Analog4 (ADC1-IN13) input pin----------------------------------------------------------------------*/
#define MAIN_12V_ADC_PIN          			GPIO_Pin_3
#define MAIN_12V_ADC_PORT         			GPIOC
#define MAIN_12V_ADC_CLK          			RCC_APB2Periph_GPIOC

/*----------------------------------------------------------------------------------------------------*/
										/*DIGITAL IO INPUTS*/
/*----------------------------------------------------------------------------------------------------*/
/*  SOS(EXT-GPIO-IRQ-1) PIN input----------------------------------------------------------------------*/
#define SOS_PIN          								GPIO_Pin_4
#define SOS_PORT         								GPIOC
#define SOS_CLK          								RCC_APB2Periph_GPIOC
#define SOS_PINSOURCE     							GPIO_PinSource4
#define SOS_PORTSOURCE     							GPIO_PortSourceGPIOC
#define SOS_EXTI_LINE     							EXTI_Line4
#define SOS_EXTI_IRQn		    						EXTI4_IRQn
/*DIO1(EXT-GPIO-IRQ-1) PIN input----------------------------------------------------------------------*/
#define D1_OPTO_PIN          						GPIO_Pin_5
#define D1_OPTO_PORT         						GPIOC
#define D1_OPTO_CLK          						RCC_APB2Periph_GPIOC

/*DIO2(EXT-GPIO-IRQ-2) PIN input----------------------------------------------------------------------*/
#define D2_OPTO_PIN          						GPIO_Pin_6
#define D2_OPTO_PORT         						GPIOC
#define D2_OPTO_CLK          						RCC_APB2Periph_GPIOC

/*DIO3(EXT-GPIO-IRQ-2) PIN input----------------------------------------------------------------------*/
#define D3_OPTO_PIN          						GPIO_Pin_7
#define D3_OPTO_PORT         						GPIOC
#define D3_OPTO_CLK          						RCC_APB2Periph_GPIOC


/* IGN(MCU-IGN-INT) PIN input----------------------------------------------------------------------*/
#define CAR_IGNITION_ON_PIN          		GPIO_Pin_8
#define CAR_IGNITION_ON_PORT         		GPIOC
#define CAR_IGNITION_ON_CLK          		RCC_APB2Periph_GPIOC
#define CAR_IGNITION_ON_PINSOURCE     	GPIO_PinSource8
#define CAR_IGNITION_ON_PORTSOURCE     	GPIO_PortSourceGPIOC
#define CAR_IGNITION_ON_EXTI_LINE     	EXTI_Line8
#define CAR_IGNITION_ON_EXTI_IRQn		    EXTI9_5_IRQn

/* Accelerometer BMA_INT PIN(AM_uC_INT) input------------------------------------------------------*/
#define BMA_INT_PIN          						GPIO_Pin_0
#define BMA_INT_PORT         						GPIOD
#define BMA_INT_CLK          						RCC_APB2Periph_GPIOD
#define BMA_INT_PINSOURCE     					GPIO_PinSource0
#define BMA_INT_PORTSOURCE     					GPIO_PortSourceGPIOD
#define BMA_INT_EXTI_LINE     					EXTI_Line0
#define BMA_INT_EXTI_IRQn		    				EXTI0_IRQn

/* ------------WIRE-1-COMM-------------------------------------------------------*/
#define WIRE_1_PIN          						GPIO_Pin_10
#define WIRE_1_PORT         						GPIOD
#define WIRE_1_CLK          						RCC_APB2Periph_GPIOD
#define WIRE_1_PINSOURCE     						GPIO_PinSource10
#define WIRE_1_PORTSOURCE     					GPIO_PortSourceGPIOD
#define WIRE_1_EXTI_LINE     						EXTI_Line10
#define WIRE_1_EXTI_IRQn		    				EXTI15_10_IRQn

/* MAIN_POWER_DETECT input pin----------------------------------------------------------------------*/
#define MAIN_POWER_DETECT_PIN         	GPIO_Pin_11
#define MAIN_POWER_DETECT_PORT        	GPIOD
#define MAIN_POWER_DETECT_CLK         	RCC_APB2Periph_GPIOD


/*----------------------------------------------------------------------------------------------------*/
										/*GSM PINS*/
/*----------------------------------------------------------------------------------------------------*/

/* GSM M95_DCD Pin input----------------------------------------------------------------------*/
#define GSM_DCD_PIN          						GPIO_Pin_12
#define GSM_DCD_PORT         						GPIOD
#define GSM_DCD_CLK          						RCC_APB2Periph_GPIOD
/* GSM M95_RI Pin Input----------------------------------------------------------------------*/
#define GSM_RI_PIN          						GPIO_Pin_13
#define GSM_RI_PORT         						GPIOD
#define GSM_RI_CLK          						RCC_APB2Periph_GPIOD
#define GSM_RI_PINSOURCE     						GPIO_PinSource13
#define GSM_RI_PORTSOURCE     					GPIO_PortSourceGPIOD
#define GSM_RI_EXTI_LINE     						EXTI_Line13
#define GSM_RI_EXTI_IRQn		    				EXTI15_10_IRQn
/* GSM M95_DTR Pin Output----------------------------------------------------------------------*/
#define GSM_DTR_PIN          						GPIO_Pin_0
#define GSM_DTR_PORT         						GPIOE
#define GSM_DTR_CLK          						RCC_APB2Periph_GPIOE
/* GSM_STATUS_PIN input----------------------------------------------------------------------*/
#define GSM_STATUS_PIN          				GPIO_Pin_1
#define GSM_STATUS_PORT         				GPIOE
#define GSM_STATUS_CLK          				RCC_APB2Periph_GPIOE
#define GSM_STATUS_PINSOURCE     				GPIO_PinSource1
#define GSM_STATUS_PORTSOURCE     			GPIO_PortSourceGPIOE
#define GSM_STATUS_EXTI_LINE     				EXTI_Line1
#define GSM_STATUS_EXTI_IRQn		    		EXTI1_IRQn
/* GSM_PWRKEY_PIN output CONTROL----------------------------------------------------------------------*/
#define GSM_PWRKEY_PIN          				GPIO_Pin_2
#define GSM_PWRKEY_PORT         				GPIOE
#define GSM_PWRKEY_CLK          				RCC_APB2Periph_GPIOE

/* GPS_RESET_PIN output CONTROL----------------------------------------------------------------------*/
#define GPS_RESET_PIN          					GPIO_Pin_8
#define GPS_RESET_PORT         					GPIOE
#define GPS_RESET_CLK          					RCC_APB2Periph_GPIOE

/* Relay1(MCU-KILL-SWITCH-1) output CONTROL----------------------------------------------------------------------*/
#define RELAY1_PIN          						GPIO_Pin_9
#define RELAY1_PORT         						GPIOE
#define RELAY1_CLK          						RCC_APB2Periph_GPIOE
/* Relay2(MCU-KILL-SWITCH-2) output CONTROL----------------------------------------------------------------------*/
#define RELAY2_PIN          						GPIO_Pin_10
#define RELAY2_PORT         						GPIOE
#define RELAY2_CLK          						RCC_APB2Periph_GPIOE

/* GPS_FIX status LED(MCU-LED-G 2) output----------------------------------------------------------------------*/
#define GPS_FIX_LED_PIN          				GPIO_Pin_14
#define GPS_FIX_LED_PORT         				GPIOE
#define GPS_FIX_LED_CLK          				RCC_APB2Periph_GPIOE
/* STATE_MACHINE_STATUS LED(MCU-LED-G 3) output----------------------------------------------------------------------*/
#define STATE_MACHINE_LED_PIN          	GPIO_Pin_15
#define STATE_MACHINE_LED_PORT         	GPIOE
#define STATE_MACHINE_LED_CLK          	RCC_APB2Periph_GPIOE

/* BMA150 CS pin----------------------------------------------------------------------*/
#define BMA_CS_PIN              GPIO_Pin_12              
#define BMA_CS_PORT             GPIOB                       
#define BMA_CS_CLK              RCC_APB2Periph_GPIOB


/**
 * @brief Defines for Timers
 */ 
 #define TIMERn                            3
/* TIMER2 defines ------------------------------------------------------------------*/
#define TIMER_NO2		                    	TIM2
#define TIMER2_CLK                    		RCC_APB1Periph_TIM2
#define TIMER2_IRQn                   		TIM2_IRQn
#define TIMER2_AUTORELOAD_VLAUE           10338 // 8 sec Timer
/* TIMER2 defines ------------------------------------------------------------------*/
#define TIMER_NO3		                    	TIM3
#define TIMER3_CLK                    		RCC_APB1Periph_TIM3
#define TIMER3_IRQn                   		TIM3_IRQn
#define TIMER3_AUTORELOAD_VLAUE           6461 // 5 sec Timer
/* TIMER4 defines ------------------------------------------------------------------*/
#define TIMER_NO4		                    	TIM4
#define TIMER4_CLK                    		RCC_APB1Periph_TIM4
#define TIMER4_IRQn                   		TIM4_IRQn
#define TIMER4_AUTORELOAD_VLAUE           1292 // 1sec  //387 //300msec /*1292*2 // 2 sec Timer*/


/**
 * @brief Defines for ADCs
 */ 

/* ADC3 defines ------------------------------------------------------------------*/
#define ADC_NO1		                    	ADC1
#define ADC1_CLK                    		RCC_APB2Periph_ADC1
#define NO_OF_ADC_CHANNELS							4
#define ADC_INT_BATT_CH		 		          ADC_Channel_12
#define ADC_MAIN_12V_CH		       		    ADC_Channel_13
#define ADC_IO1_CH		  			    			ADC_Channel_10
#define ADC_IO2_CH		 			        		ADC_Channel_11

#define MAIN_12V_ADC		              	0
#define INT_BATT_ADC		              	1
#define IO1_ADC		              				2
#define IO2_ADC		              				3





/**
 * @brief Defines for COM ports USART
 */ 
#define COMn                            5

#define GSM_COM													COM5
#define GPS_COM													COM4
#define DEBUG_COM												COM1 

#define GSM_USART    										UART5
#define GSM_USART_IRQHandler    				UART5_IRQHandler

#define GPS_USART    										UART4
#define GPS_USART_IRQHandler    				UART4_IRQHandler

#define DEBUG_USART    									USART1
#define DEBUG_USART_IRQHandler    			USART1_IRQHandler

#define COM_BAUDRATE_4800								4800
#define COM_BAUDRATE_9600								9600
#define COM_BAUDRATE_19200							19200
#define COM_BAUDRATE_38400							38400
#define COM_BAUDRATE_115200							115200

/* COM port USART1(used for debugging) defines -------------------------------------------------------*/
#define COM_NO1		                    	USART1
#define COM1_CLK                    		RCC_APB2Periph_USART1
#define COM1_TX_PIN                 		GPIO_Pin_9
#define COM1_TX_PINSOURCE								GPIO_PinSource9
#define COM1_TX_PORT           					GPIOA
#define COM1_TX_CLK           			 		RCC_APB2Periph_GPIOA
#define COM1_RX_PIN                 		GPIO_Pin_10
#define COM1_RX_PINSOURCE								GPIO_PinSource10
#define COM1_RX_PORT           					GPIOA
#define COM1_RX_CLK            					RCC_APB2Periph_GPIOA
#define COM1_IRQn                   		USART1_IRQn
#define COM1_BAUDRATE										COM_BAUDRATE_115200

/* COM port UART4(Connected to GPS Module) defines ------------------------------------------------------------------*/
#define COM_NO4		                    	UART4
#define COM4_CLK                    		RCC_APB1Periph_UART4
#define COM4_TX_PIN                 		GPIO_Pin_10
#define COM4_TX_PINSOURCE								GPIO_PinSource10
#define COM4_TX_PORT           					GPIOC
#define COM4_TX_CLK           			 		RCC_APB2Periph_GPIOC
#define COM4_RX_PIN                 		GPIO_Pin_11
#define COM4_RX_PINSOURCE								GPIO_PinSource11
#define COM4_RX_PORT           					GPIOC
#define COM4_RX_CLK            					RCC_APB2Periph_GPIOA
#define COM4_IRQn                   		UART4_IRQn
#define COM4_BAUDRATE										COM_BAUDRATE_9600

/* COM port UART5(Connected to GSM Module) defines ------------------------------------------------------------------*/
#define COM_NO5		                    	UART5
#define COM5_CLK                    		RCC_APB1Periph_UART5
#define COM5_TX_PIN                 		GPIO_Pin_12
#define COM5_TX_PINSOURCE								GPIO_PinSource12
#define COM5_TX_PORT           					GPIOC
#define COM5_TX_CLK           			 		RCC_APB2Periph_GPIOC
#define COM5_RX_PIN                 		GPIO_Pin_2
#define COM5_RX_PINSOURCE								GPIO_PinSource2
#define COM5_RX_PORT           					GPIOD
#define COM5_RX_CLK            					RCC_APB2Periph_GPIOD
#define COM5_IRQn                   		UART5_IRQn
#define COM5_BAUDRATE										COM_BAUDRATE_9600

/**
 * @brief Defines for I2C ports
 */ 
#define I2Cn                            2

#define I2C_BMC													I2C1
#define I2C_EEPROM											I2C1

#define I2C_RSV													I2C1
#define I2C_GPS													I2C2

#define I2C_SPEED_100KHZ								100000
#define I2C_SPEED_200KHZ								200000
#define I2C_SPEED_400KHZ								400000

/* I2C2 defines ------------------------------------------------------------------*/
#define I2C_NO1		                    	I2C1
#define I2C1_CLK                    		RCC_APB1Periph_I2C1
#define I2C1_SCL_PIN                 		GPIO_Pin_6
#define I2C1_SCL_PORT           				GPIOB
#define I2C1_SCL_CLK           			 		RCC_APB2Periph_GPIOB
#define I2C1_SDA_PIN                 		GPIO_Pin_7
#define I2C1_SDA_PORT           				GPIOB
#define I2C1_SDA_CLK            				RCC_APB2Periph_GPIOB
#define I2C1_SPEED											I2C_SPEED_100KHZ

/* I2C2 defines ------------------------------------------------------------------*/
#define I2C_NO2		                    	I2C2
#define I2C2_CLK                    		RCC_APB1Periph_I2C2
#define I2C2_SCL_PIN                 		GPIO_Pin_10
#define I2C2_SCL_PORT           				GPIOB
#define I2C2_SCL_CLK           			 		RCC_APB2Periph_GPIOB
#define I2C2_SDA_PIN                 		GPIO_Pin_11
#define I2C2_SDA_PORT           				GPIOB
#define I2C2_SDA_CLK            				RCC_APB2Periph_GPIOB
#define I2C2_SPEED											I2C_SPEED_100KHZ


/**
 * @brief Defines for SPI ports
 */ 
#define SPIn                            3

#define EEPROM_SPI									SPIBus1
#define BMA150_SPI									SPIBus2

#define SPI_DEBUG												SPI1

/* SPI1 defines ------------------------------------------------------------------*/
#define SPI_NO1		                  SPI1
#define SPI1_CLK                    RCC_APB2Periph_SPI1

#define SPI1_CS_PIN                	GPIO_Pin_4
#define SPI1_CS_PORT           			GPIOA
#define SPI1_CS_CLK            			RCC_APB2Periph_GPIOA

#define SPI1_SCK_PIN                GPIO_Pin_5
#define SPI1_SCK_PINSOURCE          GPIO_PinSource5
#define SPI1_SCK_PORT           		GPIOA
#define SPI1_SCK_CLK           			RCC_APB2Periph_GPIOA

#define SPI1_MISO_PIN               GPIO_Pin_6
#define SPI1_MISO_PINSOURCE         GPIO_PinSource6
#define SPI1_MISO_PORT           		GPIOA
#define SPI1_MISO_CLK            		RCC_APB2Periph_GPIOA

#define SPI1_MOSI_PIN               GPIO_Pin_7
#define SPI1_MOSI_PINSOURCE         GPIO_PinSource7
#define SPI1_MOSI_PORT           		GPIOA
#define SPI1_MOSI_CLK            		RCC_APB2Periph_GPIOA

#define SD_CD_PIN                 	GPIO_Pin_2
#define SD_CD_PORT           				GPIOA
#define SD_CD_CLK            				RCC_APB2Periph_GPIOA

/* SPI2 defines ------------------------------------------------------------------*/
#define SPI_NO2		                  SPI2
#define SPI2_CLK                    RCC_APB1Periph_SPI2

#define SPI2_CS_PIN                 GPIO_Pin_12
#define SPI2_CS_PORT           			GPIOB
#define SPI2_CS_CLK            			RCC_APB2Periph_GPIOB

#define SPI2_SCK_PIN                GPIO_Pin_13
#define SPI2_SCK_PINSOURCE          GPIO_PinSource13
#define SPI2_SCK_PORT           		GPIOB
#define SPI2_SCK_CLK           			RCC_APB2Periph_GPIOB

#define SPI2_MISO_PIN               GPIO_Pin_14
#define SPI2_MISO_PINSOURCE         GPIO_PinSource14
#define SPI2_MISO_PORT           		GPIOB
#define SPI2_MISO_CLK            		RCC_APB2Periph_GPIOB

#define SPI2_MOSI_PIN               GPIO_Pin_15
#define SPI2_MOSI_PINSOURCE         GPIO_PinSource15
#define SPI2_MOSI_PORT           		GPIOB
#define SPI2_MOSI_CLK            		RCC_APB2Periph_GPIOB


#define MOSI 								2
#define SCK 								1
#define SS 									3


#define POLLING_CONFIG 			1
#define INTERRUPT_CONFIG 		1
#define TRUE							  1
#define FALSE 							0
#define RESET 							0
#define NULL_END 						0x00
#define CARRIAGE_RETURN 		0x0D
#define LINE_FEED 					0x0A
#define GPS_PORT  					1
#define SERIAL_PORT  				0


#define IGNITION 						0
#define SOS 								1
#define DI_1 								2
#define DI_2 								3
#define DI_3 								4

#define GEO1 								5
#define GEO2 								6
#define GEO3 								7
#define GEO4 								8
#define GEO5 								9
#define GEO6 								10
#define GEO7 								11
#define GEO8 								12
#define GEO9 								13
#define GEO10 							14
#define GEO11 							15
#define GEO12 							16
#define GEO13 							17
#define GEO14 							18
#define GEO15 							19
#define GEO16 							20
#define GEO17 							21
#define GEO18 							22
#define GEO19 							23
#define GEO20								24

#define SIM1							  1
#define SIM2 								2
#define SIM3 								3

#define SERVERIP1							  1
#define SERVERIP2 								2


#define BIT0 								0b00000001
#define BIT1 								0b00000010 
#define BIT2 								0b00000100
#define BIT3 								0b00001000
#define BIT4 								0b00010000
#define BIT5 								0b00100000
#define BIT6 								0b01000000
#define BIT7 								0b10000000



#define BIT(x)							(0x01 << (x)) 

#define BITVAL(x,y) (((x)>>(y)) & 1) 

#define bit_set(p,m) 				((p) |= (m))  

#define bit_flip(p,m) 			((p) ^= (m)) 

#define bit_clear(p,m)		  ((p) &= ~(m)) 

//////////////////////   LED Defines /////////////////////////////////////////////////////////
#define GPS_FIX_LED_ON()      			GPIO_ResetBits(GPS_FIX_LED_PORT, GPS_FIX_LED_PIN)  
#define GPS_FIX_LED_OFF()     			GPIO_SetBits(GPS_FIX_LED_PORT, GPS_FIX_LED_PIN)

#define STATE_MACHINE_LED_ON()     GPIO_ResetBits(STATE_MACHINE_LED_PORT,STATE_MACHINE_LED_PIN)  
#define STATE_MACHINE_LED_OFF()    GPIO_SetBits(STATE_MACHINE_LED_PORT,STATE_MACHINE_LED_PIN)
#define STATE_MACHINE_LED_TOGGLE()    GPIO_ToggleBits(STATE_MACHINE_LED_PORT,STATE_MACHINE_LED_PIN)

#define GPS_RESET_ON()      	GPIO_ResetBits(GPS_RESET_PORT, GPS_RESET_PIN)  
#define GPS_RESET_OFF()     	GPIO_SetBits(GPS_RESET_PORT, GPS_RESET_PIN)

#define SIM_MUX_A_LOW 			//*PORTC&=~_BV(1);
#define SIM_MUX_A_HI 				//*PORTC|=_BV(1);

#define SIM_MUX_B_LOW 			//*PORTC&=~_BV(2);//low
#define SIM_MUX_B_HI 				//*PORTC|=_BV(2);//hi

/* Relay1(EXT-GPIO-OUT-1) output CONTROL-------------------------------*/
/* Relay2(EXT-GPIO-OUT-2) output CONTROL------------------------------------------*/
/* Relay3(EXT-GPIO-OUT-3) Fuel Cutoff output CONTROL-------------------------------------*/
#define	FUEL_KS_CUT 					GPIO_SetBits(RELAY1_PORT, RELAY1_PIN)  //*PORTA|=_BV(0);//Pulse High kill switch
#define	RELAY1_CUT 					GPIO_SetBits(RELAY2_PORT, RELAY2_PIN)  //*PORTA|=_BV(1);//Pulse High kill switch
#define	RELAY2_CUT 					GPIO_SetBits(RELAY2_PORT, RELAY2_PIN)  //*PORTA|=_BV(1);//Pulse High kill switch

#define	FUEL_KS_MAKE 				GPIO_ResetBits(RELAY1_PORT, RELAY1_PIN)  //*PORTA&=~_BV(0);//low
#define	RELAY1_MAKE 				GPIO_ResetBits(RELAY2_PORT, RELAY2_PIN)  //*PORTA&=~_BV(1);//low
#define	RELAY2_MAKE 				GPIO_ResetBits(RELAY2_PORT, RELAY2_PIN)  //*PORTA&=~_BV(1);//low

  
#define analogIoDataSize  				50
#define digitalIoDataSize  				6
#define tripDistanceAasciSize  		12
#define travelTimeAasciSize  			12
#define totalTripTimeAasciSize  	12

#define DriverIDSize   						6
#define AuthenticateMethodSize   	3
#define DriverFPIDSize   					5
#define DriverRFIDSize  				  9


struct _ioRead
{	
	unsigned char analogIoData[analogIoDataSize];
	unsigned char digitalIoData[digitalIoDataSize];
		
	unsigned char inputIoByte1;
	unsigned char inputIoByte1LastState;
	
	unsigned char inputIoByte2;
	unsigned char inputIoByte2LastState;
	
	
	unsigned char ignitionOnOffState;

	float analog1;
	float analog1AvgSend;
	float analog2;
	
	float analog3;
	float analog3Avg;
	float analog3AvgSend;
	unsigned char analog3AvgCount;
	
	float analog4;
	float analog4Avg;
	float analog4AvgSend;
	unsigned char analog4AvgCount;
	
	float analog5;
	float analog5Avg;
	float analog5AvgSend;
	unsigned char analog5AvgCount;
	
	float analog6;
	float analog6Avg;
	float analog6AvgSend;
	unsigned char analog6AvgCount;




	unsigned int tripId;
	unsigned char ignitionEvenFlag;
	
	//unsigned char ignitionSMSFlag;
	
	unsigned char trackerOnFlag;	
	unsigned char trackerFirstOn;
	unsigned char powerStateChange;
	
	unsigned char FuelrelayState;
	unsigned char relay1State;
	unsigned char relay2State;
	
	unsigned char FuelrelayStateChangeFlag;
	unsigned char relay1StateChangeFlag;
	unsigned char relay2StateChangeFlag;
	unsigned char relaySwitchSpeedInteger;	
	
	
	unsigned char anyIOChangeFlag;
	
	unsigned char lowBattFlag;
	unsigned char lowBattFlagClear;
	
	unsigned char D1eventFlag;
	unsigned char D2eventFlag;
	unsigned char D3eventFlag;
	unsigned char mainPowerFlag;
	unsigned char mainPowerFlagStateClear;
	
	unsigned char tripCompEvent;
	
	unsigned char ignitionStateClear;
	unsigned char DIO1StateClear;
	unsigned char DIO2StateClear;
	unsigned char DIO3StateClear;
	unsigned char DIO4StateClear;
	unsigned char DIO5StateClear;
	unsigned char DIO6StateClear;
	unsigned char sosActive;
	unsigned char sosFlagSMS;
	
	unsigned char motionDetect;	
	
	unsigned char lastMotionDetected;
	unsigned char lastMotionDetDist;
	
	unsigned char o2TriggerActiveG8;
	unsigned char o2TriggerActiveG7;

	
	unsigned char geo1State;
	unsigned char geo1StateClear;	
	unsigned char geo2State;
	unsigned char geo2StateClear;
	unsigned char geo3State;
	unsigned char geo3StateClear;
	unsigned char geo4State;
	unsigned char geo4StateClear;
	unsigned char geo5State;
	unsigned char geo5StateClear;
	unsigned char geo6State;
	unsigned char geo6StateClear;
	unsigned char geo7State;
	unsigned char geo7StateClear;
	unsigned char geo8State;
	unsigned char geo8StateClear;
	unsigned char geo9State;
	unsigned char geo9StateClear;
	unsigned char geo10State;
	unsigned char geo10StateClear;
	unsigned char geo11State;
	unsigned char geo11StateClear;
	unsigned char geo12State;
	unsigned char geo12StateClear;
	unsigned char geo13State;
	unsigned char geo13StateClear;
	unsigned char geo14State;
	unsigned char geo14StateClear;
	unsigned char geo15State;
	unsigned char geo15StateClear;
	unsigned char geo16State;
	unsigned char geo16StateClear;
	unsigned char geo17State;
	unsigned char geo17StateClear;
	unsigned char geo18State;
	unsigned char geo18StateClear;
	unsigned char geo19State;
	unsigned char geo19StateClear;
	unsigned char geo20State;
	unsigned char geo20StateClear;
	
	
	unsigned char A3Event;
	unsigned char A3StateClear;
	
	unsigned char A4Event;
	unsigned char A4StateClear;
	
	unsigned char A5Event;
	unsigned char A5StateClear;
	
	unsigned char A6Event;
	unsigned char A6StateClear;

	
	long initDistance;
	long finalDistance;
	long tripDistance;
	
	long tripDistanceTripGen;
	
	
	unsigned char tripDistanceAasci[tripDistanceAasciSize];
	
	
	long initTime;
	long FinalTime;
	long totalTravelTime;
	
	unsigned char travelTimeAasci[travelTimeAasciSize];
	
	long totalTripTime;
	long totalIdleTime;
	
	unsigned char idleTimeAlert;
	unsigned char RepeatIdleTimeAlert;
	unsigned char totalTripTimeAasci[totalTripTimeAasciSize];
	
	uint16_t ADCConvertedValue[NO_OF_ADC_CHANNELS];
	
	unsigned char FuelKSTriggerEn;
	unsigned char FuelKSTriggerEnter;
	unsigned char FuelKSTriggerExit;
	
	unsigned char KS1TriggerEn;
	unsigned char KS1TriggerEnter;
	unsigned char KS1TriggerExit;
	
	unsigned char KS2TriggerEn;
	unsigned char KS2TriggerEnter;
	unsigned char KS2TriggerExit;
	
	unsigned char NextMotionAlertCount;

};
_DECL  struct _ioRead ioRead;

/* Global function prototypes -----------------------------------------------*/
void Timer_Init(TIMER_TypeDef  TIM, FunctionalState NewState );
//void Timer_Stop(TIMER_TypeDef  TIM);
void COM_Init(COM_TypeDef COM, FunctionalState NewState);
void USART_SendData_s(COM_TypeDef COM, unsigned char *pucBuffer);
void USART_Put(COM_TypeDef COM, uint8_t ch);
uint8_t USART_Get(COM_TypeDef COM);
void GPIO_EXTI_Init(void);
void I2CBus_Init(I2CBus_TypeDef I2C, FunctionalState NewState );


void SPIBus_Init(SPIBus_TypeDef SPI, FunctionalState NewState );
void SPIBus_CS_high_low(SPIBus_TypeDef SPI, uint8_t CS_state);

void NVIC_Interrupt(FunctionalState NewState );

void ADC_Channel_Init(FunctionalState NewState );
void ADC_StartConversion(void);
void ADC_StopConversion(void);

void SleepMode_Interrupt(FunctionalState NewState );

#endif /* __MCU_IO_H */
/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
