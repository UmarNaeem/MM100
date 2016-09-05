/**
  ******************************************************************************
  * @file    Global_Defines.h
  * @author  TeReSol
  * @version V1.0.0
  * @date    6-Oct-2012
  * @brief   This file contains all the functions prototypes for the Global Variables.
  ******************************************************************************
*/
//Global_Defines.h

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_DEFINES_H
#define __GLOBAL_DEFINES_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_conf.h"

#include "GPS.h"
#include "MCU_IO.h"
#include "main.h"
#include "GSM.h"
#include "EEPROM.h"
#include "eCompass.h"
#include "FLASH.h"
#include "Configuration.h"
#include "Tracker_Misc.h"

//#include "systick.h"
//#include "stm32f10x_systick.h"


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
//#include <arm_math.h>
/* Global Defines ------------------------------------------------------------------*/

#define GPIO_CLOCK_SPEED 					GPIO_Speed_50MHz

#define Increment           			1
#define Decrement									0

#define countof(a)   							(sizeof(a) / sizeof(*(a)))

/* Global Structures initiate ------------------------------------------------------------------*/
_DECL GPIO_InitTypeDef   					GPIO_InitStructure;
_DECL NVIC_InitTypeDef   					NVIC_InitStructure;
_DECL EXTI_InitTypeDef   					EXTI_InitStructure;
_DECL ADC_InitTypeDef    					ADC_InitStructure;
//_DECL DAC_InitTypeDef 	  				DAC_InitStructure;
_DECL DMA_InitTypeDef    					DMA_InitStructure;
_DECL TIM_TimeBaseInitTypeDef    	TIM_TimeBaseStructure;
_DECL TIM_ICInitTypeDef  					TIM_ICInitStructure;
//_DECL I2C_InitTypeDef             I2C_InitStructure;
_DECL SPI_InitTypeDef             SPI_InitStructure;
_DECL USART_InitTypeDef					  USART_InitStructure;
_DECL RCC_ClocksTypeDef  					Clock_Frequency;

// Global Structure for Interrupt Flags




/* External Parameters to be accessed Globally  ------------------------------------------------*/
_DECL uint8_t G_sprintfBuffer[30];
_DECL volatile uint32_t G_TimingDelay;

#define DELAY_VALUE_1us 			  0x29
#define DELAY_VALUE_1ms 			  0xA300
#define DELAY_MILLI_SEC         0x00
#define DELAY_MICRO_SEC         0x01


typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* External Parameters to be accessed Globally  ------------------------------------------------*/

/* Global Function prototypes ------------------------------------------------------------------*/
void Delay_viaCounter(uint8_t delay_type, uint32_t delay);
void Delay_ms(__IO uint32_t nTime);
void Delay (__IO uint32_t nCount);
void TimingDelay_Decrement(void);

void arrayInit2Zero(uint8_t* a_tempArray,uint16_t arraySize);
size_t strnlen (const char* s,uint16_t arraySize);
void displaySysClocks(void);
uint8_t ascii_to_hex(char ch1,char ch2);
uint16_t hex_to_ascii(char hexin);
uint8_t checksumCalc(uint8_t* mainbCMDArray, uint8_t startIndex, uint8_t stopIndex);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
#endif /* __GLOBAL_DEFINES_H */

/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
