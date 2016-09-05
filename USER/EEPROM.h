
/**
  ******************************************************************************
  * @file    MCU_IO.c
  * @author  TeReSol
  * @version V1.0.0
  * @date    24-July-2013
  * @brief   This file contains all the initilization functions for USART,CAN,I2C,ADC,DMA,DAC,NVIC and SPI.
  ******************************************************************************
*/

#include "Global_Defines.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

#define EEPROM_CS_LOW()       GPIO_ResetBits(SPI1_CS_PORT, SPI1_CS_PIN)
#define EEPROM_CS_HIGH()      GPIO_SetBits(SPI1_CS_PORT, SPI1_CS_PIN) 

#define EEPROM_CMD_WR_STATUS_REG         			0x01  	//!< Write Status Register
#define EEPROM_CMD_WR_PAGE			              0x02  	//!< Page Program
#define EEPROM_CMD_RD_DATA  	        				0x03  	//!< Read Data
#define EEPROM_CMD_WR_DISABLE            		  0x04  	//!< Write Disable Command
#define EEPROM_CMD_RD_STATUS_REG         			0x05  	//!< Read Status Register
#define EEPROM_CMD_WR_ENABLE  	        			0x06  	//!< Write Enable Command

#define EEPROM_DUMMY_BYTE         						0x00		//!< Dummy byte for SPI
#define EEPROM_BUSY_FLAG          						0x01  	//!< BUSY flag 

#define FALSE 												0
#define TRUE 													1

#define READ 													0
#define WRITE 												1



#define EEPROM_STRINGS_LIMIT 					350  //Remaining 162 pages are for Config setting
#define EEPROM_PAGE_SIZE 							256  //AT24C1024 Page size= 256 bytes, 24LC128=64 bytes 
#define READ_POINTER_IS_GREATER 			0
#define WRITE_POINTER_IS_GREATER 			1

#define NEW_DATA_OVERWRITE_ALGO				0
#define NEW_DATA_DISCARD_ALGO					1

#define LIVE_ALGO											0
#define FIFO_ALGO											1
#define LIFO_ALGO											2
#define ALERT_PRIORITY_ALGO						3

#define CONFIG_START_OFFSET           492  //Last 12 pages are for Config setting
#define CONFIG_OFFSET                 (CONFIG_START_OFFSET*256)


#define DATABASE_OFFSET								360
#define LIFO_OFFSET										(DATABASE_OFFSET*256) //1024(2048 bytes) entries for LIFO(16 bit)
#define USER_DATA_OFFSET					  	(LIFO_OFFSET+(8*256))  //32 bytes each for 1024 users; 128 pages

struct _EEPROM
{
	uint32_t eepromWritePointer;
	uint32_t eepromReadPointer;
	unsigned char eepromRecoveryEnable;
	uint8_t eepromWriteLogicAlgo;
};
_DECL  struct _EEPROM eeprom;


uint8_t EEPROM_Test(void);

unsigned char EEPROM_Handler(unsigned char*,uint32_t,uint16_t);
unsigned char EEPROM_Write_Module(unsigned char*);
unsigned char EEPROM_Read_Module(unsigned char*);
uint8_t EEWriteByte(uint32_t,uint8_t);
void EEPROM_I2C_Configuration(void);
uint8_t EEPROM_I2C_SQ_Write(uint32_t SW_addr, uint8_t* Wx_Buf, uint16_t NB2Write);
uint8_t EEPROM_I2C_SQ_Read( uint32_t SR_addr,uint8_t* Rx_buf, uint16_t NB2Read);

void eeprom_write_word(uint32_t SW_addr,uint16_t value);
uint16_t eeprom_read_word( uint32_t SR_addr);
uint8_t eeprom_write_byte(uint32_t SW_addr,uint8_t value);
uint8_t eeprom_read_byte( uint32_t SR_addr);

void eeprom_write_dword(uint32_t SW_addr,uint32_t value);
uint32_t eeprom_read_dword(uint32_t SW_addr);

uint8_t EEPROM_Config_Reset_Enable(void);
void EEPROM_LIFO_Reset_Enable(void);

void EEPROM_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void EEPROM_WaitForWriteEnd(void);
uint8_t EEPROM_SendByte(uint8_t byte);
void EEPROM_WriteEnable(void);
void EEPROM_UnlockChip(void);
void EEPROM_LockChip(void);

void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void sFLASH_DMA_Configuration(uint8_t* Tx_Buffer,uint16_t DMABufferSize);
void sFLASH_DMA_WritePage(uint32_t WriteAddr);

#endif /*__EEPROM_H*/
/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
