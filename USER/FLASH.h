///**
//  ******************************************************************************
//  * @file    SPI_FLASH_Configuration.h
//  * @author  M.Uzair Afzal	
//  * @version V1.0.0
//  * @date    02-May-2013
//  * @brief   This file contains all the functions prototypes for the SST25VF064C 8MB SPI Flash firmware driver.
//  ******************************************************************************
//  * @attention
//  *
//  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
//  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
//  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
//  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//  *
//  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
//  ******************************************************************************  
//  */  

///* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef __SPIFUNCS_H
//#define __SPIFUNCS_H

///* Define to declare and initialize global variables -------------------------------------*/
//#ifndef VAR_DECLS
//# define _DECL extern
//# define _INIT(x)
//#else
//# define _DECL
//# define _INIT(x)  = x
//#endif


///* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
//	 
///**
//  * @brief  SST25VF064C SPI Flash supported commands
//  */	 
//#define sFLASH_CMD_WR_ENABLE  	        			0x06  	//!< Write Enable Command
//#define sFLASH_CMD_WR_DISABLE            		  0x04  	//!< Write Disable Command
//#define sFLASH_CMD_RD_STATUS_REG         			0x05  	//!< Read Status Register
//#define sFLASH_CMD_EN_WR_STATUS_REG          	0x50  	//!< Enable-Write-Status-Register
//#define sFLASH_CMD_WR_STATUS_REG         			0x01  	//!< Write Status Register
//#define sFLASH_CMD_WR_PAGE			              0x02  	//!< Page Program
//#define sFLASH_CMD_WR_DUAL_IN_PAGE  		      0xA2  	//!< Dual-Input Page-Program
//#define sFLASH_CMD_ER_BLOCK_64KB          		0xD8  	//!< Block Erase (64KB)
//#define sFLASH_CMD_ER_BLOCK_32KB          		0x52  	//!< Block Erase (32KB) 
//#define sFLASH_CMD_ER_SECTOR_4KB              0x20  	//!< Sector Erase (4KB) 
//#define sFLASH_CMD_ER_CHIP				         		0xC7  	//!< Chip Erase (0xC7 or 0x60)

//#define sFLASH_CMD_RD_DATA  	        				0x03  	//!< Read Data 
//#define sFLASH_CMD_RD_DATA_FAST            		0x0B  	//!< Fast Read
//#define sFLASH_CMD_RD_FAST_DUAL_OUTPUT        0x3B  	//!< Fast Read Dual Output
//#define sFLASH_CMD_RD_FAST_DUAL_IO          	0xBB  	//!< Fast Read Dual I/O  

//#define sFLASH_CMD_EN_HOLD_FUNCTION        		0xAA  	//!< Enable HOLD# pin functionality of RST#/HOLD# pin
//#define sFLASH_CMD_RD_SID			      					0x88  	//!< Read Security ID
//#define sFLASH_CMD_WR_USER_SID_AREA  					0xA5  	//!< Program User Security ID area
//#define sFLASH_CMD_LOCK_SID_WR  							0x85  	//!< Lockout Security ID Programming

//#define sFLASH_CMD_RD_MANUF_DEVICEID          0x90  	//!< Read Manufacturer/Device ID (0xAB or 0x90)
//#define sFLASH_CMD_RD_JEDECID          			  0x9F  	//!< Read JEDEC ID 

//#define sFLASH_SST25VF064C_DEVICE_ID					0x4B		//!< 8-bit Device-ID 
//#define sFLASH_SST25VF064C_MANUFACTURER_ID		0xBF		//!< 8-bit Manufacturer-ID   (Byte 1 of JEDECID)
//#define sFLASH_SST25VF064C_MEMORY_TYPE				0x25		//!< 8-bit Device Memory-type(Byte 2 of JEDECID)
//#define sFLASH_SST25VF064C_MEMORY_CAPACITY		0x4B		//!< 8-bit Device Memory-type(Byte 3 of JEDECID)

//#define sFLASH_DUMMY_BYTE         						0x00		//!< Dummy byte for SPI
//#define sFLASH_BUSY_FLAG          						0x01  	//!< BUSY flag 
////#define sFLASH_SPI_PAGESIZE       						0x100		//!< Page Size of Flash(256 bytes)
//#define sFLASH_PAGESIZE       								0x100		//!< Page Size of Flash(256 bytes)

///**
//  * @brief  SST25VF064C SPI Flash Interface pins
//  */  
////#define sFLASH_SPI                       SPI1

////#define sFLASH_CS_PIN                    GPIO_Pin_0                  /* PB.00 */
////#define sFLASH_CS_GPIO_PORT              GPIOB                       /* GPIOB */
////#define sFLASH_CS_GPIO_CLK               RCC_AHB1Periph_GPIOB


/////* Definition for DMAx resources */
////#define sFLASH_DMA_PERIPH_CLK          	RCC_AHB1Periph_DMA2

////#define sFLASH_TX_DMA_CHANNEL			      DMA_Channel_3
////#define sFLASH_TX_DMA_STREAM      			DMA2_Stream3
////#define sFLASH_TX_DMA_IT_TCIF	    			DMA_IT_TCIF3
////#define sFLASH_TX_DMA_FLAG_TCIF	    		DMA_FLAG_TCIF3

////#define sFLASH_TX_DMA_IRQn          		DMA2_Stream3_IRQn
////#define sFLASH_TX_DMA_IRQHandler    		DMA2_Stream3_IRQHandler

///** @defgroup SPI_FLASH_CONFIGURATION_Exported_Macros
//  * @{
//  */
///**
//  * @brief  Select sFLASH: Chip Select pin low
//  */
////#define sFLASH_CS_LOW()       GPIO_ResetBits(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN)
/////**
////  * @brief  Deselect sFLASH: Chip Select pin high
////  */
////#define sFLASH_CS_HIGH()      GPIO_SetBits(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN)   

///** @defgroup SPI_FLASH_CONFIGURATION_Exported_Functions
//  * @{
//  */
//	
////#ifdef  FLASH_ENABLED
///**
//  * @brief  High layer functions
//  */
////void sFLASH_DMA_Configuration(uint8_t* Tx_Buffer,uint16_t DMABufferSize);
////void sFLASH_Erase4KB_32KB_64KB_Chip(uint32_t BlockAddr,uint8_t EraseCMD);
////void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
////void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
////void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
////void sFLASH_StartReadSequence(uint32_t ReadAddr);
////uint8_t sFLASH_ReadStatusRegister(uint8_t RD_STATUS_REG);
////void sFLASH_WriteStatusRegisters(uint8_t status_reg);
////uint8_t sFLASH_Command_Read_ID(uint8_t* buffer, uint8_t cmd);
////void sFLASH_LockChip(void);
////void sFLASH_UnlockChip(void);
////uint8_t sFLASH_Test(void);


////void sFLASH_WriteByte(uint32_t WriteAddr, uint8_t data );
////void sFLASH_DMA_WritePage(uint32_t WriteAddr);
/////**
////  * @brief  Low layer functions
////  */
//////uint8_t sFLASH_ReadByte(uint32_t ReadAddr);
////uint8_t sFLASH_SendByte(uint8_t byte);
////uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
////void sFLASH_WriteEnable(void);
//////void sFLASH_WaitForWriteEnd(void);
////#endif

//#endif /* __SPIFUNCS_H */
///*********************************************************************************************************
//																					END FILE
//*********************************************************************************************************/
