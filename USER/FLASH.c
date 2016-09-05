///*
// * SPIFuncs.c
// *
// * Created: 5/26/2012 3:06:57 PM
// * Author: Fakhar
// Last Update : 05:July_2012 
// SPIFuncs.cpp Includes

//1 - SPI Data write 
//2 - SPI Initialization
//3 - SPIO Data Reading
// 
// */


//#include "Global_Defines.h"

///**
//  ******************************************************************************
//  * @file    SPI_FLASH_Configuration.c
//  * @author  M.Uzair Afzal
//  * @version V1.0.0
//  * @date    02-May-2013
//  * @brief   This file provides a set of functions needed to manage the SPI SST25VF064C
//  *          FLASH memory.It implements a high level communication layer for read and write 
//  *          from/to this memory. The needed STM32 hardware resources (SPI and 
//  *          GPIO) are defined and the initialization is performed in sFLASH_LowLevel_Init() 
//	*					 function.You can easily tailor this driver to any other development board, 
//  *          by just adapting the defines for hardware resources and 
//  *          sFLASH_LowLevel_Init() function.
//  *            
//  *          +-----------------------------------------------------------+
//  *          |                     Pin assignment                        |
//  *          +-----------------------------+---------------+-------------+
//  *          |  STM32 SPI Pins             |     sFLASH    |    Pin      |
//  *          +-----------------------------+---------------+-------------+
//  *          | sFLASH_CS_PIN               |ChipSelect(/CS)|    1        |
//  *          | sFLASH_SPI_MISO_PIN / MISO  |   DataOut(DO) |    2        |
//  *          |                             |   /WP         |    3        |
//  *          |                             |   GND         |    4 (0 V)  |
//  *          | sFLASH_SPI_MOSI_PIN / MOSI  |   DataIn(DI)  |    5        |
//  *          | sFLASH_SPI_SCK_PIN / SCLK   |   Clock(CLK)  |    6        |
//  *          |                             |   /HOLD       |    7        |
//  *          |                             |   VCC         |    8 (3.3 V)|  
//  *          +-----------------------------+---------------+-------------+  
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

///* Includes ------------------------------------------------------------------*/
//#include "Global_Defines.h"
///////////////////////////////////// SST25VF064C ///////////////////////////////////////////
//#ifdef  FLASH_ENABLED

//#define  FLASH_WriteAddress     0x600000
//#define  FLASH_ReadAddress      FLASH_WriteAddress
//#define  FLASH_EraseAddress     FLASH_WriteAddress
//#define  countof(a) (sizeof(a) / sizeof(*(a)))
//#define  BufferSize (countof(Tx_Buffer)-1)
//////////////////////////////////////////////////////////////////////////////////
///**
//  * @brief  FLASH Test Protocol
//  * @param  None
//  * @retval Result of Flash test
//  */
//uint8_t sFLASH_Test(void)
//{	
//	//uint8_t  Tx_Buffer[256] = "STM32F1071G SST25VF064C SPI Firmware Library Example// Author: Muammad Uzair Afzal// Date: 26-12-2013// Version: V1.0.0// Test version::";
//	//uint8_t  Rx_Buffer[BufferSize];
//	//uint16_t Index = 0x00;
//	uint8_t  RxBuff[40];
//	uint8_t  ReadBytes = 0;
//	//uint8_t  i=0;
//	//uint8_t  StatusReg;
//	TestStatus TransferStatus1 = PASSED;
//	
//	USART_SendData_s( DEBUG_COM,"\r\nFLASH Test Start\r\n");

//	USART_SendData_s( DEBUG_COM,"\r\nFlash JEDEC ID(3 Bytes)=");
//	ReadBytes = sFLASH_Command_Read_ID(RxBuff,sFLASH_CMD_RD_JEDECID);

//	if(RxBuff[0]==0xBF && RxBuff[1]==0x25  && RxBuff[2]==0x4B)
//	{
//		USART_SendData_s( DEBUG_COM,"\r\nJEDECID OK\r\n");
//	}
//	
//	sFLASH_UnlockChip();
//	/*USART_SendData_s( DEBUG_COM,"\r\nFlash Security ID(32 Bytes)=");
//	ReadBytes = sFLASH_Command_Read_ID(RxBuff,sFLASH_CMD_RD_SID);
//	for(i=0;i<ReadBytes;i++)
//	{
//		USART_Put( DEBUG_COM,RxBuff[i]);
//	}
//	*/
//	
//	
//	/*USART_SendData_s( DEBUG_COM,"\r\nFlash Erasing.....");
//	// Perform a write in the Flash followed by a read of the written data 
//	// Erase SPI FLASH Sector to write on 
//	sFLASH_Erase4KB_32KB_64KB_Chip(IAP_FLASH_PAGE_ERASE_ADDRESS,sFLASH_CMD_ER_CHIP);
//	//   Read data from SPI FLASH memory 
//  sFLASH_ReadBuffer(Rx_Buffer, FLASH_ReadAddress, BufferSize);
//  // Check the correctness of erasing operation  
//  for (Index = 0; Index < BufferSize; Index++)
//  {
//		if (Rx_Buffer[Index] != 0xFF)
//    {
//      TransferStatus1 = FAILED;
//    }	
//  }
//	if(TransferStatus1==PASSED)
//	{
//		USART_SendData_s( DEBUG_COM,"\n\rFlash Erase OK::\n\r");
//	}
//	else
//	{
//		USART_SendData_s( DEBUG_COM,"\n\rFlash Erase FAILED::\n\r");
//	}
//	
//	USART_SendData_s( DEBUG_COM,"Flash Writing.....\r\n");
//	// Write Tx_Buffer data to SPI FLASH memory 
//  //sFLASH_WriteBuffer(Tx_Buffer, FLASH_WriteAddress, BufferSize);
//	// Read data from SPI FLASH memory 
//  //sFLASH_ReadBuffer(Rx_Buffer, FLASH_ReadAddress, BufferSize);
//	sFLASH_DMA_Configuration(Tx_Buffer,256);
//	sFLASH_DMA_WritePage(FLASH_ReadAddress);
//	Delay_ms(10);
//	sFLASH_ReadBuffer(Rx_Buffer,FLASH_ReadAddress, 256);
//	
//	// Check the correctness of written dada 
//  TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);
//	// TransferStatus1 = PASSED, if the transmitted and received data by SPI1 are the same 
//  // TransferStatus1 = FAILED, if the transmitted and received data by SPI1 are different 
//	if(TransferStatus1 == PASSED)
//	{
//		USART_SendData_s( DEBUG_COM,"Flash Write OK::\n\r");
//    for (Index = 0; Index < BufferSize; Index++)
//    {
//			USART_Put( DEBUG_COM,Rx_Buffer[Index]);
//    }
//	}
//	else
//	{
//		USART_SendData_s( DEBUG_COM,"Flash Write Error::\n\r");
//		for (Index = 0; Index < BufferSize; Index++)
//    {
//			USART_Put( DEBUG_COM,Rx_Buffer[Index]);
//    }
//	}*/
//	USART_SendData_s( DEBUG_COM,"FLASH Test End\n\r");
//	return TransferStatus1;
//}

///**
//  * @brief  Configures the DMA for SPI.
//  * @param  None
//  * @retval None
//  */

////void sFLASH_DMA_Configuration(uint8_t* Tx_Buffer,uint16_t DMABufferSize)
////{
//////	// DMA clock enable 
//////  RCC_AHB1PeriphClockCmd(sFLASH_DMA_PERIPH_CLK, ENABLE);
//////	
//////	DMA_DeInit(sFLASH_TX_DMA_STREAM);

//////  DMA_InitStructure.DMA_Channel = sFLASH_TX_DMA_CHANNEL;  
//////  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&sFLASH_SPI->DR;	
//////  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buffer;
//////  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//////  DMA_InitStructure.DMA_BufferSize = DMABufferSize;
//////  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//////  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//////  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//////  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//////  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//////  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//////  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
//////  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//////  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//////  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//////     
//////  DMA_Init(sFLASH_TX_DMA_STREAM, &DMA_InitStructure); 
//////	

//////  DMA_ITConfig(sFLASH_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
//////  
//////  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
////// 
//////  NVIC_InitStructure.NVIC_IRQChannel = sFLASH_TX_DMA_IRQn;
//////  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//////  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//////  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//////  NVIC_Init(&NVIC_InitStructure);
//////	
//////	// Enable SPI DMA TX request 
//////  SPI_I2S_DMACmd(sFLASH_SPI,SPI_I2S_DMAReq_Tx, ENABLE);
////}

////void sFLASH_TX_DMA_IRQHandler(void)
////{
////  /* Test on DMA Stream Transfer Complete interrupt */
//////	 if (DMA_GetITStatus(sFLASH_TX_DMA_STREAM, sFLASH_TX_DMA_IT_TCIF))
//////  {
//////		/* Clear DMA Stream Transfer Complete interrupt pending bit */
//////    DMA_ClearITPendingBit(sFLASH_TX_DMA_STREAM, sFLASH_TX_DMA_IT_TCIF);
//////		/*!< Disable DMA */
//////		DMA_Cmd(sFLASH_TX_DMA_STREAM, DISABLE);
//////		Delay(0x1);//This Delay is necessary
//////		/*!< Deselect the FLASH: Chip Select high */
//////		sFLASH_CS_HIGH();
//////		//USART_SendData_s( DEBUG_COM,"DMA Transfer Complete\r\n");
//////		/*!< Check If SPI Transmission is complete i.e; DR register is empty */
//////		while(SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);
//////		/*!< Check If SPI bus is free i.e; DR shift register is shifted out completely */
//////		while(SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_BSY) == SET);
//////		/*!< Read out DR register to clear it */
//////		SPI_I2S_ReceiveData(sFLASH_SPI);
//////  }
////}

///**
//  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle using DMA
//  *         (Page WRITE sequence).
//  * @note   The number of byte can't exceed the FLASH page size.
//  * @param  WriteAddr: FLASH's internal address to write to.
//  * @retval None
//  */
////void sFLASH_DMA_WritePage(uint32_t WriteAddr)
////{	
////  /*!< Enable the write access to the FLASH */
////  sFLASH_WriteEnable();
////  /*!< Select the FLASH: Chip Select low */
////  sFLASH_CS_LOW();
////  /*!< Send "Write to Memory " instruction */
////  sFLASH_SendByte(sFLASH_CMD_WR_PAGE);
////  /*!< Send WriteAddr high nibble address byte to write to */
////  sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
////  /*!< Send WriteAddr medium nibble address byte to write to */
////  sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
////  /*!< Send WriteAddr low nibble address byte to write to */
////  sFLASH_SendByte(WriteAddr & 0xFF);
////	
//////  DMA_Cmd(sFLASH_TX_DMA_STREAM, ENABLE);
////}
///**
//  * @brief  FLASH ID read Commands.
//  * @param  None
//  * @retval FLASH identification
//  */
//uint8_t sFLASH_Command_Read_ID(uint8_t* buffer, uint8_t cmd)
//{	
//	uint8_t l_counter=0;
//	uint8_t NumOfReadBytes=0;
//	switch(cmd)
//	{
//		case sFLASH_CMD_RD_SID:
//			/*!< Select the FLASH: Chip Select low */
//			sFLASH_CS_LOW();
//			/*!< Send "Read Security ID" instruction */
//			sFLASH_SendByte(sFLASH_CMD_RD_SID);
//			/*!< Send dummy byte(starting address 0x00) */
//			sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			/*!< Send dummy byte */
//			sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			/*!< Send dummy byte and receive the 32-byte SID*/
//			for(l_counter=0;l_counter<32;l_counter++)
//			{
//				*buffer++ = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			}
//			/*!< Deselect the FLASH: Chip Select high */
//			sFLASH_CS_HIGH();
//			NumOfReadBytes=32;
//			break;
//		
//		case sFLASH_CMD_RD_MANUF_DEVICEID:
//			/*!< Select the FLASH: Chip Select low */
//			sFLASH_CS_LOW();
//			/*!< Send "Read Manufacturer/Device ID" instruction */
//			sFLASH_SendByte(sFLASH_CMD_RD_MANUF_DEVICEID);
//			//Delay_ms(10);	
//			/*!< Send dummy byte */
//			sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Send dummy byte */
//			sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Send dummy byte */
//			sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Send dummy byte and receive the 8-bit Manufacturer ID(MF7-MF0)*/
//			*buffer++ = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Send dummy byte and receive the 8-bit Device ID(ID7-ID0)*/
//			*buffer++ = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Deselect the FLASH: Chip Select high */
//			sFLASH_CS_HIGH();
//			NumOfReadBytes=2;
//			break;
//		
//		case sFLASH_CMD_RD_JEDECID:
//			/*!< Select the FLASH: Chip Select low */
//			sFLASH_CS_LOW();
//			/*!< Send "Read JEDEC ID" instruction */
//			sFLASH_SendByte(sFLASH_CMD_RD_JEDECID);
//			//Delay_ms(10);	
//			/*!< Send dummy byte and receive the 8-bit Manufacturer ID(MF7-MF0)*/
//			*buffer++=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Send dummy byte and receive the 8-bit Memory type(ID15-ID8)*/
//			*buffer++=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Send dummy byte and receive the 8-bit Capacity(ID7-ID0)*/
//			*buffer++=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//			//Delay_ms(10);	
//			/*!< Deselect the FLASH: Chip Select high */
//			sFLASH_CS_HIGH();
//			NumOfReadBytes=3;
//			break;
//		
//		default:
//			break;
//	}
//	return NumOfReadBytes;
//}
///**
//  * @brief  Erases the specified FLASH memory.
//  * @param  BlockAddr: address of the Block to erase.Any address inside the Sector/Block (see manual for memory distribution) 
//	* is a valid address for the Sector/Block Erase instruction 
//	* @param  EraseCMD: Select erase area of the Block (can be 4KB sector/32KB block/64KB block or entire Chip).
//  *   This parameter can be one of the following values:
//  *     @arg sFLASH_CMD_ER_SECTOR_4KB: Erase 4KB sector
//	*     @arg sFLASH_CMD_ER_BLOCK_32KB: Erase 32KB Block
//	*     @arg sFLASH_CMD_ER_BLOCK_64KB: Erase 64KB Block
//	*     @arg sFLASH_CMD_ER_CHIP: Erase entire chip
//  * @retval None
//  */
//void sFLASH_Erase4KB_32KB_64KB_Chip(uint32_t BlockAddr,uint8_t EraseCMD)
//{
//  /*!< Send write enable instruction */
//  sFLASH_WriteEnable();
//	
//  /*!< Sector Erase */
//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send 4KB Sector Erase instruction */
//  sFLASH_SendByte(EraseCMD);
//	if(EraseCMD!=sFLASH_CMD_ER_CHIP)
//	{
//		/*!< Send SectorAddr high nibble address byte */
//		sFLASH_SendByte((BlockAddr & 0xFF0000) >> 16);
//		/*!< Send SectorAddr medium nibble address byte */
//		sFLASH_SendByte((BlockAddr & 0xFF00) >> 8);
//		/*!< Send SectorAddr low nibble address byte */
//		sFLASH_SendByte(BlockAddr & 0xFF);
//	}
//	/*!< Deselect the FLASH: Chip Select high */
//	sFLASH_CS_HIGH();
//  /*!< Wait the end of Flash writing */
//  sFLASH_WaitForWriteEnd();
//}

///**
//  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
//  *         (Page WRITE sequence).
//  * @note   The number of byte can't exceed the FLASH page size.
//  * @param  pBuffer: pointer to the buffer  containing the data to be written
//  *         to the FLASH.
//  * @param  WriteAddr: FLASH's internal address to write to.
//  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
//  *         or less than "sFLASH_PAGESIZE" value.
//  * @retval None
//  */
//void sFLASH_WriteByte(uint32_t WriteAddr, uint8_t data )
//{
//  /*!< Enable the write access to the FLASH */
//  sFLASH_WriteEnable();
//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send "Write to Memory " instruction */
//  sFLASH_SendByte(sFLASH_CMD_WR_PAGE);
//  /*!< Send WriteAddr high nibble address byte to write to */
//  sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
//  /*!< Send WriteAddr medium nibble address byte to write to */
//  sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
//  /*!< Send WriteAddr low nibble address byte to write to */
//  sFLASH_SendByte(WriteAddr & 0xFF);

//  /*!< Send the current byte */
//  sFLASH_SendByte(data);


//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();

//  /*!< Wait the end of Flash writing */
//  sFLASH_WaitForWriteEnd();
//}
///**
//  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
//  *         (Page WRITE sequence).
//  * @note   The number of byte can't exceed the FLASH page size.
//  * @param  pBuffer: pointer to the buffer  containing the data to be written
//  *         to the FLASH.
//  * @param  WriteAddr: FLASH's internal address to write to.
//  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
//  *         or less than "sFLASH_PAGESIZE" value.
//  * @retval None
//  */
//void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
//{
//  /*!< Enable the write access to the FLASH */
//  sFLASH_WriteEnable();
//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send "Write to Memory " instruction */
//  sFLASH_SendByte(sFLASH_CMD_WR_PAGE);
//  /*!< Send WriteAddr high nibble address byte to write to */
//  sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
//  /*!< Send WriteAddr medium nibble address byte to write to */
//  sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
//  /*!< Send WriteAddr low nibble address byte to write to */
//  sFLASH_SendByte(WriteAddr & 0xFF);
//  /*!< while there is data to be written on the FLASH */
//  while (NumByteToWrite--)
//  {
//    /*!< Send the current byte */
//    sFLASH_SendByte(*pBuffer);
//    /*!< Point on the next byte to be written */
//    pBuffer++;
//  }

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();

//  /*!< Wait the end of Flash writing */
//  sFLASH_WaitForWriteEnd();
//}
///**
//  * @brief  Writes block of data to the FLASH. In this function, the number of
//  *         WRITE cycles are reduced, using Page WRITE sequence.
//  * @param  pBuffer: pointer to the buffer  containing the data to be written
//  *         to the FLASH.
//  * @param  WriteAddr: FLASH's internal address to write to.
//  * @param  NumByteToWrite: number of bytes to write to the FLASH.
//  * @retval None
//  */
//void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
//{
//  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

//  Addr = WriteAddr % sFLASH_PAGESIZE;
//  count = sFLASH_PAGESIZE - Addr;
//  NumOfPage =  NumByteToWrite / sFLASH_PAGESIZE;
//  NumOfSingle = NumByteToWrite % sFLASH_PAGESIZE;

//  if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
//  {
//    if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
//    {
//      sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
//    }
//    else /*!< NumByteToWrite > sFLASH_PAGESIZE */
//    {
//      while (NumOfPage--)
//      {
//        sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_PAGESIZE);
//        WriteAddr +=  sFLASH_PAGESIZE;
//        pBuffer += sFLASH_PAGESIZE;
//      }

//      sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
//    }
//  }
//  else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
//  {
//    if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
//    {
//      if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
//      {
//        temp = NumOfSingle - count;

//        sFLASH_WritePage(pBuffer, WriteAddr, count);
//        WriteAddr +=  count;
//        pBuffer += count;

//        sFLASH_WritePage(pBuffer, WriteAddr, temp);
//      }
//      else
//      {
//        sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
//      }
//    }
//    else /*!< NumByteToWrite > sFLASH_PAGESIZE */
//    {
//      NumByteToWrite -= count;
//      NumOfPage =  NumByteToWrite / sFLASH_PAGESIZE;
//      NumOfSingle = NumByteToWrite % sFLASH_PAGESIZE;

//      sFLASH_WritePage(pBuffer, WriteAddr, count);
//      WriteAddr +=  count;
//      pBuffer += count;

//      while (NumOfPage--)
//      {
//        sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_PAGESIZE);
//        WriteAddr +=  sFLASH_PAGESIZE;
//        pBuffer += sFLASH_PAGESIZE;
//      }

//      if (NumOfSingle != 0)
//      {
//        sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
//      }
//    }
//  }
//}
///**
//  * @brief  Reads a block of data from the FLASH.
//  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
//  * @param  ReadAddr: FLASH's internal address to read from.
//  * @param  NumByteToRead: number of bytes to read from the FLASH.
//  * @retval None
//  */
//void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
//{
//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send "Read from Memory " instruction */
//  sFLASH_SendByte(sFLASH_CMD_RD_DATA);
//  /*!< Send ReadAddr high nibble address byte to read from */
//  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//  /*!< Send ReadAddr medium nibble address byte to read from */
//  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//  /*!< Send ReadAddr low nibble address byte to read from */
//  sFLASH_SendByte(ReadAddr & 0xFF);
//  while (NumByteToRead--) /*!< while there is data to be read */
//  {
//    /*!< Read a byte from the FLASH */
//    *pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//    /*!< Point to the next location where the byte read will be saved */
//    pBuffer++;
//  }

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//}
///**
//  * @brief  Initiates a read data byte (READ) sequence from the Flash.
//  *   This is done by driving the /CS line low to select the device, then the READ
//  *   instruction is transmitted followed by 3 bytes address. This function exit
//  *   and keep the /CS line low, so the Flash still being selected. With this
//  *   technique the whole content of the Flash is read with a single READ instruction.
//  * @param  ReadAddr: FLASH's internal address to read from.
//  * @retval None
//  */
//void sFLASH_StartReadSequence(uint32_t ReadAddr)
//{
//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();

//  /*!< Send "Read from Memory " instruction */
//  sFLASH_SendByte(sFLASH_CMD_RD_DATA);

//  /*!< Send the 24-bit address of the address to read from -------------------*/
//  /*!< Send ReadAddr high nibble address byte */
//  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//  /*!< Send ReadAddr medium nibble address byte */
//  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//  /*!< Send ReadAddr low nibble address byte */
//  sFLASH_SendByte(ReadAddr & 0xFF);
//}
///**
//  * @brief  Reads a byte from the SPI Flash.
//  * @note   This function must be used only if the Start_Read_Sequence function
//  *         has been previously called.
//  * @param  None
//  * @retval Byte Read from the SPI Flash.
//  */
//uint8_t sFLASH_ReadByte(uint32_t ReadAddr)
//{
//	uint8_t temp;
//	/*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();

//  /*!< Send "Read from Memory " instruction */
//  sFLASH_SendByte(sFLASH_CMD_RD_DATA);

//  /*!< Send the 24-bit address of the address to read from -------------------*/
//  /*!< Send ReadAddr high nibble address byte */
//  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//  /*!< Send ReadAddr medium nibble address byte */
//  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//  /*!< Send ReadAddr low nibble address byte */
//  sFLASH_SendByte(ReadAddr & 0xFF);
//	
//  temp= sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//	/*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//	return temp;
//}
///**
//  * @brief  Sends a byte through the SPI interface and return the byte received
//  *         from the SPI bus.
//  * @param  byte: byte to send.
//  * @retval The value of the received byte.
//  */
//uint8_t sFLASH_SendByte(uint8_t byte)
//{
//  /*!< Loop while DR register in not emplty */
//  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

//  /*!< Send byte through the SPI1 peripheral */
//  SPI_I2S_SendData(sFLASH_SPI, byte);

//  /*!< Wait to receive a byte */
//  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

//  /*!< Return the byte read from the SPI bus */
//  return SPI_I2S_ReceiveData(sFLASH_SPI);
//}
///**
//  * @brief  Sends a Half Word through the SPI interface and return the Half Word
//  *         received from the SPI bus.
//  * @param  HalfWord: Half Word to send.
//  * @retval The value of the received Half Word.
//  */
//uint16_t sFLASH_SendHalfWord(uint16_t HalfWord)
//{
//  /*!< Loop while DR register in not emplty */
//  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

//  /*!< Send Half Word through the sFLASH peripheral */
//  SPI_I2S_SendData(sFLASH_SPI, HalfWord);

//  /*!< Wait to receive a Half Word */
//  while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

//  /*!< Return the Half Word read from the SPI bus */
//  return SPI_I2S_ReceiveData(sFLASH_SPI);
//}
///**
//  * @brief  Enables the write access to the FLASH.
//  * @param  None
//  * @retval None
//  */
//void sFLASH_WriteEnable(void)
//{

//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();

//  /*!< Send "Write Enable" instruction */
//  sFLASH_SendByte(sFLASH_CMD_WR_ENABLE);

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//	
//}
///**
//  * @brief  Polls the status of the BUSY flag in the FLASH's
//  *         status register and loop until write opertaion has completed.
//  * @param  None
//  * @retval None
//  */
//void sFLASH_WaitForWriteEnd(void)
//{
//  uint8_t flashstatus = 0;

//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();

//  /*!< Send "Read Status Register" instruction */
//  sFLASH_SendByte(sFLASH_CMD_RD_STATUS_REG);

//  /*!< Loop as long as the memory is busy with a write cycle */
//  do
//  {
//    /*!< Send a dummy byte to generate the clock needed by the FLASH
//    and put the value of the status register in FLASH_Status variable */
//    flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

//  }
//  while ((flashstatus & sFLASH_BUSY_FLAG) == SET); /* Write in progress */

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//}
///**
//  * @brief  Reads Status Register 1 or 2.
//  * @param  sFLASH_CMD_RD_STATUS_REG: Specifies Whether Status register 1 or 2 is to be read.
//  *   This parameter can be one of the following values:
//  *     @arg sFLASH_CMD_RD_STATUS_REG1: Read Status Register 1
//	*     @arg sFLASH_CMD_RD_STATUS_REG2: Read Status Register 2
//  * @retval status register 8-bit value
//  */
//uint8_t sFLASH_ReadStatusRegister(uint8_t RD_STATUS_REG)
//{
//  uint8_t flashstatus = 0;

//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();

//  /*!< Send "Read Status Register" instruction */
//  sFLASH_SendByte(sFLASH_CMD_RD_STATUS_REG);
//	/*!< Send Dummy byte and receive 8-bit status register content*/
//	flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//	
//	return(flashstatus);
//}
///**
//  * @brief  Write Status Register 1 & 2.
//  * @param  status_reg_1: Value of status register 1 to be written
//  * @param  status_reg_2: Value of status register 2 to be written
//  * @retval None
//  */
//void sFLASH_WriteStatusRegisters(uint8_t status_reg)
//{
//	////// Write Status Register
//	/*!< Send write enable instruction */
//  sFLASH_WriteEnable();
//	/*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send "Write Status Register" instruction */
//  sFLASH_SendByte(sFLASH_CMD_WR_STATUS_REG);
//	
//	 /*!< Send Status Register value to be written*/
//	sFLASH_SendByte(status_reg);

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//	
//	/*!< Wait the end of Flash writing */
//  sFLASH_WaitForWriteEnd();
//}

///**
//  * @brief  Write Status Register BP3 BP2 BP1 BP0 bits with '0' to disable all blocks protection
//  * @param  None
//  * @retval None
//  */
//void sFLASH_UnlockChip(void)
//{
//	////// Write Status Register
//	/*!< Send write enable instruction */
//  sFLASH_WriteEnable();
//	/*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send "Write Status Register" instruction */
//  sFLASH_SendByte(sFLASH_CMD_WR_STATUS_REG);
//	
//	 /*!< Send Status Register value to be written*/
//	sFLASH_SendByte(0x00);

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//	
//	/*!< Wait the end of Flash writing */
//  sFLASH_WaitForWriteEnd();
//}
///**
//  * @brief  Write Status Register BP3 BP2 BP1 BP0 bits with '1' to enable all blocks protection
//  * @param  None
//  * @retval None
//  */
//void sFLASH_LockChip(void)
//{
//	////// Write Status Register
//	/*!< Send write enable instruction */
//  sFLASH_WriteEnable();
//	/*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();
//  /*!< Send "Write Status Register" instruction */
//  sFLASH_SendByte(sFLASH_CMD_WR_STATUS_REG);
//	
//	 /*!< Send Status Register value to be written*/
//	sFLASH_SendByte(0x3C);

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
//	
//	/*!< Wait the end of Flash writing */
//  sFLASH_WaitForWriteEnd();
//}

//#endif

///*********************************************************************************************************
//																					END FILE
//*********************************************************************************************************/
