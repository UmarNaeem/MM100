/**
  ******************************************************************************
  * @file    EEPROM.c
  * @author  TeReSol
  * @version V1.0.0
  * @date    29-June-2014
  * @brief   This file contains all the initilization and data transfer functions for EEPROM.
  ******************************************************************************
*/

#include "Global_Defines.h"
/* Private Defines ------------------------------------------------------------------*/
unsigned char l_tempPacketLength1[20];	

static uint8_t g_FIFOsequence = WRITE_POINTER_IS_GREATER;	
uint8_t g_overWriteEndPointer=0;

#define SPI1_DR_ADDRESS    ((uint32_t)0x4001300C)

/*********************************************************************************************************
																			EEPROM_Test
													Test EEPROM by read and write datato EEPROM
*********************************************************************************************************/
uint8_t EEPROM_Test(void)
{
	uint8_t Write_Buf[] = "111111111111111111111111111111222222222222222222222222222222333333333333333333333333333333444444444444444444444444444444555555555555555555555555555555666666666666666666666666666666777777777777777777777777777777000000000000000000000000000000888888888888888888888888888888999999999999999999999999999999";
	uint8_t Read_Buf[300]={0x00};
	uint8_t Test_result=FALSE;

		USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nEEPROM Test Start\n\r");
		Test_result=EEPROM_I2C_SQ_Write(0x0000,Write_Buf,300);
		if(Test_result==TRUE)
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"EEPROM Sequential Write OK\n\r");
		}
		else if(Test_result==FALSE)
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"EEPROM Sequential Write FAIL\n\r");
		}
	
		Test_result=EEPROM_I2C_SQ_Read(0x0000,Read_Buf,300);
		Read_Buf[256]='\0';
		if(Test_result==TRUE)
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"EEPROM Sequential Read OK\n\r");
			USART_SendData_s( DEBUG_COM,(unsigned char*)Read_Buf);
			USART_SendData_s( DEBUG_COM,(unsigned char*)"\n\r");
		}
		else if(Test_result==FALSE)
		{
			USART_SendData_s( DEBUG_COM,(unsigned char*)"EEPROM Sequential Read FAIL\n\r");
		}
		USART_SendData_s( DEBUG_COM,(unsigned char*)"EEPROM Test End\n\r");

	return Test_result;
}

/*********************************************************************************************************
															sFLASH_DMA_Configuration
															Configure SPI 1 with DMA
*********************************************************************************************************/
void sFLASH_DMA_Configuration(uint8_t* Tx_Buffer,uint16_t DMABufferSize)
{
	// DMA clock enable 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Tx_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = DMABufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	// Enable SPI DMA TX request 
  SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx, ENABLE);
}

/*********************************************************************************************************
															sFLASH_TX_DMA_IRQHandler
															
*********************************************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{
		/* Test on DMA Stream Transfer Complete interrupt */
		if (DMA_GetITStatus(DMA1_IT_TC3))
		{
			/* Clear DMA Stream Transfer Complete interrupt pending bit */
			DMA_ClearITPendingBit(DMA1_IT_TC3);
			/*!< Disable DMA */
			DMA_Cmd(DMA1_Channel3, DISABLE);
			Delay(0x1);//This Delay is necessary
			/*!< Deselect the FLASH: Chip Select high */
			EEPROM_CS_HIGH();
			//USART_SendData_s( DEBUG_COM,(unsigned char*)"DMA Transfer Complete\r\n");
			/*!< Check If SPI Transmission is complete i.e; DR register is empty */
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			/*!< Check If SPI bus is free i.e; DR shift register is shifted out completely */
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
			/*!< Read out DR register to clear it */
			SPI_I2S_ReceiveData(SPI1);
		}
}

/*********************************************************************************************************
															sFLASH_DMA_WritePage
															
*********************************************************************************************************/
void sFLASH_DMA_WritePage(uint32_t WriteAddr)
{	
  
  EEPROM_WriteEnable();																/*!< Enable the write access to the FLASH */
  
  EEPROM_CS_LOW();																		/*!< Select the FLASH: Chip Select low */
  
  EEPROM_SendByte(EEPROM_CMD_WR_PAGE);								/*!< Send "Write to Memory " instruction */
  
  EEPROM_SendByte((WriteAddr & 0xFF0000) >> 16);			/*!< Send WriteAddr high nibble address byte to write to */
 
  EEPROM_SendByte((WriteAddr & 0xFF00) >> 8); 				/*!< Send WriteAddr medium nibble address byte to write to */
  
  EEPROM_SendByte(WriteAddr & 0xFF);									/*!< Send WriteAddr low nibble address byte to write to */
	
  DMA_Cmd(DMA1_Channel3, ENABLE);
}

/*********************************************************************************************************
															sFLASH_ReadBuffer
															
*********************************************************************************************************/
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  EEPROM_CS_LOW();																	/*!< Select the FLASH: Chip Select low */
  
  EEPROM_SendByte(EEPROM_CMD_RD_DATA);							/*!< Send "Read from Memory " instruction */
  
  EEPROM_SendByte((ReadAddr & 0xFF0000) >> 16);			/*!< Send ReadAddr high nibble address byte to read from */
 
  EEPROM_SendByte((ReadAddr& 0xFF00) >> 8); 				/*!< Send ReadAddr medium nibble address byte to read from */
  
  EEPROM_SendByte(ReadAddr & 0xFF);									/*!< Send ReadAddr low nibble address byte to read from */
	
  while (NumByteToRead--) 													/*!< while there is data to be read */
  {
    
    *pBuffer = EEPROM_SendByte(EEPROM_DUMMY_BYTE);	/*!< Read a byte from the FLASH */
    
    pBuffer++;																			/*!< Point to the next location where the byte read will be saved */
  }
  EEPROM_CS_HIGH();																	/*!< Deselect the FLASH: Chip Select high */
}
/*********************************************************************************************************
																			EEPROM Sequential write
													Writes specified bytes to the EEPROM using SPI
*********************************************************************************************************/
uint8_t EEPROM_I2C_SQ_Write(uint32_t WriteAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	uint8_t NumOfPage = 0, NumOfSingle = 0;
	
	uint8_t Status=TRUE;

  NumOfPage =  NumByteToWrite / 256;												/* Calcilate # of pages to write */
  NumOfSingle = NumByteToWrite % 256;												/* Calculate # of bytes to write */
	
	if (NumOfPage == 0) 																			/* NumByteToWrite < PAGESIZE */
	{
		EEPROM_WritePage(pBuffer, WriteAddr, NumByteToWrite);
	}
	else 																											/* NumByteToWrite > PAGESIZE */
	{
		while (NumOfPage--)
		{
			EEPROM_WritePage(pBuffer, WriteAddr, 256);
			WriteAddr +=  256;
			pBuffer += 256;
		}

    EEPROM_WritePage(pBuffer, WriteAddr, NumOfSingle);
	}
	return Status;
}

/*********************************************************************************************************
																			EEPROM Sequential read
													Reads specified bytes from the EEPROM using SPI
*********************************************************************************************************/
uint8_t EEPROM_I2C_SQ_Read( uint32_t ReadAddr,uint8_t* pBuffer, uint16_t NumByteToRead)
{
	uint8_t Status=TRUE;
	
  EEPROM_CS_LOW();																	/*!< Select the FLASH: Chip Select low */
  
  EEPROM_SendByte(EEPROM_CMD_RD_DATA);							/*!< Send "Read from Memory " instruction */
  
  EEPROM_SendByte((ReadAddr & 0xFF0000) >> 16);			/*!< Send ReadAddr high nibble address byte to read from */
  
  EEPROM_SendByte((ReadAddr& 0xFF00) >> 8);					/*!< Send ReadAddr medium nibble address byte to read from */
  
  EEPROM_SendByte(ReadAddr & 0xFF);									/*!< Send ReadAddr low nibble address byte to read from */
	
  while (NumByteToRead--) 													/*!< while there is data to be read */
  {
    *pBuffer = EEPROM_SendByte(EEPROM_DUMMY_BYTE);	/*!< Read a byte from the FLASH */
    
    pBuffer++;																			/*!< Point to the next location where the byte read will be saved */
  }

  EEPROM_CS_HIGH();																	/*!< Deselect the FLASH: Chip Select high */

	return Status;
}
///**
//  * @brief  enables factory default variables values reset in eeprom at firt startup 
//  * @param  None
//  * @retval Result of EEPROM test
//  */
//void EEPROM_LIFO_Reset_Enable(void)
//{
//	uint32_t l_loop=0;
//	for(l_loop=0;l_loop<DBBufferSize;l_loop++)
//	{
//		DBBuffer[l_loop]=0xAA;
//	}
//	
//	
//	for(l_loop=LIFO_OFFSET;l_loop<USER_DATA_OFFSET;l_loop=l_loop+DBBufferSize)
//	{
//		EEPROM_I2C_SQ_Write(l_loop,DBBuffer,DBBufferSize);
//	}
//	eeprom_write_byte(LIFO_OFFSET,0x00);
//	eeprom_write_byte(LIFO_OFFSET+1,0x00);

//	
//	USART_SendData_s( DEBUG_COM,"\r\nEEPROM LIFO Reset\n\r");
//}

/*********************************************************************************************************
																EEPROM_Config_Reset_Enable
							Enables factory default variables values reset in eeprom at firt startup
*********************************************************************************************************/
uint8_t EEPROM_Config_Reset_Enable(void)
{
	uint8_t Test_result=FALSE;

	uint32_t l_loop=0;
	for(l_loop=CONFIG_OFFSET;l_loop<(CONFIG_OFFSET+10);l_loop++)
	{
		Test_result=eeprom_write_byte(l_loop,0xFF);
	}
	
	USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nEEPROM Reset Enabled\n\r");
	return Test_result;
}

/*********************************************************************************************************
																			EEPROM Write Byte
														Write a byte to the EEPROM using SPI
*********************************************************************************************************/
uint8_t eeprom_write_byte(uint32_t WriteAddr, uint8_t data)
{
	uint8_t Status=TRUE;
	
	EEPROM_CS_HIGH();
	//Delay_ms(5);
	
  EEPROM_WriteEnable();															/*!< Enable the write access to the FLASH */
  
	//Delay_ms(10);
	
  EEPROM_CS_LOW();																	/*!< Select the FLASH: Chip Select low */
  
  EEPROM_SendByte(EEPROM_CMD_WR_PAGE);							/*!< Send "Write to Memory " instruction */
  
  EEPROM_SendByte((WriteAddr & 0xFF0000) >> 16);		/*!< Send WriteAddr high nibble address byte to write to */
  EEPROM_SendByte((WriteAddr & 0xFF00) >> 8);				/*!< Send WriteAddr medium nibble address byte to write to */
  EEPROM_SendByte(WriteAddr & 0xFF);								/*!< Send WriteAddr low nibble address byte to write to */

  EEPROM_SendByte(data);														/*!< Send the current byte */

  EEPROM_CS_HIGH();																	/*!< Deselect the FLASH: Chip Select high */

  EEPROM_WaitForWriteEnd();													/*!< Wait the end of Flash writing */

	return Status;
}

/*********************************************************************************************************
																			EEPROM Read Byte
														Reads a byte from the EEPROM using SPI
*********************************************************************************************************/
uint8_t eeprom_read_byte(uint32_t ReadAddr)
{
	uint8_t temp;
	
  EEPROM_CS_LOW();															/*!< Select the FLASH: Chip Select low */

  EEPROM_SendByte(EEPROM_CMD_RD_DATA);					/*!< Send "Read from Memory " instruction */

  EEPROM_SendByte((ReadAddr & 0xFF0000) >> 16);	/*!< Send ReadAddr high nibble address byte */
  EEPROM_SendByte((ReadAddr& 0xFF00) >> 8);			/*!< Send ReadAddr medium nibble address byte */
  EEPROM_SendByte(ReadAddr & 0xFF);							/*!< Send ReadAddr low nibble address byte */
	
  temp= EEPROM_SendByte(EEPROM_DUMMY_BYTE);
	
  EEPROM_CS_HIGH();/*!< Deselect the FLASH: Chip Select high */
	
	return temp;
}

/*********************************************************************************************************
																			EEPROM Write word
														Write a word to the EEPROM using SPI
*********************************************************************************************************/
void eeprom_write_word(uint32_t SW_addr,uint16_t value)
{
	eeprom_write_byte(SW_addr,(uint8_t)((value & 0xFF00) >> 8));//MSB
	eeprom_write_byte(SW_addr+1,(uint8_t)(value & 0x00FF));//LSB
}

/*********************************************************************************************************
																			EEPROM Read word
														Reads a word from the EEPROM using SPI
*********************************************************************************************************/
/**
  * @brief  .
  */
uint16_t eeprom_read_word( uint32_t SR_addr)
{
	uint8_t temp_array[2];
	uint32_t return_value=0;
	
	temp_array[0]=eeprom_read_byte(SR_addr);//MSB
	
	temp_array[1]=eeprom_read_byte(SR_addr+1);//LSB
	
	return_value=temp_array[0];
	return_value=return_value<<8;
	
	return_value=return_value | temp_array[1];
	
	return return_value;
}

/*********************************************************************************************************
																			EEPROM Write double word
														Write a double-word to the EEPROM using SPI
*********************************************************************************************************/
void eeprom_write_dword(uint32_t SW_addr,uint32_t value)
{
	eeprom_write_byte(SW_addr,(uint8_t)((value & 0xFF000000) >> 24));
	eeprom_write_byte(SW_addr+1,(uint8_t)((value & 0x00FF0000) >> 16));
	eeprom_write_byte(SW_addr+2,(uint8_t)((value & 0x0000FF00) >> 8));
	eeprom_write_byte(SW_addr+3,(uint8_t)(value & 0x000000FF));
}

/*********************************************************************************************************
																			EEPROM Read double word
														Read a double-word to the EEPROM using SPI
*********************************************************************************************************/
uint32_t eeprom_read_dword(uint32_t SW_addr)
{
	uint8_t temp_array[4];
	uint32_t return_value=0;
	
	temp_array[0]=eeprom_read_byte(SW_addr);//MSB
	
	temp_array[1]=eeprom_read_byte(SW_addr+1);
	
	temp_array[2]=eeprom_read_byte(SW_addr+2);
	
	temp_array[3]=eeprom_read_byte(SW_addr+3);//LSB
	
	return_value=temp_array[0];
	return_value=return_value<<8;
	
	return_value=return_value | temp_array[1];
	return_value=return_value<<8;
	
	return_value=return_value | temp_array[2];
	return_value=return_value<<8;
	
	return_value=return_value | temp_array[3];
	
	return return_value;
}

/*********************************************************************************************************
																			EEPROM_Write_Module
								writes the tracking strings to external EEPROM and handles exceptions
								unsigned char *: Pointer to string or array
								unsigned char : indicates whether EEPROM write is executed correctly or not
								TRUE: A string is correctly write to EEPROM
								FALSE: NO string is correctly write to EEPROM
*********************************************************************************************************/
unsigned char EEPROM_Write_Module(unsigned char *arrayPointer)
{
	unsigned char replyFlag = FALSE;	
	switch(config.eepromAlgoSet)
	{
		case FIFO_ALGO:
		{
			if(eeprom.eepromWriteLogicAlgo == NEW_DATA_DISCARD_ALGO )
			{
				if(!(eeprom.eepromReadPointer == eeprom.eepromWritePointer && g_FIFOsequence==READ_POINTER_IS_GREATER))
				{
					eeprom.eepromRecoveryEnable = TRUE ;
					
					replyFlag = EEPROM_I2C_SQ_Write( (eeprom.eepromWritePointer*EEPROM_PAGE_SIZE) ,arrayPointer, EEPROM_PAGE_SIZE);
					eeprom.eepromWritePointer++;
					if(eeprom.eepromWritePointer>=EEPROM_STRINGS_LIMIT)
					{
						eeprom.eepromWritePointer = 0;
						if(g_FIFOsequence==WRITE_POINTER_IS_GREATER)
						{
							g_FIFOsequence=READ_POINTER_IS_GREATER;
						}	
						else if(g_FIFOsequence==READ_POINTER_IS_GREATER)
						{
							g_FIFOsequence=WRITE_POINTER_IS_GREATER;
						}
					}
				}
			
				else
				{
					///USART_SendData_s( DEBUG_COM,"Write is not permitted:\r\n");
				}
			}
			
			else if(eeprom.eepromWriteLogicAlgo == NEW_DATA_OVERWRITE_ALGO )
			{
					eeprom.eepromRecoveryEnable = TRUE ;
					replyFlag = EEPROM_I2C_SQ_Write( (eeprom.eepromWritePointer*EEPROM_PAGE_SIZE)  ,arrayPointer, EEPROM_PAGE_SIZE);	
					eeprom.eepromWritePointer++;
					if(g_overWriteEndPointer<EEPROM_STRINGS_LIMIT)
					{
						g_overWriteEndPointer=g_overWriteEndPointer++;
					}
					if(eeprom.eepromWritePointer>=EEPROM_STRINGS_LIMIT)
					{
						eeprom.eepromWritePointer = 0;
						if(g_FIFOsequence==WRITE_POINTER_IS_GREATER)
						{
							g_FIFOsequence=READ_POINTER_IS_GREATER;
						}	
						else if(g_FIFOsequence==READ_POINTER_IS_GREATER)
						{
							g_FIFOsequence=WRITE_POINTER_IS_GREATER;
						}
					}
			}
			
			break;
		}
		
		case LIFO_ALGO:
		{
			break;
		}
		
		default:
			break;
	}
	/*USART_SendData_s( DEBUG_COM,"Read:");
	sprintf(( char*)l_tempPacketLength1,"%d",eeprom.eepromReadPointer);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( DEBUG_COM,"Write:");
	sprintf(( char*)l_tempPacketLength1,"%d",eeprom.eepromWritePointer);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( DEBUG_COM,"FIFO:");
	sprintf(( char*)l_tempPacketLength1,"%d",g_FIFOsequence);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( DEBUG_COM,"WriteEndPointer:");
	sprintf(( char*)l_tempPacketLength1,"%d",g_overWriteEndPointer);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	*/
	
	return replyFlag;
}

/*********************************************************************************************************
																			EEPROM_Read_Module
							Reads the tracking strings from external EEPROM and handles exceptions
							unsigned char *: Pointer to string or array
							unsigned char : indicates whether EEPROM read is executed correctly or not
							TRUE: A string is correctly read from EEPROM
							FALSE: NO string is correctly read from EEPROM
*********************************************************************************************************/
unsigned char EEPROM_Read_Module(unsigned char *arrayPointer)
{
	unsigned char replyFlag = FALSE;
	///USART_SendData_s( DEBUG_COM,"EEPROM READ SECTION\r\n");
	if(eeprom.eepromRecoveryEnable == TRUE )
	{
		if(eeprom.eepromWriteLogicAlgo == NEW_DATA_DISCARD_ALGO )
		{
			if((eeprom.eepromReadPointer != eeprom.eepromWritePointer )|| 
				(eeprom.eepromReadPointer == eeprom.eepromWritePointer && g_FIFOsequence==READ_POINTER_IS_GREATER))
			{
				
				replyFlag = EEPROM_I2C_SQ_Read( (eeprom.eepromReadPointer*EEPROM_PAGE_SIZE) ,arrayPointer, EEPROM_PAGE_SIZE);	
				eeprom.eepromReadPointer++;
				arrayPointer[1] = 'E';										
			
				if(eeprom.eepromReadPointer == eeprom.eepromWritePointer && g_FIFOsequence==WRITE_POINTER_IS_GREATER )				
				{											
					eeprom.eepromRecoveryEnable = FALSE;// recovery completed					
				}
			
				if(eeprom.eepromReadPointer==EEPROM_STRINGS_LIMIT)
				{
					eeprom.eepromReadPointer = 0;
					if(g_FIFOsequence==WRITE_POINTER_IS_GREATER)
					{
						g_FIFOsequence=READ_POINTER_IS_GREATER;
					}
					else if(g_FIFOsequence==READ_POINTER_IS_GREATER)
					{
						g_FIFOsequence=WRITE_POINTER_IS_GREATER;
					}
				}
			}
			else
			{
				///USART_SendData_s( DEBUG_COM,"Read is not permitted:\r\n");			
			}
		}
		
		
		else if(eeprom.eepromWriteLogicAlgo == NEW_DATA_OVERWRITE_ALGO )
		{
			if((g_overWriteEndPointer!=0))
			{			

				replyFlag = EEPROM_I2C_SQ_Read((eeprom.eepromReadPointer*EEPROM_PAGE_SIZE)  ,arrayPointer, EEPROM_PAGE_SIZE);
				g_overWriteEndPointer--;
				eeprom.eepromReadPointer++;
				arrayPointer[1] = 'E';										
			
				if(g_overWriteEndPointer==0)				
				{											
					eeprom.eepromRecoveryEnable = FALSE;// recovery completed
					eeprom.eepromReadPointer=eeprom.eepromWritePointer;
				}
			
				if(eeprom.eepromReadPointer==EEPROM_STRINGS_LIMIT)
				{
					eeprom.eepromReadPointer = 0;
					if(g_FIFOsequence==WRITE_POINTER_IS_GREATER)
					{
						g_FIFOsequence=READ_POINTER_IS_GREATER;
					}
					else if(g_FIFOsequence==READ_POINTER_IS_GREATER)
					{
						g_FIFOsequence=WRITE_POINTER_IS_GREATER;
					}
				}
			}
			else
			{
				///USART_SendData_s( DEBUG_COM,"Read is not permitted:\r\n");			
			}
		}
		
	}
	
	else
	{
		
		///USART_SendData_s( DEBUG_COM,"Recovery FLAG False:\r\n");
		
	}

	/*USART_SendData_s( DEBUG_COM,"Read:");
	sprintf(( char*)l_tempPacketLength1,"%d",eeprom.eepromReadPointer);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( DEBUG_COM,"Write:");
	sprintf(( char*)l_tempPacketLength1,"%d",eeprom.eepromWritePointer);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( DEBUG_COM,"FIFO:");
	sprintf(( char*)l_tempPacketLength1,"%d",g_FIFOsequence);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	USART_SendData_s( DEBUG_COM,"WriteEndPointer:");
	sprintf(( char*)l_tempPacketLength1,"%d",g_overWriteEndPointer);	
	USART_SendData_s( DEBUG_COM,(unsigned char*)l_tempPacketLength1);
	USART_SendData_s( DEBUG_COM,"\r\n");
	*/
	
	return replyFlag;
}

/*********************************************************************************************************
																			EEPROM_WritePage
								 Writes more than one byte to the FLASH with a single WRITE cycle
*********************************************************************************************************/
void EEPROM_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /*!< Enable the write access to the FLASH */
  EEPROM_WriteEnable();
  /*!< Select the FLASH: Chip Select low */
  EEPROM_CS_LOW();
  /*!< Send "Write to Memory " instruction */
  EEPROM_SendByte(EEPROM_CMD_WR_PAGE);
  /*!< Send WriteAddr high nibble address byte to write to */
  EEPROM_SendByte((WriteAddr & 0xFF0000) >> 16);
  /*!< Send WriteAddr medium nibble address byte to write to */
  EEPROM_SendByte((WriteAddr & 0xFF00) >> 8);
  /*!< Send WriteAddr low nibble address byte to write to */
  EEPROM_SendByte(WriteAddr & 0xFF);
  /*!< while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    EEPROM_SendByte(*pBuffer);
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  EEPROM_CS_HIGH();

  /*!< Wait the end of Flash writing */
  EEPROM_WaitForWriteEnd();
}

/*********************************************************************************************************
																			EEPROM_WaitForWriteEnd
												Polls the status of the BUSY flag in the EEPROM
*********************************************************************************************************/
void EEPROM_WaitForWriteEnd(void)
{
  uint8_t status = 0;
	long a = 40000;

  EEPROM_CS_LOW();															/*!< Select the FLASH: Chip Select low */

  EEPROM_SendByte(EEPROM_CMD_RD_STATUS_REG);		/*!< Send "Read Status Register" instruction */

  do																						/*!< Loop as long as the memory is busy with a write cycle */
  {
		a--;
		if(a == 0)
		{
			USART_SendData_s( DEBUG_COM,"EEPROM_WaitForWriteEnd\r\n");
			break;
		}
    /*!< Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    status = EEPROM_SendByte(EEPROM_DUMMY_BYTE);

  }
  while ((status & EEPROM_BUSY_FLAG) == SET); 	/* Write in progress */

  EEPROM_CS_HIGH();															/*!< Deselect the FLASH: Chip Select high */
}

/*********************************************************************************************************
																			EEPROM_SendByte
							Sends a byte through the SPI interface and return the byte received
*********************************************************************************************************/
uint8_t EEPROM_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/*********************************************************************************************************
																			EEPROM_WriteEnable
														Enables the write access to the FLASH
*********************************************************************************************************/
void EEPROM_WriteEnable(void)
{
  EEPROM_CS_LOW();													/*!< Select the FLASH: Chip Select low */

  EEPROM_SendByte(EEPROM_CMD_WR_ENABLE);		/*!< Send "Write Enable" instruction */

  EEPROM_CS_HIGH();													/*!< Deselect the FLASH: Chip Select high */
}

/*********************************************************************************************************
																			EEPROM_UnlockChip
			Write Status Register BP3 BP2 BP1 BP0 bits with '0' to disable all blocks protection
*********************************************************************************************************/
void EEPROM_UnlockChip(void)
{
	////// Write Status Register
  EEPROM_WriteEnable();												/*!< Send write enable instruction */
	
  EEPROM_CS_LOW();														/*!< Select the FLASH: Chip Select low */
 
  EEPROM_SendByte(EEPROM_CMD_WR_STATUS_REG);	/*!< Send "Write Status Register" instruction */
	
	EEPROM_SendByte(0x00);											/*!< Send Status Register value to be written*/

  EEPROM_CS_HIGH();														/*!< Deselect the FLASH: Chip Select high */

  EEPROM_WaitForWriteEnd();										/*!< Wait the end of Flash writing */
}

/*********************************************************************************************************
																			EEPROM_LockChip
			Write Status Register BP3 BP2 BP1 BP0 bits with '1' to enable all blocks protection
*********************************************************************************************************/
void EEPROM_LockChip(void)
{
	////// Write Status Register
  EEPROM_WriteEnable();												/*!< Send write enable instruction */
	
  EEPROM_CS_LOW();														/*!< Select the FLASH: Chip Select low */
  
  EEPROM_SendByte(EEPROM_CMD_WR_STATUS_REG);	/*!< Send "Write Status Register" instruction */
	
	EEPROM_SendByte(0x3C);											/*!< Send Status Register value to be written*/

  EEPROM_CS_HIGH();														/*!< Deselect the FLASH: Chip Select high */
	
  EEPROM_WaitForWriteEnd();										/*!< Wait the end of Flash writing */
}

/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
