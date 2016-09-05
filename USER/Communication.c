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
/* Private Defines ------------------------------------------------------------------*/





#ifdef  MB_COMM_ENABLED




/**
  * @brief  Verifies checksum, extracts received Main board command  
  * @param  mainbCMDArray: Pointer to string or array
  * @retval uint8_t:
  */
uint8_t mainbPktExtractor (uint8_t* mainbCMDArray)
{
	
	return 0;	
}



/**
  * @brief  send response/data to Main Board 
  * @param  None
  * @retval None
  */
void mainbSendPktFormatter(uint8_t Control_type, uint8_t CMD_ID)
{

}

#endif



/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
