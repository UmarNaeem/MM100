/**
  ******************************************************************************
  * @file    Global_Defines.c
  * @author  TeReSol
  * @version V1.0.0
  * @date    18-Oct-2012
  * @brief   This file contains all the functions prototypes for the global variables and functions.
  ******************************************************************************
*/

#include "Global_Defines.h"

/* Private defines -----------------------------------------------------------------------------------------------------------*/
static __IO uint32_t l_TimingDelay;

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length depending on clock speed
  * @retval None
  */
void Delay_viaCounter(uint8_t delay_type, uint32_t delay)
{
	uint32_t  counter=0;
	
	
	switch(delay_type)
	{
		case DELAY_MILLI_SEC:
			for(; delay != 0; delay--)
			{
				for(counter=0; counter != DELAY_VALUE_1ms; counter++);
			}
			break;
		case DELAY_MICRO_SEC:
			for(; delay != 0; delay--)
			{
				for(counter=0; counter != DELAY_VALUE_1us; counter++);
			}
			break;
		default:
			break;
	}
	//GPIO3_TRK_PORT->ODR ^= GPIO3_TRK_PIN;
}
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length depending on clock speed
  * @retval None
  */
void  Delay (__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_ms(__IO uint32_t nTime)
{ 
  G_TimingDelay = nTime;

  while(G_TimingDelay != 0)
	{}
}

/**
  * @brief  wash array
  * @param  unsigned char*: Pointer to array to be washed
  * @retval None
  */
void arrayInit2Zero(uint8_t* a_tempArray,uint16_t arraySize)
{
	unsigned int l_loop=0;

	for(l_loop=0;l_loop<arraySize;l_loop++)
	{
		a_tempArray[l_loop] = 0x00;
	}	
	
}
/**
  * @brief  Calculates length of string 
  * @param  const char *: Pointer to string or array
  * @param  size_t: maximum size of array to avoid memory crash
  * @retval None
  */
size_t strnlen (const char* s,uint16_t arraySize)
{
	size_t l_len = 0;
	while ((l_len <arraySize) && (*s))
  {
		s++;
    l_len++;
  }
	return l_len;
}
/**
  * @brief  Displays all the main system clocks of STM32F microcontroller on USART
  * @param  None
  * @retval None
  */
void displaySysClocks(void)
{
	unsigned char l_tempPacket[20];		
	USART_SendData_s(DEBUG_COM,"Main RCC clock values (in Hz):\n\r");	
	RCC_GetClocksFreq(&Clock_Frequency);
	USART_SendData_s(DEBUG_COM,"SYSCLK(Main)=");
	sprintf(( char*)l_tempPacket,"%d",Clock_Frequency.SYSCLK_Frequency);	
	USART_SendData_s(DEBUG_COM,l_tempPacket);
	USART_SendData_s(DEBUG_COM,"\r\nHCLK(AHB)=");
	sprintf(( char*)l_tempPacket,"%d",Clock_Frequency.HCLK_Frequency);	
	USART_SendData_s(DEBUG_COM,l_tempPacket);
	USART_SendData_s(DEBUG_COM,"\r\nPCLK1(APB1)=");
	sprintf(( char*)l_tempPacket,"%d",Clock_Frequency.PCLK1_Frequency);	
	USART_SendData_s(DEBUG_COM,l_tempPacket);
	USART_SendData_s(DEBUG_COM,"\r\nPCLK2(APB2)=");
	sprintf(( char*)l_tempPacket,"%d",Clock_Frequency.PCLK2_Frequency);	
	USART_SendData_s(DEBUG_COM,l_tempPacket);
	USART_SendData_s(DEBUG_COM,"\r\n");
}
/**
  * @brief  convert 2 ASCII bytes into one hex byte
  * @param  ch1: first byte
	* @param  ch2: second byte
  * @retval resultant byte after conversion
  */
uint8_t ascii_to_hex(char ch1,char ch2)
{
	uint8_t hex_value=0;
	
	if ((ch1 >= 0x41) && (ch1 <= 0x46 )) 
  { 
   hex_value= ch1 - 0x37;
   hex_value= hex_value<<4;
  }  
	else if(((ch1 >= 0x61) && (ch1 <= 0x66 )) )
	{
		hex_value |= (ch1 - 0x57);
	}
  else
  { 
   hex_value= ch1 - 0x30;
   hex_value= hex_value<<4; 
  }  
  
  if  ((ch2 >= 0x41) && (ch2<= 0x46)) 
  {
   hex_value |= (ch2 - 0x37);
  }
	else if(((ch2 >= 0x61) && (ch2 <= 0x66 )) )
	{
		hex_value |= (ch2 - 0x57);
	}
  else
  { 
   hex_value |= (ch2 - 0x30); 
  }
	return hex_value;
}
/**
  * @brief  convert one hex byte into 2 asci bytes
  * @param  hexin: hex byte
	* @param  final_asci: 2 bytes of asci, 
  * @retval resultant byte after conversion
  */
uint16_t hex_to_ascii(char hexin)
{
	uint16_t final_asci = 0;
	uint8_t asc1 = 0;
	uint8_t asc2 = 0;
	asc1 = (hexin >> 4);
	asc1 &= 0x0F;
	if(asc1 < 0x0A)
	{
		asc1 |= 0x30;
	}
	else
	{
		asc1 -= 0x01;
		asc1 &= 0x07;
		asc1 |= 0x40;
	}
	asc2 = (hexin & 0x0F);
	if(asc2 < 0x0A)
	{
		asc2 |= 0x30;
	}
	else
	{
		asc2 -= 0x01;
		asc2 &= 0x07;
		asc2 |= 0x40;
	}
	
	final_asci = asc1;
	final_asci = (final_asci << 8);
	final_asci |= asc2;
	return final_asci;
}
/**
  * @brief  Calculates checksum of array using XOR 
  * @param  mainbCMDArray: Pointer to string or array
  * @param  startIndex: start index of array to calculate checksum
  * @param  stopIndex: stop index of array to calculate checksum
  * @retval uint8_t: returns checksum of array
  */
uint8_t checksumCalc(uint8_t* mainbCMDArray, uint8_t startIndex, uint8_t stopIndex)
{
	uint8_t l_checksum=0,l_counter=0;
	for(l_counter=startIndex;l_counter<=stopIndex;l_counter++)
	{
		l_checksum=l_checksum^mainbCMDArray[l_counter];
	}
	return l_checksum;
}
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}
/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
