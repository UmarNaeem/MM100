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

float previous_value = 0.0;
uint8_t count =0;

/*********************************************************************************************************
																Accelerometer_Test
										Test BMA150 by reading and writing registers
*********************************************************************************************************/	
uint8_t Accelerometer_Test(void)
{
	uint8_t Test_result=FALSE;
	uint8_t temp=0;

	USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\nBMA150 Test Start\n\r");
	
	temp=BMC_ByteRead(BMA150_CHIP_ID_REG);
	if((temp & 0X03) == BMA150_CHIP_ID)
	{
		USART_SendData_s( DEBUG_COM,(unsigned char*)"Accelerometer Test Passed\n\r");
		Test_result=TRUE;
	}
	else 
	{
		USART_SendData_s( DEBUG_COM,(unsigned char*)"AccelerometerTest Failed\n\r");
	}
	
	USART_SendData_s( DEBUG_COM,(unsigned char*)"BMC Test End\n\r\r\n");
	
	return Test_result;
}

/*********************************************************************************************************
																BMC_ByteWrite
											Write a byte to BMA150 using SPI
*********************************************************************************************************/
uint8_t BMC_ByteWrite(uint8_t WriteAddr, uint8_t data)
{
	uint8_t Status=TRUE;
	  
  BMA150_CS_LOW();										/*!< Select the FLASH: Chip Select low */
  
  BMA_SendByte((WriteAddr & 0x7F));		/*!< Send WriteAddr high nibble address byte to write to */
  
  BMA_SendByte(data);									/*!< Send the current byte */
  
  BMA150_CS_HIGH();										/*!< Deselect the FLASH: Chip Select high */
	
	return Status;
}

/*********************************************************************************************************
																BMC_ByteRead
										Read a byte from the BMA150 using SPI
*********************************************************************************************************/
uint8_t BMC_ByteRead(uint8_t WriteAddr)
{
	uint8_t byte;
  
  BMA150_CS_LOW();										/*!< Select the FLASH: Chip Select low */
  
  BMA_SendByte((WriteAddr | 0x80));		/*!< Send WriteAddr high nibble address byte to write to */
  
  byte=BMA_ReadByte();								/*!< Send the current byte */
  
  BMA150_CS_HIGH();										/*!< Deselect the FLASH: Chip Select high */
	
	return byte;
}

/*********************************************************************************************************
																		BMC_init
								Initialize the accelerometer with the required settings 
*********************************************************************************************************/
uint8_t BMC_init(uint8_t range, uint8_t BW)
{
	unsigned char temp;
	unsigned char temp1;	

	
	temp =BMC_ByteRead(BMA150_EN_ACCESS_REG) ;			/* Set Accelerometer Ee_w (enable access to registers) */
	temp |= 0x10;																		/* ee_w bit position is 4th */
	BMC_ByteWrite(BMA150_EN_ACCESS_REG,temp);
	
	temp =BMC_ByteRead(BMA150_RANGE_BW_SEL_REG) ;		/* Set Accelerometer Bandwidth */
	temp = (temp & 0xF8);  													/* Unchanging upper 5 bits */
	temp1 = temp | (BW & 0x07) ;										/* keeping first 3 bits */
	BMC_ByteWrite(BMA150_RANGE_BW_SEL_REG,temp1);
	
	temp =BMC_ByteRead(BMA150_RANGE_BW_SEL_REG) ;		/* Set Accelerometer Range */
	temp = (temp & 0xE7);
	temp1 = temp | ((range & 0x03)<<3) ;						/* left shifting by 3 */
	BMC_ByteWrite(BMA150_RANGE_BW_SEL_REG,temp1);
	
	return 1;
}

/*********************************************************************************************************
															AnyMotionCriteriaSet
										Set accelerometer in any detection mode 
*********************************************************************************************************/
void AnyMotionCriteriaSet(void)
{
	BMC_ByteWrite(BMA150_ANYMOTION_INT_ENABLE_REG,0x40);	/* Enabling adv_INT and Any_motion first,Writing */ 
																												/* 1 to any_motion bit and disabling others */
	
	BMC_ByteWrite(BMA150_SPI_ADV_REG,0xC0);								/* Setting SPI4 and Adv_INT to 1 */
	
	BMC_ByteWrite(BMA150_LOW_THRES_REG,motionThresh);			/* Setting Any_motion Threshold as 20 (0-255). */
	
	BMC_ByteWrite(BMA150_LOW_DURN_REG,0x80);							/* Setting Any_motion duration as 1 time */
	
	USART_SendData_s( DEBUG_COM,(unsigned char*)"AnyMotionCriteriaSet\r\n");
}

/*********************************************************************************************************
																					gFormat
							Reads g values from register and converts the value into floating point 
*********************************************************************************************************/
float gFormat(uint8_t lsb, uint8_t msb)
{
	float g_value;
	uint16_t temp;
	uint8_t tempLSB;
	
	tempLSB = (lsb>>6);		//LSB 2 bits
	temp=msb;
	temp = (temp<<2) | tempLSB;		//Shifting temp MSB by 2 towards left and adding it to LSB
	
	if(temp>0 && temp<=511 )// Positive Acceleration
	{
		g_value = (1.996/512)*temp;// 0g to +2g scale
		return g_value;
	}
	else if(temp>=512 && temp<=1024)// Negative Acceleration
	{
		g_value = (1.996/512)*temp - 4;// 0g to -2g scale
		return g_value;
	}
	
	return g_value;
}

/*********************************************************************************************************
																	AccelrationXYZ
								Reads the value of x, y, z acceleraton and measure 
								them for harsh break and acceleration detection 
*********************************************************************************************************/
void AccelrationXYZ(void)
{
	uint8_t tempLSB;  
	uint8_t tempMSB;
	float Net_Acceleration;

//	USART_SendData_s( DEBUG_COM,(unsigned char*)"Compass-in\r\n");

  //Acquiring X-axis g value; For MX70,+X-Axis is in forward direction
  tempLSB = BMC_ByteRead(BMA150_X_AXIS_LSB_REG) ; 
  tempMSB = BMC_ByteRead(BMA150_X_AXIS_MSB_REG) ; 
  accMeter.G_valueX = gFormat(tempLSB,tempMSB);
	
	//Acquiring Y-axis g value; 
  tempLSB = BMC_ByteRead(BMA150_Y_AXIS_LSB_REG) ; 
  tempMSB = BMC_ByteRead(BMA150_Y_AXIS_MSB_REG) ; 
	accMeter.G_valueY = gFormat(tempLSB,tempMSB);
		
	//Acquiring Z-axis g value;
  tempLSB = BMC_ByteRead(BMA150_Z_AXIS_LSB_REG) ; 
  tempMSB = BMC_ByteRead(BMA150_Z_AXIS_MSB_REG) ;
	accMeter.G_valueZ = gFormat(tempLSB,tempMSB);
	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"************************G_valueX:");
//	sprintf ((char *)G_sprintfBuffer, "%f",accMeter.G_valueX );
//	USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n");
//	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"************************G_valueY:");
//	sprintf ((char *)G_sprintfBuffer, "%f",accMeter.G_valueY );
//	USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n\n\n");
	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"************************G_valueZ:");
//	sprintf ((char *)G_sprintfBuffer, "%f",accMeter.G_valueZ );
//	USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"\r\n\n");
	
	Net_Acceleration=sqrt((accMeter.G_valueX*accMeter.G_valueX) + (accMeter.G_valueY*accMeter.G_valueY));	//Calculating sum of means;

	if (Net_Acceleration>=(1+config.AccelerationDetectTHR) && Net_Acceleration<=(1.5+config.AccelerationDetectTHR))//Acceleration threshold for Accelerate
  {
		accMeter.harshAcceFlag = TRUE;
		accMeter.SendValueX=accMeter.G_valueX ;
		accMeter.SendValueY=accMeter.G_valueY ;
		accMeter.SendValueZ=accMeter.G_valueZ ;
		USART_SendData_s( DEBUG_COM,(unsigned char *)"Acc\r\n");
  }
  else if (Net_Acceleration>(1.5+config.AccelerationDetectTHR) && Net_Acceleration<(3+config.AccelerationDetectTHR))  //Acceleration threshold for Harsh Brake
  {
		accMeter.harshBreakFlag = TRUE;
		accMeter.SendValueX=accMeter.G_valueX ;
		accMeter.SendValueY=accMeter.G_valueY ;
		accMeter.SendValueZ=accMeter.G_valueZ ;
		USART_SendData_s( DEBUG_COM,(unsigned char *)"H_B\r\n");

  }
	else if (Net_Acceleration>(3+config.ImpactDetectTHR))//Acceleration threshold for Impact
  {
		accMeter.impactFlag = TRUE;
		accMeter.SendValueX=accMeter.G_valueX ;
		accMeter.SendValueY=accMeter.G_valueY ;
		accMeter.SendValueZ=accMeter.G_valueZ ;
		USART_SendData_s( DEBUG_COM,(unsigned char *)"Imp\r\n");
  }
	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"Compass-out\r\n");
}

/*********************************************************************************************************
															BMA_ReadByte
												Reads a byte using SPI 
*********************************************************************************************************/
uint8_t BMA_ReadByte(void)
{
  return (BMA_SendByte(0x00));
}

/*********************************************************************************************************
															BMA_SendByte
											  Writes a byte using SPI 
*********************************************************************************************************/
uint8_t BMA_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

/*********************************************************************************************************
													magnetometer_Heading_Convert
					magnetometer compass heading conversion relative to 180 degree
*********************************************************************************************************/
int16_t magnetometer_Heading_Convert(void)
{
	int16_t l_limit;
	int16_t l_convertValue;
	
	l_limit=accMeter.previousHeading_360+180;
	if(accMeter.currentHeading_360<=180)
	{
		if((accMeter.currentHeading_360+360)<=l_limit)
		{
			l_convertValue=accMeter.currentHeading_360+360+180-accMeter.previousHeading_360;
		}
		else
		{
			l_convertValue=accMeter.currentHeading_360+180-accMeter.previousHeading_360;
		}
	}
	else
	{
		l_convertValue=accMeter.currentHeading_360+180-accMeter.previousHeading_360;
	}
	if(l_convertValue>360)
	{
		l_convertValue -=360;
	}
	return l_convertValue;
}

/*********************************************************************************************************
													magnetometer_Heading
					     magnetometer compass heading/Sharp Turn logic
*********************************************************************************************************/
uint8_t magnetometer_Heading(void)
{
	accMeter.previousHeading_180=180;
	
	accMeter.currentHeading_360=atoi((const char*)GPS_LOG.gpsHeading);
	
	accMeter.currentHeading_180=magnetometer_Heading_Convert();
	
	accMeter.currentSpeed = atoi((const char*)GPS_LOG.speed);
	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"currentHeading_360=");
//	sprintf(( char*)G_sprintfBuffer,"%d",accMeter.currentHeading_360);	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)G_sprintfBuffer);
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"\n\r");
//	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"previousHeading_360=");
//	sprintf(( char*)G_sprintfBuffer,"%d",accMeter.previousHeading_360);	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)G_sprintfBuffer);
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"\n\r");
//	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"currentHeading_180=");
//	sprintf(( char*)G_sprintfBuffer,"%d",accMeter.currentHeading_180);	
//	USART_SendData_s( DEBUG_COM,(unsigned char*)G_sprintfBuffer);
//	USART_SendData_s( DEBUG_COM,(unsigned char*)"\n\r");
	

//	if(AccBreakFlag==TRUE)
//	{
//		AccBreakFlag=FALSE;
//		return 1;
//	}
	
	
	if((accMeter.currentHeading_180-accMeter.previousHeading_180)<-config.SharpTurnThresh && 
		  accMeter.sharpTurnEvent==FALSE && accMeter.currentHeading_360!=0 && 
	    accMeter.currentSpeed > accMeter.previousSpeed && accMeter.currentSpeed > 40)
	{
		accMeter.sharpTurnEvent=1;//Right Turn
		
		accMeter.HeadingChange=accMeter.currentHeading_180-accMeter.previousHeading_180;
		
		USART_SendData_s( DEBUG_COM,(unsigned char *)"R-S-HeadingChange=");
		sprintf(( char*)G_sprintfBuffer,"%d",accMeter.HeadingChange);	
		USART_SendData_s( DEBUG_COM,(unsigned char*)G_sprintfBuffer);
		USART_SendData_s( DEBUG_COM,(unsigned char *)"\n\r");
		
		
		
	}
	else if((accMeter.currentHeading_180-accMeter.previousHeading_180)>config.SharpTurnThresh && 
		       accMeter.sharpTurnEvent==FALSE && accMeter.currentHeading_360!=0 && 
					 accMeter.currentSpeed > accMeter.previousSpeed && accMeter.currentSpeed > 40)
	{
		accMeter.sharpTurnEvent=2;//Left Turn
		
		accMeter.HeadingChange=accMeter.currentHeading_180-accMeter.previousHeading_180;
		
		USART_SendData_s( DEBUG_COM,(unsigned char *)"L-S-HeadingChange=");
		sprintf(( char*)G_sprintfBuffer,"%d",accMeter.HeadingChange);	
		USART_SendData_s( DEBUG_COM,(unsigned char*)G_sprintfBuffer);
		USART_SendData_s( DEBUG_COM,(unsigned char *)"\n\r");
	}
	
	accMeter.previousHeading_360=accMeter.currentHeading_360;
	accMeter.previousSpeed = accMeter.currentSpeed+10;
	
	return 0;
}
/*********************************************************************************************************
																					END FILE
*********************************************************************************************************/
