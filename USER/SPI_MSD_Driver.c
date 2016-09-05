/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               SPI_MSD_Driver.c
** Descriptions:            The SPI SD Card application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              Ya Dan
** Created date:            2011-1-4
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "Global_Defines.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define PRINT_INFO  1	

/* Private variables ---------------------------------------------------------*/
MSD_CARDINFO CardInfo;

FRESULT scan_files (char* path);
int SD_TotalSize(void);
/////////////////////  SD Section /////////////////////////////////////
/* Private variables ---------------------------------------------------------*/
FATFS fs;         /* Work area (file system object) for logical drive */
FIL fsrc;         /* file objects */   
FRESULT res;
UINT br;

char path[512]="0:";
uint8_t textFileBuffer[] = "Thank you for using STM32F407IG ARM MicroController! Author :Uzair Afzal! \r\n";   
char readFileBuffer[100];

uint8_t sd_wr_buff[512]={0x00};
uint8_t sd_rd_buff[512]={0x00};

int SD_TotalSize(void);
FRESULT scan_files (char* path);
/**
  * @brief  MSD Test
  * @param  None
  * @retval None
  */
void MSD_Test(void)
{
	uint8_t send=0;
	
	MSD_SPI_Configuration();

	if( _card_insert() == 0 )
  {
	  USART_SendData_s( DEBUG_COM,"\r\nSD card detected OK\r\n");
  }
  else
  {
		USART_SendData_s( DEBUG_COM,"Please connect SD card\r\n");
    while( _card_insert() != 0 );
    USART_SendData_s( DEBUG_COM,"SD card connection detected\r\n");
	  Delay(0xffffff);
  }
		
	f_mount(0,&fs);	
	
	res = f_open( &fsrc , "0:/Demo.TXT" , FA_WRITE);	
  if ( res == FR_OK )
  { 
    // Write buffer to file 
    res = f_write(&fsrc, textFileBuffer, sizeof(textFileBuffer), &br); 
		
		USART_SendData_s( DEBUG_COM,"Demo.TXT successfully created\r\n");
    
    // close file 
    f_close(&fsrc);      
  }
		
	else
		USART_SendData_s( DEBUG_COM,"Demo.TXT not successfully created\r\n");
	
	res = f_open( &fsrc , "0:/Demo.TXT" , FA_READ);
	if ( res == FR_OK )
	{
		f_read (&fsrc, &readFileBuffer, sizeof(readFileBuffer),&br);
			
		USART_SendData_s( DEBUG_COM,readFileBuffer);
			
		//	for(i=0;i<br-1;i++)
		//	{
		//		UART4_Put(readFileBuffer[i]);
		//	}
			
		// close file 
    f_close(&fsrc);
	}
		
  else if ( res == FR_EXIST )
  {
		USART_SendData_s( DEBUG_COM,"Acces denied due to prohibited access\r\n");
  	//USART_SendData_s( DEBUG_COM,"Demo.TXT File is already exist in the card\r\n");
  }

	else
		USART_SendData_s( DEBUG_COM,"Demo.TXT not successfully read\r\n");	
	///scan_files(path);
	///SD_TotalSize();
	
	sd_wr_buff[0]='a';
	sd_wr_buff[1]='b';
	sd_wr_buff[2]='c';
	sd_wr_buff[3]='d';
	sd_wr_buff[4]='e';
	sd_wr_buff[5]='f';
	sd_wr_buff[510]='g';
	sd_wr_buff[511]='h';

	USART_SendData_s( DEBUG_COM,"Write start\r\n");
	MSD_WriteSingleBlock(0x00,sd_wr_buff);
	USART_SendData_s( DEBUG_COM,"Write end\r\n");
	MSD_ReadSingleBlock(0x00,sd_rd_buff);
	
	for(send=0;send<512;send++)
	{
		USART_Put( DEBUG_COM,sd_rd_buff[send]);
	}
	USART_SendData_s( DEBUG_COM,"read end\r\n");
	
	sd_wr_buff[0]='0';
	sd_wr_buff[1]='1';
	sd_wr_buff[2]='2';
	sd_wr_buff[3]='3';
	sd_wr_buff[4]='4';
	sd_wr_buff[5]='5';
	sd_wr_buff[510]='6';
	sd_wr_buff[511]='7';

	USART_SendData_s( DEBUG_COM,"Write start\r\n");
	MSD_WriteSingleBlock(0x00,sd_wr_buff);
	USART_SendData_s( DEBUG_COM,"Write end\r\n");
	MSD_ReadSingleBlock(0x00,sd_rd_buff);
	
	for(send=0;send<512;send++)
	{
		USART_Put( DEBUG_COM,sd_rd_buff[send]);
	}
	USART_SendData_s( DEBUG_COM,"read end\r\n");
	
	
	sd_wr_buff[0]='a';
	sd_wr_buff[1]='b';
	sd_wr_buff[2]='c';
	sd_wr_buff[3]='d';
	sd_wr_buff[4]='e';
	sd_wr_buff[5]='f';
	sd_wr_buff[510]='g';
	sd_wr_buff[511]='h';

	USART_SendData_s( DEBUG_COM,"Write start\r\n");
	MSD_WriteSingleBlock(512,sd_wr_buff);
	USART_SendData_s( DEBUG_COM,"Write end\r\n");
	MSD_ReadSingleBlock(512,sd_rd_buff);
	
	for(send=0;send<512;send++)
	{
		USART_Put( DEBUG_COM,sd_rd_buff[send]);
	}
	USART_SendData_s( DEBUG_COM,"read end\r\n");
	
	
	
	
	
	USART_SendData_s( DEBUG_COM,"read1\r\n");
	MSD_ReadSingleBlock(0x00,sd_rd_buff);
	
	for(send=0;send<512;send++)
	{
		USART_Put( DEBUG_COM,sd_rd_buff[send]);
	}
	USART_SendData_s( DEBUG_COM,"read end\r\n");
	
	
	USART_SendData_s( DEBUG_COM,"read2\r\n");
	MSD_ReadSingleBlock(512,sd_rd_buff);
	
	for(send=0;send<512;send++)
	{
		USART_Put( DEBUG_COM,sd_rd_buff[send]);
	}
	USART_SendData_s( DEBUG_COM,"read end\r\n");
	
}

/*******************************************************************************
* Function Name  : scan_files
* Description    : 搜索文件目录下所有文件
* Input          : - path: 根目录
* Output         : None
* Return         : FRESULT
* Attention		 : 不支持长文件名
*******************************************************************************/
FRESULT scan_files (char* path)
{
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;
#if _USE_LFN
    static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0) break;
            if (fno.fname[0] == '.') continue;
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {
                //sprintf(&path[i], "/%s", fn);
							USART_SendData_s( DEBUG_COM, &path[i]);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {
                //printf("%s/%s \r\n", path, fn);
							USART_SendData_s( DEBUG_COM, path);
							USART_SendData_s( DEBUG_COM, fn);
            }
        }
    }

    return res;
}

/*******************************************************************************
* Function Name  : SD_TotalSize
* Description    : 文件空间占用情况
* Input          : None
* Output         : None
* Return         : 返回1成功 返回0失败
* Attention		 : None
*******************************************************************************/
int SD_TotalSize(void)
{
	FATFS *fs;
  DWORD fre_clust;        

  res = f_getfree("0:", &fre_clust, &fs);  /* 必须是根目录，选择磁盘0 */
  if ( res==FR_OK ) 
  {
	  /* Print free space in unit of MB (assuming 512 bytes/sector) */
			
		USART_SendData_s(DEBUG_COM,"\r\nTotal drive space(in MB)=");
		sprintf(( char*)G_sprintfBuffer,"%d",( (fs->n_fatent - 2) * fs->csize ) / 2 /1024);	
		USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
		USART_SendData_s(DEBUG_COM,"\r\n");
		
		USART_SendData_s(DEBUG_COM,"\r\nAvailable(in MB)=");
		sprintf(( char*)G_sprintfBuffer,"%d",(fre_clust * fs->csize) / 2 /1024 );	
		USART_SendData_s(DEBUG_COM,G_sprintfBuffer);
		USART_SendData_s(DEBUG_COM,"\r\n");
			
    /*printf("\r\n%d MB total drive space.\r\n"
           "%d MB available.\r\n",
           ( (fs->n_fatent - 2) * fs->csize ) / 2 /1024 , (fre_clust * fs->csize) / 2 /1024 );*/
		
	  return ENABLE;
	}
	else
	{		
	  return DISABLE;   
	}
}
/*******************************************************************************
* Function Name  : _spi_read_write
* Description    : None
* Input          : - data:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
__inline int _spi_read_write(uint8_t data)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, data);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI1 bus */
  return SPI_I2S_ReceiveData(SPI1);
}

/*******************************************************************************
* Function Name  : MSD_SPI_Configuration
* Description    : SD Card SPI Configuration
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void MSD_SPI_Configuration(void)
{	
	/*
	// Enable peripheral clock and GPIO clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);

	// Configure SPI1 Pins (MOSI,MISO,SCK) as alternate function 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	
  // SPI1_SCK -> PB3 , SPI1_MISO -> PB4 , SPI1_MOSI ->	PB5
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_CLOCK_SPEED;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
		
  // SD_CS -> PC11 			
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_CLOCK_SPEED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  // SD_CD -> PC7 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_CLOCK_SPEED;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  _card_disable(); 
	SPI_Cmd(SPI_SD, DISABLE);

  MSD_SPIHighSpeed(0);		

	SPI_Cmd(SPI_SD, ENABLE);*/
	
	RCC_AHB1PeriphClockCmd(SD_CS_CLK | SD_CD_CLK, ENABLE);
	/* SD_CS -> PC11 */			
  GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SD_CS_PORT, &GPIO_InitStructure);
	
  /* SD_CD -> PC7 */	
  GPIO_InitStructure.GPIO_Pin = SD_CD_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SD_CD_PORT, &GPIO_InitStructure);
	_card_disable(); 
	SPI_Cmd(SPI_SD, DISABLE);

  MSD_SPIHighSpeed(0);		

	SPI_Cmd(SPI_SD, ENABLE);
}

/*******************************************************************************
* Function Name  : MSD_SPIHighSpeed
* Description    : SD Card Speed Set
* Input          : - b_high: 1 = 21MHz, 0 = 328kHz
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void MSD_SPIHighSpeed(uint8_t b_high)
{
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  /* Speed select */
  if(b_high == 0)
  {
	 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  }
  else
  {
	 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  }

  SPI_Init(SPI_SD, &SPI_InitStructure);
}

/*******************************************************************************
* Function Name  : MSD_Init
* Description    : SD Card initializtion
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD_Init(void)
{
	uint8_t r1;	
	uint8_t buff[6] = {0};
	uint16_t retry; 

	/* Check , if no card insert */
  if( _card_insert() )
	{ 

		USART_SendData_s( DEBUG_COM,"There is no card detected! \r\n");

	  /* FATFS error flag */
    return -1;
	}
	/* Power on and delay some times */	
	for(retry=0; retry<0x100; retry++)
	{
		_card_power_on();
	}	

	/* Satrt send 74 clocks at least */
	for(retry=0; retry<10; retry++)
	{
		_spi_read_write(DUMMY_BYTE);
	}	
	/* Start send CMD0 till return 0x01 means in IDLE state */
	for(retry=0; retry<0xFFF; retry++)
	{
		r1 = _send_command(CMD0, 0, 0x95);
		if(r1 == 0x01)
		{
			retry = 0;
			break;
		}
	}
	/* Timeout return */
	if(retry == 0xFFF)
	{
		USART_SendData_s( DEBUG_COM,"Reset card into IDLE state failed!\r\n");
		return 1;
	}
	/* Get the card type, version */
	r1 = _send_command_hold(CMD8, 0x1AA, 0x87);
	/* r1=0x05 -> V1.0 */
	if(r1 == 0x05)
	{
	  CardInfo.CardType = CARDTYPE_SDV1;

	  /* End of CMD8, chip disable and dummy byte */
	  _card_disable();
	  _spi_read_write(DUMMY_BYTE);
		
	  /* SD1.0/MMC start initialize */
	  /* Send CMD55+ACMD41, No-response is a MMC card, otherwise is a SD1.0 card */
	  for(retry=0; retry<0xFFF; retry++)
	  {
	     r1 = _send_command(CMD55, 0, 0);			/* should be return 0x01 */
		 if(r1 != 0x01)
		 {
			///printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
			USART_SendData_s( DEBUG_COM,"Send CMD55 should return 0x01\r\n");
			return r1;
		 }

		 r1 = _send_command(ACMD41, 0, 0);			/* should be return 0x00 */
		 if(r1 == 0x00)
		 {
			retry = 0;
			break;
		 }
	  }

	  /* MMC card initialize start */
	  if(retry == 0xFFF)
	  {
			for(retry=0; retry<0xFFF; retry++)
			{
				r1 = _send_command(CMD1, 0, 0);		/* should be return 0x00 */
				if(r1 == 0x00)
				{
					retry = 0;
					break;
				}
			}

			/* Timeout return */
			if(retry == 0xFFF)
			{
				USART_SendData_s( DEBUG_COM,"Send CMD1 should return 0x00\r\n");
				///printf("Send CMD1 should return 0x00, response=0x%02x\r\n", r1);

				return 2;
			}	
			
			CardInfo.CardType = CARDTYPE_MMC;		
			USART_SendData_s( DEBUG_COM,"Card Type: MMC\r\n");
	  }		
		/* SD1.0 card detected, print information */
	  else
	  {
			USART_SendData_s( DEBUG_COM,"Card Type: SD V1\r\n");
	  }

	  /* Set spi speed high */
	  MSD_SPIHighSpeed(1);		
	  /* CRC disable */
	  r1 = _send_command(CMD59, 0, 0x01);
	  if(r1 != 0x00)
	  {
		  //printf("Send CMD59 should return 0x00, response=0x%02x\r\n", r1);
			USART_SendData_s( DEBUG_COM,"Send CMD59 should return 0x00\r\n");
		  return r1;		/* response error, return r1 */
	  }
		  
	  /* Set the block size */
	  r1 = _send_command(CMD16, MSD_BLOCKSIZE, 0xFF);
	  if(r1 != 0x00)
	  {
		  //printf("Send CMD16 should return 0x00, response=0x%02x\r\n", r1);
			USART_SendData_s( DEBUG_COM,"Send CMD16 should return 0x00\r\n");
		  return r1;		/* response error, return r1 */
	  }
	}	
	
  /* r1=0x01 -> V2.x, read OCR register, check version */
  else if(r1 == 0x01)
  {
		/* 4Bytes returned after CMD8 sent	*/
		buff[0] = _spi_read_write(DUMMY_BYTE);				/* should be 0x00 */
		buff[1] = _spi_read_write(DUMMY_BYTE);				/* should be 0x00 */
		buff[2] = _spi_read_write(DUMMY_BYTE);				/* should be 0x01 */
		buff[3] = _spi_read_write(DUMMY_BYTE);				/* should be 0xAA */
		
		/* End of CMD8, chip disable and dummy byte */ 
		_card_disable();
		_spi_read_write(DUMMY_BYTE);
		/* Check voltage range be 2.7-3.6V	*/
		if(buff[2]==0x01 && buff[3]==0xAA)
		{
			for(retry=0; retry<0xFFF; retry++)
			{
				r1 = _send_command(CMD55, 0, 0);			/* should be return 0x01 */
				if(r1!=0x01)
				{
					USART_SendData_s( DEBUG_COM,"Send CMD55 should return 0x01\r\n");
					//printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
					return r1;
				}				

				r1 = _send_command(ACMD41, 0x40000000, 0);	/* should be return 0x00 */
				if(r1 == 0x00)
				{
					retry = 0;
					break;
				}
			}
			/* Timeout return */
			if(retry == 0xFFF)
			{	
				//printf("Send ACMD41 should return 0x00, response=0x%02x\r\n", r1);
				USART_SendData_s( DEBUG_COM,"Send ACMD41 should return 0x00\r\n");
				return 3;
			}
			/* Read OCR by CMD58 */
	    r1 = _send_command_hold(CMD58, 0, 0);
	    if(r1!=0x00)
	    {
				USART_SendData_s( DEBUG_COM,"Send CMD58 should return 0x00\r\n");
				//printf("Send CMD58 should return 0x00, response=0x%02x\r\n", r1);

        return r1;		/* response error, return r1 */
	    }

	    buff[0] = _spi_read_write(DUMMY_BYTE);					
			buff[1] = _spi_read_write(DUMMY_BYTE);					
			buff[2] = _spi_read_write(DUMMY_BYTE);					
			buff[3] = _spi_read_write(DUMMY_BYTE);					

			/* End of CMD58, chip disable and dummy byte */
			_card_disable();
			_spi_read_write(DUMMY_BYTE);
	    /* OCR -> CCS(bit30)  1: SDV2HC	 0: SDV2 */
	    if(buff[0] & 0x40)
	    {
        CardInfo.CardType = CARDTYPE_SDV2HC;
				USART_SendData_s( DEBUG_COM,"Card Type: SD V2HC\r\n");
	    }
	    else
	    {
        CardInfo.CardType = CARDTYPE_SDV2;
				USART_SendData_s( DEBUG_COM,"Card Type: SD V2\r\n");
	    }

			/* Set spi speed high */
			////MSD_SPIHighSpeed(1);
		}	
   }
   return 0;
}

/*******************************************************************************
* Function Name  : MSD_GetCardInfo
* Description    : Get SD Card Information
* Input          : None
* Output         : None
* Return         : 0：NO_ERR; TRUE: Error
* Attention		 : None
*******************************************************************************/
int MSD_GetCardInfo(PMSD_CARDINFO cardinfo)
{
  uint8_t r1;
  uint8_t CSD_Tab[16];
  uint8_t CID_Tab[16];

  /* Send CMD9, Read CSD */
  r1 = _send_command(CMD9, 0, 0xFF);
  if(r1 != 0x00)
  {
    return r1;
  }

  if(_read_buffer(CSD_Tab, 16, RELEASE))
  {
	return 1;
  }

  /* Send CMD10, Read CID */
  r1 = _send_command(CMD10, 0, 0xFF);
  if(r1 != 0x00)
  {
    return r1;
  }

  if(_read_buffer(CID_Tab, 16, RELEASE))
  {
	return 2;
  }  

  /* Byte 0 */
  cardinfo->CSD.CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
  cardinfo->CSD.SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
  cardinfo->CSD.Reserved1 = CSD_Tab[0] & 0x03;
  /* Byte 1 */
  cardinfo->CSD.TAAC = CSD_Tab[1] ;
  /* Byte 2 */
  cardinfo->CSD.NSAC = CSD_Tab[2];
  /* Byte 3 */
  cardinfo->CSD.MaxBusClkFrec = CSD_Tab[3];
  /* Byte 4 */
  cardinfo->CSD.CardComdClasses = CSD_Tab[4] << 4;
  /* Byte 5 */
  cardinfo->CSD.CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
  cardinfo->CSD.RdBlockLen = CSD_Tab[5] & 0x0F;
  /* Byte 6 */
  cardinfo->CSD.PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
  cardinfo->CSD.WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
  cardinfo->CSD.RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
  cardinfo->CSD.DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
  cardinfo->CSD.Reserved2 = 0; /* Reserved */
  cardinfo->CSD.DeviceSize = (CSD_Tab[6] & 0x03) << 10;
  /* Byte 7 */
  cardinfo->CSD.DeviceSize |= (CSD_Tab[7]) << 2;
  /* Byte 8 */
  cardinfo->CSD.DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
  cardinfo->CSD.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
  cardinfo->CSD.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
  /* Byte 9 */
  cardinfo->CSD.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
  cardinfo->CSD.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
  cardinfo->CSD.DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
  /* Byte 10 */
  cardinfo->CSD.DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
  cardinfo->CSD.EraseGrSize = (CSD_Tab[10] & 0x7C) >> 2;
  cardinfo->CSD.EraseGrMul = (CSD_Tab[10] & 0x03) << 3;
  /* Byte 11 */
  cardinfo->CSD.EraseGrMul |= (CSD_Tab[11] & 0xE0) >> 5;
  cardinfo->CSD.WrProtectGrSize = (CSD_Tab[11] & 0x1F);
  /* Byte 12 */
  cardinfo->CSD.WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
  cardinfo->CSD.ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
  cardinfo->CSD.WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
  cardinfo->CSD.MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
  /* Byte 13 */
  cardinfo->CSD.MaxWrBlockLen |= (CSD_Tab[13] & 0xc0) >> 6;
  cardinfo->CSD.WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
  cardinfo->CSD.Reserved3 = 0;
  cardinfo->CSD.ContentProtectAppli = (CSD_Tab[13] & 0x01);
  /* Byte 14 */
  cardinfo->CSD.FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
  cardinfo->CSD.CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
  cardinfo->CSD.PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
  cardinfo->CSD.TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
  cardinfo->CSD.FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
  cardinfo->CSD.ECC = (CSD_Tab[14] & 0x03);
  /* Byte 15 */
  cardinfo->CSD.CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
  cardinfo->CSD.Reserved4 = 1;

  if(cardinfo->CardType == CARDTYPE_SDV2HC)
  {
	 /* Byte 7 */
	 cardinfo->CSD.DeviceSize = (u16)(CSD_Tab[8]) *256;
	 /* Byte 8 */
	 cardinfo->CSD.DeviceSize += CSD_Tab[9] ;
  }

  cardinfo->Capacity = cardinfo->CSD.DeviceSize * MSD_BLOCKSIZE * 1024;
  cardinfo->BlockSize = MSD_BLOCKSIZE;

  /* Byte 0 */
  cardinfo->CID.ManufacturerID = CID_Tab[0];
  /* Byte 1 */
  cardinfo->CID.OEM_AppliID = CID_Tab[1] << 8;
  /* Byte 2 */
  cardinfo->CID.OEM_AppliID |= CID_Tab[2];
  /* Byte 3 */
  cardinfo->CID.ProdName1 = CID_Tab[3] << 24;
  /* Byte 4 */
  cardinfo->CID.ProdName1 |= CID_Tab[4] << 16;
  /* Byte 5 */
  cardinfo->CID.ProdName1 |= CID_Tab[5] << 8;
  /* Byte 6 */
  cardinfo->CID.ProdName1 |= CID_Tab[6];
  /* Byte 7 */
  cardinfo->CID.ProdName2 = CID_Tab[7];
  /* Byte 8 */
  cardinfo->CID.ProdRev = CID_Tab[8];
  /* Byte 9 */
  cardinfo->CID.ProdSN = CID_Tab[9] << 24;
  /* Byte 10 */
  cardinfo->CID.ProdSN |= CID_Tab[10] << 16;
  /* Byte 11 */
  cardinfo->CID.ProdSN |= CID_Tab[11] << 8;
  /* Byte 12 */
  cardinfo->CID.ProdSN |= CID_Tab[12];
  /* Byte 13 */
  cardinfo->CID.Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  /* Byte 14 */
  cardinfo->CID.ManufactDate = (CID_Tab[13] & 0x0F) << 8;
  /* Byte 15 */
  cardinfo->CID.ManufactDate |= CID_Tab[14];
  /* Byte 16 */
  cardinfo->CID.CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
  cardinfo->CID.Reserved2 = 1;

  return 0;  
}

/*******************************************************************************
* Function Name  : _read_buffer
* Description    : None
* Input          : - *buff:
*				   - len:
*				   - release:
* Output         : None
* Return         : 0：NO_ERR; TRUE: Error
* Attention		 : None
*******************************************************************************/
int _read_buffer(uint8_t *buff, uint16_t len, uint8_t release)
{
  uint8_t r1;
  uint16_t retry;

  /* Card enable, Prepare to read	*/
  _card_enable();

  /* Wait start-token 0xFE */
  for(retry=0; retry<2000; retry++)
  {
	 r1 = _spi_read_write(DUMMY_BYTE);
	 if(r1 == 0xFE)
	 {
		 retry = 0;
		 break;
	 }
  }

  /* Timeout return	*/
  if(retry == 2000)
  {
	 _card_disable();
	 return 1;
  }

  /* Start reading */
  for(retry=0; retry<len; retry++)
  {
     *(buff+retry) = _spi_read_write(DUMMY_BYTE);
  }

  /* 2bytes dummy CRC */
  _spi_read_write(DUMMY_BYTE);
  _spi_read_write(DUMMY_BYTE);

  /* chip disable and dummy byte */ 
  if(release)
  {
	 _card_disable();
	 _spi_read_write(DUMMY_BYTE);
  }

  return 0;
}

/*******************************************************************************
* Function Name  : MSD_ReadSingleBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD_ReadSingleBlock(uint32_t sector, uint8_t *buffer)
{
  uint8_t r1;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  /* Send CMD17 : Read single block command */
  r1 = _send_command(CMD17, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }
	
  /* Start read and return the result */
  r1 = _read_buffer(buffer, MSD_BLOCKSIZE, RELEASE);

  /* Send stop data transmit command - CMD12 */
  _send_command(CMD12, 0, 0);

  return r1;
}

/*******************************************************************************
* Function Name  : MSD_ReadMultiBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector)
{
  uint8_t r1;
  uint32_t i;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }

  /* Send CMD18 : Read multi block command */
  r1 = _send_command(CMD18, sector, 0);
  if(r1 != 0x00)
  {
     return 1;
  }

  /* Start read	*/
  for(i=0; i<NbrOfSector; i++)
  {
     if(_read_buffer(buffer+i*MSD_BLOCKSIZE, MSD_BLOCKSIZE, HOLD))
     {
		 /* Send stop data transmit command - CMD12	*/
		 _send_command(CMD12, 0, 0);
		 /* chip disable and dummy byte */
		 _card_disable();
		 return 2;
     }
  }
	
  /* Send stop data transmit command - CMD12 */
  _send_command(CMD12, 0, 0);

  /* chip disable and dummy byte */
  _card_disable();
  _spi_read_write(DUMMY_BYTE);
	
  return 0;
}

/*******************************************************************************
* Function Name  : MSD_WriteSingleBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD_WriteSingleBlock(uint32_t sector, uc8 *buffer)
{
  uint8_t r1;
  uint16_t i;
  uint32_t retry;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	 sector = sector<<9;
  }
	
  /* Send CMD24 : Write single block command */
  r1 = _send_command(CMD24, sector, 0);
	
  if(r1 != 0x00)
  {
	 return 1;
  }

  /* Card enable, Prepare to write */
  _card_enable();
  _spi_read_write(DUMMY_BYTE);
  _spi_read_write(DUMMY_BYTE);
  _spi_read_write(DUMMY_BYTE);
  /* Start data write token: 0xFE */
  _spi_read_write(0xFE);
	
  /* Start single block write the data buffer */
  for(i=0; i<MSD_BLOCKSIZE; i++)
  {
    _spi_read_write(*buffer++);
  }

  /* 2Bytes dummy CRC */
  _spi_read_write(DUMMY_BYTE);
  _spi_read_write(DUMMY_BYTE);
	
  /* MSD card accept the data */
  r1 = _spi_read_write(DUMMY_BYTE);
  if((r1&0x1F) != 0x05)
  {
    _card_disable();
    return 2;
  }
	
  /* Wait all the data programm finished */
  retry = 0;
  while(_spi_read_write(DUMMY_BYTE) == 0x00)
  {	
	 /* Timeout return */
	 if(retry++ == 0x40000)
	 {
	    _card_disable();
	    return 3;
	 }
  }

  /* chip disable and dummy byte */ 
  _card_disable();
  _spi_read_write(DUMMY_BYTE);
	
  return 0;
}

/*******************************************************************************
* Function Name  : MSD_WriteMultiBlock
* Description    : None
* Input          : - sector:
*				   - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int MSD_WriteMultiBlock(uint32_t sector, uc8 *buffer, uint32_t NbrOfSector)
{
  uint8_t r1;
  uint16_t i;
  uint32_t n;
  uint32_t retry;

  /* if ver = SD2.0 HC, sector need <<9 */
  if(CardInfo.CardType != CARDTYPE_SDV2HC)
  {
	  sector = sector<<9;
  }

  /* Send command ACMD23 berfore multi write if is not a MMC card */
  if(CardInfo.CardType != CARDTYPE_MMC)
  {
	  _send_command(ACMD23, NbrOfSector, 0x00);
  }
	
  /* Send CMD25 : Write nulti block command	*/
  r1 = _send_command(CMD25, sector, 0);
	
  if(r1 != 0x00)
  {
	  return 1;
  }

  /* Card enable, Prepare to write */
  _card_enable();
  _spi_read_write(DUMMY_BYTE);
  _spi_read_write(DUMMY_BYTE);
  _spi_read_write(DUMMY_BYTE);

  for(n=0; n<NbrOfSector; n++)
  {	
	 /* Start multi block write token: 0xFC */
	 _spi_read_write(0xFC);

	 for(i=0; i<MSD_BLOCKSIZE; i++)
	 {
		_spi_read_write(*buffer++);
	 }	

	 /* 2Bytes dummy CRC */
	 _spi_read_write(DUMMY_BYTE);
	 _spi_read_write(DUMMY_BYTE);

	 /* MSD card accept the data */
	 r1 = _spi_read_write(DUMMY_BYTE);
	 if((r1&0x1F) != 0x05)
	 {
	    _card_disable();
	    return 2;
	 }

	 /* Wait all the data programm finished	*/
	 retry = 0;
	 while(_spi_read_write(DUMMY_BYTE) != 0xFF)
	 {	
		/* Timeout return */
		if(retry++ == 0x40000)
		{
		   _card_disable();
		   return 3;
		}
	 }
  }

  /* Send end of transmit token: 0xFD */
  r1 = _spi_read_write(0xFD);
  if(r1 == 0x00)
  {
	 return 4;
  }

  /* Wait all the data programm finished */
  retry = 0;
  while(_spi_read_write(DUMMY_BYTE) != 0xFF)
  {	
	 /* Timeout return */
	 if(retry++ == 0x40000)
	 {
	     _card_disable();
	     return 5;
	 }
  }

  /* chip disable and dummy byte */
  _card_disable();
  _spi_read_write(DUMMY_BYTE);

  return 0;
}


/*******************************************************************************
* Function Name  : _send_command
* Description    : None
* Input          : - cmd:
*				   - arg:
*                  - crc:
* Output         : None
* Return         : R1 value, response from card
* Attention		 : None
*******************************************************************************/
int _send_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1;
  uint8_t retry;

  /* Dummy byte and chip enable */
  _spi_read_write(DUMMY_BYTE);
  _card_enable();

  /* Command, argument and crc */
  _spi_read_write(cmd | 0x40);
  _spi_read_write(arg >> 24);
  _spi_read_write(arg >> 16);
  _spi_read_write(arg >> 8);
  _spi_read_write(arg);
  _spi_read_write(crc);
  
  /* Wait response, quit till timeout */
  for(retry=0; retry<200; retry++)
  {
	 r1 = _spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }

  /* Chip disable and dummy byte */ 
  _card_disable();
  _spi_read_write(DUMMY_BYTE);

  return r1;
}	

/*******************************************************************************
* Function Name  : _send_command_hold
* Description    : None
* Input          : - cmd:
*				   - arg:
*                  - crc:
* Output         : None
* Return         : R1 value, response from card
* Attention		 : None
*******************************************************************************/
int _send_command_hold(uint8_t cmd, uint32_t arg, uint8_t crc)
{
  uint8_t r1;
  uint8_t retry;

  /* Dummy byte and chip enable */
  _spi_read_write(DUMMY_BYTE);
  _card_enable();

  /* Command, argument and crc */
  _spi_read_write(cmd | 0x40);
  _spi_read_write(arg >> 24);
  _spi_read_write(arg >> 16);
  _spi_read_write(arg >> 8);
  _spi_read_write(arg);
  _spi_read_write(crc);
  
  /* Wait response, quit till timeout */
  for(retry=0; retry<200; retry++)
  {
	 r1 = _spi_read_write(DUMMY_BYTE);
	 if(r1 != 0xFF)
	 {
		 break;
	 }
  }

  return r1;
}

