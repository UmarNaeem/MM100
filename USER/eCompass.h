/*
 * BMA150.h
 *
 * Created: 5/26/2012 2:27:17 PM
 *  Author: Fakhar
 */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMA150_H
#define __BMA150_H

/* Define to declare and initialize global variables -------------------------------------*/
#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(x)
#else
# define _DECL
# define _INIT(x)  = x
#endif

#include <stdint.h>

#define BMA150_CS_HIGH()      	GPIO_SetBits(BMA_CS_PORT, BMA_CS_PIN)  
#define BMA150_CS_LOW()     		GPIO_ResetBits(BMA_CS_PORT, BMA_CS_PIN)

#define motionThresh 														6
#define BMA150_CHIP_ID         									0x02

/* Definitions for Range */
#define ACC_RANGE_2g 														0x00		// sets 2g
#define ACC_RANGE_4g 														0x01		// sets 4g
#define ACC_RANGE_8g 														0x02		// sets 8g

/* Definitions for Bandwidth */
#define ACC_BW_25Hz 														0x00		
#define ACC_BW_50Hz 														0x01		
#define ACC_BW_100Hz 														0x02		
#define ACC_BW_190Hz 														0x03		
#define ACC_BW_375Hz 														0x04		
#define ACC_BW_750Hz 														0x05		
#define ACC_BW_1500Hz 													0x06


/* BMA150 register address  */
#define BMA150_CHIP_ID_REG                      0x00
#define BMA150_VERSION_REG                      0x01

#define BMA150_X_AXIS_LSB_REG                   0x02
#define BMA150_X_AXIS_MSB_REG                   0x03
#define BMA150_Y_AXIS_LSB_REG                   0x04
#define BMA150_Y_AXIS_MSB_REG                   0x05
#define BMA150_Z_AXIS_LSB_REG                   0x06
#define BMA150_Z_AXIS_MSB_REG                   0x07


#define BMA150_EN_ACCESS_REG                    0x0A

#define BMA150_RANGE_BW_SEL_REG                 0x14

#define BMA150_ANYMOTION_INT_ENABLE_REG         0x0B

#define BMA150_SPI_ADV_REG                      0x15


#define BMA150_LOW_THRES_REG                    0x10
#define BMA150_LOW_DURN_REG               			0x11

struct _accMeter
{		
	
	unsigned char harshBreakFlag;
	unsigned char harshAcceFlag;
	unsigned char impactFlag;
	
	int16_t previousHeading_360;
	int16_t currentHeading_360;
	
	int16_t previousHeading_180;
	int16_t currentHeading_180;
	
	int16_t HeadingChange;
	uint8_t sharpTurnEvent;
	
	float G_valueX;
	float G_valueY;
	float G_valueZ;
	
	float SendValueX;
	float SendValueY;
	float SendValueZ;
	
	int16_t currentSpeed;
	int16_t previousSpeed;
	
	
	
};
_DECL  struct _accMeter accMeter;



uint8_t Accelerometer_Test(void);
uint8_t BMC_init(uint8_t,uint8_t);
void AnyMotionCriteriaSet(void);
void HighGCriteria(void);
float gFormat(uint8_t,uint8_t);
void AccelrationXYZ(void);
uint8_t BMC_ByteRead(uint8_t WriteAddr);
uint8_t BMC_ByteWrite(uint8_t WriteAddr, uint8_t data);
uint8_t BMA_SendByte(uint8_t byte);
uint8_t BMA_ReadByte(void);

uint8_t magnetometer_Heading(void);
int16_t magnetometer_Heading_Convert(void);

#endif /* __BMA150_H */
