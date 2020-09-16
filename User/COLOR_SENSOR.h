#ifndef __COLOR_SENSOR_H
#define __COLOR_SENSOR_H

#include "GLOBAL.h"

//COLOR SENSOR PIN
//COLOR LED PA0
//COLOR SDA PA1
//COLOR INT PA2
//COLOR SCL PA3

#define COLOR_SDA_IN()  {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=4<<4;}
#define COLOR_SDA_OUT() {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=3<<4;}

#define COLOR_IIC_SCL    PAout(3) 		//SCL
#define COLOR_IIC_SDA    PAout(1) 		//SDA	 
#define COLOR_READ_SDA   PAin(1) 		//ÊäÈëSDA 

void COLOR_IIC_Delay_Short(void);
void COLOR_IIC_Dealy_Long(void);
void COLOR_IIC_Start(void);
void COLOR_IIC_Stop(void);
u8 COLOR_IIC_Wait_Ack(void);
void COLOR_IIC_Ack(void);
void COLOR_IIC_NAck(void);
void COLOR_IIC_Send_Byte(u8 txd);
u8 COLOR_IIC_Read_Byte(unsigned char ack);
u8 COLOR_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 COLOR_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 COLOR_Write_Byte(u8 reg,u8 data);
u8 COLOR_Read_Byte(u8 reg);


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define APDS9960_ADDRESS                (0x39)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/

enum 
{
		APDS9960_RAM = 0x00,
		APDS9960_ENABLE = 0x80,
		APDS9960_ATIME = 0x81,
		APDS9960_WTIME = 0x83,
		APDS9960_AILTIL = 0x84,
		APDS9960_AILTH = 0x85,
		APDS9960_AIHTL = 0x86,
		APDS9960_AIHTH = 0x87,
		APDS9960_PILT = 0x89,
		APDS9960_PIHT = 0x8B,
		APDS9960_PERS = 0x8C,
		APDS9960_CONFIG1 = 0x8D,
		APDS9960_PPULSE = 0x8E,
		APDS9960_CONTROL = 0x8F,
		APDS9960_CONFIG2 = 0x90,
		APDS9960_ID = 0x92,
		APDS9960_STATUS = 0x93,
		APDS9960_CDATAL = 0x94,
		APDS9960_CDATAH = 0x95,
		APDS9960_RDATAL = 0x96,
		APDS9960_RDATAH = 0x97,
		APDS9960_GDATAL = 0x98,
		APDS9960_GDATAH = 0x99,
		APDS9960_BDATAL = 0x9A,
		APDS9960_BDATAH = 0x9B,
		APDS9960_PDATA = 0x9C,
		APDS9960_POFFSET_UR = 0x9D,
		APDS9960_POFFSET_DL = 0x9E,
		APDS9960_CONFIG3 = 0x9F,
		APDS9960_GPENTH = 0xA0,
		APDS9960_GEXTH = 0xA1,
		APDS9960_GCONF1 = 0xA2,
		APDS9960_GCONF2 = 0xA3,
		APDS9960_GOFFSET_U = 0xA4,
		APDS9960_GOFFSET_D = 0xA5,
		APDS9960_GOFFSET_L = 0xA7,
		APDS9960_GOFFSET_R = 0xA9,
		APDS9960_GPULSE = 0xA6,
		APDS9960_GCONF3 = 0xAA,
		APDS9960_GCONF4 = 0xAB,
		APDS9960_GFLVL = 0xAE,
		APDS9960_GSTATUS = 0xAF,
		APDS9960_IFORCE = 0xE4,
		APDS9960_PICLEAR = 0xE5,
		APDS9960_CICLEAR = 0xE6,
		APDS9960_AICLEAR = 0xE7,
		APDS9960_GFIFO_U = 0xFC,
		APDS9960_GFIFO_D = 0xFD,
		APDS9960_GFIFO_L = 0xFE,
		APDS9960_GFIFO_R = 0xFF,
};


/*=========================================================================*/

typedef enum
{
  APDS9960_AGAIN_1X                = 0x00,   /**<  No gain  */
  APDS9960_AGAIN_4X                = 0x01,   /**<  2x gain  */
  APDS9960_AGAIN_16X               = 0x02,   /**<  16x gain */
  APDS9960_AGAIN_64X               = 0x03    /**<  64x gain */
}
apds9960AGain_t;


typedef enum
{
  APDS9960_PGAIN_1X                = 0x00,   /**<  1x gain  */
  APDS9960_PGAIN_2X                = 0x04,   /**<  2x gain  */
  APDS9960_PGAIN_4X                = 0x08,   /**<  4x gain  */
  APDS9960_PGAIN_8X                = 0x0C    /**<  8x gain  */
}
apds9960PGain_t;


typedef enum
{
  APDS9960_PPULSELEN_4US                = 0x00,   /**<  4uS   */
  APDS9960_PPULSELEN_8US                = 0x40,   /**<  8uS   */
  APDS9960_PPULSELEN_16US               = 0x80,   /**<  16uS  */
  APDS9960_PPULSELEN_32US               = 0xC0    /**<  32uS  */
}
apds9960PPulseLen_t;


typedef enum
{
  APDS9960_LEDDRIVE_100MA              = 0x00,   /**<  100mA   */
  APDS9960_LEDDRIVE_50MA               = 0x40,   /**<  50mA    */
  APDS9960_LEDDRIVE_25MA               = 0x80,   /**<  25mA    */
  APDS9960_LEDDRIVE_12MA               = 0xC0    /**<  12.5mA  */
}
apds9960LedDrive_t;


typedef enum
{
  APDS9960_LEDBOOST_100PCNT               = 0x00,   /**<  100%   */
  APDS9960_LEDBOOST_150PCNT               = 0x10,   /**<  150%   */
  APDS9960_LEDBOOST_200PCNT               = 0x20,   /**<  200%   */
  APDS9960_LEDBOOST_300PCNT               = 0x30    /**<  300%   */
}
apds9960LedBoost_t;

enum
{
	APDS9960_DIMENSIONS_ALL		= 0x00,
	APDS9960_DIMENSIONS_UP_DOWM  = 0x01,
	APGS9960_DIMENSIONS_LEFT_RIGHT = 0x02,	
};

enum
{
	APDS9960_GFIFO_1	= 0x00,
	APDS9960_GFIFO_4	= 0x01,
	APDS9960_GFIFO_8	= 0x02,
	APDS9960_GFIFO_16	= 0x03,
};

enum
{
	APDS9960_GGAIN_1	= 0x00,
	APDS9960_GGAIN_2	= 0x01,
	APDS9960_GGAIN_4	= 0x02,
	APDS9960_GGAIN_8	= 0x03,
};

enum
{
	APDS9960_GPULSE_4US		= 0x00,
	APDS9960_GPULSE_8US		= 0x01,
	APDS9960_GPULSE_16US	= 0x02,
	APDS9960_GPULSE_32US	= 0x03,
};

typedef struct COLOR_CHANNEL_DATA
{
	u16 ClearData;
	u16 RedData;
	u16 GreenData;
	u16 BlueData;
}COLOR_CHANNEL_DATA;

extern COLOR_CHANNEL_DATA CCD;

#define APDS9960_TIME_MULT 2.78 //millisec

#define APDS9960_UP 0x01
#define APDS9960_DOWN 0x02
#define APDS9960_LEFT 0x03
#define APDS9960_RIGHT 0x04

uint8_t APDS_SETUP(void);

void setADCIntegrationTime(uint16_t iTimeMS);
void setADCGain(apds9960AGain_t aGain);
void EnableGesture(uint8_t en);
void DispableAllFunction(void);
void ClearAllInterrupt(void);

uint8_t ColorDataReady(void);
void ReadLastColorData(void);

#endif 