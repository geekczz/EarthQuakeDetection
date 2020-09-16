#ifndef __ADXL313_H
#define __ADXL313_H
#include "GLOBAL.h"

#define SPI2_ADXL313_CS_H GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define SPI2_ADXL313_CS_L GPIO_ResetBits(GPIOB,GPIO_Pin_12)

#define REG_DEVID_0 0x00
#define REG_DEVID_1 0x01
#define REG_PARTID 0x02
#define REG_REVID 0X03
#define REG_XID 0x04
#define REG_SOFT_RESET 0x18
#define REG_OFSX 0x1E
#define REG_OFSY 0x1F
#define REG_OFSZ 0x20
#define REG_THRESH_ACT 0X24
#define REG_THRESH_INACT 0X25
#define REG_TIME_INACT 0x26
#define REG_ACT_INACT_CTL 0X27
#define REG_BW_RATE 0X2C
#define REG_POWER_CTL 0X2D
#define REG_INT_ENABLE 0X2E
#define REG_INT_MAP 0X2F
#define REG_INT_SOURCE 0X30
#define REG_DATA_FORMAT 0X31
#define REG_DATA_X0 0X32
#define REG_DATA_X1 0X33
#define REG_DATA_Y0 0X34
#define REG_DATA_Y1 0X35
#define REG_DATA_Z0 0X36
#define REG_DATA_Z1 0X37
#define REG_FIFO_CTL 0X38
#define REG_FIFO_STATUS 0X39

uint8_t ADXL313_Init(void);
uint8_t ADXL313_spiSwap(uint8_t data);
uint8_t ADXL313_GetRegData(uint8_t Reg);
void ADXL313_WriteRagData(uint8_t Reg,uint8_t Data);
void ADXL313_Get3AxisData(void);
void ADXL313_SetThresholdValue(float ThresholdValue);

typedef struct ADXL313_DATA
{
	uint8_t DATA[6];
	float Acc_AxisX;
	float Acc_AxisY;
	float Acc_AxisZ;
}ADXL313_DATA;

extern ADXL313_DATA adxl313_data;

#endif
