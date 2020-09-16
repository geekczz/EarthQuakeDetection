#include "ADXL313.h"

ADXL313_DATA adxl313_data;

uint8_t ADXL313_Init(void)
{
	uint8_t Datatemp = 0;
	
	//测试ADXL313标准返回值，检测ADXL313存在
	Datatemp = ADXL313_GetRegData(REG_DEVID_0);
	if(Datatemp != 0xAD)return 0;
	Datatemp = ADXL313_GetRegData(REG_DEVID_1);
	if(Datatemp != 0x1D)return 0;
	Datatemp = ADXL313_GetRegData(REG_PARTID);
	if(Datatemp != 0xCB)return 0;

	//对ADXL313进行软复位
	ADXL313_WriteRagData(REG_SOFT_RESET,0x18);
	//失能I2C，Link，关闭auto_sleep，待机，关闭睡眠
	ADXL313_WriteRagData(REG_POWER_CTL,0x60);
	
	ADXL313_WriteRagData(REG_BW_RATE,0x0F);
	
	ADXL313_WriteRagData(REG_FIFO_CTL,0x00);
	
	ADXL313_WriteRagData(REG_DATA_FORMAT,0x02);
	
	//设置活动检测阈值
	ADXL313_WriteRagData(REG_THRESH_ACT,0x06);
	//设置静止检测阈值
	ADXL313_WriteRagData(REG_THRESH_INACT,0x03);
	//设置静止维持时间
	ADXL313_WriteRagData(REG_TIME_INACT,0x03);
	//设置测量轴及耦合(只打开Z轴）
//	ADXL313_WriteRagData(REG_ACT_INACT_CTL,0x99);
	//设置测量轴及耦合(3轴全开）
	ADXL313_WriteRagData(REG_ACT_INACT_CTL,0xff);
	
	//设置中断映射
	ADXL313_WriteRagData(REG_INT_MAP,0x08);
	//读中断寄存器值
	Datatemp = ADXL313_GetRegData(REG_INT_SOURCE);
	//设置中断使能
	ADXL313_WriteRagData(REG_INT_ENABLE,0x18);
	
	//失能I2C，Link，关闭auto_sleep，启动测量模式，关闭睡眠
	ADXL313_WriteRagData(REG_POWER_CTL,0x68);

	return 1;
}

void ADXL313_SetThresholdValue(float ThresholdValue)
{
//	uint8_t DataTemp = 0;
	
//	if(ThresholdValue<0)ThresholdValue = 0;
//	if(ThresholdValue>2.0)ThresholdValue = 2.0;
	
//	DataTemp = (uint8_t)(ThresholdValue / 0.015625f);
//	
//	ADXL313_WriteRagData(REG_THRESH_ACT,DataTemp);
	
	if(ThresholdValue<0)ThresholdValue = 0;
	if(ThresholdValue>2.0)ThresholdValue = 2.0;
	mainData.MixThresholdValue = ThresholdValue*1000;
}

uint8_t ADXL313_GetRegData(uint8_t Reg)
{
	uint8_t DataTemp = 0;
	SPI2_ADXL313_CS_L;
	DataTemp = ADXL313_spiSwap(Reg|0x80);
	DataTemp = ADXL313_spiSwap(0x00);
	SPI2_ADXL313_CS_H;
	return DataTemp;
}

void ADXL313_WriteRagData(uint8_t Reg,uint8_t Data)
{
	SPI2_ADXL313_CS_L;
	ADXL313_spiSwap(Reg);
	ADXL313_spiSwap(Data);
	SPI2_ADXL313_CS_H;
}

void ADXL313_Get3AxisData(void)
{
	int16_t DataTemp_X = 0;
	int16_t DataTemp_Y = 0;
	int16_t DataTemp_Z = 0;
	
	SPI2_ADXL313_CS_L;
	ADXL313_spiSwap(REG_DATA_X0|0xC0);
	adxl313_data.DATA[0] = ADXL313_spiSwap(0x00);
	adxl313_data.DATA[1] = ADXL313_spiSwap(0x00);
	adxl313_data.DATA[2] = ADXL313_spiSwap(0x00);
	adxl313_data.DATA[3] = ADXL313_spiSwap(0x00);
	adxl313_data.DATA[4] = ADXL313_spiSwap(0x00);
	adxl313_data.DATA[5] = ADXL313_spiSwap(0x00);
	SPI2_ADXL313_CS_H;
	
	DataTemp_X = (adxl313_data.DATA[1]<<8)|(adxl313_data.DATA[0]);
	DataTemp_Y = (adxl313_data.DATA[3]<<8)|(adxl313_data.DATA[2]);
	DataTemp_Z = (adxl313_data.DATA[5]<<8)|(adxl313_data.DATA[4]);
	
	adxl313_data.Acc_AxisX = DataTemp_X*125.0f/32.0f;
	adxl313_data.Acc_AxisY = DataTemp_Y*125.0f/32.0f;
	adxl313_data.Acc_AxisZ = DataTemp_Z*125.0f/32.0f;
}

uint8_t ADXL313_spiSwap(uint8_t data)
{
	return fun_spiSwap(SPI2,data);
}


