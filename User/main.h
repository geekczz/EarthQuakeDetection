#ifndef __MAIN_H
#define __MAIN_H

#include "GLOBAL.h"

#define VOL_MIN  20  //备用电源最低电压，计算电量时使用 
#define VOL_MAX  24  //备用电源最高电压
#define VOL_Threshold 5 //主动电源电压阈值，小于多少V为断电

#define CHARGER_ON    PBout(3) = 0
#define CHARGER_OFF   PBout(3) = 1
#define RELAY_ON      PAout(11) = 0
#define RELAY_OFF     PAout(11) = 1

#define CLEAN_U2 memset(mainData.U2ReceiveData,0,U2_LENGTH)
#define U2_LENGTH   30 //U2数组长度

#define U1_TEST fun_usartSends(USART1, mainData.U2ReceiveData, U2_LENGTH)

typedef struct MAIN_DATA
{
	u16 PowerAdcData[2];
	float MainsSupplyVoltage;
	float BatteryVoltage;
	uint8_t U2ReceiveData[U2_LENGTH];
	uint8_t U1ReceiveData[8];
  
  u8 ServerTx[5]; //单片机发送数据包
  u8 ServerRx[4]; //服务器发送数据包  单片机接收
  u16 SerThresholdValue; //服务器发送阈值
  u8 AccFlag;   //加速度计中断标志
  u8 AccFirst;  //加速度计第一次报警判断
	u8 AccFirst_Sent;
  u8 ClockFlag; //定时标志
  u8 PowerFlag; //主动电源标志
  u8 SerResTimes;   //服务器应答次数
	u8 BuzzerFlag; //主程标志位
  
  float test;
  u8 LoraRx[U2_LENGTH];
	
	float MixAccData;
	u16 MixThresholdValue;
}MAIN_DATA;

extern MAIN_DATA mainData;

void BuzzerOn(void);
void BuzzerOff(void);
void BuzzerRing(u8 times, u16 interval);
void AdcData2Voltage(void);
void Start(void);
void SendServer(void);

#endif 
