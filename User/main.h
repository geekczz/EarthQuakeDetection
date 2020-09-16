#ifndef __MAIN_H
#define __MAIN_H

#include "GLOBAL.h"

#define VOL_MIN  20  //���õ�Դ��͵�ѹ���������ʱʹ�� 
#define VOL_MAX  24  //���õ�Դ��ߵ�ѹ
#define VOL_Threshold 5 //������Դ��ѹ��ֵ��С�ڶ���VΪ�ϵ�

#define CHARGER_ON    PBout(3) = 0
#define CHARGER_OFF   PBout(3) = 1
#define RELAY_ON      PAout(11) = 0
#define RELAY_OFF     PAout(11) = 1

#define CLEAN_U2 memset(mainData.U2ReceiveData,0,U2_LENGTH)
#define U2_LENGTH   30 //U2���鳤��

#define U1_TEST fun_usartSends(USART1, mainData.U2ReceiveData, U2_LENGTH)

typedef struct MAIN_DATA
{
	u16 PowerAdcData[2];
	float MainsSupplyVoltage;
	float BatteryVoltage;
	uint8_t U2ReceiveData[U2_LENGTH];
	uint8_t U1ReceiveData[8];
  
  u8 ServerTx[5]; //��Ƭ���������ݰ�
  u8 ServerRx[4]; //�������������ݰ�  ��Ƭ������
  u16 SerThresholdValue; //������������ֵ
  u8 AccFlag;   //���ٶȼ��жϱ�־
  u8 AccFirst;  //���ٶȼƵ�һ�α����ж�
	u8 AccFirst_Sent;
  u8 ClockFlag; //��ʱ��־
  u8 PowerFlag; //������Դ��־
  u8 SerResTimes;   //������Ӧ�����
	u8 BuzzerFlag; //���̱�־λ
  
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
