#ifndef __RTC_H
#define __RTC_H	    

#include "Global.h"

//ʱ��ṹ��
typedef struct 
{
	vu8 hour;
	vu8 min;
	vu8 sec;			
	//������������
	vu16 w_year;
	vu8  w_month;
	vu8  w_date;
	vu8  week;		 
}_calendar_obj;					 
extern _calendar_obj calendar;	//�����ṹ��

extern u8 const mon_table[12];	//�·��������ݱ�
void Disp_Time(u8 x,u8 y,u8 size);//���ƶ�λ�ÿ�ʼ��ʾʱ��
void Disp_Week(u8 x,u8 y,u8 size,u8 lang);//��ָ��λ����ʾ����
extern void RTC_NVIC_Config(void);
u8 RTC_Init(void);        //��ʼ��RTC,����0,ʧ��;1,�ɹ�;
extern void IWDG_Init(void);
extern void IWDG_Feed(void);

#endif


