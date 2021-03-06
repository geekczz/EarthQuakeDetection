#ifndef __RTC_H
#define __RTC_H	    

#include "Global.h"

//时间结构体
typedef struct 
{
	vu8 hour;
	vu8 min;
	vu8 sec;			
	//公历日月年周
	vu16 w_year;
	vu8  w_month;
	vu8  w_date;
	vu8  week;		 
}_calendar_obj;					 
extern _calendar_obj calendar;	//日历结构体

extern u8 const mon_table[12];	//月份日期数据表
void Disp_Time(u8 x,u8 y,u8 size);//在制定位置开始显示时间
void Disp_Week(u8 x,u8 y,u8 size,u8 lang);//在指定位置显示星期
extern void RTC_NVIC_Config(void);
u8 RTC_Init(void);        //初始化RTC,返回0,失败;1,成功;
extern void IWDG_Init(void);
extern void IWDG_Feed(void);

#endif


