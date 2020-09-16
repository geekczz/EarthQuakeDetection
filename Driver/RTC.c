#include "RTC.h"

//RTC初始化
u8 RTC_Init(void)
{
  {	 			
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);	//使能PWR和BKP外设时钟   
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);	//使能后备寄存器访问 
    BKP_DeInit();	//复位备份区域 	
    
    //use LSE 
    RCC_LSEConfig(RCC_LSE_ON);	//设置外部低速晶振(LSE),使用外设低速晶振
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)	//检查指定的RCC标志位设置与否,等待低速晶振就绪
      {
//      temp++;
//      delay_ms(10);
      }
//    if(temp>=250)return 1;//初始化时钟失败,晶振有问题	    
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		//设置RTC时钟(RTCCLK),选择LSE作为RTC时钟    
  
    RCC_RTCCLKCmd(ENABLE);	//使能RTC时钟  
    RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成
    RTC_WaitForSynchro();		//等待RTC寄存器同步  
    RTC_ITConfig(RTC_IT_ALR, ENABLE);		//使能RTC闹钟中断
    RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成
    RTC_EnterConfigMode();/// 允许配置	
    RTC_SetPrescaler(32767); //设置RTC预分频的值
    RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成
    RTC_SetCounter(0);
    RTC_WaitForLastTask();
    
    RTC_SetAlarm(86399);  //86399(1d)
    RTC_WaitForLastTask();
    RTC_ExitConfigMode(); //退出配置模式  
//    BKP_WriteBackupRegister(BKP_DR1, 0X5050);	//向指定的后备寄存器中写入用户程序数据
  }
  
	RTC_NVIC_Config();//RCT中断分组设置		    				     

	return 0; //ok

}		 				    
//RTC时钟中断
//每秒触发一次  
//extern u16 tcnt; 
void RTC_IRQHandler(void)
{		 
//	if (RTC_GetITStatus(RTC_IT_SEC) != RESET)//秒钟中断
//	{							
////    LED = !LED;

//		RTC_Get();//更新时间   
// 	}
	if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)//闹钟中断
	{
		RTC_ClearITPendingBit(RTC_IT_ALR);		//清闹钟中断	  	 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	//使能PWR和BKP外设时钟   
    PWR_BackupAccessCmd(ENABLE);	//使能后备寄存器访问  
		RTC_SetCounter(0);
    RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成  	    
    mainData.ClockFlag = 1;
  } 				  								 
	RTC_ClearITPendingBit(RTC_IT_SEC|RTC_IT_OW);		//清闹钟中断
	RTC_WaitForLastTask();	  	    						 	   	 
}
extern void RTC_NVIC_Config(void)
{	
  NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;		//RTC全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//先占优先级1位,从优先级3位
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	//先占优先级0位,从优先级4位
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//使能该通道中断
	NVIC_Init(&NVIC_InitStructure);		//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

//看门狗初始化
void IWDG_Init(void)
{	//40KHz
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //解除寄存器保护
  IWDG_SetPrescaler(IWDG_Prescaler_256); // 写入初始化分频值
  IWDG_SetReload(0xF43); //写入自动装载值 //25s   256*3907/40000=25s
  IWDG_ReloadCounter(); //开启寄存器保护
  IWDG_Enable(); //启动看门狗
}

//喂狗
void IWDG_Feed(void)
{
  IWDG->KR=0XAAAA;      
//  IWDG_ReloadCounter();
}
