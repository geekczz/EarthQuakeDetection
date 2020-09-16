#include "RTC.h"

//RTC��ʼ��
u8 RTC_Init(void)
{
  {	 			
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);	//ʹ��PWR��BKP����ʱ��   
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ������� 
    BKP_DeInit();	//��λ�������� 	
    
    //use LSE 
    RCC_LSEConfig(RCC_LSE_ON);	//�����ⲿ���پ���(LSE),ʹ��������پ���
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)	//���ָ����RCC��־λ�������,�ȴ����پ������
      {
//      temp++;
//      delay_ms(10);
      }
//    if(temp>=250)return 1;//��ʼ��ʱ��ʧ��,����������	    
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		//����RTCʱ��(RTCCLK),ѡ��LSE��ΪRTCʱ��    
  
    RCC_RTCCLKCmd(ENABLE);	//ʹ��RTCʱ��  
    RTC_WaitForLastTask();	//�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_WaitForSynchro();		//�ȴ�RTC�Ĵ���ͬ��  
    RTC_ITConfig(RTC_IT_ALR, ENABLE);		//ʹ��RTC�����ж�
    RTC_WaitForLastTask();	//�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_EnterConfigMode();/// ��������	
    RTC_SetPrescaler(32767); //����RTCԤ��Ƶ��ֵ
    RTC_WaitForLastTask();	//�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_SetCounter(0);
    RTC_WaitForLastTask();
    
    RTC_SetAlarm(86399);  //86399(1d)
    RTC_WaitForLastTask();
    RTC_ExitConfigMode(); //�˳�����ģʽ  
//    BKP_WriteBackupRegister(BKP_DR1, 0X5050);	//��ָ���ĺ󱸼Ĵ�����д���û���������
  }
  
	RTC_NVIC_Config();//RCT�жϷ�������		    				     

	return 0; //ok

}		 				    
//RTCʱ���ж�
//ÿ�봥��һ��  
//extern u16 tcnt; 
void RTC_IRQHandler(void)
{		 
//	if (RTC_GetITStatus(RTC_IT_SEC) != RESET)//�����ж�
//	{							
////    LED = !LED;

//		RTC_Get();//����ʱ��   
// 	}
	if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)//�����ж�
	{
		RTC_ClearITPendingBit(RTC_IT_ALR);		//�������ж�	  	 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	//ʹ��PWR��BKP����ʱ��   
    PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ�������  
		RTC_SetCounter(0);
    RTC_WaitForLastTask();	//�ȴ����һ�ζ�RTC�Ĵ�����д�������  	    
    mainData.ClockFlag = 1;
  } 				  								 
	RTC_ClearITPendingBit(RTC_IT_SEC|RTC_IT_OW);		//�������ж�
	RTC_WaitForLastTask();	  	    						 	   	 
}
extern void RTC_NVIC_Config(void)
{	
  NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;		//RTCȫ���ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�1λ,�����ȼ�3λ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	//��ռ���ȼ�0λ,�����ȼ�4λ
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//ʹ�ܸ�ͨ���ж�
	NVIC_Init(&NVIC_InitStructure);		//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

//���Ź���ʼ��
void IWDG_Init(void)
{	//40KHz
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //����Ĵ�������
  IWDG_SetPrescaler(IWDG_Prescaler_256); // д���ʼ����Ƶֵ
  IWDG_SetReload(0xF43); //д���Զ�װ��ֵ //25s   256*3907/40000=25s
  IWDG_ReloadCounter(); //�����Ĵ�������
  IWDG_Enable(); //�������Ź�
}

//ι��
void IWDG_Feed(void)
{
  IWDG->KR=0XAAAA;      
//  IWDG_ReloadCounter();
}
