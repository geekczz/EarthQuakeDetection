#include "main.h"
#include "math.h"

MAIN_DATA mainData;
int32_t Counter_i = 0;
u32 TimeStampA = 0;
u32 TimeStampB = 0;
u32 TimeStampDiff = 0;

u8 serverRes[] = {0xAA,0xF5,0xF5,0xA6};  //Ӧ���
u8 logIn[] = {0xA1, 0x00, 0x00, 0xA6}; //���߰�
u8 txData[] = {0xA5,0x64,0x01,0x01,0xA6};
/*************************
*txData[1]:�����ٷֱ�
*txData[2]:���������״̬   0(������������)
*txData[3]:������Դ����״̬ 0(�رգ�������)
*************************/
int main(void)
{
  u32 num=0;
  int temp;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Config(SystemCoreClock/1000);
	BSP_Init();
	
	BuzzerRing(2,200);
	
	PBout(5) = 0;
	PBout(6) = 0;
	
	bsp_delayms(1000);
	RTC_Init();
  IWDG_Init(); //���ߵ���ʱ��Ҫע�͸þ�
  
  //��ʼ������
  mainData.SerResTimes = 0; //������Ӧ�����
  mainData.ClockFlag = 0; //��ʱ�жϱ�־
  mainData.PowerFlag = 0;
  mainData.BatteryVoltage = VOL_MAX; 
  
  //�Լ����
  Start();
  
  mainData.AccFlag = 0;   //���ٶȼ��жϱ�־
  mainData.AccFirst = 0;  //���ٶȼƵ�һ�α����ж�
	mainData.AccFirst_Sent = 0;
	
	PBout(5) = 1;
	PBout(6) = 1; 
  
	while(1)
	{
/************************  �������  ******************************/
		AdcData2Voltage();
    if(mainData.MainsSupplyVoltage < VOL_Threshold) //������Դ���ضϿ�
    {
      if(mainData.PowerFlag == 0) //��֤�ϵ�ֻ����һ��
      {
        txData[3] = 0x00;
        SendServer();//�������ݰ�
				BuzzerRing(3,200);
        mainData.PowerFlag = 1;
      }
    }
    else
    {
      if(mainData.PowerFlag == 1)
      {
        txData[3] = 0x01;
        SendServer();//�������ݰ�
        mainData.PowerFlag = 0; //��λ��Դ��־
      }
    }
    IWDG_Feed();
    
/************************  ���ٶȼƼ��  ******************************/    
    if(mainData.AccFlag == 1)
    {
      if(mainData.AccFirst_Sent == 0 && mainData.AccFirst == 1) //��֤���ٶȼ�ֻ����һ��
      {
				mainData.AccFirst_Sent = 1;
				//�������ݰ�
        txData[2] = 0x00;
        SendServer();
      }
//      txData[2] = 0x01;
      mainData.AccFlag = 0; 
    }
    IWDG_Feed();
   
/************************  ��ʱ���  ******************************/        
    if(mainData.ClockFlag == 1)
    {
      SendServer();
      mainData.ClockFlag = 0;
//      mainData.AccFlag = 0; //��λ���ٶȼƱ�־
    }

/************************  ��ֵ���ü��  ******************************/    
    temp = DATACmp(mainData.U2ReceiveData, 0xA5, U2_LENGTH);
    if(temp!= -1) //�յ�����������
    {
      bsp_delayms(100);//�ȴ�����
      fun_usartSends(USART2,serverRes, 4); //����Ӧ��
      
      //������ֵ
      U1_TEST;
      mainData.SerThresholdValue = mainData.U2ReceiveData[temp+1]<<8 | mainData.U2ReceiveData[temp+2] ;
      if(mainData.SerThresholdValue >=  0x64 && mainData.SerThresholdValue <=  0x12C) //ȷ����ֵ����ȷ��Χ
      {
        //����ֵд��FLash
        FLASH_WriteHalfWord(PAGE63_ADDR,mainData.SerThresholdValue);  
        //����ֵд��Ĵ���
        ADXL313_SetThresholdValue(mainData.SerThresholdValue/1000.0);
      }
      CLEAN_U2;    //????
      IWDG_Feed();
    }
    
		/************************  ���ڶ�ʱ�ϴ�����  ******************************/  
		TimeStampA = bspdata.systime;
//		fun_printf(USART1,"%.3f %.3f %.3f\n",adxl313_data.Acc_AxisX,adxl313_data.Acc_AxisY,mainData.MixAccData);
		fun_printf(USART1,"%.3f\n",mainData.MixAccData);
		TimeStampB = bspdata.systime;
		TimeStampDiff = TimeStampB - TimeStampA;
	}
}

void BuzzerOn(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
}

void BuzzerOff(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}

void BuzzerRing(u8 times, u16 interval)
{
  u16 i=0;
	mainData.BuzzerFlag = 0;
  BuzzerOff();
  for(i=0;i<times;i++)
  {
//    BuzzerOn();
    bsp_delayms(interval);
//    BuzzerOff();
    bsp_delayms(interval);
  }
	bsp_delayms(100);
	mainData.BuzzerFlag = 1;
}

void AdcData2Voltage(void)
{
	mainData.MainsSupplyVoltage = mainData.PowerAdcData[0]/4096.0f*3.3*4.0;
	mainData.BatteryVoltage = mainData.PowerAdcData[1]/4096.0f*3.3*7.65;
}

//�Լ���򣬿�������һ��
void Start(void)
{
  u16 read=0;
  int temp;
/************************  ���ٶȼ���ֵ����  ******************************/
  read = FLASH_ReadHalfWord(PAGE63_ADDR); //��ȡ��ֵ
  if(read == 0xFFFF)  //��һ������
  {
    FLASH_WriteHalfWord(PAGE63_ADDR,0x007D);  //��Ĭ����ֵд��FLash,0.125g
    //��Ĭ����ֵд��Ĵ���
    ADXL313_SetThresholdValue(0x007D/1000.0);
  }
  else{
    //����ֵд��Ĵ���
    ADXL313_SetThresholdValue(read/1000.0);
    mainData.test = read/1000.0;
  }
  IWDG_Feed();
  
  LORA_Config(); //LORA���ó���
  
/************************  ������ͨѶ����    ******************************/
  fun_usartSends(USART2,logIn, 4);  //�������߰�
  while(1)
  {
    bsp_delayms(10000);
		IWDG_Feed();
    U1_TEST; //��ʾ��������
    if( (temp = DATACmps(mainData.U2ReceiveData,serverRes, U2_LENGTH, 4))!= -1 ) //��������Ӧ�����˳�
      break;
    else
    {
      if(mainData.SerResTimes++ >=6) //ͨѶδ����
      {
        BuzzerRing(5,300);
        mainData.SerResTimes = 0;
        return;
      }
      fun_usartSends(USART2,logIn, 4);
    }
  }
  
  CLEAN_U2;    
  
  //�Լ������ʾ
  BuzzerRing(2,500);
  bsp_delayms(100);
  SendServer();
}

//����������͸����ݰ�
void SendServer(void)
{
  //�������
  if(mainData.BatteryVoltage <= VOL_MIN)
    txData[1] = 0x00;
  else if(mainData.BatteryVoltage >= VOL_MAX)
    txData[1] = 0x64;
  else
    txData[1] = (mainData.BatteryVoltage - VOL_MIN) / (VOL_MAX-VOL_MIN) *100; //��������ٷֱ�
  
  fun_usartSends(USART2,txData, 5);  //�������ݰ�
  
  while(1)  //�ȴ�Ӧ��
  {
		bsp_delayms(3000);
    if( DATACmps(mainData.U2ReceiveData,serverRes, U2_LENGTH, 4)!= -1 ) //��������Ӧ�����˳�
      break;
    else
    {
      if(mainData.SerResTimes++ >=3) 
      {
        mainData.SerResTimes = 0;
        return;
      }
      fun_usartSends(USART2,txData,5);
    }
  }
  
  CLEAN_U2;
  IWDG_Feed();
}
