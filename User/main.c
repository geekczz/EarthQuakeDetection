#include "main.h"
#include "math.h"

MAIN_DATA mainData;
int32_t Counter_i = 0;
u32 TimeStampA = 0;
u32 TimeStampB = 0;
u32 TimeStampDiff = 0;

u8 serverRes[] = {0xAA,0xF5,0xF5,0xA6};  //应答包
u8 logIn[] = {0xA1, 0x00, 0x00, 0xA6}; //上线包
u8 txData[] = {0xA5,0x64,0x01,0x01,0xA6};
/*************************
*txData[1]:电量百分比
*txData[2]:电磁铁开关状态   0(报警，不正常)
*txData[3]:主动电源开关状态 0(关闭，不正常)
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
  IWDG_Init(); //在线调试时需要注释该句
  
  //初始化参数
  mainData.SerResTimes = 0; //服务器应答次数
  mainData.ClockFlag = 0; //定时中断标志
  mainData.PowerFlag = 0;
  mainData.BatteryVoltage = VOL_MAX; 
  
  //自检程序
  Start();
  
  mainData.AccFlag = 0;   //加速度计中断标志
  mainData.AccFirst = 0;  //加速度计第一次报警判断
	mainData.AccFirst_Sent = 0;
	
	PBout(5) = 1;
	PBout(6) = 1; 
  
	while(1)
	{
/************************  电量检测  ******************************/
		AdcData2Voltage();
    if(mainData.MainsSupplyVoltage < VOL_Threshold) //主动电源开关断开
    {
      if(mainData.PowerFlag == 0) //保证断电只发送一次
      {
        txData[3] = 0x00;
        SendServer();//发送数据包
				BuzzerRing(3,200);
        mainData.PowerFlag = 1;
      }
    }
    else
    {
      if(mainData.PowerFlag == 1)
      {
        txData[3] = 0x01;
        SendServer();//发送数据包
        mainData.PowerFlag = 0; //复位电源标志
      }
    }
    IWDG_Feed();
    
/************************  加速度计检测  ******************************/    
    if(mainData.AccFlag == 1)
    {
      if(mainData.AccFirst_Sent == 0 && mainData.AccFirst == 1) //保证加速度计只报警一次
      {
				mainData.AccFirst_Sent = 1;
				//发送数据包
        txData[2] = 0x00;
        SendServer();
      }
//      txData[2] = 0x01;
      mainData.AccFlag = 0; 
    }
    IWDG_Feed();
   
/************************  定时检测  ******************************/        
    if(mainData.ClockFlag == 1)
    {
      SendServer();
      mainData.ClockFlag = 0;
//      mainData.AccFlag = 0; //复位加速度计标志
    }

/************************  阈值设置检测  ******************************/    
    temp = DATACmp(mainData.U2ReceiveData, 0xA5, U2_LENGTH);
    if(temp!= -1) //收到服务器数据
    {
      bsp_delayms(100);//等待接收
      fun_usartSends(USART2,serverRes, 4); //发送应答
      
      //解析阈值
      U1_TEST;
      mainData.SerThresholdValue = mainData.U2ReceiveData[temp+1]<<8 | mainData.U2ReceiveData[temp+2] ;
      if(mainData.SerThresholdValue >=  0x64 && mainData.SerThresholdValue <=  0x12C) //确保阈值在正确范围
      {
        //将阈值写入FLash
        FLASH_WriteHalfWord(PAGE63_ADDR,mainData.SerThresholdValue);  
        //将阈值写入寄存器
        ADXL313_SetThresholdValue(mainData.SerThresholdValue/1000.0);
      }
      CLEAN_U2;    //????
      IWDG_Feed();
    }
    
		/************************  串口定时上传数据  ******************************/  
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

//自检程序，开机运行一次
void Start(void)
{
  u16 read=0;
  int temp;
/************************  加速度计阈值处理  ******************************/
  read = FLASH_ReadHalfWord(PAGE63_ADDR); //读取阈值
  if(read == 0xFFFF)  //第一次运行
  {
    FLASH_WriteHalfWord(PAGE63_ADDR,0x007D);  //将默认阈值写入FLash,0.125g
    //将默认阈值写入寄存器
    ADXL313_SetThresholdValue(0x007D/1000.0);
  }
  else{
    //将阈值写入寄存器
    ADXL313_SetThresholdValue(read/1000.0);
    mainData.test = read/1000.0;
  }
  IWDG_Feed();
  
  LORA_Config(); //LORA配置程序
  
/************************  服务器通讯测试    ******************************/
  fun_usartSends(USART2,logIn, 4);  //发送上线包
  while(1)
  {
    bsp_delayms(10000);
		IWDG_Feed();
    U1_TEST; //显示接收数据
    if( (temp = DATACmps(mainData.U2ReceiveData,serverRes, U2_LENGTH, 4))!= -1 ) //服务器有应答则退出
      break;
    else
    {
      if(mainData.SerResTimes++ >=6) //通讯未建立
      {
        BuzzerRing(5,300);
        mainData.SerResTimes = 0;
        return;
      }
      fun_usartSends(USART2,logIn, 4);
    }
  }
  
  CLEAN_U2;    
  
  //自检完成提示
  BuzzerRing(2,500);
  bsp_delayms(100);
  SendServer();
}

//向服务器发送给数据包
void SendServer(void)
{
  //计算电量
  if(mainData.BatteryVoltage <= VOL_MIN)
    txData[1] = 0x00;
  else if(mainData.BatteryVoltage >= VOL_MAX)
    txData[1] = 0x64;
  else
    txData[1] = (mainData.BatteryVoltage - VOL_MIN) / (VOL_MAX-VOL_MIN) *100; //计算电量百分比
  
  fun_usartSends(USART2,txData, 5);  //发送数据包
  
  while(1)  //等待应答
  {
		bsp_delayms(3000);
    if( DATACmps(mainData.U2ReceiveData,serverRes, U2_LENGTH, 4)!= -1 ) //服务器有应答则退出
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
