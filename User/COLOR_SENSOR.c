#include "COLOR_SENSOR.h"

#define COLOR_ADDR 0x39

 //COLOR IIC 延时函数
void COLOR_IIC_Delay_Short(void)
{
	uint8_t counter_I = 0;
	for(counter_I = 0;counter_I<10;counter_I++)
	{
		__nop();
	}
}

void COLOR_IIC_Dealy_Long(void)
{
	uint8_t counter_I = 0;
	for(counter_I = 0;counter_I<20;counter_I++)
	{
		__nop();
	}
}

//产生IIC起始信号
void COLOR_IIC_Start(void)
{
	COLOR_SDA_OUT();     //sda线输出
	COLOR_IIC_SDA=1;	  	  
	COLOR_IIC_SCL=1;
	COLOR_IIC_Dealy_Long();
 	COLOR_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	COLOR_IIC_Delay_Short();
	COLOR_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void COLOR_IIC_Stop(void)
{
	COLOR_SDA_OUT();//sda线输出
	COLOR_IIC_SCL=0;
	COLOR_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	COLOR_IIC_Delay_Short();
	COLOR_IIC_SCL=1; 
	COLOR_IIC_Delay_Short();
	COLOR_IIC_SDA=1;//发送I2C总线结束信号
	COLOR_IIC_Delay_Short();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 COLOR_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	COLOR_SDA_IN();      //SDA设置为输入  
	COLOR_IIC_SDA=1;COLOR_IIC_Dealy_Long();	   
	COLOR_IIC_SCL=1;COLOR_IIC_Dealy_Long();	 
	while(COLOR_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			COLOR_IIC_Stop();
			return 1;
		}
	}
	COLOR_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void COLOR_IIC_Ack(void)
{
	COLOR_IIC_SCL=0;
	COLOR_SDA_OUT();
	COLOR_IIC_SDA=0;
	COLOR_IIC_Dealy_Long();
	COLOR_IIC_SCL=1;
	COLOR_IIC_Dealy_Long();
	COLOR_IIC_SCL=0;
}
//不产生ACK应答		    
void COLOR_IIC_NAck(void)
{
	COLOR_IIC_SCL=0;
	COLOR_SDA_OUT();
	COLOR_IIC_SDA=1;
	COLOR_IIC_Dealy_Long();
	COLOR_IIC_SCL=1;
	COLOR_IIC_Dealy_Long();
	COLOR_IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void COLOR_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		COLOR_SDA_OUT(); 	    
    COLOR_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        COLOR_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		    COLOR_IIC_SCL=1;
		    COLOR_IIC_Dealy_Long(); 
		    COLOR_IIC_SCL=0;	
		    COLOR_IIC_Dealy_Long();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 COLOR_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	COLOR_SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    COLOR_IIC_SCL=0; 
    COLOR_IIC_Dealy_Long();
		COLOR_IIC_SCL=1;
    receive<<=1;
    if(COLOR_READ_SDA)receive++;   
		COLOR_IIC_Dealy_Long(); 
    }					 
    if (!ack)
        COLOR_IIC_NAck();//发送nACK
    else
        COLOR_IIC_Ack(); //发送ACK   
    return receive;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 COLOR_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
  COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(COLOR_IIC_Wait_Ack())	//等待应答
	{
		COLOR_IIC_Stop();		 
		return 1;		
	}
  COLOR_IIC_Send_Byte(reg);	//写寄存器地址
  COLOR_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		COLOR_IIC_Send_Byte(buf[i]);	//发送数据
		if(COLOR_IIC_Wait_Ack())		//等待ACK
		{
			COLOR_IIC_Stop();	 
			return 1;		 
		}		
	}    
  COLOR_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 COLOR_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(COLOR_IIC_Wait_Ack())	//等待应答
	{
		COLOR_IIC_Stop();		 
		return 1;		
	}
	COLOR_IIC_Send_Byte(reg);	//写寄存器地址
	COLOR_IIC_Wait_Ack();		//等待应答
	COLOR_IIC_Start();
	COLOR_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
  COLOR_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=COLOR_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=COLOR_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
  COLOR_IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 COLOR_Write_Byte(u8 reg,u8 data) 				 
{ 
  COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((COLOR_ADDR<<1)|0);//发送器件地址+写命令	
	if(COLOR_IIC_Wait_Ack())	//等待应答
	{
		COLOR_IIC_Stop();		 
		return 1;		
	}
  COLOR_IIC_Send_Byte(reg);	//写寄存器地址
  COLOR_IIC_Wait_Ack();		//等待应答 
	COLOR_IIC_Send_Byte(data);//发送数据
	if(COLOR_IIC_Wait_Ack())	//等待ACK
	{
		COLOR_IIC_Stop();	 
		return 1;		 
	}		 
    COLOR_IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 COLOR_Read_Byte(u8 reg)
{
	u8 res;
  COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((COLOR_ADDR<<1)|0);//发送器件地址+写命令	
	COLOR_IIC_Wait_Ack();		//等待应答 
	COLOR_IIC_Send_Byte(reg);	//写寄存器地址
	COLOR_IIC_Wait_Ack();		//等待应答
	COLOR_IIC_Start();
	COLOR_IIC_Send_Byte((COLOR_ADDR<<1)|1);//发送器件地址+读命令	
  COLOR_IIC_Wait_Ack();		//等待应答 
	res=COLOR_IIC_Read_Byte(0);//读取数据,发送nACK 
  COLOR_IIC_Stop();			//产生一个停止条件 
	return res;		
}

uint8_t APDS_SETUP(void)
{
//	uint8_t Setup_Status = 0;
	uint8_t IIC_GetData = 0;
	
	/* Make sure we're actually connected */
	IIC_GetData = COLOR_Read_Byte(APDS9960_ID);
	if(IIC_GetData != 0xAB)
	{
		return 0;
	}
	
	/* Set default integration time and gain */
	setADCIntegrationTime(100);
	setADCGain(APDS9960_AGAIN_4X);
	
	// disable everything to start
	DispableAllFunction();
	ClearAllInterrupt();

  /* Note: by default, the device is in power down mode on bootup */
  COLOR_Write_Byte(APDS9960_ENABLE, 0x00);
  bsp_delayms(10);
  COLOR_Write_Byte(APDS9960_ENABLE, 0x01);
  bsp_delayms(10);
  
  //打开ALS功能
	COLOR_Write_Byte(APDS9960_ENABLE, 0x03);
	
	return 1;
}

void setADCIntegrationTime(uint16_t iTimeMS)
{
  float temp;

  // convert ms into 2.78ms increments
  temp = iTimeMS;
  temp /= 2.78;
  temp = 256 - temp;
  if (temp > 255) temp = 255;
  if (temp < 0)   temp = 0;
  
  /* Update the timing register */
  COLOR_Write_Byte(APDS9960_ATIME, (uint8_t)temp);
}

void setADCGain(apds9960AGain_t aGain)
{
  uint8_t temp = 0;
	
	//ALS and Color gain control
	uint8_t AGAIN = aGain;
	//proximity gain control
	uint8_t PGAIN = 2;
	//led drive strength
	uint8_t LDRIVE = 2;
	
	temp = (LDRIVE << 6) | (PGAIN << 2) | AGAIN;
	
  /* Update the timing register */
  COLOR_Write_Byte(APDS9960_CONTROL, temp);
}

void DispableAllFunction(void)
{
	uint8_t temp = 0x01;
	COLOR_Write_Byte(APDS9960_ENABLE, temp);
}

void ClearAllInterrupt(void)
{
	COLOR_Write_Byte(APDS9960_AICLEAR,0x00);
}

uint8_t ColorDataReady(void)
{
	uint8_t ReadyState = 0;
	uint8_t temp = 0;
	temp = COLOR_Read_Byte(APDS9960_STATUS);
	if((temp&0x01))ReadyState = 1;
	return ReadyState;
}

COLOR_CHANNEL_DATA CCD;

void ReadLastColorData(void)
{
	uint8_t temp,temp1 = 0;
	
	temp = COLOR_Read_Byte(APDS9960_CDATAL);
	temp1 = COLOR_Read_Byte(APDS9960_CDATAH);
	CCD.ClearData = ((temp1<<8)|temp);
	
	temp = COLOR_Read_Byte(APDS9960_RDATAL);
	temp1 = COLOR_Read_Byte(APDS9960_RDATAH);
	CCD.RedData = ((temp1<<8)|temp);
	
	temp = COLOR_Read_Byte(APDS9960_GDATAL);
	temp1 = COLOR_Read_Byte(APDS9960_GDATAH);
	CCD.GreenData = ((temp1<<8)|temp);
	
	temp = COLOR_Read_Byte(APDS9960_BDATAL);
	temp1 = COLOR_Read_Byte(APDS9960_BDATAH);
	CCD.BlueData = ((temp1<<8)|temp);
}
