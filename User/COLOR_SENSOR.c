#include "COLOR_SENSOR.h"

#define COLOR_ADDR 0x39

 //COLOR IIC ��ʱ����
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

//����IIC��ʼ�ź�
void COLOR_IIC_Start(void)
{
	COLOR_SDA_OUT();     //sda�����
	COLOR_IIC_SDA=1;	  	  
	COLOR_IIC_SCL=1;
	COLOR_IIC_Dealy_Long();
 	COLOR_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	COLOR_IIC_Delay_Short();
	COLOR_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void COLOR_IIC_Stop(void)
{
	COLOR_SDA_OUT();//sda�����
	COLOR_IIC_SCL=0;
	COLOR_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	COLOR_IIC_Delay_Short();
	COLOR_IIC_SCL=1; 
	COLOR_IIC_Delay_Short();
	COLOR_IIC_SDA=1;//����I2C���߽����ź�
	COLOR_IIC_Delay_Short();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 COLOR_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	COLOR_SDA_IN();      //SDA����Ϊ����  
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
	COLOR_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void COLOR_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		COLOR_SDA_OUT(); 	    
    COLOR_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 COLOR_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	COLOR_SDA_IN();//SDA����Ϊ����
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
        COLOR_IIC_NAck();//����nACK
    else
        COLOR_IIC_Ack(); //����ACK   
    return receive;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 COLOR_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
  COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(COLOR_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		COLOR_IIC_Stop();		 
		return 1;		
	}
  COLOR_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
  COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		COLOR_IIC_Send_Byte(buf[i]);	//��������
		if(COLOR_IIC_Wait_Ack())		//�ȴ�ACK
		{
			COLOR_IIC_Stop();	 
			return 1;		 
		}		
	}    
  COLOR_IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 COLOR_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(COLOR_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		COLOR_IIC_Stop();		 
		return 1;		
	}
	COLOR_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ��
	COLOR_IIC_Start();
	COLOR_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
  COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=COLOR_IIC_Read_Byte(0);//������,����nACK 
		else *buf=COLOR_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
  COLOR_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 COLOR_Write_Byte(u8 reg,u8 data) 				 
{ 
  COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((COLOR_ADDR<<1)|0);//����������ַ+д����	
	if(COLOR_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		COLOR_IIC_Stop();		 
		return 1;		
	}
  COLOR_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
  COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	COLOR_IIC_Send_Byte(data);//��������
	if(COLOR_IIC_Wait_Ack())	//�ȴ�ACK
	{
		COLOR_IIC_Stop();	 
		return 1;		 
	}		 
    COLOR_IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 COLOR_Read_Byte(u8 reg)
{
	u8 res;
  COLOR_IIC_Start(); 
	COLOR_IIC_Send_Byte((COLOR_ADDR<<1)|0);//����������ַ+д����	
	COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	COLOR_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
	COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ��
	COLOR_IIC_Start();
	COLOR_IIC_Send_Byte((COLOR_ADDR<<1)|1);//����������ַ+������	
  COLOR_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=COLOR_IIC_Read_Byte(0);//��ȡ����,����nACK 
  COLOR_IIC_Stop();			//����һ��ֹͣ���� 
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
  
  //��ALS����
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
