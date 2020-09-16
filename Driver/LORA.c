#include "LORA.h"

//Lora��������
u8 LORA_Config(void)
{
  int temp,num=0;
  bsp_delayms(100); //ȷ��Lora׼��
  CLEAN_U2;
  fun_usartSends(USART2,"+++",3);
  while(1)
  {
    bsp_delayms(1000);
    U1_TEST;
    LORA_Check(); //ȥ�������е�"LoRa Start!"
    if((temp = DATACmp(mainData.U2ReceiveData, 'a', U2_LENGTH) )!= -1)      
      break;
    else if((temp = DATACmp(mainData.U2ReceiveData, '+', U2_LENGTH) ) != -1)
      break;
    if(num++ >= 3)     return 0;
    fun_usartSends(USART2,"+++",3);
  }
  CLEAN_U2;
  num=0;
  
  
  fun_usartSend(USART2,'a');
  while(1)
  {
    bsp_delayms(50);
    U1_TEST;
    LORA_Check(); //ȥ�������е�"LoRa Start!"
    if((temp = DATACmps(mainData.U2ReceiveData, "OK", U2_LENGTH,2) )!= -1)   break;
    else if((temp = DATACmp(mainData.U2ReceiveData, 'a', U2_LENGTH) ) != -1)
      break;
    if(num++ >= 3)  return 0;
    fun_usartSend(USART2,'a');
  }
  CLEAN_U2;
  num=0;
  
  for(num=0;num<3;num++)
  {
    bsp_delayms(500);
    if( LORA_Tx("AT+AID=00000055\r\n") == 1)
      break;
  }
  for(num=0;num<3;num++)
  {
    bsp_delayms(500);
    if( LORA_Tx("AT+ENTM\r\n")== 1)   //�˳�ָ��ģʽ
      break;
  }
  
  return 1;
}

//����ATָ��
u8 LORA_Tx(u8 *s)
{
  int temp;
  u8 *data;
  u8 num = 0;
  fun_usartSends(USART2,s,strlen(s));
  while(1)//�ȴ�����
  {
    bsp_delayms(500);
    U1_TEST;
    temp = DATACmps(mainData.U2ReceiveData, "AT", U2_LENGTH, 2);
    if(temp!= -1)   break;
    if(num++ >= 3)  return 0;
    fun_usartSends(USART2,s,strlen(s));
  }
  bsp_delayms(500);
  
  //�ж������Ƿ�ɹ�
  data = mainData.U2ReceiveData+temp;
  
  strncat(data,mainData.U2ReceiveData,temp);
  memcpy(mainData.LoraRx,data,U2_LENGTH);
  
  CLEAN_U2;
  temp = DATACmps(mainData.LoraRx, "OK", U2_LENGTH, 2);
  if(temp!= -1)   return 1; 
  else   return 0;
}

//����Lora�����ᷢ��Lora Start����Ҫɾ��"LoRa Start!"�Է�ֹ����a�ļ��
void LORA_Check(void)
{
  int i;
  i = DATACmps(mainData.U2ReceiveData, "LoRa", U2_LENGTH, 4);
  if(i!=-1)
  {
    memset(mainData.U2ReceiveData+i,0,11);
  }
}

//Ѱ��s2��s1�е�λ��
int DATACmps(u8 *s1, u8 *s2, u8 s1Len, u8 s2Len)
{
  u8 i,j;

  for(i=0;i<s1Len;i++)
  {
    for(j=0;j<s2Len;j++)
    {
      if(*(s1+i+j)==*(s2+j))
        continue;
      else
        break;
    }
    if(j == s2Len)
      return i;
  }
  return -1;
}

//Ѱ��s2�Ƿ���s1��
int DATACmp(u8 *s1, u8 s2, u8 s1Len)
{
  u8 i;
  for(i=0;i<s1Len;i++)
  {
    if(*(s1+i)==s2)
      return i;
  }
  return -1;
}
