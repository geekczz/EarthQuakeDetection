#ifndef _FLASH_H_
#define _FLASH_H_

#include "GLOBAL.h"

#define FLASH_SIZE 64          //FLASH������С(K)

#if FLASH_SIZE<256
  #define SECTOR_SIZE           1024    //��С����ÿ������1k�ֽ�
#else 
  #define SECTOR_SIZE           2048    //������2k�ֽ�
#endif

#define PAGE61_ADDR  (0x08000000 + 61 * SECTOR_SIZE) 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)
#define PAGE62_ADDR  (0x08000000 + 62 * SECTOR_SIZE) 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)
#define PAGE63_ADDR  (0x08000000 + 63 * SECTOR_SIZE) 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

extern void FLASH_ReadMoreData(uint32_t startAddress,uint16_t *readData,uint16_t countToRead);
extern uint16_t FLASH_ReadHalfWord(uint32_t address);
extern uint32_t FLASH_ReadWord(uint32_t address);
extern void FLASH_WriteMoreData(uint32_t startAddress,uint16_t *writeData,uint16_t countToWrite);
extern void FLASH_WriteHalfWord(uint32_t startAddress,uint16_t writeData);

#endif
