#ifndef _LORA_H
#define _LORA_H_

#include "GLOBAL.h"

extern u8 LORA_Config(void);
extern u8 LORA_Tx(u8 *s);
static void LORA_Check(void);
extern int DATACmps(u8 *s1, u8 *s2, u8 s1Len, u8 s2Len);
extern int DATACmp(u8 *s1, u8 s2, u8 s1Len);
#endif
