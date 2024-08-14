#ifndef __FWDGT_H
#define __FWDGT_H
#include "sys.h"

void FWDGT_Init(uint8_t prer,uint16_t rlr);//初始化FWDGT，并使能FWDGT
void FWDGT_Feed(void);                     //喂独立看门狗 
#endif
