#ifndef _KEY_H
#define _KEY_H
#include "sys.h"



#define	KEY_ON	 1		//按键按下
#define	KEY_OFF	 0		//按键放开

void KEY_Init(void);              //按键初始化函数
uint8_t	KEY_Scan(void);   //按键扫描函数
#endif
