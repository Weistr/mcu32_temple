#ifndef _KEY_H
#define _KEY_H
#include "sys.h"



#define	KEY_ON	 1		//��������
#define	KEY_OFF	 0		//�����ſ�

void KEY_Init(void);              //������ʼ������
uint8_t	KEY_Scan(void);   //����ɨ�躯��
#endif
