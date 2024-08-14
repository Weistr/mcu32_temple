#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "wwdgt.h"

int main(void)
{ 	
	  delay_init(120);                            //��ʼ����ʱ���� 
	  usart_init(115200);                         //��ʼ������
	  LED_Init();							                    //��ʼ��LED
	  LED1(0);                                    //����LED1
	  delay_ms(300);                  	          //��ʱ300ms�ٳ�ʼ�����Ź�,LED1�ı仯"�ɼ�"
	  WWDGT_Init(0X7F, 0X5F, WWDGT_CFG_PSC_DIV8); //������ֵΪ7F������ֵΪ5F��Ԥ��ƵֵΪ8
    while(1)
		{
       LED1(1);                                 //�ر�LED1  
		}
}


