#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "key.h"
#include "fwdgt.h"

int main(void)
{ 	
	  delay_init(120);                          //��ʼ����ʱ���� 
	  usart_init(115200);                       //��ʼ������
	  LED_Init();							                  //��ʼ��LED
	  KEY_Init();							                  //��ʼ������
	  delay_ms(100);                  	        //��ʱ100ms�ٳ�ʼ�����Ź�,LED0�ı仯"�ɼ�"
	  FWDGT_Init(FWDGT_PSC_DIV64,625);  	      //��Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s	
	  LED1(0);                                  //����LED0
    while(1)
		{
        if(KEY_Scan()==KEY_ON)  	        //���WK_UP���£�ι��
        {
            FWDGT_Feed();    			            //ι��
        }
        delay_ms(10); 
		}
}


