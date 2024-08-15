#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"


int main(void)
{ 
	  uint8_t t;
		uint8_t len;	
	  uint16_t times=0;
	
	  delay_init(120);                          //��ʼ����ʱ���� 
	  usart_init(115200);                       //��ʼ������
	  LED_Init();							                  //��ʼ��LED
    while(1)
		{
        if(USART_RX_STA&0x8000)    //��������һ������
				{					    
					len=USART_RX_STA&0x3fff; //�õ��˴ν��յ������ݳ���
					printf("\r\n�����͵���ϢΪ:\r\n");
					for(t=0;t<len;t++)
					{
						usart_data_transmit(USART0, USART_RX_BUF[t]);         //���ͽ��յ�������

						while(RESET == usart_flag_get(USART0, USART_FLAG_TC));//�ȴ����ͽ���
					}
					printf("\r\n\r\n");      //���뻻��
					USART_RX_STA=0;
				}else
				{
					times++;
					if(times%5000==0)
					{
						printf("\r\nGD32������ ����ʵ��\r\n\r\n");
					}
					if(times%200==0)printf("����������,�Իس�������\r\n");  
					if(times%30==0) LED1_TOGGLE();//��˸LED,��ʾϵͳ��������.
					delay_ms(10);   
				} 
		}
}




