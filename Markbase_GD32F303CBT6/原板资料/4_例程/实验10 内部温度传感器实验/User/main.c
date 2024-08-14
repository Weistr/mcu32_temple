#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "usmart.h"
#include "led.h"

#include "adc.h" 	


int main(void)
{ 	
    short temp;
	
	  delay_init(120);                 //��ʼ����ʱ���� 
	  usart_init(115200);              //��ʼ������
	  LED_Init();							         //��ʼ��LED
    adc_temperature_init();          //��ʼ��ADC�ڲ��¶ȴ�����

	

	
  	while(1) 
	  {		 
				temp=Get_Temprate();	//�õ��¶�ֵ 
			
				if(temp<0)
				{
					temp=-temp;
				}else printf("");	//�޷���
				
				printf("%d.",temp/100);
				printf("%d��C",temp%100);
				printf("\n");
				
				
				LED1_TOGGLE();    //LED0��˸,��ʾ�������� 
				delay_ms(500);	
	 } 
}


