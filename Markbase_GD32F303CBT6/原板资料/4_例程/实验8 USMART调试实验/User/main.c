#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "usmart.h"


//LED״̬���ú��� 
void led_set(uint8_t sta)
{
    LED1(sta);
}

//�����������ò��Ժ��� 
void test_fun(void(*ledset)(uint8_t), uint8_t sta)
{
    ledset(sta);
}

int main(void)
{ 	
	  delay_init(120);                 //��ʼ����ʱ���� 
	  usart_init(115200);              //��ʼ������
	  usmart_init(120);	               //��ʼ��USMART
	  LED_Init();							         //��ʼ��LED

  	while(1) 
	 {		 

	 } 
}
