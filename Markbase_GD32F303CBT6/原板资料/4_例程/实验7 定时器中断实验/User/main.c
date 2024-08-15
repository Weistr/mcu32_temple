#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "timer.h"

int main(void)
{ 
	  delay_init(120);                          //��ʼ����ʱ���� 
	  usart_init(115200);                       //��ʼ������
	  LED_Init();							                  //��ʼ��LED
	  TIM2_Int_Init(5000-1,12000-1);            //����10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms
    while(1)
		{
      printf("OK\n");
      delay_ms(200);
		}
}
