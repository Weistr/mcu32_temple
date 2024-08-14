#include "key.h"
#include "delay.h"


//������ʼ������
void KEY_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);  //GPIOAʱ��ʹ��
	
		gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);    //����PA0Ϊ��������
}


uint8_t	KEY_Scan(void)
{
	if( gpio_input_bit_get ( GPIOA,GPIO_PIN_15) == 0 )	//��ⰴ���Ƿ񱻰���
	{	
		delay_ms(10);	//��ʱ����
		if(gpio_input_bit_get ( GPIOA,GPIO_PIN_15) == 0)	//�ٴμ���Ƿ�Ϊ�͵�ƽ
		{
			while(gpio_input_bit_get ( GPIOA,GPIO_PIN_15) == 0);	//�ȴ������ſ�
			return KEY_ON;	//���ذ������±�־
		}
	}
	return KEY_OFF;	
}

