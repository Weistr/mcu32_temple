/************************************************ 
* WKS Mini GD32������
* �������� ��������	   
* �汾��V1.0								  
************************************************/	

#include "key.h"
#include "delay.h"


//������ʼ������
void KEY_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);  //GPIOAʱ��ʹ��
	
		gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_0);    //����PA0Ϊ��������
		gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_13);   //����PA13Ϊ��������
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);   //����PA15Ϊ��������   
}


//����ɨ�躯��
//�ú�������Ӧ���ȼ�(ͬʱ���¶������): WK_UP > KEY1 > KEY0!!
//mode:0 / 1, ���庬������:
//0,  ��֧��������(���������²���ʱ, ֻ�е�һ�ε��û᷵�ؼ�ֵ,
//�����ɿ��Ժ�, �ٴΰ��²Ż᷵��������ֵ)
//1,  ֧��������(���������²���ʱ, ÿ�ε��øú������᷵�ؼ�ֵ)
//��ֵ, ��������:
//KEY0_PRES, 1, KEY0����
//KEY1_PRES, 2, KEY1����
//WKUP_PRES, 3, WKUP����
 
uint8_t KEY_Scan(uint8_t mode)
{
    static uint8_t key_up = 1;  //�������ɿ���־ 
    uint8_t keyval = 0;

    if (mode) key_up = 1;       //֧������ 
    if (key_up && (KEY0 == 0 || KEY1 == 0 || WK_UP == 1))  //�����ɿ���־Ϊ1, ��������һ������������ 
    {
        delay_ms(10);           //ȥ���� 
        key_up = 0;

        if (KEY0 == 0)  keyval = KEY0_PRES;
        if (KEY1 == 0)  keyval = KEY1_PRES;
        if (WK_UP == 1) keyval = WKUP_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1 && WK_UP == 0) //û���κΰ�������, ��ǰ����ɿ� 
    {
        key_up = 1;
    }

    return keyval;              //���ؼ�ֵ 
}
