#include "key.h"
#include "delay.h"


//按键初始化函数
void KEY_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);  //GPIOA时钟使能
	
		gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);    //设置PA0为下拉输入
}


uint8_t	KEY_Scan(void)
{
	if( gpio_input_bit_get ( GPIOA,GPIO_PIN_15) == 0 )	//检测按键是否被按下
	{	
		delay_ms(10);	//延时消抖
		if(gpio_input_bit_get ( GPIOA,GPIO_PIN_15) == 0)	//再次检测是否为低电平
		{
			while(gpio_input_bit_get ( GPIOA,GPIO_PIN_15) == 0);	//等待按键放开
			return KEY_ON;	//返回按键按下标志
		}
	}
	return KEY_OFF;	
}

