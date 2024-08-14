#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"


int main(void)
{ 
	  uint8_t t;
		uint8_t len;	
	  uint16_t times=0;
	
	  delay_init(120);                          //初始化延时函数 
	  usart_init(115200);                       //初始化串口
	  LED_Init();							                  //初始化LED
    while(1)
		{
        if(USART_RX_STA&0x8000)    //接收完了一次数据
				{					    
					len=USART_RX_STA&0x3fff; //得到此次接收到的数据长度
					printf("\r\n您发送的消息为:\r\n");
					for(t=0;t<len;t++)
					{
						usart_data_transmit(USART0, USART_RX_BUF[t]);         //发送接收到的数据

						while(RESET == usart_flag_get(USART0, USART_FLAG_TC));//等待发送结束
					}
					printf("\r\n\r\n");      //插入换行
					USART_RX_STA=0;
				}else
				{
					times++;
					if(times%5000==0)
					{
						printf("\r\nGD32开发板 串口实验\r\n\r\n");
					}
					if(times%200==0)printf("请输入数据,以回车键结束\r\n");  
					if(times%30==0) LED1_TOGGLE();//闪烁LED,提示系统正在运行.
					delay_ms(10);   
				} 
		}
}




