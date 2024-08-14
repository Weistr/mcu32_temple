#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "malloc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_ctlreq.h"
#include "hw_config.h"
#include "usbd_core.h"


int main(void)
{ 	
    uint16_t len;
    uint16_t times = 0;
    uint8_t usbstatus = 0;
		  
	  delay_init(120);                 //初始化延时函数 
	  usart_init(115200);              //初始化串口
	  LED_Init();							         //初始化LED


		usbd_port_config(0);    //USB先断开 
    delay_ms(500);
    usbd_port_config(1);    //USB再次连接 
    delay_ms(500);
		
		
    MX_USB_DEVICE_Init();        //初始化USB设备,添加类,开启USB    
  	while(1) 
	  {		 		   
        if(usbstatus!=USB_GetStatus())   //USB连接状态发生了改变
        {
            usbstatus = USB_GetStatus(); //记录新的状态
            if(usbstatus==USBD_STATE_CONFIGURED)
            {
                LED1(0); //LED1亮
            } else
            {
                LED1(1); //LED1灭
            }
        }
        if(USB_USART_RX_STA&0x8000)
        {
            len=USB_USART_RX_STA&0x3FFF;//得到此次接收到的数据长度
            USB_Printf("\r\n您发送的消息长度为:%d\r\n",len);
						CDC_Transmit_FS(USB_USART_RX_BUF,len);
            USB_Printf("\r\n");//插入换行
            USB_USART_RX_STA=0;
        } 
				else
        {
            times++;
            if(times%5000==0)
            {
                USB_Printf("\r\nMakerbase GD32开发板 串口实验\r\n\r\n");
            }
            if(times%200==0)  USB_Printf("请输入数据,以回车键结束\r\n");
            if(times%30==0) 
						{	
								LED1_TOGGLE(); //LED0闪烁,提示系统正在运行.
						}
            delay_ms(10);
        }
	  } 
}


