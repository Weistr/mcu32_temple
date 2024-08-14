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
		  
	  delay_init(120);                 //��ʼ����ʱ���� 
	  usart_init(115200);              //��ʼ������
	  LED_Init();							         //��ʼ��LED


		usbd_port_config(0);    //USB�ȶϿ� 
    delay_ms(500);
    usbd_port_config(1);    //USB�ٴ����� 
    delay_ms(500);
		
		
    MX_USB_DEVICE_Init();        //��ʼ��USB�豸,�����,����USB    
  	while(1) 
	  {		 		   
        if(usbstatus!=USB_GetStatus())   //USB����״̬�����˸ı�
        {
            usbstatus = USB_GetStatus(); //��¼�µ�״̬
            if(usbstatus==USBD_STATE_CONFIGURED)
            {
                LED1(0); //LED1��
            } else
            {
                LED1(1); //LED1��
            }
        }
        if(USB_USART_RX_STA&0x8000)
        {
            len=USB_USART_RX_STA&0x3FFF;//�õ��˴ν��յ������ݳ���
            USB_Printf("\r\n�����͵���Ϣ����Ϊ:%d\r\n",len);
						CDC_Transmit_FS(USB_USART_RX_BUF,len);
            USB_Printf("\r\n");//���뻻��
            USB_USART_RX_STA=0;
        } 
				else
        {
            times++;
            if(times%5000==0)
            {
                USB_Printf("\r\nMakerbase GD32������ ����ʵ��\r\n\r\n");
            }
            if(times%200==0)  USB_Printf("����������,�Իس�������\r\n");
            if(times%30==0) 
						{	
								LED1_TOGGLE(); //LED0��˸,��ʾϵͳ��������.
						}
            delay_ms(10);
        }
	  } 
}


