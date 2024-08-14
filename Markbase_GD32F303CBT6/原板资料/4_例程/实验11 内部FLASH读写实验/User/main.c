#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "usmart.h"
#include "led.h"
#include "bsp_ili9341_lcd.h"
#include "key.h" 
#include "fmc.h"


//Ҫд�뵽GD32 FLASH���ַ������� 
const uint8_t TEXT_Buffer[] = {"GD32 FMC TEST"};

#define TEXT_LENTH sizeof(TEXT_Buffer)   //���鳤�� 

//Ҫд��������С������2��������, ������ǵĻ�, ǿ�ƶ��뵽2�������� 
#define SIZE    TEXT_LENTH / 2 + ((TEXT_LENTH % 2) ? 1 : 0)

#define FLASH_SAVE_ADDR     0X08015000   //����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С + 0X08000000) 
	 
int main(void)
{ 	
	  uint8_t key;
    uint16_t i = 0;
    uint16_t datatemp[SIZE];
		  
	  delay_init(120);                 //��ʼ����ʱ���� 
	  usart_init(115200);              //��ʼ������
	  usmart_init(120);                //��ʼ��USMART
	  LED_Init();							         //��ʼ��LED
    ILI9341_Init ();                 //��ʼ��LCD    
	  KEY_Init();                      //��ʼ������
	
		LCD_SetFont(&Font8x16);
	  LCD_SetColors(RED,BLACK);

    ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* ��������ʾȫ�� */
	
	
	  LCD_SetTextColor(RED);    //��������Ϊ��ɫ 
    ILI9341_DispStringLine_EN(LINE(0),"Makerbase GD32F3");
    ILI9341_DispStringLine_EN(LINE(1),"MCU FLASH TEST");
    ILI9341_DispStringLine_EN(LINE(2),"genbotter.com");
    ILI9341_DispStringLine_EN(LINE(3),"2024/3/14");

	  ILI9341_DispString_EN(30,110,"KEY0:Write  KEY1:Read");	
	  LCD_SetTextColor(BLUE);   //��������Ϊ��ɫ	

  	while(1) 
	  {		 
//			key=KEY_Scan(0);   //����ɨ��
//			if(key==KEY1_PRES) //KEY1����,д��GD32 FLASH
//			{
//					ILI9341_Clear(0,150,LCD_X_LENGTH,LCD_Y_LENGTH/2);   //�������    
//					ILI9341_DispString_EN(30,150,"Start Write FLASH....");
//					GDFLASH_Write(FLASH_SAVE_ADDR, (uint16_t *)TEXT_Buffer, SIZE);              
//					ILI9341_DispString_EN(30,150,"FLASH Write Finished!"); //��ʾ�������
//			}
//			if(key==KEY0_PRES) //KEY0 ����,��ȡ�ַ�������ʾ
//			{
//					ILI9341_DispString_EN(30,150,"Start Read FLASH.... ");
//					GDFLASH_Read(FLASH_SAVE_ADDR, (uint16_t *)datatemp, SIZE);                        
//					ILI9341_DispString_EN(30,150,"The Data Readed Is:  "); //��ʾ�������
//					ILI9341_DispString_EN(30,170,(char *)datatemp);        //��ʾ�������ַ���
//				
//				
//			}
			
			if(KEY_Scan() == KEY_ON)
			{
				GDFLASH_Write(FLASH_SAVE_ADDR, (uint16_t *)TEXT_Buffer, SIZE);   
				GDFLASH_Read(FLASH_SAVE_ADDR, (uint16_t *)datatemp, SIZE);  
				
				printf("%s",(char *)datatemp);
			}
			
			
			i++;
			delay_ms(10);
			if(i==20)
			{
					LED1_TOGGLE(); //LED0��ʾϵͳ��������	
					i=0;
			}		   
	 } 
}


