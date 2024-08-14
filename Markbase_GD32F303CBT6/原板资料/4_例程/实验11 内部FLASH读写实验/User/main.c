#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "usmart.h"
#include "led.h"
#include "bsp_ili9341_lcd.h"
#include "key.h" 
#include "fmc.h"


//要写入到GD32 FLASH的字符串数组 
const uint8_t TEXT_Buffer[] = {"GD32 FMC TEST"};

#define TEXT_LENTH sizeof(TEXT_Buffer)   //数组长度 

//要写入的数组大小必须是2的整数倍, 如果不是的话, 强制对齐到2的整数倍 
#define SIZE    TEXT_LENTH / 2 + ((TEXT_LENTH % 2) ? 1 : 0)

#define FLASH_SAVE_ADDR     0X08015000   //设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小 + 0X08000000) 
	 
int main(void)
{ 	
	  uint8_t key;
    uint16_t i = 0;
    uint16_t datatemp[SIZE];
		  
	  delay_init(120);                 //初始化延时函数 
	  usart_init(115200);              //初始化串口
	  usmart_init(120);                //初始化USMART
	  LED_Init();							         //初始化LED
    ILI9341_Init ();                 //初始化LCD    
	  KEY_Init();                      //初始化按键
	
		LCD_SetFont(&Font8x16);
	  LCD_SetColors(RED,BLACK);

    ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* 清屏，显示全黑 */
	
	
	  LCD_SetTextColor(RED);    //设置字体为红色 
    ILI9341_DispStringLine_EN(LINE(0),"Makerbase GD32F3");
    ILI9341_DispStringLine_EN(LINE(1),"MCU FLASH TEST");
    ILI9341_DispStringLine_EN(LINE(2),"genbotter.com");
    ILI9341_DispStringLine_EN(LINE(3),"2024/3/14");

	  ILI9341_DispString_EN(30,110,"KEY0:Write  KEY1:Read");	
	  LCD_SetTextColor(BLUE);   //设置字体为蓝色	

  	while(1) 
	  {		 
//			key=KEY_Scan(0);   //按键扫描
//			if(key==KEY1_PRES) //KEY1按下,写入GD32 FLASH
//			{
//					ILI9341_Clear(0,150,LCD_X_LENGTH,LCD_Y_LENGTH/2);   //清除半屏    
//					ILI9341_DispString_EN(30,150,"Start Write FLASH....");
//					GDFLASH_Write(FLASH_SAVE_ADDR, (uint16_t *)TEXT_Buffer, SIZE);              
//					ILI9341_DispString_EN(30,150,"FLASH Write Finished!"); //提示传送完成
//			}
//			if(key==KEY0_PRES) //KEY0 按下,读取字符串并显示
//			{
//					ILI9341_DispString_EN(30,150,"Start Read FLASH.... ");
//					GDFLASH_Read(FLASH_SAVE_ADDR, (uint16_t *)datatemp, SIZE);                        
//					ILI9341_DispString_EN(30,150,"The Data Readed Is:  "); //提示传送完成
//					ILI9341_DispString_EN(30,170,(char *)datatemp);        //显示读到的字符串
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
					LED1_TOGGLE(); //LED0提示系统正在运行	
					i=0;
			}		   
	 } 
}


