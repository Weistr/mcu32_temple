#ifndef __FMC_H__
#define __FMC_H__
#include "sys.h"  


#define FLASH_PAGE_SIZE	        ((uint32_t)0x0800)   //页大小
#define GD32_FLASH_SIZE         0x20000              //GD32内部FLASH总大小 
#define GD32_FLASH_BASE         0x08000000 			     //GD32内部FLASH起始地址




uint16_t GDFLASH_ReadHalfWord(uint32_t faddr);		  	                       //FLASH读出半字  
void GDFLASH_Write_NoCheck(uint32_t waddr, uint16_t *pbuf, uint16_t length); //不检查的写入
void GDFLASH_Write(uint32_t waddr, uint16_t *pbuf, uint16_t length);		     //在FLASH指定位置,写入指定长度的数据(自动擦除)
void GDFLASH_Read(uint32_t raddr, uint16_t *pbuf, uint16_t length);   		   //从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(uint32_t WriteAddr,uint16_t WriteData);		

#endif

















