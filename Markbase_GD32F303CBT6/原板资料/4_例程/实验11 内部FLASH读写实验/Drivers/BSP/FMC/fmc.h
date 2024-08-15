#ifndef __FMC_H__
#define __FMC_H__
#include "sys.h"  


#define FLASH_PAGE_SIZE	        ((uint32_t)0x0800)   //ҳ��С
#define GD32_FLASH_SIZE         0x20000              //GD32�ڲ�FLASH�ܴ�С 
#define GD32_FLASH_BASE         0x08000000 			     //GD32�ڲ�FLASH��ʼ��ַ




uint16_t GDFLASH_ReadHalfWord(uint32_t faddr);		  	                       //FLASH��������  
void GDFLASH_Write_NoCheck(uint32_t waddr, uint16_t *pbuf, uint16_t length); //������д��
void GDFLASH_Write(uint32_t waddr, uint16_t *pbuf, uint16_t length);		     //��FLASHָ��λ��,д��ָ�����ȵ�����(�Զ�����)
void GDFLASH_Read(uint32_t raddr, uint16_t *pbuf, uint16_t length);   		   //��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(uint32_t WriteAddr,uint16_t WriteData);		

#endif

















