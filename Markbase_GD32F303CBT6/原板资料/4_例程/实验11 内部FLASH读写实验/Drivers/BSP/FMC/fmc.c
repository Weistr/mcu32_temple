#include "fmc.h"
#include "delay.h"
#include "usart.h"


//从指定地址读取一个半字 (16位数据) 
//faddr : 读取地址 (此地址必须为2的倍数!!) 
//返回值:读取到的数据 (16位)
uint16_t GDFLASH_ReadHalfWord(uint32_t faddr)
{
		return  REG16(faddr); 
}

//不检查的写入
//这个函数假设已经把原来的扇区擦除过再写入
//waddr  : 起始地址 (此地址必须为2的倍数!!)
//pbuf   : 数据指针
//length : 要写入的 半字(16位)数 
void GDFLASH_Write_NoCheck(uint32_t waddr, uint16_t *pbuf, uint16_t length)   
{ 			 		 
		uint16_t i;
		for(i=0;i<length;i++)
		{  
				fmc_halfword_program(waddr, pbuf[i]);
				waddr+=2; //地址增加2,指向下一个半字
		}  
} 

//在FLASH指定位置,写入指定长度的数据(自动擦除)
//该函数往GD32闪存控制器FMC指定位置写入指定长度的数据
//该函数会先检测要写入的页是否是空(全0XFFFF)的?, 如果
//不是, 则先擦除, 如果是, 则直接往页里面写入数据.
//数据长度不足一页时，自动写回擦除前的数据
//waddr  : 起始地址 (此地址必须为2的倍数!!)
//pbuf   : 数据指针
//length : 要写入的 半字(16位)数 
uint16_t FLASH_BUF[FLASH_PAGE_SIZE/2]; //最多是2K字节
void GDFLASH_Write(uint32_t waddr, uint16_t *pbuf, uint16_t length)	
{
    uint32_t secpos;    //扇区地址 
    uint16_t secoff;    //扇区内偏移地址(16位字计算) 
    uint16_t secremain; //扇区内剩余地址(16位字计算) 
    uint16_t i;
    uint32_t offaddr;   //去掉0X08000000后的地址 
	
		if(waddr<GD32_FLASH_BASE||(waddr>=(GD32_FLASH_BASE+GD32_FLASH_SIZE)))
		{
			  return;           //非法地址
		}
		
		fmc_unlock();					//解锁FLASH
		
		offaddr=waddr-GD32_FLASH_BASE;		    //实际偏移地址.
		secpos=offaddr/FLASH_PAGE_SIZE;			  //扇区地址  0~127 for GD32F303RBT6
		secoff=(offaddr%FLASH_PAGE_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
		secremain=FLASH_PAGE_SIZE/2-secoff;		//扇区剩余空间大小   
		if(length<=secremain) secremain=length;  //不大于该扇区范围
		while(1) 
		{	
			GDFLASH_Read(secpos*FLASH_PAGE_SIZE+GD32_FLASH_BASE,FLASH_BUF,FLASH_PAGE_SIZE/2); //读出整个扇区的内容
			for(i=0;i<secremain;i++)               //校验数据
			{
				if(FLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
			}
			if(i<secremain)//需要擦除
			{
				fmc_page_erase(secpos*FLASH_PAGE_SIZE+GD32_FLASH_BASE); //擦除这个扇区
				for(i=0;i<secremain;i++) //复制
				{
					FLASH_BUF[i+secoff]=pbuf[i];	  
				}
				GDFLASH_Write_NoCheck(secpos*FLASH_PAGE_SIZE+GD32_FLASH_BASE,FLASH_BUF,FLASH_PAGE_SIZE/2);//写入整个扇区  
			}else GDFLASH_Write_NoCheck(waddr,pbuf,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
			if(length==secremain) break;//写入结束了
			else //写入未结束
			{
				  secpos++;				    //页地址增1
				  secoff=0;				    //偏移位置为0 	 
					pbuf+=secremain;  	//指针偏移
				  waddr+=secremain*2;	//写地址偏移(16位数据地址,需要*2)	   
					length-=secremain;	//要写入字节(16位)数递减
				  if(length>(FLASH_PAGE_SIZE/2))secremain=FLASH_PAGE_SIZE/2;//下一个扇区还是写不完
				  else secremain=length;//下一个扇区可以写完了
			}	 
		}	
		fmc_lock();//FLASH上锁
}


//从指定地址开始读出指定长度的数据
//raddr : 起始地址
//pbuf  : 数据指针
//length: 要读取的半字(16位)数,即2个字节的整数倍
void GDFLASH_Read(uint32_t raddr, uint16_t *pbuf, uint16_t length)   	
{
		uint16_t i;
		for(i=0;i<length;i++)
		{
			pbuf[i]=GDFLASH_ReadHalfWord(raddr); //读取2个字节.
			raddr+=2; //偏移2个字节.	
		}
}

//////////////////////////////////////////测试用代码///////////////////////////////////////////
//测试写数据(写1个半字)
//WriteAddr:起始地址
//WriteData:要写入的数据
void Test_Write(uint32_t WriteAddr,uint16_t WriteData)   	
{
		GDFLASH_Write(WriteAddr,&WriteData,1); //写入一个半字
}
















