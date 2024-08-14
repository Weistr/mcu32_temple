/************************************************ 
* WKS Mini GD32开发板
* FATFS底层(diskio) 驱动代码	   
* 版本：V1.0								  
************************************************/	

#include "diskio.h"			
#include "spi_sdcard.h"
#include "norflash.h"
#include "malloc.h"	 	 


#define SD_CARD	 	0  			//SD卡,卷标为0
#define EX_FLASH 	1			  //外部spi flash,卷标为1


//对于W25Q64 
//前4.8M字节给fatfs用,4.8M字节后~4.8M+100K给用户用,4.9M以后,用于存放字库,字库占用3.09M.			 
#define FLASH_SECTOR_SIZE 	512
#define	FLASH_SECTOR_COUNT  9832;	   //4.8M字节,默认为W25Q64
#define FLASH_BLOCK_SIZE   	8     	 //每个BLOCK有8个扇区
  
 
//获得磁盘状态
//pdrv:磁盘编号
DSTATUS disk_status (
	BYTE pdrv		      /* Physical drive nmuber to identify the drive */
)
{ 
		return RES_OK;
}  

//初始化磁盘
//pdrv:磁盘编号
DSTATUS disk_initialize (
	BYTE pdrv				  /* Physical drive nmuber to identify the drive */
)
{
		uint8_t res=0;	    
		switch(pdrv)
		{
			case SD_CARD:		          //SD卡
				  res=SD_Initialize();	//SD卡初始化 
					break;
			case EX_FLASH:		        //外部flash
				  NORFLASH_Init();      //NORFLASH初始化
				  break;
			default:
				  res=1; 
		}		 
		if(res) return STA_NOINIT;
		else    return 0;           //初始化成功 
} 

//读扇区
//pdrv  :磁盘编号0~9
//buff  :数据接收缓冲首地址
//sector:扇区地址
//count :需要读取的扇区数
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	  uint8_t res=0; 
    if (!count)return RES_PARERR; //count不能等于0，否则返回参数错误		 	 
		switch(pdrv)
		{
			case SD_CARD:         //SD卡
				res=SD_ReadDisk(buff,sector,count);	 
				while(res)          //读出错
				{
					SD_Initialize();	//重新初始化SD卡
					res=SD_ReadDisk(buff,sector,count);	
					//printf("sd rd error:%d\r\n",res);
				}
				break;
			case EX_FLASH:        //外部flash
				for(;count>0;count--)
				{
					NORFLASH_Read(buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
					sector++;
					buff+=FLASH_SECTOR_SIZE;
				}
				res=0;
				break;
			default:
				res=1; 
		}
    //处理返回值，将返回值转成ff.c的返回值
    if(res==0x00) return RES_OK;	 
    else          return RES_ERROR;	   
}

//写扇区
//pdrv  :磁盘编号0~9
//buff  :发送数据首地址
//sector:扇区地址
//count :需要写入的扇区数 
DRESULT disk_write (
	BYTE pdrv,			 /* Physical drive nmuber to identify the drive */
	const BYTE *buff,/* Data to be written */
	DWORD sector,		 /* Sector address in LBA */
	UINT count			 /* Number of sectors to write */
)
{
	  uint8_t res=0;  
    if (!count)return RES_PARERR;//count不能等于0，否则返回参数错误		 	 
		switch(pdrv)
		{
			case SD_CARD:        //SD卡
				res=SD_WriteDisk((uint8_t *)buff,sector,count);
				while(res)         //写出错
				{
					SD_Initialize(); //重新初始化SD卡
					res=SD_WriteDisk((uint8_t *)buff,sector,count);	
					//printf("sd rd error:%d\r\n",res);
				}
				break;
			case EX_FLASH:       //外部flash
				for(;count>0;count--)
				{										    
					NORFLASH_Write((uint8_t *)buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
					sector++;
					buff+=FLASH_SECTOR_SIZE;
				}
				res=0;
				break;
			default:
				res=1; 
		}
    //处理返回值，将返回值转成ff.c的返回值
    if(res == 0x00) return RES_OK;	 
    else            return RES_ERROR;	
} 

//获取其他控制参数
//pdrv:磁盘编号0~9
//ctrl:控制代码
//buff:发送/接收缓冲区指针 
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		  /* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	  DRESULT res;						  			     
		if(pdrv==SD_CARD) //SD卡
		{
				switch(cmd)
				{
					case CTRL_SYNC:
					    res = RES_OK; 
							break;	 
					case GET_SECTOR_SIZE:
					    *(DWORD*)buff = 512; 
							res = RES_OK;
							break;	 
					case GET_BLOCK_SIZE:
							*(WORD*)buff = 8;
							res = RES_OK;
							break;	 
					case GET_SECTOR_COUNT:
							*(DWORD*)buff = SD_GetSectorCount();
							res = RES_OK;
							break;
					default:
							res = RES_PARERR;
							break;
				}
		}else if(pdrv==EX_FLASH)	//外部FLASH  
		{
				switch(cmd)
				{
					case CTRL_SYNC:
					    res = RES_OK; 
							break;	 
					case GET_SECTOR_SIZE:
							*(WORD*)buff = FLASH_SECTOR_SIZE;
							res = RES_OK;
							break;	 
					case GET_BLOCK_SIZE:
							*(WORD*)buff = FLASH_BLOCK_SIZE;
							res = RES_OK;
							break;	 
					case GET_SECTOR_COUNT:
							*(DWORD*)buff = FLASH_SECTOR_COUNT;
							res = RES_OK;
							break;
					default:
							res = RES_PARERR;
							break;
				}
		}
		else   res=RES_ERROR; //其他的不支持
    return res;
} 

//获得时间
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
		return 0;
}		

//动态分配内存
void *ff_memalloc (UINT size)			
{
		return (void*)mymalloc(size);
}

//释放内存
void ff_memfree (void* mf)		 
{
		myfree(mf);
}









