/************************************************ 
* WKS Mini GD32������
* FATFS�ײ�(diskio) ��������	   
* �汾��V1.0								  
************************************************/	

#include "diskio.h"			
#include "spi_sdcard.h"
#include "norflash.h"
#include "malloc.h"	 	 


#define SD_CARD	 	0  			//SD��,���Ϊ0
#define EX_FLASH 	1			  //�ⲿspi flash,���Ϊ1


//����W25Q64 
//ǰ4.8M�ֽڸ�fatfs��,4.8M�ֽں�~4.8M+100K���û���,4.9M�Ժ�,���ڴ���ֿ�,�ֿ�ռ��3.09M.			 
#define FLASH_SECTOR_SIZE 	512
#define	FLASH_SECTOR_COUNT  9832;	   //4.8M�ֽ�,Ĭ��ΪW25Q64
#define FLASH_BLOCK_SIZE   	8     	 //ÿ��BLOCK��8������
  
 
//��ô���״̬
//pdrv:���̱��
DSTATUS disk_status (
	BYTE pdrv		      /* Physical drive nmuber to identify the drive */
)
{ 
		return RES_OK;
}  

//��ʼ������
//pdrv:���̱��
DSTATUS disk_initialize (
	BYTE pdrv				  /* Physical drive nmuber to identify the drive */
)
{
		uint8_t res=0;	    
		switch(pdrv)
		{
			case SD_CARD:		          //SD��
				  res=SD_Initialize();	//SD����ʼ�� 
					break;
			case EX_FLASH:		        //�ⲿflash
				  NORFLASH_Init();      //NORFLASH��ʼ��
				  break;
			default:
				  res=1; 
		}		 
		if(res) return STA_NOINIT;
		else    return 0;           //��ʼ���ɹ� 
} 

//������
//pdrv  :���̱��0~9
//buff  :���ݽ��ջ����׵�ַ
//sector:������ַ
//count :��Ҫ��ȡ��������
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	  uint8_t res=0; 
    if (!count)return RES_PARERR; //count���ܵ���0�����򷵻ز�������		 	 
		switch(pdrv)
		{
			case SD_CARD:         //SD��
				res=SD_ReadDisk(buff,sector,count);	 
				while(res)          //������
				{
					SD_Initialize();	//���³�ʼ��SD��
					res=SD_ReadDisk(buff,sector,count);	
					//printf("sd rd error:%d\r\n",res);
				}
				break;
			case EX_FLASH:        //�ⲿflash
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
    //������ֵ��������ֵת��ff.c�ķ���ֵ
    if(res==0x00) return RES_OK;	 
    else          return RES_ERROR;	   
}

//д����
//pdrv  :���̱��0~9
//buff  :���������׵�ַ
//sector:������ַ
//count :��Ҫд��������� 
DRESULT disk_write (
	BYTE pdrv,			 /* Physical drive nmuber to identify the drive */
	const BYTE *buff,/* Data to be written */
	DWORD sector,		 /* Sector address in LBA */
	UINT count			 /* Number of sectors to write */
)
{
	  uint8_t res=0;  
    if (!count)return RES_PARERR;//count���ܵ���0�����򷵻ز�������		 	 
		switch(pdrv)
		{
			case SD_CARD:        //SD��
				res=SD_WriteDisk((uint8_t *)buff,sector,count);
				while(res)         //д����
				{
					SD_Initialize(); //���³�ʼ��SD��
					res=SD_WriteDisk((uint8_t *)buff,sector,count);	
					//printf("sd rd error:%d\r\n",res);
				}
				break;
			case EX_FLASH:       //�ⲿflash
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
    //������ֵ��������ֵת��ff.c�ķ���ֵ
    if(res == 0x00) return RES_OK;	 
    else            return RES_ERROR;	
} 

//��ȡ�������Ʋ���
//pdrv:���̱��0~9
//ctrl:���ƴ���
//buff:����/���ջ�����ָ�� 
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		  /* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	  DRESULT res;						  			     
		if(pdrv==SD_CARD) //SD��
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
		}else if(pdrv==EX_FLASH)	//�ⲿFLASH  
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
		else   res=RES_ERROR; //�����Ĳ�֧��
    return res;
} 

//���ʱ��
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
		return 0;
}		

//��̬�����ڴ�
void *ff_memalloc (UINT size)			
{
		return (void*)mymalloc(size);
}

//�ͷ��ڴ�
void ff_memfree (void* mf)		 
{
		myfree(mf);
}









