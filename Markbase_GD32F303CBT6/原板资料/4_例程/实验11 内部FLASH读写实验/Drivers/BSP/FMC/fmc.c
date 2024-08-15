#include "fmc.h"
#include "delay.h"
#include "usart.h"


//��ָ����ַ��ȡһ������ (16λ����) 
//faddr : ��ȡ��ַ (�˵�ַ����Ϊ2�ı���!!) 
//����ֵ:��ȡ�������� (16λ)
uint16_t GDFLASH_ReadHalfWord(uint32_t faddr)
{
		return  REG16(faddr); 
}

//������д��
//������������Ѿ���ԭ����������������д��
//waddr  : ��ʼ��ַ (�˵�ַ����Ϊ2�ı���!!)
//pbuf   : ����ָ��
//length : Ҫд��� ����(16λ)�� 
void GDFLASH_Write_NoCheck(uint32_t waddr, uint16_t *pbuf, uint16_t length)   
{ 			 		 
		uint16_t i;
		for(i=0;i<length;i++)
		{  
				fmc_halfword_program(waddr, pbuf[i]);
				waddr+=2; //��ַ����2,ָ����һ������
		}  
} 

//��FLASHָ��λ��,д��ָ�����ȵ�����(�Զ�����)
//�ú�����GD32���������FMCָ��λ��д��ָ�����ȵ�����
//�ú������ȼ��Ҫд���ҳ�Ƿ��ǿ�(ȫ0XFFFF)��?, ���
//����, ���Ȳ���, �����, ��ֱ����ҳ����д������.
//���ݳ��Ȳ���һҳʱ���Զ�д�ز���ǰ������
//waddr  : ��ʼ��ַ (�˵�ַ����Ϊ2�ı���!!)
//pbuf   : ����ָ��
//length : Ҫд��� ����(16λ)�� 
uint16_t FLASH_BUF[FLASH_PAGE_SIZE/2]; //�����2K�ֽ�
void GDFLASH_Write(uint32_t waddr, uint16_t *pbuf, uint16_t length)	
{
    uint32_t secpos;    //������ַ 
    uint16_t secoff;    //������ƫ�Ƶ�ַ(16λ�ּ���) 
    uint16_t secremain; //������ʣ���ַ(16λ�ּ���) 
    uint16_t i;
    uint32_t offaddr;   //ȥ��0X08000000��ĵ�ַ 
	
		if(waddr<GD32_FLASH_BASE||(waddr>=(GD32_FLASH_BASE+GD32_FLASH_SIZE)))
		{
			  return;           //�Ƿ���ַ
		}
		
		fmc_unlock();					//����FLASH
		
		offaddr=waddr-GD32_FLASH_BASE;		    //ʵ��ƫ�Ƶ�ַ.
		secpos=offaddr/FLASH_PAGE_SIZE;			  //������ַ  0~127 for GD32F303RBT6
		secoff=(offaddr%FLASH_PAGE_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
		secremain=FLASH_PAGE_SIZE/2-secoff;		//����ʣ��ռ��С   
		if(length<=secremain) secremain=length;  //�����ڸ�������Χ
		while(1) 
		{	
			GDFLASH_Read(secpos*FLASH_PAGE_SIZE+GD32_FLASH_BASE,FLASH_BUF,FLASH_PAGE_SIZE/2); //������������������
			for(i=0;i<secremain;i++)               //У������
			{
				if(FLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����  	  
			}
			if(i<secremain)//��Ҫ����
			{
				fmc_page_erase(secpos*FLASH_PAGE_SIZE+GD32_FLASH_BASE); //�����������
				for(i=0;i<secremain;i++) //����
				{
					FLASH_BUF[i+secoff]=pbuf[i];	  
				}
				GDFLASH_Write_NoCheck(secpos*FLASH_PAGE_SIZE+GD32_FLASH_BASE,FLASH_BUF,FLASH_PAGE_SIZE/2);//д����������  
			}else GDFLASH_Write_NoCheck(waddr,pbuf,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
			if(length==secremain) break;//д�������
			else //д��δ����
			{
				  secpos++;				    //ҳ��ַ��1
				  secoff=0;				    //ƫ��λ��Ϊ0 	 
					pbuf+=secremain;  	//ָ��ƫ��
				  waddr+=secremain*2;	//д��ַƫ��(16λ���ݵ�ַ,��Ҫ*2)	   
					length-=secremain;	//Ҫд���ֽ�(16λ)���ݼ�
				  if(length>(FLASH_PAGE_SIZE/2))secremain=FLASH_PAGE_SIZE/2;//��һ����������д����
				  else secremain=length;//��һ����������д����
			}	 
		}	
		fmc_lock();//FLASH����
}


//��ָ����ַ��ʼ����ָ�����ȵ�����
//raddr : ��ʼ��ַ
//pbuf  : ����ָ��
//length: Ҫ��ȡ�İ���(16λ)��,��2���ֽڵ�������
void GDFLASH_Read(uint32_t raddr, uint16_t *pbuf, uint16_t length)   	
{
		uint16_t i;
		for(i=0;i<length;i++)
		{
			pbuf[i]=GDFLASH_ReadHalfWord(raddr); //��ȡ2���ֽ�.
			raddr+=2; //ƫ��2���ֽ�.	
		}
}

//////////////////////////////////////////�����ô���///////////////////////////////////////////
//����д����(д1������)
//WriteAddr:��ʼ��ַ
//WriteData:Ҫд�������
void Test_Write(uint32_t WriteAddr,uint16_t WriteData)   	
{
		GDFLASH_Write(WriteAddr,&WriteData,1); //д��һ������
}
















