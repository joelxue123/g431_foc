#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32g4xx_hal.h"

//Flash 256K(0x0800 0000 - 0x0803 FFFF)   ,CmdMap����Ϊ 128*2B,�����һ��Page��Ϊ�洢CmdMap�ĵ�ַ 0x0803 FE00-0x0803 FFFF
#define PAGE_SIZE  0x200	        //�ڲ�Flashÿҳ��С��С������Ʒ����������ƷFlashÿҳ1K�ֽڣ�1024������������ƷFlashÿҳ2K�ֽڣ�2048����������32λ��Ϊ��λ
//#define CMDMAP_ADD	0x0803FE00 			    // �ڴ���Ʊ���ʼ��ַ    ADDR_FLASH_SECTOR_10  // �ڴ���Ʊ���ʼ��ַ 128

#define FLASH_PARA_START_ADDR(PAGE)  	(0x08000000 + PAGE*0x00000800) 																		//Page 0-63;
#define FLASH_PARA_END_ADDR(PAGE)  	 	(0x08003fff + PAGE*0x00000400) 																		//Page 0-63;

#define CMDMAP_ADD 					FLASH_PARA_START_ADDR(62)
#define USERCMD_MAP_ADD 		FLASH_PARA_START_ADDR(63) 


s32 Flash_Write(u32 Address, const u64* pData, u32 Len);
s32 Flash_Read(u32 Address, u16* Readbuff, u32 Len);
extern s32 Flash_Init(void);
extern s32 Flash_SaveCmdmap(void);
extern s32 Flash_SaveUserCmdmap(void);
extern s32 Flash_ReadCmdmap(void);
void clear_flash_erro(void);
#endif
