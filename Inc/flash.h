#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32g4xx_hal.h"

//Flash 256K(0x0800 0000 - 0x0803 FFFF)   ,CmdMap长度为 128*2B,将最后一个Page作为存储CmdMap的地址 0x0803 FE00-0x0803 FFFF
#define PAGE_SIZE  0x200	        //内部Flash每页大小，小容量产品和中容量产品Flash每页1K字节（1024），大容量产品Flash每页2K字节（2048）。这里以32位字为单位
//#define CMDMAP_ADD	0x0803FE00 			    // 内存控制表起始地址    ADDR_FLASH_SECTOR_10  // 内存控制表起始地址 128

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
