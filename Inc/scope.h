#ifndef __SCOPE_H__
#define __SCOPE_H__

#include "stm32g4xx_hal.h"
extern void ScopeProDebug(void);//调试时监控数据输出
extern void ScopeProDebug_2(void);//调试时监控数据输出
extern void ReturnCLibADC_Frame(void);//调试时监控数据输出
//返回状态信息帧,状态信息帧的每一位都可以配置
extern void Return_state_frame(u8 cmd_type,u8 index);
extern void Return_OldFrame_state(u8 cmd_type,u8 index,u8 data) ;
extern void Return_OldFrame_data(u8 cmd_type,u8 index,u8 length) ;
#endif

