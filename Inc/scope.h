#ifndef __SCOPE_H__
#define __SCOPE_H__

#include "stm32g4xx_hal.h"
extern void ScopeProDebug(void);//����ʱ����������
extern void ScopeProDebug_2(void);//����ʱ����������
extern void ReturnCLibADC_Frame(void);//����ʱ����������
//����״̬��Ϣ֡,״̬��Ϣ֡��ÿһλ����������
extern void Return_state_frame(u8 cmd_type,u8 index);
extern void Return_OldFrame_state(u8 cmd_type,u8 index,u8 data) ;
extern void Return_OldFrame_data(u8 cmd_type,u8 index,u8 length) ;
#endif

