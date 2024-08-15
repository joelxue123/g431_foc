#ifndef __LIMIT_POS_DETECTION_H__
#define __LIMIT_POS_DETECTION_H__
#include "stm32g4xx_hal.h"
#include "global.h"
/*------------------------------Ѱ�Ҽ���λ����ر���---------------------------------*/
extern volatile s16 g_pos_act_array[8];		      		// ʵ��λ�ü�¼����
extern volatile u8 g_reachUpperLimit;		      		// ����λ�ü��� ����
extern volatile u8 g_reachLowerLimit;		      		// ����λ�ü��� ����		      
/*-------------------------------------                    --------------------------*/

void clear_limit_event(void);
extern void detect_limitPos(s16 pos_act,s16 current_set);
#endif



