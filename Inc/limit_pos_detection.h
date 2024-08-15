#ifndef __LIMIT_POS_DETECTION_H__
#define __LIMIT_POS_DETECTION_H__
#include "stm32g4xx_hal.h"
#include "global.h"
/*------------------------------寻找极限位置相关变量---------------------------------*/
extern volatile s16 g_pos_act_array[8];		      		// 实际位置记录序列
extern volatile u8 g_reachUpperLimit;		      		// 到达位置极限 上限
extern volatile u8 g_reachLowerLimit;		      		// 到达位置极限 下限		      
/*-------------------------------------                    --------------------------*/

void clear_limit_event(void);
extern void detect_limitPos(s16 pos_act,s16 current_set);
#endif



