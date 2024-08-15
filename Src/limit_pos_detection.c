#include "limit_pos_detection.h"
#include "global.h"
/*------------------------------寻找极限位置相关变量---------------------------------*/
volatile s16 g_pos_act_array[8] = {0};		      		// 实际位置记录序列
volatile s16 g_pos_act_average = 0;		      				// 实际位置记录序列
volatile s16 g_current_array[8] = {0};		      		// 实际位置记录序列
volatile s16 g_current_average = 0;		      				// 实际位置记录序列
volatile u8 g_reachUpperLimit = 0;		      		// 到达位置极限 上限
volatile u8 g_reachLowerLimit = 0;		      		// 到达位置极限 下限		      
volatile u8 g_posNotChange = 0;		      		    // 位置不变化  
/*-------------------------------------                    --------------------------*/
/*
funtion:
para 

*/

enum
{
	ALERT = 0x01,
	FAULT = 0x02,
	RUNNNING = 0x04,
};


	#define MIN_CUR_UPPER_LIMIT_PU (5*16384/100)
	#define MAX_CUR_UPPER_LIMIT_PU (60*16384/100)
	
	#define MIN_CUR_LOWER_LIMIT_PU  (-60*16384/100)
	#define MAX_CUR_LOWER_LIMIT_PU   (-5*16384/100)


void detect_limitPos(s16 pos_act,s16 current_set)
{
	s16 i=0;
	static int time_after_power_on = 0;
	static int repeat_cnt =0;
	
/***********增加标志位******/		
	if(g_reachUpperLimit == 1)
	{
		g_CmdMap[CMD_STATE_WORD] = g_CmdMap[CMD_STATE_WORD]|(0x0001<<6);
	}
	else
	{
		g_CmdMap[CMD_STATE_WORD] = g_CmdMap[CMD_STATE_WORD]&(~(0x0001<<6));
	}
	
	if(g_reachLowerLimit == 1)
	{
		g_CmdMap[CMD_STATE_WORD] = g_CmdMap[CMD_STATE_WORD]|(0x0001<<7);
	}
	else
	{
		g_CmdMap[CMD_STATE_WORD] = g_CmdMap[CMD_STATE_WORD]&(~(0x0001<<7));
	}
	
	if(( g_CmdMap[CMD_ERROR] & 0x0fff ) )
	{
		g_CmdMap[CMD_STATE_WORD] |= FAULT;
	}
	else
	{
		g_CmdMap[CMD_STATE_WORD] &= (~FAULT);
	}
	
	if(( g_CmdMap[CMD_ERROR] & 0xf000) )
	{
		g_CmdMap[CMD_STATE_WORD] |= ALERT;
	}
	else
	{
		g_CmdMap[CMD_STATE_WORD] &= (~ALERT);
	}
	
	if(g_CmdMap[TAG_MOTOR_ENABLE] == 0) 				
	{
		g_CmdMap[CMD_STATE_WORD] &=  (~RUNNNING);
		
		time_after_power_on = 0;
		return;
	}
	
	if( target_position_arrived(  g_CmdMap[CMD_POS_SET_PU] , g_CmdMap[CMD_POS_ACT_PU] ) == 0 )
	{
		g_CmdMap[CMD_STATE_WORD] &=  (~RUNNNING);
	}
	else
	{
		g_CmdMap[CMD_STATE_WORD] |=  RUNNNING; 
	}
	/*************增加标志位**********/
	
	
	
	/*****************不使能找极限功能，退出************/
	if( g_CmdMap[CMD_LIMIT_DETECT_ENABLE] != 1 )  
	{
		return;
	}
	/*******************不使能找极限功能，退出*************/

/****使能开始后延时 100ms 再启动找极限功能*****/	
	time_after_power_on++;
	
	if(  time_after_power_on < 100)
	{
		return;
	}
	else
	{
		time_after_power_on = 101;
	}
	/****使能开始后延时 100ms 再启动找极限功能*****/	
	
	
	/******找极限代码*******/
	
	/*******每10ms才允许执行一次********/
	if( ++repeat_cnt < 10)
	{
		return;
	}	
	repeat_cnt = 0;
	
	for(i=0;i<7;i++)
	{
		g_pos_act_array[i] = g_pos_act_array[i+1];
		g_current_array[i] = g_current_array[i+1];
	}
	g_pos_act_array[7] = pos_act;
	g_current_array[7] = current_set;
	g_pos_act_average = (g_pos_act_array[0]+g_pos_act_array[1]+g_pos_act_array[2]+g_pos_act_array[3]
										+g_pos_act_array[4]+g_pos_act_array[5]+g_pos_act_array[6]+g_pos_act_array[7])/8;
	g_posNotChange = 1;
	for(i=0;i<8;i++)
	{
		if(ABS(g_pos_act_average-g_pos_act_array[i])>10)
		{
			//位置有变化
			g_posNotChange = 0;
			break;
		}
	}
	

	
	
	if( g_CmdMap[CMD_CUR_UPPER_LIMIT_PU] < MIN_CUR_UPPER_LIMIT_PU )
	{
		g_CmdMap[CMD_CUR_UPPER_LIMIT_PU] = MIN_CUR_UPPER_LIMIT_PU;
	}
	
	if( g_CmdMap[CMD_CUR_UPPER_LIMIT_PU] > MAX_CUR_UPPER_LIMIT_PU )
	{
		g_CmdMap[CMD_CUR_UPPER_LIMIT_PU] = MAX_CUR_UPPER_LIMIT_PU;
	}
	
	if( g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] < MIN_CUR_LOWER_LIMIT_PU )
	{
		g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] = MIN_CUR_LOWER_LIMIT_PU;
	}
	
	if( g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] > MAX_CUR_LOWER_LIMIT_PU )
	{
		g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] = MAX_CUR_LOWER_LIMIT_PU;
	}
	
	
	
	if(g_current_array[0]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[1]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[2]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[3]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[4]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[5]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[6]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_current_array[7]>= g_CmdMap[CMD_CUR_UPPER_LIMIT_PU]
		&&g_posNotChange == 1
		&&g_CmdMap[TAG_MOTOR_ENABLE] == 1
	)
	{
		g_CmdMap[CMD_LIMIT_UPPER] = g_pos_act_average;
		g_reachUpperLimit = 1;
		g_CmdMap[CMD_PAUSE] = 1;
	}
	if(g_current_array[0]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[1]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[2]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[3]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[4]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[5]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[6]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_current_array[7]<= g_CmdMap[CMD_CUR_LOWER_LIMIT_PU]
		&&g_posNotChange == 1
		&&g_CmdMap[TAG_MOTOR_ENABLE] == 1
	)
	{
		g_CmdMap[CMD_LIMIT_LOWER] = g_pos_act_average;
		g_reachLowerLimit = 1;
		g_CmdMap[CMD_PAUSE] = 1;
	}	
	
//	if(current_set > 163)
//	{
//		g_reachLowerLimit = 0;
//	}
//	if(current_set < -163)
//	{
//		g_reachUpperLimit = 0;
//	}	
		
	

	

	
}




void clear_limit_event(void)
{
	g_reachLowerLimit = 0;
	g_reachUpperLimit = 0;
}

