#ifndef __PID_H__
#define __PID_H__

#include "stm32g4xx_hal.h"
#define CUR_ARRAY_NUM  512 //1024	  // 电流环PID输出右移位数
#define CUR_SHIFT_BIT  9  //	  // 电流环PID输出右移位数

#define CUR_PID_CUTOFF_BIT  12	  // 电流环PID输出右移位数
#define SPD_PID_CUTOFF_BIT  10  //10  // 速度环PID输出右移位数

#define MAX_SPDI		20000000   //20000000		// 最大速度积分
#define MAX_POSI		20000000		// 最大位置积分
#define MAX_SPDADDI	(MAX_VELADD*65336)		// 最大位置调节（附加速度）积分 100 *65336

struct PID_TYPE{
	u32 enable;
	u32 is_at_dead_zone;
	s32 Kp,Ki,Kd,dead_zone;
	s32 error;
	s32 error_last;
	s32 Part_P;	
	int64_t Part_I;
	s32 Part_D;
	s32 out;
	s32 frequency;
	s32 Up_limit;
	s32 Low_limit;
	//float integral;//
	s32 integral;//
};
extern struct PID_TYPE pid_IQ,pid_ID,pid_force,pid_spd,pid_pos;
void PID_ini(struct PID_TYPE * p_pid,u32 freq);
void PID_control(s32 set,s32 act,struct PID_TYPE * p_pid);   //PID
//Force PID output
void force_PID_out(s32 out,struct PID_TYPE * p_pid);   //PID

extern volatile s32 g_Iq;			// 交流电流，mA
extern volatile s32 g_Id;			// 直流电流，mA
extern volatile s32 g_Iq_filter;			// 交流电流，mA
extern volatile s32 g_Id_filter;			// 直流电流，mA
extern volatile s32  g_Iq_Array[CUR_ARRAY_NUM];			        // 交流电流，mA
extern volatile s32  g_Id_Array[CUR_ARRAY_NUM];			        // 直流电流，mA
extern volatile s32 g_I_Array_Index;
extern volatile s32 g_I_Array_Index_Invalid;
extern volatile s32 g_Iq_sum;
extern volatile s32 g_Id_sum;
extern s32 g_Vq;			// 交流电压
extern s32 g_Sector;		      // 电压矢量所在区间

extern s32 g_Vd;
extern s32 g_V_Alpha;
extern s32 g_V_Beta;
extern s16 g_IA,g_IA_Raw;			// A相电流，mA
extern s16 g_IB,g_IB_Raw;			// B相电流，mA
extern s16 g_IC,g_IC_Raw;
extern s16 g_Sum_ABC;
extern int64_t g_Pos_Icnt;       // 位置环I累积值
extern float Speed_P_out;
extern float Speed_I_out;
extern s32 g_spd_error; 
extern u8 PosAtDead_Flag_Last;
extern u8 PosAtDead_Flag;
//extern u8 PostionPID_Enable;
//extern u8 SpeedPID_Enable;
extern volatile u8 Grriper_Ready;
extern volatile u8 Grriper_Run,Grriper_Run_last;
extern volatile u8 Grriper_Run_on_delay_status,Grriper_Run_on_delay_status_last;
extern volatile u8 Grriper_Run_off_delay_status,Grriper_Run_off_delay_status_last;
extern volatile s32 Grriper_Run_on_delay_cnt,Grriper_Run_off_delay_cnt;
extern volatile s32 V_AddReg;//附加速度调节量
extern volatile u8 Flag_Set_OpenLen;
extern volatile s32 pos_error_to_target;
extern void GetElectricAngle_FromSPI(void);
extern void ServoPro_Fast(void);
extern void work_var_updata(void);
extern void GetCurrent(void);
extern void Loop3_EnableManage(void);
//位置控制，采用高低速简单控制的方法
extern void Gripper_control(void);
void park_transform(int alpha, int beta, int*q, int*d,int ElectricAngle );
#endif

