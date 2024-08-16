#include "pid.h"
#include "global.h"
#include "icmu.h"
#include "parameter_identification.h"
#include "sensorless.h"
//#include "ouj_cmd.h"
//3�ջ��ŷ���ز���
s16 g_IA = 0,g_IA_Raw = 0;			        
s16 g_IB = 0,g_IB_Raw = 0;			        
s16 g_IC = 0,g_IC_Raw = 0;
s16 g_Sum_ABC = 0;
volatile s32  g_Iq = 0;			        // ����������mA
volatile s32  g_Id = 0;			        // ֱ��������mA
volatile s32 g_Iq_filter;			// ����������mA
volatile s32 g_Id_filter;			// ֱ��������mA
volatile s32  g_Iq_Array[CUR_ARRAY_NUM] = {0};			        // ����������mA
volatile s32  g_Id_Array[CUR_ARRAY_NUM] = {0};			        // ֱ��������mA
volatile s32 g_I_Array_Index = 0;
volatile s32 g_I_Array_Index_Invalid = 0;
volatile s32 g_Iq_sum = 0;
volatile s32 g_Id_sum = 0;
s32 g_Vq = 0;			        // ������ѹ
s32 g_Vd = 0;			        // ֱ����ѹ
s32 g_Vq_Pu = 0;			    // ֱ����ѹ
s32 g_Vd_Pu = 0;			    // ֱ����ѹ
s32 g_V_Alpha = 0;
s32 g_V_Beta = 0;

s32 g_Sector = 0;		      // ��ѹʸ����������
float g_Spd_Icnt = 0;		    // �ٶ�I�ۻ�ֵ
int64_t g_Pos_Icnt = 0;       // λ�û�I�ۻ�ֵ
float Speed_P_out = 0;
float Speed_I_out = 0;
s32 g_Pos_PID_16BIT = 0;  // λ�û�PI��
s32 g_Pos_Spd = 0;        // λ�û����ٶ����ֵ
volatile s32 V_AddReg = 0;     // λ�û����ٶ����ֵ �����ٶȵ�����
volatile s32 V_SetTotal = 0;     // λ�û����ٶ����ֵ �����ٶȵ�����


s16 g_testAngle = 0;           //


//u8 SpeedPID_Enable = 0;
//u8 PostionPID_Enable = 0;
s32 g_posPID_e = 0;
u8 PosAtDead_Flag = 0;
u8 PosAtDead_Flag_Last = 0;

volatile int time4_capture_ch1;
extern int g_ElectricAngle_add ;
extern float  pll_estimated_spd;
extern float pll_estimated_phase ;


s32 g_spd_error = 0;           //
extern volatile int32_t SPI_X[9] ;
extern volatile int32_t SPI_Y[9];
extern volatile int16_t SPI_0_Array[250];
s16 anlge_reg;
s32 g_decLen = 0;
s32 g_X, g_Y, g_Z;
s16 Spd_RPM_CatchLimit = 0;//�г��ٶ�����
//�����բ�߼������ر���
volatile u8 Grriper_Ready = 0;
volatile u8 Grriper_Run = 0,Grriper_Run_last = 0;
volatile s32 Grriper_Run_on_delay_cnt = 0,Grriper_Run_off_delay_cnt = 0;
volatile u8  Brake_rellease_delay = 0;
volatile s32 Brake_rellease_delay_cnt = 0;
volatile u8  Flag_Set_OpenLen = 0;
volatile u8 Test_perm = 0;
volatile s32 Test_dirt = 1;
volatile s32 Test_dirt_last = 1;
volatile s32 sin_wave_t_us = 0;//���Ҳ���ʱ us
volatile s32 motor_para_identification_count_Tms = 0;//���Ҳ���ʱms
volatile s32 I_phase[3][100] = {0};//���Ҳ���ʱms
volatile s32 index_I_phase = 0;//���Ҳ���ʱms
volatile s32 Para_Identificaion = 0;
double T_t = 0.0;
double T_t_1 = 0.0;
volatile s32 pos_error_to_target = 0;
struct PID_TYPE pid_IQ,pid_ID,pid_force,pid_spd,pid_pos,pid_pll;

void hfi_update_angle(void);
void PID_ini(struct PID_TYPE * p_pid,u32 freq)   //PID
{
		p_pid->integral = 0;
		p_pid->error = 0;
		p_pid->error_last = 0;
		p_pid->out = 0;
		p_pid->frequency = freq;
}
void PID_control(s32 set,s32 act,struct PID_TYPE * p_pid)   //PID
{
	s32 deriv=0;
	
	if(p_pid->enable == 0)
	{
		p_pid->integral = 0;
		p_pid->error = 0;
		p_pid->error_last = 0;
		p_pid->out = 0;
		p_pid->Part_P = 0;
		p_pid->Part_I = 0;
		p_pid->Part_D = 0;
		return;
	}
	p_pid->error = set - act; 
	if((p_pid->error<p_pid->dead_zone && p_pid->error>-1*p_pid->dead_zone))
	{
		p_pid->is_at_dead_zone = 1;
		p_pid->error = 0;
	}	
	if ((p_pid->out < p_pid->Up_limit && p_pid->out > p_pid->Low_limit) ||
			(p_pid->out >= p_pid->Up_limit && p_pid->error < 0) ||
			(p_pid->out <= p_pid->Low_limit && p_pid->error > 0))
	{
//		if(p_pid->Ki>1000)
//		{
//			p_pid->integral += (float)(p_pid->error*(p_pid->Ki/100));
//		}
//		else
//		{
//			p_pid->integral += (float)(p_pid->error*p_pid->Ki);
//		}
		if(p_pid->Ki>1000)
		{
			p_pid->integral += (p_pid->error*(p_pid->Ki/100));
		}
		else
		{
			p_pid->integral += (p_pid->error*p_pid->Ki);
		}
	}
	p_pid->Part_P = p_pid->error * p_pid->Kp/100;
	
	
//	if(p_pid->Ki>1000)
//	{
//		p_pid->Part_I = (s32)(100.0f*p_pid->integral/(float)p_pid->frequency);
//	}
//	else
//	{
//		p_pid->Part_I = (s32)(p_pid->integral/(float)p_pid->frequency);
//	}
	if(p_pid->Ki>1000)
	{
		p_pid->Part_I = (s32)(100*p_pid->integral/p_pid->frequency);
	}
	else
	{
		p_pid->Part_I = (s32)(p_pid->integral/p_pid->frequency);
	}	
	if(p_pid->Part_I>0x7FFFFFFF)
	{
		p_pid->Part_I = 0x7FFFFFFF;
	}
	else if(p_pid->Part_I<(-1*0x7FFFFFFF))
	{
		p_pid->Part_I = -1*0x7FFFFFFF;
	}
	deriv = p_pid->Part_D;
	p_pid->Part_D = p_pid->Kd * ( 9*deriv + 1*(p_pid->error-p_pid->error_last) )   /1000 ;
	p_pid->out = p_pid->Part_P+p_pid->Part_I+p_pid->Part_D;
	if(p_pid->out >p_pid->Up_limit)
	{
		p_pid->out = p_pid->Up_limit;
		p_pid->integral = (p_pid->out)*p_pid->frequency;
	}
	if(p_pid->out <p_pid->Low_limit)
	{
		p_pid->out = p_pid->Low_limit;
		p_pid->integral = (p_pid->out)*p_pid->frequency;
	}	
	return;
}
//Force PID output
void force_PID_out(s32 out,struct PID_TYPE * p_pid)   //PID
{
	p_pid->out = out;
	if(p_pid->out >p_pid->Up_limit)
	{
		p_pid->out = p_pid->Up_limit;
		//p_pid->integral = (p_pid->out-p_pid->Part_P - p_pid->Part_D)*p_pid->frequency;
	}
	if(p_pid->out <p_pid->Low_limit)
	{
		p_pid->out = p_pid->Low_limit;
		//p_pid->integral = (p_pid->out-p_pid->Part_P - p_pid->Part_D)*p_pid->frequency;
	}	
	p_pid->error = 0;
	p_pid->error_last = 0;
	p_pid->Part_P = 0;
	p_pid->Part_I = out;
	p_pid->integral = p_pid->Part_I*p_pid->frequency;
	p_pid->Part_D = 0;
}
static void ForcePID(void)
{
	s16 error_force = 0;
	s16 error_force_abs = 0;
//	static u8 flag_in_dead = 0;
//	static u16 delayCnt_in_dead = 0;
	if(g_CmdMap[CMD_FORCE_DIR] == 0)
	{
		force_set = g_CmdMap[CMD_FORCE_SET_PU];
		force_act = g_CmdMap[CMD_FORCE_ACT_RAW_PU];
	}
	else
	{
		force_set = -1*g_CmdMap[CMD_FORCE_SET_PU];
		force_act = -1*g_CmdMap[CMD_FORCE_ACT_RAW_PU];
	}
////	if(g_CmdMap[CMD_POS_ACT_PU]<(g_CmdMap[SYS_POS_UPPER_LIMIT_PU]-819) && g_CmdMap[CMD_POS_ACT_PU]>(g_CmdMap[SYS_POS_LOWER_LIMIT_PU]+819))
////	{
//				if(force_set>16)
//				{
//					pid_force.Up_limit = g_CmdMap[SYS_RESISTANCE_CURPU]+((force_set*g_CmdMap[SYS_K_PERCENT_FORCE_CURPU])>>14);
//					pid_force.Low_limit = g_CmdMap[SYS_CUR_LOWER_LIMIT_PU];
//				}
//				else if(force_set<-16)
//				{
//					pid_force.Up_limit = g_CmdMap[SYS_CUR_UPPER_LIMIT_PU];
//					pid_force.Low_limit = -g_CmdMap[SYS_RESISTANCE_CURPU]+((force_set*g_CmdMap[SYS_K_PERCENT_FORCE_CURPU])>>14);
//				}
//				else
//				{
//					pid_force.Up_limit = g_CmdMap[SYS_RESISTANCE_CURPU]+(g_CmdMap[SYS_K_PERCENT_FORCE_CURPU]>>7);
//					pid_force.Low_limit = -g_CmdMap[SYS_RESISTANCE_CURPU]-(g_CmdMap[SYS_K_PERCENT_FORCE_CURPU]>>7);					
//				}
//////	}
////	if(pid_force.Up_limit<g_CmdMap[SYS_RESISTANCE_CURPU])
////	{
////		pid_force.Up_limit=g_CmdMap[SYS_RESISTANCE_CURPU];
////	}
////	if(pid_force.Low_limit>(-1*g_CmdMap[SYS_RESISTANCE_CURPU]))
////	{
////		pid_force.Low_limit=-1*g_CmdMap[SYS_RESISTANCE_CURPU];
////	}
	pid_force.Up_limit = ((g_CmdMap[SYS_SPD_UPPER_LIMIT_PU])>>0);
	pid_force.Low_limit = ((g_CmdMap[SYS_SPD_LOWER_LIMIT_PU]>>0));
	
	if(g_CmdMap[CMD_POS_ACT_PU]>g_CmdMap[SYS_POS_UPPER_LIMIT_PU])
	{
		pid_force.Up_limit = 0;
		
	}
	else if(g_CmdMap[CMD_POS_ACT_PU]>(g_CmdMap[SYS_POS_UPPER_LIMIT_PU]-409)&& g_CmdMap[CMD_CUR_SET_PU]>0)
	{
		pid_force.Up_limit = 0;
		
	}
	if(g_CmdMap[CMD_POS_ACT_PU]<g_CmdMap[SYS_POS_LOWER_LIMIT_PU])
	{
		pid_force.Low_limit = 0;
		
	}	
	else if(g_CmdMap[CMD_POS_ACT_PU]<(g_CmdMap[SYS_POS_LOWER_LIMIT_PU]+409)  && g_CmdMap[CMD_CUR_SET_PU]<0)
	{
		pid_force.Low_limit = 0;
		
	}
	

	
//	error_force = force_set - force_act;
//	error_force_abs = ABS(error_force);
//	if(error_force < (pid_force.dead_zone>>1))
//	{
//		if(delayCnt_in_dead<1000)
//		{
//			delayCnt_in_dead++;
//		}
//		else
//		{
//			flag_in_dead = 1;
//		}
//	}
//	if(error_force_abs > pid_force.dead_zone)
//	{
//		delayCnt_in_dead = 0;
//		flag_in_dead = 0;
//	}
//	if(flag_in_dead == 1)
//	{
//		force_PID_out(0,&pid_force);
//	}
//	else
//	{
		PID_control(force_set,force_act,&pid_force);
//	}
	//
	g_CmdMap[CMD_SPD_ADD_PU] = pid_force.out;
	
	if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_4_FORCE || (g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE && status_Mode_POSITION_FORCE == 3))
	{
		Flag_Profile_Enable = 0;
		g_CmdMap[CMD_PROFILE_POS_PU] = g_CmdMap[CMD_POS_ACT_PU];
		g_CmdMap[CMD_POS_SET_PU] = g_CmdMap[CMD_POS_ACT_PU];
	}
}

// SVPWM����ռ�ձ�
static void Calculate_CCR_SVP(TIM_TypeDef* TIMx, int sector, int X, int Y, int Z)
{
  s32 t0, tk, tk_1;
  s32 CCR1 = 0;
  s32 CCR2 = 0;
  s32 CCR3 = 0;
  switch (sector)
  {
    case 1:
			tk_1 = Z;
			tk = Y;
			t0 = REAL_CCR - tk - tk_1;
			CCR3 = t0>>1;
			CCR1 = CCR3 + tk;
			CCR2 = CCR1 + tk_1;
		break;		
		case 2:
			tk_1 = Y;
			tk = -X;
			t0 = REAL_CCR - tk - tk_1;
			CCR2 = t0>>1;
			CCR3 = CCR2 + tk;
			CCR1 = CCR3 + tk_1;
		break;		
		case 3:
			tk_1 = X;
			tk = -Z;
			t0 = REAL_CCR - tk - tk_1;
			CCR3 = t0>>1;
			CCR2 = CCR3 + tk_1;
			CCR1 = CCR2 + tk;
		break;		
		case 4:
			tk_1 = -X;
			tk = Z;
			t0 = REAL_CCR - tk - tk_1;
			CCR1 = t0>>1;
			CCR2 = CCR1 + tk;
			CCR3 = CCR2 + tk_1;
		break;
		case 5:
			tk_1 = -Y;
			tk = X;
			t0 = REAL_CCR - tk - tk_1;
			CCR1 = t0>>1;
			CCR3 = CCR1 + tk_1;
			CCR2 = CCR3 + tk;
		break;
		case 6:
			tk_1 = -Z;
			tk = -Y;
			t0 = REAL_CCR - tk - tk_1;
			CCR2 = t0>>1;
			CCR1 = CCR2 + tk_1;
			CCR3 = CCR1 + tk;
		break;
		default:
			CCR1 = REAL_CCR >> 1;
			CCR2 = REAL_CCR >> 1;
			CCR3 = REAL_CCR >> 1;
		break;
  }
	CCR1 = CCR1>N97_CCR?N97_CCR:(CCR1<0?0:CCR1);
  CCR2 = CCR2>N97_CCR?N97_CCR:(CCR2<0?0:CCR2);
  CCR3 = CCR3>N97_CCR?N97_CCR:(CCR3<0?0:CCR3);
	if(g_CmdMap[SYS_MOT_TEST] == 0)
	{
			motor_para_identification_count_Tms = 0;
			sin_wave_t_us = 0;
	}
   // �������ռ�ձ�97%
		//���������ʶ Step1:����������Step2�����в�������Vq ����1KHz �����Ҳ�
	if(g_CmdMap[SYS_MOT_TEST] == 4 || g_CmdMap[SYS_MOT_TEST] == 5)
	{
		sin_wave_t_us = sin_wave_t_us + 4;
		if(sin_wave_t_us > 1000)
		{
			sin_wave_t_us = 0;
			motor_para_identification_count_Tms++;
		}
	}
	if(g_CmdMap[SYS_MOT_TEST] == 4)
	{
		CCR1 = 1;
		CCR2 = REAL_CCR-1;
		CCR3 = REAL_CCR-1;
		Para_Identificaion++;
		if(Para_Identificaion>=0)
		{
			Para_Identificaion = 0;
			if(index_I_phase<100)
			{
				I_phase[0][index_I_phase] = -g_IA_Raw;
				I_phase[1][index_I_phase] = -g_IB_Raw;
				I_phase[2][index_I_phase] = -g_IC_Raw;		
			}
			index_I_phase++;
		}
		if(motor_para_identification_count_Tms > 200)//1s��������
		{	
			T_t = 0.000040/(log(-1.0*(double)I_phase[0][99]/((double)(I_phase[0][2] - I_phase[0][99])))   );
			T_t_1 = -1.0/T_t;	
			g_CmdMap[SYS_MOT_TEST] = 0;
			g_CmdMap[MOT_RES] = 2*(12000*1000)/(3*I_phase[0][99]);// ����裺0.5*(U_Dc*0.866*Duty/g_IA);��������������
			g_CmdMap[MOT_INDUC] = (u16)(1000.0*T_t*(double)g_CmdMap[MOT_RES]);
		}
	}	 
  TIMx->CCR3 = CCR3;
  TIMx->CCR2 = CCR2;
  TIMx->CCR1 = CCR1;
	//TIMx->CCER = (u16)0x5FFF;
	TIMx->CCER = (u16)PWM_OUTPUT_REG;
}

extern volatile int g_high_fre_pulse;

// ��������20KHz
static void CurrentPID(void)
{            
  s32 temp32 = 0;       
  s32 X, Y, Z;
	static u16 test = 0;
	s32 abs_g_Vq;
	s16 set_current = 0;
	static int inr_angle =0;
	
	if (g_CmdMap[TAG_WORK_MODE] != MODE_CURRENT)  // ��ǰģʽ���ǵ�����
	{
		if(g_CmdMap[TAG_WORK_MODE] == MODE_SPEED || g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
		{
		//	g_CmdMap[CMD_CUR_FWD_PU] = 1*((s32)g_CmdMap[CMD_ACC_PRE_PU]*5)>>4;
			g_CmdMap[CMD_CUR_FWD_PU] = 1*(s32)g_CmdMap[CMD_ACC_PRE_PU];
			g_CmdMap[CMD_CUR_SET_PU] = (s32)pid_spd.out + (s32)g_CmdMap[CMD_CUR_FWD_PU];//ǰ������
		}
		else if(g_CmdMap[TAG_WORK_MODE] == MODE_FORCE)
		{
//			g_CmdMap[CMD_CUR_SET_PU] = pid_force.out;
			g_CmdMap[CMD_CUR_SET_PU] = (s32)pid_spd.out ;
		
		}
		else
		{
			g_CmdMap[CMD_CUR_SET_PU] = 0;
		}	
	}
		//������������ԣ��ܲ��������ƣ�ֻ����1msΪ��λ
	if(g_CmdMap[SYS_SELF_TUNING] == 1)//
	{
		g_CmdMap[TAG_MOTOR_ENABLE] = 1;
		g_CmdMap[TAG_WORK_MODE] = MODE_CURRENT;
		if(test++ > (50))
		{
			test = 0;
		}
		if(test>(25))
		{
			g_CmdMap[CMD_CUR_SET_PU] = 16*200;
		}
		else
		{
			g_CmdMap[CMD_CUR_SET_PU] = -16*200;		
		}	
	}
	else if(g_CmdMap[SYS_SELF_TUNING] == 3)//����ת������
	{
		g_CmdMap[TAG_MOTOR_ENABLE] = 1;
		g_CmdMap[TAG_WORK_MODE] = MODE_CURRENT;
		if(flag_processing_Jtest == 0)
		{
			state_Jtest = 0;
			g_CmdMap[CMD_JTEST_TIME1] = 0;
			g_CmdMap[CMD_JTEST_TIME2] = 0;
		}
		switch(state_Jtest)
		{
			case 0:
			{	
				flag_processing_Jtest = 1;
				g_CmdMap[CMD_CUR_SET_PU] = 8192;
				if(g_CmdMap[CMD_SPD_ACT_PU]>819)
				{
					g_CmdMap[CMD_JTEST_TIME1] = g_CmdMap[CMD_JTEST_TIME1] + SVPWM_PERIOD_US;					
				}
				if(g_CmdMap[CMD_SPD_ACT_PU]>(8192+819))
				{
					state_Jtest++;
					g_CmdMap[CMD_JTEST_TIME2] = 0;
				}
				break;
			}
			case 1:
			{	
				g_CmdMap[CMD_CUR_SET_PU] = -8192;
				if(g_CmdMap[CMD_SPD_ACT_PU]<8192+819)
				{
					g_CmdMap[CMD_JTEST_TIME2] = g_CmdMap[CMD_JTEST_TIME2] + SVPWM_PERIOD_US;					
				}
				if(g_CmdMap[CMD_SPD_ACT_PU]<819)
				{
					state_Jtest++;
				}
				break;
			}
			case 2:
			{	
				if(g_CmdMap[CMD_SPD_ACT_PU]<819)
				{
					state_Jtest++;
				}				
				break;
			}
			default:
			{	
				g_CmdMap[SYS_SELF_TUNING] = 0;
				g_CmdMap[CMD_CUR_SET_PU] = 0;
				flag_processing_Jtest = 0;
				break;
			}
		}
	}
		else if(g_CmdMap[SYS_SELF_TUNING] == 4)//
	{
		g_CmdMap[TAG_MOTOR_ENABLE] = 1;
		g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;

		
		test++;
		if(test < (10*10))
		{
			g_CmdMap[CMD_SPD_SET_PU] = 0;
		}
		else if(test < (10*60))
		{
			g_CmdMap[CMD_SPD_SET_PU] = 4096;
		}
		else if(test < (10*110))
		{
			g_CmdMap[CMD_SPD_SET_PU] = 0;
		}
		else if(test < (10*160))
		{
			g_CmdMap[CMD_SPD_SET_PU] = -4096;
		}
		else if(test < (10*170))
		{
			g_CmdMap[CMD_SPD_SET_PU] = 0;
		}
		else
		{
			test = 0;
			g_CmdMap[SYS_SELF_TUNING] = 0;
			g_CmdMap[CMD_SPD_SET_PU] = 0;
		}
	}
	else
	{

		if(g_CmdMap[CMD_CUR_SET_PU]> g_CmdMap[SYS_CUR_UPPER_LIMIT_PU])
		{
			g_CmdMap[CMD_CUR_SET_PU] = g_CmdMap[SYS_CUR_UPPER_LIMIT_PU];
		}
		if(g_CmdMap[CMD_CUR_SET_PU]< g_CmdMap[SYS_CUR_LOWER_LIMIT_PU])
		{
			g_CmdMap[CMD_CUR_SET_PU] = g_CmdMap[SYS_CUR_LOWER_LIMIT_PU];
		}
	}
	
	float Ierr_d = 0.f - (float)g_CmdMap[CMD_CUR_D_ACT_PU]/1000.0f;
    float Ierr_q = (float)g_CmdMap[CMD_CUR_SET_PU]/1000.0f - (float)g_CmdMap[CMD_CUR_ACT_PU]/1000.f;
	float Vd = motor_.current_control_.v_current_control_integral_d + Ierr_d * motor_.current_control_.p_gain;
    float Vq = motor_.current_control_.v_current_control_integral_q + Ierr_q * motor_.current_control_.p_gain;

    float mod_to_V = one_by_sqrt3 * bus_voltage_;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

	float mod_scalefactor =  0.9f / sqrtf(mod_d * mod_d + mod_q * mod_q);
	if(mod_scalefactor < 1.0f)
	{
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
		motor_.current_control_.v_current_control_integral_d *= 0.99f;
        motor_.current_control_.v_current_control_integral_q *= 0.99f;
	}`
	else
	{
		motor_.current_control_.v_current_control_integral_d += Ierr_d * (motor_.current_control_.i_gain * current_meas_period);
        motor_.current_control_.v_current_control_integral_q += Ierr_q * (motor_.current_control_.i_gain * current_meas_period);
	}

	pid_IQ.Up_limit = 16384;
	pid_IQ.Low_limit = -16384;
		
	pid_ID.Up_limit = 16384;
	pid_ID.Low_limit = -16384;	
	
	PID_control(g_CmdMap[CMD_CUR_SET_PU],g_CmdMap[CMD_CUR_ACT_PU],&pid_IQ);	
	PID_control(-0,g_CmdMap[CMD_CUR_D_ACT_PU],&pid_ID);	
	

	

	
	
	g_Vq = pid_IQ.out*N97_CCR/PU_REFERENCE;
	g_Vd = pid_ID.out*N97_CCR/PU_REFERENCE;	
	
//	g_Vq = 100;//g_high_fre_pulse * 500;
//	g_Vd = g_high_fre_pulse * 1000;
	
	g_high_fre_pulse = 0 - g_high_fre_pulse;
	
//	inr_angle += 1;
//	g_ElectricAngle = inr_angle;
	
  // Park���任
//	
//		g_ElectricAngle_add = (int)pll_estimated_spd /100000;
//		g_ElectricAngle = g_ElectricAngle + g_ElectricAngle_add;// * 0.000025;
	
	X = Math_Sin_EN360( g_ElectricAngle  );
	Y = Math_Cos_EN360( g_ElectricAngle );
	g_V_Alpha = Q15_MUL(g_Vd,X) + Q15_MUL(g_Vq,Y);  // ��Ƕȶ���ΪQ����Alpha��ļн�
	g_V_Beta = Q15_MUL(-g_Vd,Y) + Q15_MUL(g_Vq,X);

	update_current_control(&motor_, (float)bus_voltage_*one_by_sqrt3*g_V_Alpha/REAL_CCR, (float)bus_voltage_*one_by_sqrt3*g_V_Beta/REAL_CCR);
	// SVPWMռ�ձȼ������
	X = g_V_Beta;
	temp32 = Q16_MUL(g_V_Alpha,113512);
	Y = (g_V_Beta + temp32) >> 1;
	Z = (g_V_Beta - temp32) >> 1;  
  // �����ѹʸ�����ڵ�����
  temp32 = 0;
	/** ������ת**/	
	if (X > 0) temp32 = 1;
	if (Z < 0) temp32 += 2;
	if (Y < 0) temp32 += 4;	
	g_Sector = temp32;
	g_Sector_test = g_Sector;
	g_X = X;
	g_Y = Y;
	g_Z = Z;
  Calculate_CCR_SVP(TIM1, g_Sector, g_X, g_Y, g_Z);

}
static void SpeedPID(void)
{ 
	static s32 s_test_speed_t_count = 0;
	s16 spd_test = 0;
	
  if (g_CmdMap[TAG_WORK_MODE] == MODE_CURRENT ||g_CmdMap[TAG_WORK_MODE] == MODE_POSITION||g_CmdMap[TAG_WORK_MODE] ==MODE_POSTION_NOCURRENT)
  {
			g_CmdMap[CMD_SPDREG_REF_PU] = g_CmdMap[CMD_PROFILE_SPD_PU] + g_CmdMap[CMD_SPD_ADD_PU];
  }
	else if( g_CmdMap[TAG_WORK_MODE] == MODE_FORCE )
	{
		g_CmdMap[CMD_SPDREG_REF_PU] = g_CmdMap[CMD_SPD_ADD_PU];
		
		pid_spd.enable =1;
	}
  else if(g_CmdMap[TAG_WORK_MODE] == MODE_SPEED)
  {
			if(status_Mode_POSITION_FORCE == 2) //�������Ӵ������еĵ����ٶȿ���
			{
				if(g_CmdMap[CMD_FORCE_DIR] == 0)
				{
					force_set = g_CmdMap[CMD_FORCE_SET_PU];
					force_act = g_CmdMap[CMD_FORCE_ACT_RAW_PU];
				}
				else
				{
					force_set = -1*g_CmdMap[CMD_FORCE_SET_PU];
					force_act = -1*g_CmdMap[CMD_FORCE_ACT_RAW_PU];
				}
				if(force_set >= 0 )
				{
					if(force_act>force_set || g_CmdMap[CMD_POS_ACT_PU] > g_CmdMap[SYS_POS_UPPER_LIMIT_PU])
					{
						status_Mode_POSITION_FORCE = 3;
						g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;	
						force_PID_out(g_CmdMap[CMD_CUR_SET_PU],&pid_force);
					
					}
					else
					{
						g_CmdMap[CMD_SPDREG_REF_PU] = g_CmdMap[CMD_SPD_SOFT_SET_PU];
					}
				}
				else
				{
					if(force_act<force_set || g_CmdMap[CMD_POS_ACT_PU] < g_CmdMap[SYS_POS_LOWER_LIMIT_PU])
					{
						status_Mode_POSITION_FORCE = 3;
						g_CmdMap[TAG_WORK_MODE] = MODE_FORCE;
						force_PID_out(g_CmdMap[CMD_CUR_SET_PU],&pid_force); 
 		
					}
					else
					{
						g_CmdMap[CMD_SPDREG_REF_PU] = -1*g_CmdMap[CMD_SPD_SOFT_SET_PU];
					}
				}
			}
			else
			{
							if(g_CmdMap[SYS_SPEED_TEST] == 0)//�������ٶ���������
							{
								g_CmdMap[CMD_SPDREG_REF_PU] = g_CmdMap[CMD_SPD_SET_PU];
							}
							else if(g_CmdMap[SYS_SPEED_TEST] == 1)//�������ٶ��ڷ�Χ����������
							{
								
								if(g_CmdMap[CMD_POS_ACT_PU] < 1638)
								{
									Test_dirt = 1;					
								}
								else if(g_CmdMap[CMD_POS_ACT_PU] > (16384-1638))
								{
									Test_dirt = -1;					
								}
								g_CmdMap[CMD_SPDREG_REF_PU] = Test_dirt*g_CmdMap[CMD_SPD_SET_PU];

							}	
							else if(g_CmdMap[SYS_SPEED_TEST] == 11)//�������ٶ��ڷ�Χ����������
							{
								if(g_CmdMap[CMD_POS_ADC_12BIT] < g_CmdMap[POS_LINEARITY_ADC(0)])
								{
									Test_dirt = 1;
								}
								else if(g_CmdMap[CMD_POS_ADC_12BIT] > g_CmdMap[POS_LINEARITY_ADC(30)])
								{
									Test_dirt = -1;
								}
								spd_test = ABS(g_CmdMap[CMD_SPD_SET_PU]);
								if(s_test_speed_t_count++>4000)
								{
									s_test_speed_t_count = 0;
									if(g_flag_monitor_from_host == 0)
									{
										Test_perm = 0;   //4000����������λ����ͨѶ
									}
									else
									{
										Test_perm = 1;//4000����������λ����ͨѶ
									}
									g_flag_monitor_from_host = 0;
								}
												if(Test_perm == 1)
								{
									g_CmdMap[CMD_SPDREG_REF_PU] = Test_dirt*spd_test;
								}
								else
								{
									g_CmdMap[CMD_SPDREG_REF_PU] = 0;
								}				
							}				
							else if(g_CmdMap[SYS_SPEED_TEST] == 12)//�������ٶ����е���Сλ��
							{
								if(g_CmdMap[CMD_POS_ADC_12BIT] >g_CmdMap[POS_LINEARITY_ADC(0)])
								{
									g_CmdMap[CMD_SPDREG_REF_PU] = -8192;
								}
								else
								{
									g_CmdMap[CMD_SPDREG_REF_PU] = 0;
								}
							}
							else if(g_CmdMap[SYS_SPEED_TEST] == 13)//�������ٶ����е����λ��
							{
								if(g_CmdMap[CMD_POS_ADC_12BIT] < g_CmdMap[POS_LINEARITY_ADC(30)])
								{
									g_CmdMap[CMD_SPDREG_REF_PU] = 8192;
								}
								else
								{
									g_CmdMap[CMD_SPDREG_REF_PU] = 0;
								}
							}
							else
							{
								g_CmdMap[CMD_SPDREG_REF_PU] = 0;
							}				
			}
	}
	if(g_CmdMap[CMD_SPDREG_REF_PU]> g_CmdMap[SYS_SPD_UPPER_LIMIT_PU])
	{
		g_CmdMap[CMD_SPDREG_REF_PU] = g_CmdMap[SYS_SPD_UPPER_LIMIT_PU];
	}
	if(g_CmdMap[CMD_SPDREG_REF_PU]< g_CmdMap[SYS_SPD_LOWER_LIMIT_PU])
	{
		g_CmdMap[CMD_SPDREG_REF_PU] = g_CmdMap[SYS_SPD_LOWER_LIMIT_PU];
	}


	
	if(g_CmdMap[CMD_CUR_UPPER_LIMIT_PU] > g_CmdMap[SYS_CUR_UPPER_LIMIT_PU])
	{
		g_CmdMap[CMD_CUR_UPPER_LIMIT_PU] = g_CmdMap[SYS_CUR_UPPER_LIMIT_PU];
	}
	if(g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] < g_CmdMap[SYS_CUR_LOWER_LIMIT_PU])
	{
		g_CmdMap[CMD_CUR_LOWER_LIMIT_PU] = g_CmdMap[SYS_CUR_LOWER_LIMIT_PU];
	}
	
	pid_spd.Up_limit = g_CmdMap[CMD_CUR_UPPER_LIMIT_PU];
	pid_spd.Low_limit = g_CmdMap[CMD_CUR_LOWER_LIMIT_PU];		
	
	
	
	
	
//	
//	pid_spd.Up_limit = 1000;
//	pid_spd.Low_limit = -1000;		
//	
	PID_control(g_CmdMap[CMD_SPDREG_REF_PU],g_CmdMap[CMD_SPD_ACT_PU],&pid_spd);	
}
void PositionPID(void)   //�Ž����� λ�õ�����ϲ��η�����ʹ��
{	
	static s16 delay_cnt = 0;
	pos_error_to_target = g_CmdMap[CMD_POS_SET_PU]-g_CmdMap[CMD_POS_ACT_PU];  // ƫ��
	//����λ���ж�
	if(ABS(pos_error_to_target)<gPos_ds && Profile_moving == 0)
	{
		if(delay_cnt <50)
		{
			delay_cnt++;
		}
		else
		{
			PosAtDead_Flag = 1;
		}
	}
	if(ABS(pos_error_to_target)>2*gPos_ds || Profile_moving == 1)
	{
		PosAtDead_Flag = 0;
		delay_cnt = 0;
	}	
	PosAtDead_Flag_Last = PosAtDead_Flag;
  // ��������  
	// ���ּ���	
	if(flag_posReg_strengthen == 1)
	{
		pid_pos.Up_limit = g_CmdMap[SYS_SPD_ADD_LIMIT_PU];
		pid_pos.Low_limit = -g_CmdMap[SYS_SPD_ADD_LIMIT_PU];	
	}
	else
	{
		pid_pos.Up_limit = g_CmdMap[SYS_SPD_ADD_LIMIT_PU]>>5;
		pid_pos.Low_limit = -(g_CmdMap[SYS_SPD_ADD_LIMIT_PU]>>5);	
	}		
	PID_control(g_CmdMap[CMD_PROFILE_POS_PU],g_CmdMap[CMD_POS_ACT_PU],&pid_pos);
	g_CmdMap[CMD_SPD_ADD_PU] = pid_pos.out;
}
//int16_t Force_set_drop_detection = 0;
//λ�ÿ���
void Gripper_control(void)
{
	s32 error_Pos = g_CmdMap[CMD_POS_SET_PU] - g_CmdMap[CMD_POS_ACT_PU];
	if(	(g_CmdMap[TAG_WORK_MODE] != MODE_POSITION) || Motor_forbidden == 1)
	{
		Gripper_Move = 0x00;
		Grriper_Run = 0;
		Flag_contact = 0;
		Flag_grip_F_OK = 0;
		Flag_grip_P_OK = 0;
		Flag_rellease_P_OK = 0;
	}	
	else
	{
		
//		if(g_CmdMap[CMD_POS_SET_PU]>g_CmdMap[CMD_POS_UPPER_LIMIT_PU])
//		{
//			g_CmdMap[CMD_POS_SET_PU] =g_CmdMap[CMD_POS_UPPER_LIMIT_PU];		
//			if(g_CmdMap[CMD_POS_SET_PU] > 16384)
//			{
//				g_CmdMap[CMD_POS_SET_PU] = 16384;
//			}
//		}
//		if(g_CmdMap[CMD_POS_SET_PU]<g_CmdMap[CMD_POS_LOWER_LIMIT_PU])
//		{
//			g_CmdMap[CMD_POS_SET_PU] = g_CmdMap[CMD_POS_LOWER_LIMIT_PU];
//			if(g_CmdMap[CMD_POS_SET_PU] < 0)
//			{
//				g_CmdMap[CMD_POS_SET_PU] = 0;
//			}
//		}
		
		
		
		if(Flag_Set_OpenLen == 1  && (error_Pos>g_CmdMap[SEV_POSITION_DS]||error_Pos<-g_CmdMap[SEV_POSITION_DS]) ) //&& Gripper_Move == 0
		{
				Flag_Set_OpenLen = 0;
			
				if(g_CmdMap[CMD_E_STOP] == 0)
				{
				
				}
				
				g_CmdMap[TAG_WORK_MODE] = MODE_POSITION;	
				Profile_ParaSet();
				if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_1_SERVO)
				{
					tCnt_f = 0.0;
				}
				
				Flag_contact = 0;
				Flag_grip_F_OK = 0;
				Flag_grip_P_OK = 0;
				Flag_rellease_P_OK = 0;
				if(g_CmdMap[CMD_POS_SET_PU] < g_CmdMap[CMD_POS_ACT_PU])
				{
					Gripper_Move = 0x01;//��ȡ
					Grriper_Run = 1;
					pid_spd.enable = 1;
					pid_pos.enable = 1;
					if(g_CmdMap[CMD_E_STOP] == 0 && (g_CmdMap[CMD_ERROR] & 0x0fff) == 0x0000)
					{
						g_CmdMap[TAG_MOTOR_ENABLE] = 1;
					}
					//g_Flag_drop_detection = 1;   // 1-���ڵ����� 0-������ر�
					Grriper_Run_last = 0;
				}
				else if(g_CmdMap[CMD_POS_SET_PU] > g_CmdMap[CMD_POS_ACT_PU])
				{
					Gripper_Move = 0x02;//�ɿ�
					Grriper_Run = 1;
					pid_spd.enable = 1;
					pid_pos.enable = 1;
					if(g_CmdMap[CMD_E_STOP] == 0 && (g_CmdMap[CMD_ERROR] & 0x0fff) == 0x0000)
					{
						g_CmdMap[TAG_MOTOR_ENABLE] = 1;
					}
					Grriper_Run_last = 0;
				}
				else
				{
				}
		}
		g_pos_tag = g_CmdMap[CMD_POS_SET_PU];

		if(Gripper_Move == 1)
		{
			Flag_grip_F_OK = 1;
		}
		if(Grriper_Run == 1 && g_CmdMap[TAG_MOTOR_ENABLE] == 1)
		{		
			Flag_Profile_Enable = 1;
		}
		else
		{
			Flag_Profile_Enable = 0;
		}
		//���η���
		Profile();
		//λ�õ���
		PositionPID();	
	}
}
	float v = 11.84;
	float L,temp_L;

extern volatile  int delta_alpha;
extern volatile  int delata_beta;

int observer_ElectricAngle;
int observer_speed;
int integre_observer_ElectricAngle;

void hfi_update_angle(void)
{
	int X,Y;
	int temp_x;
	int temp_y;
	int pll_intput;
	int temp_q,temp_d;
	static float inr_angle = 0;
	int  observer_ElectricAngle1;
	
	pid_pll.dead_zone = 0;
	pid_pll.Kp = 1000;
	pid_pll.Ki = 10;
	pid_pll.Kd = 0;
	pid_pll.frequency = F_CUR_REGULATOR_HZ;
	pid_pll.enable =1;
	pid_pll.Up_limit = 0x7FFFFFFF;
	pid_pll.Low_limit = -0x7FFFFFFF;
	
	
	park_transform(delta_alpha, delata_beta, &temp_q, &temp_d,g_ElectricAngle);
	
	L = v *1.7321*500*1000 /(g_Iq * F_CUR_REGULATOR_HZ/1000 ) /3 /MAX_CCR*0.58*0.7; //0.7 ϵ���� ��ô����
	temp_L = v *1.7321*500*1000 /(temp_d * F_CUR_REGULATOR_HZ/1000 ) /3/MAX_CCR*0.58*0.7;
	
	X = Math_Sin_EN360(observer_ElectricAngle);
  Y = Math_Cos_EN360(observer_ElectricAngle);
	
	temp_x = Q15_MUL(delta_alpha,X);
	temp_y = Q15_MUL(delata_beta,Y);
	
	pll_intput = temp_x - temp_y;  //+ 90�� ���� x -y ? ,y -x ��
	
	PID_control(0,pll_intput,&pid_pll);
	observer_speed = pid_pll.out;
	
	integre_observer_ElectricAngle +=observer_speed; //update angle
	
	observer_ElectricAngle = integre_observer_ElectricAngle /100;
	
	observer_ElectricAngle  &= 0x7fff; 
	
	observer_ElectricAngle1= observer_ElectricAngle  + EN_90;// EN_90;//EN_90; 
	observer_ElectricAngle1  &= 0x7fff; 
	
	g_ElectricAngle = observer_ElectricAngle1;
	
#if 0 	 //������ת��ų�
	inr_angle  += 0.1;
	if(inr_angle  > 32768 )
		inr_angle = 0;
	g_ElectricAngle = 0;
#endif
	
}


static void OpenLoop(void)
{
	int X,Y;
	int temp_x;
	int temp_y;
	int pll_intput;
	
  s16 temp32 = g_CmdMap[TAG_OPEN_PWM] * MAX_CCR / 100;  // ��������ת�����򣬼���CCR
  g_Timer1CCR = (ABS(temp32) <= MAX_CCR) ? temp32 : g_Timer1CCR;  // ��ʱ�����յ����ռ�ձ�
	
	g_Vq =  g_CmdMap[TAG_OPEN_PWM] * 16384 / 100;  
	g_Vd = 0;

	X = Math_Sin_EN360( g_ElectricAngle  );
	Y = Math_Cos_EN360( g_ElectricAngle );
	g_V_Alpha = Q15_MUL(g_Vd,X) + Q15_MUL(g_Vq,Y);  // ��Ƕȶ���ΪQ����Alpha��ļн�
	g_V_Beta = Q15_MUL(-g_Vd,Y) + Q15_MUL(g_Vq,X);

	update_current_control(&motor_, (float)bus_voltage_*one_by_sqrt3*g_V_Alpha/REAL_CCR, (float)bus_voltage_*one_by_sqrt3*g_V_Beta/REAL_CCR);

//	g_ElectricAngle &= 0x7fff;
//	if(observer_speed < 0)
//	{
//		g_ElectricAngle = 0 - g_ElectricAngle;
//	}
//	g_ElectricAngle += 32768;
//	g_ElectricAngle &= 0x7fff;
	anlge_reg = g_ElectricAngle;//observer_ElectricAngle;
	
	if(anlge_reg < EN_0)
	{
		anlge_reg = anlge_reg+EN_360;
	}
	else if(anlge_reg > EN_360)
	{
		anlge_reg = anlge_reg-EN_360;
	}
  PWM_OutPut_Open(TIM1, anlge_reg, g_Timer1CCR);
}
// �ջ�ģʽ���
//static void CheckModeChange(void)
//{
//  static s32 s_OldMode = 0;
//  if (s_OldMode != g_CmdMap[TAG_WORK_MODE])
//  {
//    if(g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
//		{
//			g_CmdMap[CMD_SPD_SET_PU] = 16384;
//		}
//		else
//		{
//			g_CmdMap[CMD_SPD_SET_PU] = 0;
//		}
//    g_CmdMap[CMD_CUR_SET_PU] = 0;
//  }
//  s_OldMode = g_CmdMap[TAG_WORK_MODE];
//}

 volatile s32 s_Disable_Delay = 0;

static void Pos_Spd_PID(void)
{
	static s32 s_count = 0; 	// ʹ��H�ź����ʱʱ��
  static s32 s_Enable_Delay = 0; 	// ʹ��H�ź����ʱʱ��
 GPIOA->BRR  |= (uint32_t)GPIO_PIN_4;
	//CheckModeChange();
	MTPro();
	if(g_servo_cmd_period_cnt<2000)
	{
		g_servo_cmd_period_cnt++; //��Ƶλ���ŷ��£��������ڼ�ʱ
	}
	get_average_force_data();	
	
	if(g_CmdMap[TAG_WORK_MODE] == MODE_FORCE)
	{
		ForcePID();
	}
	
	
	Loop3_EnableManage();
	if(++s_count>=(F_SPD_REGULATOR_HZ/F_POS_REGULATOR_HZ))
	{
		s_count = 0;
		Flag_1ms = 1;
		if(++s_count_50ms>50)
		{
			s_count_50ms = 0;
			Flag_50ms = 1;
		}
		
		
		
		Get_Pos_Rod();
		Stroke_protection();
		
//		if(g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
		{
			Gripper_control();	
		}
	}
	
	icmu_sdtransmission_test();
	
	
	if(g_CmdMap[TAG_WORK_MODE] == MODE_POSITION || g_CmdMap[TAG_WORK_MODE] == MODE_SPEED  ||  g_CmdMap[TAG_WORK_MODE] == MODE_FORCE )
	{
		SpeedPID();
	}	
  // H�Ŵӽ��ܵ�ʹ��ʱ����ʱ���ȴ��̵�������������ֹ��������������ɵĳ��
  if(g_CmdMap[TAG_MOTOR_ENABLE] == 0 || Motor_forbidden == 1 || Motor_emergercy_stop == 1 || Motor_pause_flag == 1)
	{
		s_Enable_Delay = 0;
		Motor_forbidden = 0;
		if(g_CmdMap[TAG_WORK_MODE] == MODE_SPEED ||g_CmdMap[TAG_WORK_MODE] == MODE_CURRENT )
		{
			g_CmdMap[CMD_SPD_SET_PU] = 0;
			g_CmdMap[CMD_CUR_SET_PU] = 0;
		}
		if(g_CmdMap[CMD_ERROR] != 0)
		{	
			g_CmdMap[CMD_POS_SET_PU] = g_CmdMap[CMD_POS_ACT_PU];
		}			
//		if(s_Disable_Delay <= 1) s_Disable_Delay ++;
//		if (s_Disable_Delay == 1) // ��ʹ�ܱ�Ϊ���� 50ms
		{
//			s_Disable_Delay = 0;
			g_Timer1CCR = 0;
      Motor_emergercy_stop =0;
			Motor_pause_flag = 0;
			PWM_Disable();
		}
//		if (s_Disable_Delay > 1)
//		{
//			s_Disable_Delay =2;
//		}
		g_Vq = 0;
		return;
	}
	if(g_CmdMap[TAG_MOTOR_ENABLE] == 1 && Motor_forbidden == 0)
	{
		if(s_Enable_Delay <= 2) s_Enable_Delay ++;
		s_Disable_Delay = 0;
		if (s_Enable_Delay == 1)  // �ӽ��ܱ�Ϊʹ�� 50ms
		{
			if(g_CmdMap[TAG_WORK_MODE] == MODE_SPEED ||g_CmdMap[TAG_WORK_MODE] == MODE_CURRENT )
			{
				g_CmdMap[CMD_SPD_SET_PU] = 0;
				g_CmdMap[CMD_CUR_SET_PU] = 0;
			}
			g_Timer1CCR = 0;		
			PWM_Enable();			
		}
		return;
	}  
}
s32 I_Alpha, I_Beta; 
extern 	volatile  int hall_ture_value;
extern 	volatile  int last_hall_ture_value;
extern volatile  s32 sample_invered;

volatile  int delta_alpha;
volatile  int delata_beta;
int last_alpha = 0;
int last_beta = 0;

float phase_current_rev_gain_ = 1.0f/16.5f;
float shunt_conductance = 1.0f/0.01f;
float phase_current_from_adcval(s32 adcval_bal)
{
  float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
  float shunt_volt = amp_out_volt * phase_current_rev_gain_;
  float current = shunt_volt * shunt_conductance;
  return current ;
}
void GetCurrent(void)
{
  s32 i=0, j=0,I_B1 = 0,I_C1 = 0;


	
	g_IC_Raw = (s16)(-phase_current_from_adcval(ADC1->JDR1-g_ZeroCur_MotorC)*1000.0f); // mA	
	g_IB_Raw = (s16)(-phase_current_from_adcval(ADC2->JDR1-g_ZeroCur_MotorB)*1000.0f); // mA
	g_IA_Raw = (s16)(-phase_current_from_adcval(ADC1->JDR1-g_ZeroCur_MotorA)*1000.0f); // mA
	
	g_IA = g_IA_Raw;
	g_IB = g_IB_Raw;
	g_IC = 0 - g_IA_Raw -g_IB_Raw;
	
	update_current_meas(&motor_, g_IA/1000.0f, g_IB/1000.0f, g_IC/1000.0f);

	I_Alpha = -(g_IB + g_IC);             // �ȷ�Clark:I_Alpha = (2/3)*(I_a - 0.5*I_b - 0.5*I_c)= Ia;
	I_Beta = Q15_MUL(g_IB - g_IC,18919);  // �ȷ�Clark:I_Beta =  (2/3)*(ib-ic)*0.5*squr(3) = (ib-ic)/squr(3) = (ib-ic)*18919/32768; 
	
	delta_alpha = I_Alpha - last_alpha;
	delata_beta = I_Beta - last_beta;
	
	delta_alpha *= g_high_fre_pulse;
	delata_beta *= g_high_fre_pulse;
	
	last_alpha = I_Alpha;
	last_beta = I_Beta;
	
	i = Math_Sin_EN360(g_ElectricAngle );
	j = Math_Cos_EN360(g_ElectricAngle );
	I_B1 = Q15_MUL(j, I_Alpha) + Q15_MUL(i, I_Beta);//Park�任:Iq = i_Alpha*cos - i_Beta*sin
	I_C1 = Q15_MUL(i, I_Alpha) - Q15_MUL(j, I_Beta);//Park�任:Id = i_Alpha*sin + i_Beta*cos
//		I_B1 = (I_B1>>3)<<3;
//		I_C1 = (I_C1>>3)<<3;

  if (g_bEnableHB != 0 && (pid_IQ.enable == 1||g_CmdMap[TAG_WORK_MODE] == MODE_OPEN))
  {
		g_Iq = I_B1;
		g_Id = I_C1;	
	}
	else
	{
		g_Iq = I_B1;
		g_Id = I_C1;	
	}
	g_CmdMap[CMD_CUR_D_ACT_PU] = g_Id*PU_REFERENCE/(*g_pCur_ref_base_mA);
	g_CmdMap[CMD_CUR_ACT_PU] = g_Iq*PU_REFERENCE/(*g_pCur_ref_base_mA);
	
	
}


void park_transform(int alpha, int beta, int*q, int*d,int ElectricAngle )
{
	int i,j;
	
	i = Math_Sin_EN360(ElectricAngle);
	j = Math_Cos_EN360(ElectricAngle);
	*q = Q15_MUL(j, I_Alpha) + Q15_MUL(i, I_Beta);//Park�任:Iq = i_Alpha*cos - i_Beta*sin
	*d = Q15_MUL(i, I_Alpha) - Q15_MUL(j, I_Beta);//Park�任:Id = i_Alpha*sin + i_Beta*cos
}

s32 ServoPro_En_count = 0;
extern volatile s32 Encoder_Inc;   							// ����������ֵ�仯��
extern volatile s32 s_SpeedTimeAdd1;   					// ����������ֵ�仯��
extern volatile s32 start_speed_sample;
 volatile  int pid_hall_ture_value1;
  volatile  int pid_last_hall_ture_value1;
 volatile int last_hall_sample_time = 0;
 volatile int speed1,speed2,speed3;

extern volatile int time4_capture_ch1;

int pid_hall_sample_time_point[10];
int pid_hall_sample_time_index = 0;
volatile int last_time4_capture_ch1 = 0;

volatile int hall_speed_sample_timeout_cnt=0; //���㵽30ms
volatile int  hall_speed_sample_cnt = 0;
volatile int hall_speed_sample_timeout =0;
volatile int hall_cnt = 0;

volatile int hall_now_time = 0;
volatile int hall_previous_1_time = 0;
volatile int hall_previous_2_time = 0;
volatile int hall_previous_3_time = 0;
int64_t hall_total_time;

volatile int64_t Vel_PPS_raw1 =0;

void ServoPro_Fast(void)
{
	int64_t s_SpeedTimeAdd11=0;

	
	int64_t speed;

	
	static u8 PlusPosition_flag = 0;
  static s32 s_count = 0;
  static int s_hal_cnt=0;
	EXTI_HandleTypeDef EXTI_Handle;
	
GPIOA->BSRR  |= (uint32_t)GPIO_PIN_15;
	
//	GPIO_WriteBit(GPIOA,GPIO_Pin_15,0X01);
	//---------------------------------���´���20KHz(50us)ִ��-----------------
//	if(g_BeginPIDFlag == 1)
	{
		GetElectricAngle();//��ȡ��Ƕ�
		GetCurrent();   // ������� 	
		
//		hfi_update_angle();
		
	}
	

	measure_R();
	if(g_bEnableHB) 																											// ������ or ����
	{
		if(g_CmdMap[TAG_WORK_MODE] == MODE_OPEN)
		{
			OpenLoop();
		}
		else
		{
			CurrentPID();
		}
		motor_fault_check(); //¼ì²éµç»úÊÇ·ñÄÜÕý³£¹¤×÷
	}
	
	GPIOA->BRR |= (uint32_t)GPIO_PIN_15;
	Pll_phase(  ( (s16)g_ElectricAngle_15bit  ), &pll_estimated_phase,    &pll_estimated_spd  );


	PlusPosition_flag ^= 0x1;

	non_linear_flux_observer();
	s_count ++;
	if(s_count >= (F_CUR_REGULATOR_HZ/F_SPD_REGULATOR_HZ))
	{	
		s_count = 0;    		
		EXTI->SWIER1 |= 0x08;
		EXTI_Handle.Line = 3;
		HAL_EXTI_GenerateSWI(&EXTI_Handle);
	} 

//	GPIO_WriteBit(GPIOA,GPIO_Pin_15,0X00);
//	GPIO_WriteBit(GPIOA,GPIO_Pin_15,0X00);
	
	SPI3->DR =0X0000;
	
}
void Loop3_EnableManage(void)   //�������Ƶ�ʹ�ܹ���
{
	//ǿ�ùر��ٶ�
	if(g_CmdMap[TAG_MOTOR_ENABLE] == 1 
			&& 	(	g_CmdMap[TAG_WORK_MODE] == MODE_CURRENT	  //������
						|| g_CmdMap[TAG_WORK_MODE] == MODE_FORCE 	//���ػ�				
						|| g_CmdMap[TAG_WORK_MODE] == MODE_SPEED	//�ٶȻ�
						|| (g_CmdMap[TAG_WORK_MODE] == MODE_POSITION && PosAtDead_Flag == 0)  //λ���Ҳ������� 
					)
		)
	{	
		pid_IQ.enable = 1;
		pid_ID.enable = 1;
	}
	else
	{
		pid_IQ.enable = 0;
		pid_ID.enable = 0;
	}
	if(g_CmdMap[TAG_MOTOR_ENABLE] == 1 && g_CmdMap[TAG_WORK_MODE] == MODE_FORCE )
	{	
		pid_force.enable = 1;
	}
	else
	{
		pid_force.enable = 0;
	}	
	if(g_CmdMap[TAG_MOTOR_ENABLE] == 1 
			&& ( g_CmdMap[TAG_WORK_MODE] == MODE_SPEED	//�ٶȻ�
					|| (g_CmdMap[TAG_WORK_MODE] == MODE_POSITION && PosAtDead_Flag == 0)  //λ���Ҳ������� 
					)
		)
	{	
		pid_spd.enable = 1;
	}
	else
	{
		pid_spd.enable = 0;
	}
	if(	g_CmdMap[TAG_MOTOR_ENABLE] == 1 
			&& (g_CmdMap[TAG_WORK_MODE] == MODE_POSITION && PosAtDead_Flag == 0) //λ���Ҳ������� 
		)
	{	
		pid_pos.enable = 1;
	}
	else
	{
		pid_pos.enable = 0;
	}
}
// ���¿��Ʋ���
void work_var_updata(void)
{
  /****************************************************************************************************/
	g_Encode_offset = (int)g_CmdMap[MOT_EANGLE_OFFSET];
	if (g_CmdMap[SEV_PARAME_LOCKED] != 0)	 	// �Ƿ���������
	{
		pid_pos.Kp = g_ThreeLoopParaBefLock[(SEV_POSITION_P)&0x0F];
		pid_pos.Ki = g_ThreeLoopParaBefLock[(SEV_POSITION_I)&0x0F];
		pid_pos.Kd = 0;
		pid_pos.dead_zone = g_ThreeLoopParaBefLock[(SEV_POSITION_DS)&0x0F];
		
		pid_spd.Kp = g_ThreeLoopParaBefLock[(SEV_SPEED_P)&0x0F];
		pid_spd.Ki = g_ThreeLoopParaBefLock[(SEV_SPEED_I)&0x0F];
		pid_spd.Kd = 0;
		pid_spd.dead_zone = g_ThreeLoopParaBefLock[(SEV_SPEED_DS)&0x0F];

		pid_IQ.Kp = g_ThreeLoopParaBefLock[(SEV_CURRENT_P)&0x0F];
		pid_IQ.Ki = g_ThreeLoopParaBefLock[(SEV_CURRENT_I)&0x0F];
		pid_IQ.Kd = 0;
		pid_IQ.dead_zone = 4;	

		pid_ID.Kp = g_ThreeLoopParaBefLock[(SEV_CURRENT_P)&0x0F];
		pid_ID.Ki = g_ThreeLoopParaBefLock[(SEV_CURRENT_I)&0x0F];
		pid_ID.Kd = 0;
		pid_ID.dead_zone = 4;	

		pid_force.Kp = g_ThreeLoopParaBefLock[(SEV_FORCE_P)&0x0F];
		pid_force.Ki = g_ThreeLoopParaBefLock[(SEV_FORCE_I)&0x0F];
		pid_force.Kd = g_ThreeLoopParaBefLock[(SEV_FORCE_D)&0x0F];
		pid_force.dead_zone = g_ThreeLoopParaBefLock[(SEV_FORCE_DS)&0x0F];
		gPos_ds = g_ThreeLoopParaBefLock[(SEV_POSITION_DS)&0x0F];
	}
	else
	{
		pid_pos.Kp = g_CmdMap[SEV_POSITION_P];
		pid_pos.Ki = g_CmdMap[SEV_POSITION_I];
		pid_pos.Kd = 0;
		pid_pos.dead_zone = g_CmdMap[SEV_POSITION_DS];
		
		pid_spd.Kp = g_CmdMap[SEV_SPEED_P];
		pid_spd.Ki = g_CmdMap[SEV_SPEED_I];
		pid_spd.Kd = 0;
		pid_spd.dead_zone = g_CmdMap[SEV_SPEED_DS];

		pid_IQ.Kp = g_CmdMap[SEV_CURRENT_P];
		pid_IQ.Ki = g_CmdMap[SEV_CURRENT_I];
		pid_IQ.Kd = 0;
		pid_IQ.dead_zone = 4;	

		pid_ID.Kp = g_CmdMap[SEV_CURRENT_P];
		pid_ID.Ki = g_CmdMap[SEV_CURRENT_I];
		pid_ID.Kd = 0;
		pid_ID.dead_zone = 4;
		
		pid_force.Kp = g_CmdMap[SEV_FORCE_P];
		pid_force.Ki = g_CmdMap[SEV_FORCE_I];
		pid_force.Kd = g_CmdMap[SEV_FORCE_D];
		pid_force.dead_zone = g_CmdMap[SEV_FORCE_DS];
		gPos_ds = g_CmdMap[SEV_POSITION_DS];
	}
  /**************************************************************************/	
}


// λ�û����ٶȻ����ж� 2KHz
void EXTI3_IRQHandler(void)
{
 // if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
		//LED2_TOOGLE(); fortest
		EXTI->PR1 |= 0x0008;		// �����־
    Pos_Spd_PID();  // ����λ�û����ٶȻ�				
		//UpdateErrorStateLED(); 		
  }
}
