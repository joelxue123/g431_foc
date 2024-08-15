
#include "profile_joint.h"
#include "limit_pos_detection.h"

volatile s32 g_pos_tag = 0;
volatile s32 g_pos_ini = 0;
volatile s32 g_pos_tag_old = 0;
volatile float g_pos_tag_dlt_f = 0.0f;
volatile float g_pos_tag_dlt_abs_f = 0.0f;
volatile float dir_f = 0.0f;
volatile s32 pos_ref_um = 0;
volatile s32 pos_act_um = 0;
volatile s32 pos_interpolation_A_um = 0;
volatile s32 pos_interpolation_B_um = 0;
volatile float time_interpolation_s = 0.0;
volatile s32 vel_ref_umPs = 0;
volatile s32 vel_interpolation_umPs = 0;
volatile s32 acc_ref_umPs2 = 0;
volatile s32 V_ref = 0;
volatile s32 V_Mot_ref_RPM = 0;
volatile s32 V_Mot_AddReg_RPM = 0;
volatile s32 V_Mot_ref = 0;
volatile s32 A_ref = 0;
volatile float vel_max_f_umPs = 0.0f;
volatile float acc_set_f_umPs2 = 0.0f;
volatile float vel_tmp_f = 0.0f;
volatile float t1_tmp_f = 0.0f;
volatile float t1_f = 0.0f;
volatile float t2_f = 0.0f;
volatile float t3_f = 0.0f;
volatile float p1_f = 0.0f;
volatile float p2_f = 0.0f;
volatile float p3_f = 0.0f;
volatile float S1_f = 0.0f;
volatile float t_at_P_Dec_f = 0.0f; //到减速位置时的时间，只有在宏定义PLAN_TYPE_SET=PLAN_DYNAMIC时使用 
volatile float acc_1_sections = 0.0f;
volatile float vp_abs = 0.0f;      //两段运动时最高速度绝对值
volatile float v0 = 0.0f;          //规划开始时的规划速度
volatile float v0_abs = 0.0f;      //规划开始时的规划速度 绝对值
volatile float S_max_2_stop = 0.0f;//vmax减速到0的距离值
volatile float S_v0_2_max =   0.0f;//从v0加速到vmax的距离值
volatile float S_th1 = 0.0f;       //判断阈值1,v0减速到0的距离
volatile float S_th2 = 0.0f;       //判断阈值2,vo加速到vmax后立即减速到0的距离
volatile float S_samll_stroke = 0.0f; //小行程，一个控制周期内匀速运行的距离
volatile float S1and3_f = 0.0f;
volatile float S2_f = 0.0f;
volatile float tCnt_f = 0.0f;
volatile s32 Flag_Profile_Enable = 0;
volatile s32 Flag_Profile_Enable_last = 0;
volatile s32 Profile_mode_last= CMD_USEER_MODE_0_POSITION;
volatile s32 Profile_v_set= 0;
volatile s32 Profile_frozen_tar_pos= 0;
volatile s32 Profile_moving = 0;
volatile s16 Kaphla_Bit8_Index = 0;
volatile s16 Count_test = 0;
volatile s16 Count_test01 = 0;
volatile u8 flag_profile_start_reverse = 0;
volatile u8 flag_profile_3_sections = 0;
volatile float dis_DaHua = 0.0f;
volatile u8 flag_posReg_strengthen = 1;

 int g_flag_1= 0;

void Profile_ParaSet(void)        //参数设置_Profile不计算的时候才可以设置参数
{
	if(g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
	{
		if(g_CmdMap[CMD_SPD_SET_PU]<2)
		{
			g_CmdMap[CMD_SPD_SET_PU] = 2;
		}
		if(g_CmdMap[CMD_ACC_SET_PU]<2048)
		{
			g_CmdMap[CMD_ACC_SET_PU] = 2048;
		}
	}
	if(Profile_moving == 1)//参数设置_Profile不计算的时候才可以设置参数
	{
	//	return;
	}
	vel_max_f_umPs = 	(float)g_CmdMap[CMD_SPD_SET_PU]/(float)pos_p_vel_float;  //u/s
	acc_set_f_umPs2 =  1000.0f*(float)g_CmdMap[CMD_ACC_SET_PU]*((float)*g_pAcc_ref_base_mmPS2)/16384.0f;
	g_pos_tag_old = -100;
}

volatile s32 profile_flag =0;
void Profile(void)
{		
	static s16 ini_count = 0;
	dis_DaHua = 10; //预留保证不超调的裕量
	if(ini_count<2)
	{
		ini_count++;
		g_pos_tag_old = g_pos_tag = g_CmdMap[CMD_POS_ACT_PU];//目标值等于实际值
		pos_ref_um = g_pos_ini = g_pos_tag;
		vel_ref_umPs = 0;
		acc_ref_umPs2 = 0;
		tCnt_f = 0.0f;
		Profile_moving = 0;
		return;
	}
//	if(Motor_forbidden == 1)//如果存在系统故障，直接中断Profile功能，并初始化变量
//	{	
//		g_pos_tag_old = g_pos_tag = (int64_t)g_CmdMap[CMD_POS_ACT_PU]*(*g_pPos_ref_base_um)/16384;//目标值等于实际值
//		pos_ref_um = g_pos_ini = g_pos_tag;
//		vel_ref_umPs = 0;
//		acc_ref_umPs2 = 0;
//		V_ref = 0;
//		A_ref = 0;
//		tCnt_f = 0.0f;
//		Profile_moving = 0;
//		return;
//	}
	if(Flag_Profile_Enable == 0 && Motor_forbidden == 0)
	{
//		pos_ref_um = Profile_frozen_tar_pos;
//		vel_ref_umPs = 0;
//		acc_ref_umPs2 = 0;
//		tCnt_f = 0.0f;
//		vel_tmp_f = 0.0f;
//		t1_tmp_f = 0.0f;
		g_pos_tag_old = g_pos_tag = g_CmdMap[CMD_POS_ACT_PU];//目标值等于实际值
		pos_ref_um = g_pos_ini = g_pos_tag;
		vel_ref_umPs = 0;
		acc_ref_umPs2 = 0;
		V_ref = 0;
		A_ref = 0;
		tCnt_f = 0.0f;
		Profile_moving = 0;
	}
	//冻结命令的上升沿 将当前实际位置作为目标位置
	if(Flag_Profile_Enable_last == 1 && Flag_Profile_Enable == 0)
	{
		Profile_frozen_tar_pos = 	g_CmdMap[CMD_POS_ACT_PU];	
	}	
	if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_2_SPEED && Profile_mode_last != CMD_USEER_MODE_2_SPEED)//速度模式
	{
		tCnt_f = 0.0f;
		g_pos_ini = pos_ref_um;
	}	
	Profile_mode_last = g_CmdMap[CMD_USEER_MODE];
	//如果没有故障继续向下执行
	//使能 位置模式下 目标位置发生改变触发
	if(Flag_Profile_Enable ==1)
	{
		pos_act_um =  g_CmdMap[CMD_POS_ACT_PU];
		if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_0_POSITION || g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE)
		{
							g_CmdMap[SYS_SPD_ADD_LIMIT_PU] = 3276;			
							if(g_pos_tag_old != g_pos_tag)
							{					
												profile_flag++;
												Profile_moving = 1;
												tCnt_f = 0.0f;
												v0 = vel_ref_umPs;          //规划开始时的规划速度								
												g_pos_ini = pos_act_um;
												pos_ref_um = g_pos_ini;
												g_pos_tag_dlt_f = (float)(g_pos_tag - g_pos_ini);
												g_pos_tag_dlt_abs_f = (float)ABS(g_pos_tag - g_pos_ini);	
												if(g_pos_tag_dlt_f>0.0f)
												{
													dir_f = 1.0f;
												}
												else
												{
													dir_f = -1.0f;
												}		
												if((v0>-0.2f && g_pos_tag_dlt_f>0.0f)||(v0<0.2f && g_pos_tag_dlt_f<0.0f))//当前速度与目标方向一致
												{
													flag_profile_start_reverse = 0;
													v0_abs = fabsl(v0);
												}													
												else    //当前速度与目标方向不一致
												{
													flag_profile_start_reverse = 1;
													v0 = 0.0f;
													v0_abs = 0.0f;
												}
												S_max_2_stop = 0.5f*vel_max_f_umPs*vel_max_f_umPs/acc_set_f_umPs2;//vmax减速到0的距离值
												S_v0_2_max = 0.5f*(vel_max_f_umPs*vel_max_f_umPs-v0*v0)/acc_set_f_umPs2;
												S_th1 = 0.5f*v0*v0/acc_set_f_umPs2;
												S_th2 = S_max_2_stop+S_v0_2_max;
												S_samll_stroke = 0.001f*(float)P_POS_REGULATOR_MS*vel_max_f_umPs;
												if(g_pos_tag_dlt_abs_f>S_th2)//三段
												{
													flag_profile_3_sections = 3;
													p1_f = dir_f*S_v0_2_max + g_pos_ini;
													#if (PLAN_TYPE_SET==PLAN_STATIC)//
													p2_f = dir_f*(g_pos_tag_dlt_abs_f-S_max_2_stop) + g_pos_ini;
													#endif
													#if (PLAN_TYPE_SET==PLAN_DYNAMIC)													
													p2_f = dir_f*(g_pos_tag_dlt_abs_f-S_max_2_stop) + g_pos_ini-dir_f*dis_DaHua;
													#endif
													p3_f = g_pos_tag;
													t1_f = (vel_max_f_umPs-v0_abs)/acc_set_f_umPs2;
													t2_f = (g_pos_tag_dlt_abs_f-S_th2)/vel_max_f_umPs + t1_f;
													t3_f = vel_max_f_umPs/acc_set_f_umPs2+t2_f;
												}
												else if(g_pos_tag_dlt_abs_f>S_th1)//两段
												{
													flag_profile_3_sections = 2;
													vp_abs = sqrtl(g_pos_tag_dlt_abs_f*acc_set_f_umPs2+0.5f*v0_abs*v0_abs);
													t1_f = (vp_abs-v0_abs)/acc_set_f_umPs2;
													t2_f = vp_abs/acc_set_f_umPs2+t1_f;
													#if (PLAN_TYPE_SET==PLAN_STATIC)//
													p1_f = dir_f*(vp_abs*vp_abs - v0_abs*v0_abs)/(2.0f*acc_set_f_umPs2)+ (float)g_pos_ini;
													#endif
													#if (PLAN_TYPE_SET==PLAN_DYNAMIC)		
													p1_f = dir_f*(vp_abs*vp_abs - v0_abs*v0_abs)/(2.0f*acc_set_f_umPs2)+ (float)g_pos_ini-dir_f*dis_DaHua;
													#endif													
													p2_f = (float)g_pos_tag;
												}
												else
												{
													flag_profile_3_sections = 1;														
													t1_f = 2.0f*g_pos_tag_dlt_abs_f/v0_abs;
													acc_1_sections = 	v0_abs/t1_f;
													p1_f = (float)g_pos_tag;
												}
												//return;//2022-3-31 test
							}							
							if(Profile_moving == 1)
							{				
											tCnt_f = tCnt_f + 0.001f*(float)P_POS_REGULATOR_MS;
											if(flag_profile_3_sections == 3)//三段
											{
												#if (PLAN_TYPE_SET==PLAN_STATIC)//
												if(tCnt_f<t1_f)
												{
													pos_ref_um = (float)g_pos_ini + dir_f*(v0_abs*tCnt_f+acc_set_f_umPs2*tCnt_f*tCnt_f*0.5f);
													vel_ref_umPs = (s32)(dir_f*(acc_set_f_umPs2*tCnt_f+v0_abs));
													acc_ref_umPs2 = dir_f*acc_set_f_umPs2;
													flag_posReg_strengthen = 0;
												}
												else if(tCnt_f<t2_f)
												{
													pos_ref_um = p1_f + dir_f*vel_max_f_umPs*(tCnt_f-t1_f);
													vel_ref_umPs = dir_f*vel_max_f_umPs;
													acc_ref_umPs2 = 0;
													flag_posReg_strengthen = 0;
												}
												else if(tCnt_f<t3_f)
												{
													pos_ref_um = p2_f + dir_f*vel_max_f_umPs*(tCnt_f-t2_f)-dir_f*acc_set_f_umPs2*(tCnt_f-t2_f)*(tCnt_f-t2_f)/2;
													vel_ref_umPs = dir_f*vel_max_f_umPs-dir_f*acc_set_f_umPs2*(tCnt_f-t2_f);
													acc_ref_umPs2 = -1*dir_f*acc_set_f_umPs2;
													flag_posReg_strengthen = 1;
												}
												else
												{
													pos_ref_um = p3_f;
													vel_ref_umPs = 0;
													acc_ref_umPs2 = 0;
													Profile_moving = 0;
													tCnt_f = 0;
													flag_posReg_strengthen = 1;
													if(status_Mode_POSITION_FORCE == 1) //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
													{
														status_Mode_POSITION_FORCE = 2;
														g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;
													}
												}
												#endif
												#if (PLAN_TYPE_SET==PLAN_DYNAMIC)
													if((dir_f>0.0f && pos_act_um<(s32)p2_f) ||  (dir_f<0.0f && pos_act_um>(s32)p2_f))
													{
														if(tCnt_f<t1_f)
														{
															pos_ref_um = (float)g_pos_ini + dir_f*(v0_abs*tCnt_f+acc_set_f_umPs2*tCnt_f*tCnt_f*0.5f);
															vel_ref_umPs = (s32)(dir_f*(acc_set_f_umPs2*tCnt_f+v0_abs));
															acc_ref_umPs2 = dir_f*acc_set_f_umPs2/8000;
															flag_posReg_strengthen = 0;
														}
														else								
														{
															pos_ref_um = p1_f + dir_f*vel_max_f_umPs*(tCnt_f-t1_f);
															if(dir_f>0.0f && pos_act_um>(s32)p2_f)
															{
																pos_ref_um = p2_f;
															}
															if(dir_f<0.0f && pos_act_um<(s32)p2_f)
															{
																pos_ref_um = p2_f;
															}
															vel_ref_umPs = dir_f*vel_max_f_umPs;
															acc_ref_umPs2 = 0;
															flag_posReg_strengthen = 0;
														}
														t_at_P_Dec_f = tCnt_f;														
														t3_f = vel_max_f_umPs/acc_set_f_umPs2+t_at_P_Dec_f;
													}
													else if(tCnt_f<t3_f)
													{
														if((dir_f>0.0f && pos_act_um<((s32)p3_f -100)) ||  (dir_f<0.0f && pos_act_um>((s32)p3_f+100)))
														{
															pos_ref_um = p2_f + dir_f*vel_max_f_umPs*(tCnt_f-t_at_P_Dec_f)-dir_f*acc_set_f_umPs2*(tCnt_f-t_at_P_Dec_f)*(tCnt_f-t_at_P_Dec_f)/2;
															vel_ref_umPs = dir_f*vel_max_f_umPs-dir_f*acc_set_f_umPs2*(tCnt_f-t_at_P_Dec_f);
															acc_ref_umPs2 = -1*dir_f*acc_set_f_umPs2/8000;
															flag_posReg_strengthen = 0;
															
															if(dir_f>0)
															{
																if(vel_ref_umPs < 0)
																{
																	vel_ref_umPs =0;
																	acc_ref_umPs2 = 0;
																	tCnt_f = t3_f;
																	pid_spd.integral = 0;
																	flag_posReg_strengthen = 1;
																}
															}
															else 
															{
																if(vel_ref_umPs > 0)
																{
																	vel_ref_umPs =0;
																	acc_ref_umPs2 = 0;
																	tCnt_f = t3_f;
																	pid_spd.integral = 0;
																	flag_posReg_strengthen = 1;
																}
															}
														}
														else
														{
															pid_spd.integral = 0;
															pos_ref_um = p3_f;
															vel_ref_umPs = 0;
															acc_ref_umPs2 = 0;
														//	Profile_moving = 0;
															tCnt_f = t3_f;
															flag_posReg_strengthen = 1;
														}

													}
													else
													{
														g_flag_1= 1;
														pid_spd.integral = 0;
														pos_ref_um = p3_f;
														vel_ref_umPs = 0;
														acc_ref_umPs2 = 0;
														Profile_moving = 0;
														tCnt_f = 0;
														flag_posReg_strengthen = 1;
														if(status_Mode_POSITION_FORCE == 1) //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
														{
															status_Mode_POSITION_FORCE = 2;
															g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;

														}
													}												
												#endif
											}
											else if(flag_profile_3_sections == 2)//两段式
											{	
												#if (PLAN_TYPE_SET==PLAN_STATIC)
													if(tCnt_f<t1_f)
													{
														pos_ref_um = (float)g_pos_ini + dir_f*(v0_abs*tCnt_f+acc_set_f_umPs2*tCnt_f*tCnt_f*0.5f);
														vel_ref_umPs = (s32)(dir_f*(acc_set_f_umPs2*tCnt_f+v0_abs));
														acc_ref_umPs2 = dir_f*acc_set_f_umPs2;
														flag_posReg_strengthen = 0;
													}
													else if(tCnt_f<t2_f)																						
													{
														pos_ref_um = p1_f + dir_f*vp_abs*(tCnt_f-t1_f)-dir_f*acc_set_f_umPs2*(tCnt_f-t1_f)*(tCnt_f-t1_f)/2;
														vel_ref_umPs = dir_f*vp_abs-dir_f*acc_set_f_umPs2*(tCnt_f-t1_f);
														acc_ref_umPs2 = -1*dir_f*acc_set_f_umPs2;
														flag_posReg_strengthen = 1;
													}
													else
													{
														pos_ref_um = p2_f;
														vel_ref_umPs = 0;
														acc_ref_umPs2 = 0;
														Profile_moving = 0;
														tCnt_f = 0;
														flag_posReg_strengthen = 1;
														if(status_Mode_POSITION_FORCE == 1) //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
														{
															status_Mode_POSITION_FORCE = 2;
															g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;
 
														}
													}
												#endif
												#if (PLAN_TYPE_SET==PLAN_DYNAMIC)
													if((dir_f>0.0f && pos_act_um<(s32)p1_f) ||  (dir_f<0.0f && pos_act_um>(s32)p1_f))
													{
														if(tCnt_f<t1_f)
														{
															pos_ref_um = (float)g_pos_ini + dir_f*(v0_abs*tCnt_f+acc_set_f_umPs2*tCnt_f*tCnt_f*0.5f);
															vel_ref_umPs = (s32)(dir_f*(acc_set_f_umPs2*tCnt_f+v0_abs));
															acc_ref_umPs2 = dir_f*acc_set_f_umPs2/8000;
														}
														else								
														{
															pos_ref_um = p1_f;
															vel_ref_umPs = dir_f*vp_abs;
															acc_ref_umPs2 = 0;
														}
														t_at_P_Dec_f = tCnt_f;
														
														vp_abs= vel_ref_umPs;		
														if(vp_abs <0)
															vp_abs = 0 -vp_abs;
														
														t2_f = vp_abs/acc_set_f_umPs2+t_at_P_Dec_f;
														
														vp_abs= vel_ref_umPs;		
														if(vp_abs <0)
															vp_abs = 0 -vp_abs;
														
														flag_posReg_strengthen = 0;
													}
													else if(tCnt_f<t2_f)
													{
														if((dir_f>0.0f && pos_act_um<((s32)p2_f -100)) ||  (dir_f<0.0f && pos_act_um>((s32)p2_f+100)))
														{
																pos_ref_um = p1_f + dir_f*vp_abs*(tCnt_f-t_at_P_Dec_f)-dir_f*acc_set_f_umPs2*(tCnt_f-t_at_P_Dec_f)*(tCnt_f-t_at_P_Dec_f)/2;
																vel_ref_umPs = dir_f*vp_abs-dir_f*acc_set_f_umPs2*(tCnt_f-t_at_P_Dec_f);
																acc_ref_umPs2 = -1*dir_f*acc_set_f_umPs2/8000;
																flag_posReg_strengthen = 0;
																if(dir_f>0)
																{
																	if(vel_ref_umPs < 0)
																	{
																		vel_ref_umPs =0;
																	 acc_ref_umPs2 = 0;
																	 tCnt_f = t2_f;
																	 pid_spd.integral = 0;
																		flag_posReg_strengthen = 1;
																	}
																}
																else 
																{
																	if(vel_ref_umPs > 0)
																	{
																		vel_ref_umPs =0;
																		acc_ref_umPs2 = 0;
																		tCnt_f = t2_f;
																	  pid_spd.integral = 0;
																		flag_posReg_strengthen = 1;
																	}
																}
														}
														else
														{
															pid_spd.integral = 0;
															pos_ref_um = p2_f;
															vel_ref_umPs = 0;
															acc_ref_umPs2 = 0;
													//		Profile_moving = 0;
															tCnt_f = t2_f;
															flag_posReg_strengthen = 1;
														}

													}
													else
													{
														pid_spd.integral = 0;
														pos_ref_um = p2_f;
														vel_ref_umPs = 0;
														acc_ref_umPs2 = 0;
														Profile_moving = 0;
														tCnt_f = 0;
														flag_posReg_strengthen = 1;
														if(status_Mode_POSITION_FORCE == 1) //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
														{
															status_Mode_POSITION_FORCE = 2;
															g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;
 
														}
													}												
												#endif
											}
											else if(flag_profile_3_sections == 1)//一段式
											{
												#if (PLAN_TYPE_SET==PLAN_STATIC)
													if(tCnt_f<t1_f)
												#endif
												#if (PLAN_TYPE_SET==PLAN_DYNAMIC)
													if( (dir_f>0.0f && pos_act_um<(s32)p1_f) ||  (dir_f<0.0f && pos_act_um>(s32)p1_f))
												#endif
												{
													pos_ref_um = (float)g_pos_ini + dir_f*v0_abs*tCnt_f-dir_f*acc_1_sections*tCnt_f*tCnt_f/2;
													vel_ref_umPs = dir_f*v0_abs-dir_f*acc_1_sections*tCnt_f;
													acc_ref_umPs2 = -1*dir_f*acc_1_sections;
													flag_posReg_strengthen = 1;
												}
												else
												{
													pos_ref_um = p1_f;
													vel_ref_umPs = 0;
													acc_ref_umPs2 = 0;
													Profile_moving = 0;
													tCnt_f = 0;
													flag_posReg_strengthen = 1;
													if(status_Mode_POSITION_FORCE == 1) //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
													{
														status_Mode_POSITION_FORCE = 2;
														g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;
 
													}
												}	
											}
											else if(flag_profile_3_sections == 0)//
											{
												if(tCnt_f<0.0005f*(float)P_POS_REGULATOR_MS)
												{
													pos_ref_um = (float)g_pos_ini;
													vel_ref_umPs =  0;// 1000.0f*g_pos_tag_dlt_f/(float)P_POS_REGULATOR_MS;
													acc_ref_umPs2 = 0;
													flag_posReg_strengthen = 1;
												}
												else
												{
													pos_ref_um = g_pos_tag;
													vel_ref_umPs = 0;
													acc_ref_umPs2 = 0;
													Profile_moving = 0;
													tCnt_f = 0;
													flag_posReg_strengthen = 1;
													if(status_Mode_POSITION_FORCE == 1) //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch  g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE
													{
														status_Mode_POSITION_FORCE = 2;
														g_CmdMap[TAG_WORK_MODE] = MODE_SPEED;
 
													}
												}
											}
							}
							else
							{
											vel_ref_umPs = 0;
											acc_ref_umPs2 = 0;
											tCnt_f = 0.0f;
											vel_tmp_f = 0.0f;
											t1_tmp_f = 0.0f;
											flag_posReg_strengthen = 1;
							}
		}
		else if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_2_SPEED)
		{
						g_CmdMap[SYS_SPD_ADD_LIMIT_PU] = 3276;	
						if((Profile_v_set > 0 && pos_ref_um >g_pos_tag)||(Profile_v_set < 0 && pos_ref_um <g_pos_tag))
						{
							g_CmdMap[CMD_USEER_MODE] = CMD_USEER_MODE_0_POSITION;
							tCnt_f = 0.0f;
							vel_tmp_f = 0.0f;
							t1_tmp_f = 0.0f;
						}
						else
						{
							Profile_moving = 1;
							tCnt_f = tCnt_f + 0.001f*(float)P_POS_REGULATOR_MS;//P_POS_REGULATOR_MS;
							pos_ref_um = ((float)g_pos_ini+tCnt_f*(float)Profile_v_set);
							vel_ref_umPs = Profile_v_set;
						}
						flag_posReg_strengthen = 1;
		}
		else if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_1_SERVO)
		{
							if(g_servo_cmd_period_cnt<1000)
							{
								Profile_moving = 1;
							}
							else
							{
								Profile_moving = 0;
								vel_interpolation_umPs = 0;
							}
							{	
								g_CmdMap[SYS_SPD_ADD_LIMIT_PU] = 16384;	 //debug		
								if(tCnt_f<time_interpolation_s)
								{
									tCnt_f = tCnt_f + 0.001f*(float)P_POS_REGULATOR_MS;
									pos_ref_um = (s32)((tCnt_f/time_interpolation_s)*(float)(pos_interpolation_A_um-pos_interpolation_B_um)) + pos_interpolation_B_um;
									
									vel_ref_umPs = 9*vel_interpolation_umPs/10;
									
	
									
//									if(pos_error_to_target< 4096)
//									{
//										vel_ref_umPs = 0;//9*vel_interpolation_umPs/10;
//									}
//									else
//									{
//										vel_ref_umPs = 0;//9*vel_interpolation_umPs/10;
//									}			
									
								}
								else
								{
									vel_ref_umPs = 3*vel_interpolation_umPs/10;
									pos_ref_um = pos_interpolation_A_um;
								}
								
								if(pos_interpolation_A_um > pos_interpolation_B_um)
								{
									if(pos_act_um > pos_ref_um-10)
									{
										vel_ref_umPs =0;
										Profile_moving =0;
									}
								}
								else
								{
									if(pos_act_um < pos_ref_um+10)
									{
										vel_ref_umPs =0;
										Profile_moving =0;
									}
								}
							}
							flag_posReg_strengthen = 1;
							p3_f = pos_interpolation_A_um;
							p2_f = pos_interpolation_A_um;
		}			
		g_pos_tag_old = g_pos_tag;
	}
	else
	{			
				pos_ref_um = g_CmdMap[CMD_POS_ACT_PU];
				vel_ref_umPs = 0;
				acc_ref_umPs2 = 0;
				tCnt_f = 0.0f;
				vel_tmp_f = 0.0f;
				t1_tmp_f = 0.0f;
				flag_posReg_strengthen = 1;
	}
	Flag_Profile_Enable_last = Flag_Profile_Enable;
	g_CmdMap[CMD_PROFILE_POS_PU] = pos_ref_um;
	g_CmdMap[CMD_PROFILE_SPD_PU] = (s16)((float)vel_ref_umPs*pos_p_vel_float);  
	g_CmdMap[CMD_ACC_PRE_PU] = (s16)(acc_ref_umPs2*(s32)16384/500);
}
/**
  * @brief  接受到位置数据后，计算命令间隔，以及前馈速度值
  * @param  ptr_sinWave: 
  * @param  para_frea_exc:    
  * @retval None
  */
void recPosCmd(void)
{
	if(g_CmdMap[CMD_E_STOP] == 1)
	{
		return;
	}
	
	if(g_CmdMap[CMD_POS_SET_PU]>g_CmdMap[CMD_POS_UPPER_LIMIT_PU])
	{
		g_CmdMap[CMD_POS_SET_PU] =g_CmdMap[CMD_POS_UPPER_LIMIT_PU];		
		if(g_CmdMap[CMD_POS_SET_PU] > 16384)
		{
			g_CmdMap[CMD_POS_SET_PU] = 16384;
		}
	}
	if(g_CmdMap[CMD_POS_SET_PU]<g_CmdMap[CMD_POS_LOWER_LIMIT_PU])
	{
		g_CmdMap[CMD_POS_SET_PU] = g_CmdMap[CMD_POS_LOWER_LIMIT_PU];
		if(g_CmdMap[CMD_POS_SET_PU] < 0)
		{
			g_CmdMap[CMD_POS_SET_PU] = 0;
		}
	}
	
	Flag_Set_OpenLen = 1;
	g_servo_cmd_period = g_servo_cmd_period_cnt;//高频位置伺服下，命令周期 ms
	g_servo_cmd_period_cnt = 0; 								//高频位置伺服下，命令周期计时
	pos_interpolation_B_um = pos_interpolation_A_um;
	pos_interpolation_A_um = g_CmdMap[CMD_POS_SET_PU];
	time_interpolation_s = (double)g_servo_cmd_period/(double)(F_SPD_REGULATOR_HZ);					
	vel_interpolation_umPs = (s32)((double)(pos_interpolation_A_um-pos_interpolation_B_um)/time_interpolation_s);
	if(vel_interpolation_umPs>g_Vel_Max_PosPU_pSec)
	{
		vel_interpolation_umPs = g_Vel_Max_PosPU_pSec;
	}
	if(vel_interpolation_umPs<-g_Vel_Max_PosPU_pSec)
	{
		vel_interpolation_umPs = -g_Vel_Max_PosPU_pSec;
	}				
	if(g_servo_cmd_period < 500)
	{
	//	g_CmdMap[CMD_USEER_MODE] = CMD_USEER_MODE_1_SERVO;
	}
	else
	{
		if(g_CmdMap[CMD_USEER_MODE] == CMD_USEER_MODE_5_POSITION_FORCE)
		{
		
		}
		else
		{
			//g_CmdMap[CMD_USEER_MODE] = CMD_USEER_MODE_0_POSITION;
		}
	}
	
	
	clear_limit_event();
}

#define POS_ERRO_TOL ( 16384 /500)

int target_position_arrived(int target, int act)
{
	u32 pos_erro = 0;
	
//	pos_erro = ABS(   g_CmdMap[CMD_POS_SET_PU]  -  g_CmdMap[CMD_POS_ACT_PU] );
	pos_erro = ABS(   target  -  act );
	if( pos_erro > POS_ERRO_TOL )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}






