#include "stm32g4xx_hal.h"
#include "global.h"
#define PROFILE_BIT_AMP 6 //3   //不宜太大不让 内部运算溢出，S1 和 S1andS3 为负数
#define PROFILE_AMP 65536
#define PROFILE_F 	500   //Profile 计算频率
//#define PI_DOUBLE_DIV_PITCH 	
//#define ACC_NPS_2_NAMP_PER_CALT  (PROFILE_AMP/PROFILE_F) 
#define RPM_TO_NP500US_8BIT	(s32)((float)ENCODER_LINES*1024/120000+0.5f)  // rpm到cnt/500us的转换系数（放大了256倍) 500*1024/120000= 4.26666
#define RPMPS_TO_NP500USP500US_8BIT	(s32)((float)ENCODER_LINES*1024/(120000*2000)+0.5f)    // rpm/s到(cnt/500us)/500us的转换系数（放大了256倍)   500*1024/(120000*2000)= 0.00213333  加速度必须469的倍数
#define NP500USP500US_8BIT_TO_RPMPS	(s32)((120000*2000)/((float)ENCODER_LINES*1024)+0.5f)    // rpm/s到(cnt/500us)/500us的转换系数（放大了256倍)   500*1024/(120000*2000)= 0.00213333  加速度必须469的倍数
#define APHLA_LOW_15BIT 4000
#define APHLA_HIGH_15BIT 12500
#define APHLA_RES_15BIT 4


#define PLAN_STATIC   0 //静态位置规划，适用于传动机构不打滑的情况
#define PLAN_DYNAMIC  1 //动态位置规划，适用于传动机构打滑的情况
#define PLAN_TYPE_SET PLAN_DYNAMIC  
extern volatile float t1_f;
extern volatile float t2_f;
extern volatile float t3_f;

extern volatile s32 g_pos_tag;
extern volatile s32 g_pos_ini;
extern volatile s32 g_pos_tag_old ;
extern volatile float g_pos_tag_dlt_f;
extern volatile float g_pos_tag_dlt_abs_f ;
extern volatile float pos_ref_f ;

extern volatile float vel_ref_f ;
extern volatile float acc_ref_f;
extern volatile s32 acc_ref_umPs2;
extern volatile s32 pos_ref_um;
extern volatile s32 pos_act_um;
extern volatile s32 pos_interpolation_A_um;
extern volatile s32 pos_interpolation_B_um;
extern volatile float time_interpolation_s;
extern volatile s32 vel_interpolation_umPs;
extern volatile s32 V_Mot_ref_RPM;
extern volatile s32 V_Mot_AddReg_RPM;
extern volatile s32 V_Mot_ref;
extern volatile float vel_max_f_umPs ;
extern volatile float acc_set_f_umPs2 ;
extern volatile s32 Flag_Profile_Enable;
extern volatile s32 Flag_Profile_Enable_last;
//extern volatile s32 Profile_mode;   // 0-位置模式 1-速度模式 2-冻结当前位置模式
extern volatile s32 Profile_mode_last;
extern volatile s32 Profile_v_set;
extern volatile s32 Profile_moving ;
extern volatile s16 Count_test;
extern volatile s16 Count_test01;
extern volatile s32 vel_ref_umPs;
extern volatile u8 flag_posReg_strengthen;
extern volatile u8 flag_profile_3_sections;
extern volatile float tCnt_f;

extern void Profile(void);
extern void Profile_ParaSet(void); 
extern void Get_MotSpeedSet_From_JointSpeed(s32 Joint_v_15bitPs);
/**
  * @brief  接受到位置数据后，计算命令间隔，以及前馈速度值
  * @param  ptr_sinWave: 
  * @param  para_frea_exc:    
  * @retval None
  */
extern void recPosCmd(void);

int target_position_arrived(int target, int act);



























