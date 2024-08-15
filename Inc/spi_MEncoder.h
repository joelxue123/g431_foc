
#include "stm32g4xx_hal.h"


#define 	RES_P_180	  16384    //增量型编码器半圈的累计值
#define 	RES_P_360	  32768    //增量型编码器整圈的累计值

#define 	ABPL_P_R 1024   										//磁编一圈的增量脉冲数(没有四倍频时)

#define MA730_SPI1_PERIOD_US	20 //3 						//磁编SPI的采样周期微秒
#define MA730_SPI1_CSHIGH_US	5  //1 						//CS高电平时间微秒
#define MA730_SPI1_CLK_DELAY 	18 //9 						// 18:250ns 9:125ns

#define TIM8_PERIOD	(MA730_SPI1_PERIOD_US*72) 
#define TIM8_CCR1 (MA730_SPI1_CSHIGH_US*72) 
#define TIM8_CCR3 (MA730_SPI1_CSHIGH_US*72 + MA730_SPI1_CLK_DELAY) 

#define MT_TRPM_THRESHHOLD  200							//T测速门槛值
#define MT_MRPM_THRESHHOLD	100							//M测速门槛值
#define RPM_TO_NPS				 	546 						//RPM 转换为 NPS的转化系数 1RPM = 32768/60 = 546NPS

//#define NP_TO_UU_4BIT				8977            //NPS 转换为 UPS(步每秒，步10um) 4096脉冲(一圈) 对应 7.3 u(72um)
#define NP_TO_UU_4BIT				  87             //NPS 转换为 UPS(步每秒，步 um) 4096脉冲(一圈) 对应 75 u(750um)
#define MT_TRPM_THRESHHOLD  200							//T测速门槛值
#define MT_MRPM_THRESHHOLD	100							//M测速门槛值
#define UPS_TO_NPS	  (NP_TO_UU_4BIT>>4)    //NPS 转换为 UPS(步每秒，步10um) 4096脉冲(一圈) 对应 7.3 u(72um)
#define D_NPS_N_RPS		(4096)      //转化系数(放大16倍)分子为NPS 转换分母为RPM，16*4096/60
#define SPEED_TIM_FREQUENCY	36000000				//速度计算定时器的频率 36MHz
//#define SCREW_UM_RES				380 					//螺杆螺距（每圈行程）

// 一阶低通滤波系数
//fL = a/(2*3.14*Ts)
//a  = 6.28*Ts*fL
//Yn = a*Xn+(1-a)*Yn-1
//#define		SPEED_FL_HZ	  100.0f
//#define		SPEED_FL_A	  (int)(128.0f*6.28f*(1.0f/10000.0f)*SPEED_FL_HZ + 0.5f)
 
extern volatile s32 g_CurrentSpeed;				    //当前速度
extern volatile s32 g_Encode_AbsPos;
extern volatile s32 g_Encode_AbsPos_Last;
extern volatile s32 g_Encode_Inc;
extern volatile s32 g_Encode_offset;
extern volatile s32 g_Encode_offset_EN;
extern volatile s32 Num_Turns;
extern volatile s32 tim3_cnt;
extern volatile s32 tim4_cnt;
extern  TIM_HandleTypeDef htim2;
extern	volatile s32 flag_first_exe;
	
	
	
extern void SPI_MEncoder_Configuration(void); 
extern void AB_MEncoder_Configuration(void);
extern void MTPro(void);
extern void GetElectricAngle(void);
extern void GetPos_FromMEncoder(void);
extern void SetPos_FromMEncoder(s32 PosSetValue);
extern void sim_ElectricAngle(void);
extern void Encode_Single_to_Multi_R(u8 flag_reverse,u8 flag_cycle,s32 single_value_raw,s32* p_multi_value,s32 mod_single,s32 mod_multi);
void hall_Configuration(void);
void Pll_phase(s16 measured_phase, float *estimated_phase, float *estimated_spd );
void MX_SPI3_Init(void);