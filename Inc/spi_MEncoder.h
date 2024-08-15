
#include "stm32g4xx_hal.h"


#define 	RES_P_180	  16384    //�����ͱ�������Ȧ���ۼ�ֵ
#define 	RES_P_360	  32768    //�����ͱ�������Ȧ���ۼ�ֵ

#define 	ABPL_P_R 1024   										//�ű�һȦ������������(û���ı�Ƶʱ)

#define MA730_SPI1_PERIOD_US	20 //3 						//�ű�SPI�Ĳ�������΢��
#define MA730_SPI1_CSHIGH_US	5  //1 						//CS�ߵ�ƽʱ��΢��
#define MA730_SPI1_CLK_DELAY 	18 //9 						// 18:250ns 9:125ns

#define TIM8_PERIOD	(MA730_SPI1_PERIOD_US*72) 
#define TIM8_CCR1 (MA730_SPI1_CSHIGH_US*72) 
#define TIM8_CCR3 (MA730_SPI1_CSHIGH_US*72 + MA730_SPI1_CLK_DELAY) 

#define MT_TRPM_THRESHHOLD  200							//T�����ż�ֵ
#define MT_MRPM_THRESHHOLD	100							//M�����ż�ֵ
#define RPM_TO_NPS				 	546 						//RPM ת��Ϊ NPS��ת��ϵ�� 1RPM = 32768/60 = 546NPS

//#define NP_TO_UU_4BIT				8977            //NPS ת��Ϊ UPS(��ÿ�룬��10um) 4096����(һȦ) ��Ӧ 7.3 u(72um)
#define NP_TO_UU_4BIT				  87             //NPS ת��Ϊ UPS(��ÿ�룬�� um) 4096����(һȦ) ��Ӧ 75 u(750um)
#define MT_TRPM_THRESHHOLD  200							//T�����ż�ֵ
#define MT_MRPM_THRESHHOLD	100							//M�����ż�ֵ
#define UPS_TO_NPS	  (NP_TO_UU_4BIT>>4)    //NPS ת��Ϊ UPS(��ÿ�룬��10um) 4096����(һȦ) ��Ӧ 7.3 u(72um)
#define D_NPS_N_RPS		(4096)      //ת��ϵ��(�Ŵ�16��)����ΪNPS ת����ĸΪRPM��16*4096/60
#define SPEED_TIM_FREQUENCY	36000000				//�ٶȼ��㶨ʱ����Ƶ�� 36MHz
//#define SCREW_UM_RES				380 					//�ݸ��ݾࣨÿȦ�г̣�

// һ�׵�ͨ�˲�ϵ��
//fL = a/(2*3.14*Ts)
//a  = 6.28*Ts*fL
//Yn = a*Xn+(1-a)*Yn-1
//#define		SPEED_FL_HZ	  100.0f
//#define		SPEED_FL_A	  (int)(128.0f*6.28f*(1.0f/10000.0f)*SPEED_FL_HZ + 0.5f)
 
extern volatile s32 g_CurrentSpeed;				    //��ǰ�ٶ�
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