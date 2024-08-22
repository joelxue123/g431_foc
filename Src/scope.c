#include "scope.h"
#include "global.h"
#include "sensorless.h"
#define RECODER_LEN		    100 		// ���ݼ�¼���ֳ���
#define RECODER_LEN_HALF	50 			// ���ݼ�¼���ֳ��ȵ�һ��
#define SCPE_TXD_LEN	    80      //(18+6)			// ���ͻ��泤�ȣ�Byte��
#define DATA_BEG_POS	    0       // ������ʼλ��
s16 g_UserDefinedBuf[RECODER_LEN] = {0};		  // �Զ������ݼ�¼������
volatile u8 flag_rev_modbus = 0; 
volatile s16 modbus_rly_delay_ms = 0; 
u8 g_UserDefined_TxBuf[SCPE_TXD_LEN] = {0};	// �Զ������ݷ��ͻ���
u32 g_Rec_pos = 0;						          // ��ǰ��¼����λ��
u32 g_TxBuf_Pos = DATA_BEG_POS;			    // ��ǰ���ͻ���λ��
u32 g_TxBufDebug = 0;			    // ��ǰ���ͻ���λ��
u8 g_bWaitToSend_UserDefined = 0;

extern int g_flag_1;
extern volatile s32 s_Disable_Delay;
extern volatile s32 profile_flag;
extern volatile float p3_f;
extern volatile int G_hall_ture_value;

extern int observer_ElectricAngle;
extern volatile  int delta_alpha;
extern volatile  int delata_beta;
extern int observer_speed;
extern 	float L,temp_L;
extern s16 pll_estimated_spd;
extern volatile int estimated_SPD_filterd;
extern float pll_estimated_phase;
extern int g_measured_phase;
extern int s_Encoder_Inc;

#define ENCODER_BASE_SPEED  (30*  1000000  /100   / 23000)
void ScopeProDebug(void)//����ʱ���������� 
{
	//ֱ��д���µ�һ֡	
	s16 i=0;	
	g_TxBuf_Pos = 0;
	i=0;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_POS_SET_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_PROFILE_POS_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_POS_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_SPDREG_REF_PU];//g_CmdMap[CMD_SPDREG_REF_PU]; //CMD_SPD_SET_PU
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) =  g_CmdMap[CMD_SPD_ACT_PU];//s_Encoder_Inc * ENCODER_BASE_SPEED;//g_CmdMap[CMD_SPD_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = (s16)motor_.vel_estimate_;//estimated_SPD_filterd;//g_CmdMap[CMD_SPD_ADD_PU];//g_CmdMap[CMD_SPD_ADD_PU]/100;//g_CmdMap[CMD_SPD_SOFT_SET_PU];// CMD_CUR_FWD_PU  //CMD_SPD_ADD_PU
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = adc_measurements_[0];//(s16)(motor_.current_control_.final_v_alpha*1000.f*PU_REFERENCE/(*g_pCur_ref_base_mA));//g_CmdMap[CMD_CUR_SET_PU];//g_CmdMap[CMD_CUR_SET_PU];//g_CmdMap[CMD_CUR_SET_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = adc_measurements_[1];//(s16)(motor_.current_control_.final_v_beta*1000.f*PU_REFERENCE/(*g_pCur_ref_base_mA));//g_Iq*PU_REFERENCE/(*g_pCur_ref_base_mA);
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = 0;//g_IB_Raw*PU_REFERENCE/(*g_pCur_ref_base_mA);; // CMD_CUR_FWD_PU //CMD_CUR_D_ACT_PU
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) =g_ElectricAngle_15bit_Raw - g_Encode_offset_EN;//g_ElectricAngle;//g_ElectricAngle ;//g_ElectricAngle_15bit_Raw;//g_ElectricAngle_15bit_Raw;//g_ElectricAngle_15bit_Raw;  //g_ElectricAngle_15bit_Raw;//g_ElectricAngle_15bit_Raw;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_SysStatus;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_ERROR];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_TEMP];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = ((MU_Value_base_0)>>16)&0xFFFF;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = (MU_Value_base_0)&0xFFFF;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_FORCE_ACT_PU];
	i+=2;
	g_TxBuf_Pos+=i;
	UART_PushFrame(g_TxBuf_Pos,CMDTYPE_DEBUG,0x00,&g_UserDefined_TxBuf[0]);	
}
void ScopeProDebug_2(void)//����ʱ���������� 
{
	//ֱ��д���µ�һ֡	
	s16 i=0;	
	g_TxBuf_Pos = 0;
	i=0;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_POS_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_SPDREG_REF_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_SPD_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_CUR_SET_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_CUR_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_FORCE_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_ERROR];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_TEMP];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = ((MU_Value_base_0)>>16)&0xFFFF;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = (MU_Value_base_0)&0xFFFF;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_SysStatus;
	i+=2;g_TxBuf_Pos+=i;
	UART_PushFrame(g_TxBuf_Pos,CMDTYPE_DEBUG_2,0x00,&g_UserDefined_TxBuf[0]);	
}
void ReturnCLibADC_Frame(void)//����ʱ���������� 
{
	//ֱ��д���µ�һ֡	
	s16 i=0;	
	g_TxBuf_Pos = 0;
	i=0;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_POS_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_POS_ADC_12BIT];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_CUR_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_SPD_ACT_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_SPD_SET_PU];
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_SysStatus;
	i+=2;*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[CMD_ERROR];
	i+=2;
	g_TxBuf_Pos+=i;
	UART_PushFrame(g_TxBuf_Pos,CMDTYPE_ADC_CLIB,0x00,&g_UserDefined_TxBuf[0]);	
}
//����״̬��Ϣ֡,״̬��Ϣ֡��ÿһλ����������
void Return_state_frame(u8 cmd_type,u8 index) 
{
	//ֱ��д���µ�һ֡	
	s16 i=0,j=0;	
	g_TxBuf_Pos = 0;
	if(g_CmdMap[SYS_STATA_FRAME_NUM]<0)
	{
		g_CmdMap[SYS_STATA_FRAME_NUM] = 0;
	}
	if(g_CmdMap[SYS_STATA_FRAME_NUM]>15)
	{
		g_CmdMap[SYS_STATA_FRAME_NUM] = 15;
	}
	for(j=0;j<g_CmdMap[SYS_STATA_FRAME_NUM];j++)
	{
		if(g_CmdMap[SYS_STATA_FRAME_SET(j)]<0x0200 && g_CmdMap[SYS_STATA_FRAME_SET(j)]>0x0000)
		{
			*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = g_CmdMap[g_CmdMap[SYS_STATA_FRAME_SET(j)]];i+=2;
		}
		else
		{
			*(u16*)(&g_UserDefined_TxBuf[g_TxBuf_Pos+i]) = 0x0000;i+=2;
		}			
	}
	g_TxBuf_Pos+=i;
	UART_PushFrame(g_TxBuf_Pos,cmd_type,index,&g_UserDefined_TxBuf[0]);	
}
//����LA��ָ��,���ذ���״̬��Ϣ
void Return_OldFrame_state(u8 cmd_type,u8 index,u8 data) 
{
	//ֱ��д���µ�һ֡	
	g_TxBuf_Pos = 0;
	s16 pos_set = (s32)g_CmdMap[CMD_POS_SET_PU]*2000/PU_REFERENCE;
	s16 pos_act = (s32)g_CmdMap[CMD_POS_ACT_PU]*2000/PU_REFERENCE;
	s16 cur_act = (s32)g_CmdMap[CMD_CUR_ACT_PU]*(*g_pCur_ref_base_mA)/PU_REFERENCE;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = data;	
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = pos_set & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (pos_set>>8) & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = pos_act & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (pos_act>>8) & 0xff;	
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = g_CmdMap[CMD_TEMP];//�¶�
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = cur_act & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (cur_act>>8) & 0xff;	
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (0x00) & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = g_CmdMap[CMD_ERROR];//����	
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (0x00) & 0xff;	//����ֵ���ֽ�
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (0x00) & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (0x00) & 0xff;	//����ֵ���ֽ�
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (0x00) & 0xff;
	g_UserDefined_TxBuf[g_TxBuf_Pos++] = (0x00) & 0xff;	//����ֵ���ֽ�
	UART_PushFrame_Old(g_TxBuf_Pos,cmd_type,index,&g_UserDefined_TxBuf[0]);	
}
//����LA��ָ��-д�Ĵ���,���ذ���״̬��Ϣ
void Return_OldFrame_data(u8 cmd_type,u8 index,u8 length) 
{
	//ֱ��д���µ�һ֡	
	s16 i=0;	
	g_TxBuf_Pos = 0;
	s32 data_temp = 0;
	for(i=0;i<length;i++)
	{
		if(index+i == 2)//ID
		{
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = g_CmdMap[CMD_ID];
		}
		else if(index+i == 12)//������
		{
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = g_CmdMap[CMD_BAUDRATE_UART];
		}
		else if(index+i == 31)//��������У׼λ
		{
			
		}
		else if(index+i == 32)//��������ֵ���ֽ�
		{
			data_temp = (s32)g_CmdMap[CMD_OVER_CURRENT_SET]*(*g_pCur_ref_base_mA)/PU_REFERENCE;
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = data_temp&0x00FF;
			//g_UserDefined_TxBuf[g_TxBuf_Pos++] = (data_temp>>8)&0x00FF;
		}
		else if(index+i == 33)//��������ֵ���ֽ�
		{
			data_temp = (s32)g_CmdMap[CMD_OVER_CURRENT_SET]*(*g_pCur_ref_base_mA)/PU_REFERENCE;
			//g_UserDefined_TxBuf[g_TxBuf_Pos++] = data_temp&0x00FF;
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = (data_temp>>8)&0x00FF;
		}
		else if(index+i == 98)//���±���ֵ���ֽ�
		{
			data_temp = (g_CmdMap[CMD_OVER_TEMP_SET]*10);
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = data_temp&0x00FF;
			//g_UserDefined_TxBuf[g_TxBuf_Pos++] = (data_temp>>8)&0x00FF;
		}
		else if(index+i == 99)//���±���ֵ���ֽ�
		{
			data_temp = (g_CmdMap[CMD_OVER_TEMP_SET]*10);
			//g_UserDefined_TxBuf[g_TxBuf_Pos++] = data_temp&0x00FF;
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = (data_temp>>8)&0x00FF;
		}
		else
		{
			g_UserDefined_TxBuf[g_TxBuf_Pos++] = 0;
		}	
	}
	UART_PushFrame_Old(g_TxBuf_Pos,cmd_type,index,&g_UserDefined_TxBuf[0]);	
}


