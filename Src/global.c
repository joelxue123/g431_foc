#include "global.h"

// ȫ�ֱ���
volatile u8 Motor_forbidden = 0;        // �����ֹ�����־
volatile u8 Motor_emergercy_stop;
volatile u8 Motor_pause_flag;
volatile s16 Zero_s16 = 0x0000;         // SPI����CLK�ź�ʹ��
volatile u8 g_BeginPIDFlag = 0;		      // ��ʼPID
volatile s16 g_SysStatus = 0; 		      // ������״̬��
volatile s32 g_Timer1CCR = 0;		        // ��ʱ�����յ����ռ�ձ�
volatile s16 g_bEnableHB = 0;
s16 g_Temperature = 250; 			          // �¶�
u16 g_PowVoltage = 4800;			          // �����ѹ
double Spd_ratio =0;
s32 Encode_PPR =0;
s32 Encode_PP_Half_R = 0;
volatile s32 pos_nm_MU150 = 0;
uint8_t Flag_error_read_flash = 0; 		//flash ��ȡ����
uint8_t Flag_error_set_preDrive = 0;	//����Ԥ����оƬ��������
uint8_t Flag_error_sensor_comm = 0;		//������ģ��ͨѶ����
uint8_t Flag_error_current_clib = 0;	//����У׼ʧ��
uint8_t Flag_error_MU_SPI = 0;				//MU SPIͨѶ����
uint8_t Flag_error_MU_value_instable;		//MU �źŲ��ȶ�
uint8_t Flag_error_torsion_dect = 0;	//�α��������󣬸��ض˺͵���˵ĽǶ�ƫ��̫��

s32 range_icmu_vlaue_at_stroke;
s32 vlaue_icmu_offset_overturn;
s32 range_icmu_vlaue_need_addMax;


/*-------------------------Profile-----------------------------------*/
//����ֵ
float pos_p_vel_float;
s32* g_pPos_ref_base_um = (s32*)&g_CmdMap[SYS_POS_REF_BASE_UM_L];//λ�ñ���ֵ
s32* g_pVel_ref_base_PPS = (s32*)&g_CmdMap[SYS_SPD_REF_BASE_PPS_L];//�ٶȱ���ֵ
s32* g_pVel_ref_base_umPS = (s32*)&g_CmdMap[SYS_SPD_REF_BASE_UMPS_L];//�ٶȱ���ֵ
s32* g_pCur_ref_base_mA = (s32*)&g_CmdMap[SYS_CUR_REF_BASE_MA_L];//��������ֵ
s32* g_pAcc_ref_base_mmPS2 = (s32*)&g_CmdMap[SYS_ACC_REF_BASE_L];//���ٶȱ���ֵ
s32* g_pForce_ref_base_mg = (s32*)&g_CmdMap[SYS_FORCE_REF_BASE_L];//��������ֵ
s32  g_Vel_Max_PosPU_pSec = 0; //������ٶ� pu(λ��) /s
s32 g_servo_cmd_period_cnt = 0; //��Ƶλ���ŷ��£��������ڼ�ʱ
s32 g_servo_cmd_period = 0; //��Ƶλ���ŷ��£���������
s32 g_servo_cmd_pos_cur = 0;//��Ƶλ���ŷ��£���ǰ��λ���趨
s32 g_servo_cmd_pos_last = 0;//��Ƶλ���ŷ��£���һ���ڵ�λ���趨
s32 pos_set_current_frm_CMD_PU = 0;
s32 pos_set_last_frm_CMD_PU = 0;
u8 status_Mode_POSITION_FORCE = 0; //λ��+����ģʽ�µ�״̬�� 0 Idle 1 Profile 2 Force_touch
/*-------------------------λ�û�-----------------------------------*/
s32* g_pMU_value_offset = (s32*)&g_CmdMap[SYS_MU_MA_OFFSET_L];
s32* g_pMU_0mm_offset = (s32*)&g_CmdMap[SYS_MU_MA_0MM_BASE_L];
s32 rod_pos_Muti_Encoder = 0;							// ��������ֵ�ۼ�ֵ������Ȧ���Ȧ�������
s32 gPos_Kp;															// ��ǰ��ʵʱ��У׼���λ�û�����ϵ��
s32 gPos_Ki;															// ��ǰ��ʵʱ��У׼���λ�û�����ϵ��
s32 gPos_ds;      												// ��ǰ��ʵʱ��У׼���λ�û�����
volatile u8 g_PosReg_runing = 0;		      // λ�û�״̬ 1����0ֹͣ
volatile u8 g_PosReg_Enbale = 0;		      // λ�û�ʹ������
volatile u8 g_PosReg_Disabel = 0;		      // λ�û���������
/*-------------------------�ٶȻ�-----------------------------------*/
s32 gSpd_Kp;															// ��ǰ��ʵʱ��У׼����ٶȻ�����ϵ��
s32 gSpd_Ki;															// ��ǰ��ʵʱ��У׼����ٶȻ�����ϵ��
double gVel_PPS = 0;															// 
double Vel_PPS_raw = 0;
s32 gSpd_ds;      												// ��ǰ��ʵʱ��У׼����ٶȻ�����
volatile u8 g_SpdReg_runing = 0;		      // �ٶȻ�״̬ 1����0ֹͣ
volatile u8 g_SpdReg_Enbale = 0;		      // �ٶȻ�ʹ������
volatile u8 g_SpdReg_Disabel = 0;		      // �ٶȻ���������
/*-------------------------������-----------------------------------*/
s32 gCur_Kp;															// ��ǰ��ʵʱ��У׼��ĵ���������ϵ��
s32 gCur_Ki;															// ��ǰ��ʵʱ��У׼��ĵ���������ϵ��
volatile u8 g_CurReg_runing = 0;		      // �ٶȻ�״̬ 1����0ֹͣ
volatile u8 g_CurReg_Enbale = 0;		      // �ٶȻ�ʹ������
volatile u8 g_CurReg_Disabel = 0;		      // �ٶȻ��������
/*---------------------------�ű�Ƕ����---------------------------------*/
volatile s32 g_ElectricAngle_act = 0;	    // ��ʵ��Ƕ�
volatile s32 g_ElectricAngle_sim = 0;	    // �����Ƕ�
volatile s32 g_ElectricAngle_15bit = 0;		// ��Ƕ�_32768
volatile s32 g_ElectricAngle_15bit_Raw = 0;		// ��Ƕ�_32768
volatile s32 g_MechanicsAngle_15bit = 0;	// ��е�Ƕ�_32768
volatile s32 g_MechanicsAngle_15bit_last = 0;	// ��е�Ƕ�_32768
volatile s32 g_ElectricAngle = 0; 	      // ��ǰ��Ƕ�


volatile s16 force_set = 0,force_act = 0;

volatile u8 flag_processing_Jtest = 0;
volatile s16 state_Jtest = 0;

s16	EN_0 = 0;
s16	EN_30	 = 0;
s16	EN_45 = 0;
s16	EN_60 = 0;
s16	EN_90 = 0;
s16	EN_120 = 0;
s16	EN_150 = 0;
s16	EN_180 = 0;
s16	EN_210 = 0;
s16	EN_240 = 0;
s16	EN_270 = 0;
s16	EN_300 = 0;
s16	EN_330 = 0;
s16	EN_360 = 0;
//
volatile s32 L_Slop_Array[NUM_LINEARITY_SEG+1] = {0};
//
s32	Screw_um = 0;           //�ݾ�
s32	Mot_Pairs = 0;          //���������
// �����궨����
s32	g_ZeroCur_MotorA;    // ��������궨ֵ
s32	g_ZeroCur_MotorB;    // ��������궨ֵ
s32	g_ZeroCur_MotorC;	   // ��������궨ֵ

s16 Gripper_Move = 0;  //0 IDEL 1 ��ȡ 2 �ɿ�
u8 Flag_contact = 0;
u8 Flag_grip_F_OK = 0;
u8 Flag_grip_P_OK = 0;
u8 Flag_rellease_P_OK = 0;
// �ڴ���Ʊ�32λ����ָ��


// �ڴ���Ʊ�32λ����ָ��

s32 g_PosOffset = 0;				// �궨ƫ����
s32 g_PosSet_Calibrate = 0;


s16 s_count_5ms = 0;
s16 s_count_1ms = 0;
u8 Flag_1ms = 0;
u16 Flag_50ms = 0;
// �ڴ���Ʊ����

s16 s_count_50ms = 0;
uint16_t padding; 
volatile s16 g_CmdMap[CMDMAP_LEN+USERCMDMAP_LEN] __attribute__((aligned(4)))  = {0};
s16 g_ThreeLoopParaBefLock[CMDMAP_SUBLEN] = {0};
s16 g_CmdMap_Default[CMDMAP_INDLEN][CMDMAP_SUBLEN] = 
{
		{	  // 0x010*�ֶ�
			0xAA55,								//0x0100
			0,					      		//0x0101		//�����Ƿ��ϵ�
      0,										//0x0102		//����ģʽ
      0xFFFF,								//0x0103		//SN1
      0xFFFF,		          	//0x0104		//SN2
      0xFFFF,								//0x0105		//SN3
			0,										//0x0106		//�������ݵ�FLASH
      0,		  							//0x0107		//����ʵ��λ�ã�����ʹ�ã������޸��ۼ�ֵ
      0,		  							//0x0108		//�������г��ڵ��α������б궨
      0,			    					//0x0109		//���������ʶʹ�ܣ�1���������裬2���������
      80,										//0x010A		//��բ�ɿ���ʱ
      80,			    					//0x010B		//��բ������ʱ
      0,										//0x010C		//MOS����
			0,										//0x010D		//�������,�����������������
			0,										//0x010E		//����ٶȻ�����ģʽ��0-�ٶȻ���������ת��1-�ٶȻ����ڼ���ֵ���������˶���2-�ٶȻ�ģʽ�������¶��˶���3-�ٶȻ�ģʽ����������˶�
			0,										//0x010F		//��ԭ�궨����			
		},
		{	  // 0x011*�ֶ� ����ֵ
			0,										//0x0110    �������ۻ�׼ֵmA L_16Bit
			0,					      		//0x0111		�������ۻ�׼ֵmA H_16Bit
      0,										//0x0112		����ת�ٱ��ۻ�׼ֵPPS L_16Bit
      0,										//0x0113		����ת�ٱ��ۻ�׼ֵPPS H_16Bit
      0,				          	//0x0114		�Ƹ����ٶȱ��ۻ�׼ֵ L_16Bit
      0,										//0x0115		�Ƹ����ٶȱ��ۻ�׼ֵ H_16Bit
			0,										//0x0116		λ�ñ��ۻ�׼ֵuM L_16Bit
      0,		  							//0x0117		λ�ñ��ۻ�׼ֵuM H_16Bit
      0,		  							//0x0118		����ֵ���ۻ�׼ֵPPS L_16Bit
      0,			    					//0x0119		����ֵ���ۻ�׼ֵPPS H_16Bit
      0,										//0x011A		���ٶȱ��ۻ�׼ֵPPS L_16Bit um/s^2
      0,			    					//0x011B		���ٶȱ��ۻ�׼ֵPPS H_16Bit um/s^2
      0,										//0x011C		
			1638,									//0x011D		//ϵͳ�ڲ��ڲ�������Ӧ�ĵ�������ֵ
			8192,									//0x011E		//100% ����� ��Ӧ�ĵ�������ֵ
			0,										//0x011F		
		},
		{	  // 0x012*�ֶ� ��������Ϣ
			0,										//0x0120    �������
			0,					      		//0x0121		������
      0,										//0x0122		������ѹ
      0,										//0x0123		��������
      15,				          	//0x0124		��������
      1,										//0x0125		���������
			0,										//0x0126		�����Ƕ�ƫ����
      250,		  						//0x0127		�ݸ��ݾ� um
      11,		  							//0x0128		�����ٱ� *256
      0,			    					//0x0129		λ�ÿ������� 0�ݸ� 1�ؽڽ� 2����
      0,										//0x012A		
      0,			    					//0x012B		
      0,										//0x012C		
			0,										//0x012D		
			0,										//0x012E		
			0,										//0x012F	
		},
		{	  // 0x013*�ֶ� ����Ŀ��ֵ
			0,										//0x0130    ����ģʽ��ռ�ձ�(0-100)
			0,					      		//0x0131		Ŀ�����ֵ L16bit
      0,										//0x0132		Ŀ�����ֵ H16bit
      0,										//0x0133		Ŀ���ٶ�ֵ L16bit
      0,				          	//0x0134		Ŀ���ٶ�ֵ H16bit
      0,										//0x0135		Ŀ��λ�� L16bit
			0,										//0x0136		Ŀ��λ�� H16bit
      0,		  							//0x0137		Ŀ��ؽ� L16bit
      0,		  							//0x0138		Ŀ��ؽ� H16bit
      0,			    					//0x0139		
      0,										//0x013A		
      0,			    					//0x013B		
      0,										//0x013C		
			0,										//0x013D		
			0,										//0x013E		
			0,										//0x013F	
		},
		{	  // 0x014*�ֶ� ��������ֵ
			16384,										//0x0140    ������������ PU
			-16384,					      		//0x0141		������������ PU
      16384,										//0x0142		�ٶȿ������� PU
      -16384,										//0x0143		�ٶȿ������� PU
      16384,				          	//0x0144		���ȿ������� PU
      -16384,										//0x0145		���ȿ������� PU
			16384,										//0x0146		λ�ÿ������� PU
      0,		  									//0x0147		λ�ÿ������� PU
      16384,		  							//0x0148		���ٶȿ��Ƽ��� PU
      16384,			    					//0x0149		�����ٶȵ��ڵļ��� PU
      0,												//0x014A		
			2200,											//0x014B		
			2000,											//0x014C		
			0,												//0x014D
			0,												//0x014E
			0,												//0x014F			
		},
		{	  // 0x015*�ֶ� ���ջ���ز���
			0,										//0x0150    ���ջ�����������־
			90,					      		//0x0151		������P����
      300,										//0x0152		������I����
      0,										//0x0153		������D����
      200,				          	//0x0154		�ٶȻ�P����
      400,										//0x0155		�ٶȻ�I����
			0,										//0x0156		�ٶȻ�D����
      4,		  							//0x0157		�ٶȻ���������
      8000,		  							//0x0158		λ�û�P����
      100,			    					//0x0159		λ�û�I����
      0,										//0x015A		λ�û�D����
      2,			    					//0x015B		λ�û���������
      400,										//0x015C		//���ػ�P����
			8,										//0x015D		//���ػ�I����
			10,										//0x015E		//���ػ�D����
			50,										//0x015F		//���ػ�����
		},
		{	  // 0x016*�ֶ� ͨѶ���ã�״̬��Ϣ֡������
			6,					          //0x01A0    ״̬��Ϣ֡�ĳ���
			CMD_POS_ACT_PU,      	//0x01A1		״̬��Ϣ֡��1λ���żĴ����ĵ�ַ
			CMD_CUR_ACT_PU,				//0x01A2		״̬��Ϣ֡��2λ���żĴ����ĵ�ַ
			CMD_FORCE_ACT_PU,			//0x01A3		״̬��Ϣ֡��3λ���żĴ����ĵ�ַ
			CMD_SPD_ACT_PU,       //0x01A4		״̬��Ϣ֡��4λ���żĴ����ĵ�ַ
			CMD_ERROR,						//0x01A5		״̬��Ϣ֡��5λ���żĴ����ĵ�ַ
			CMD_TEMP,							//0x01A6		״̬��Ϣ֡��6λ���żĴ����ĵ�ַ
			0,										//0x01A7		״̬��Ϣ֡��7λ���żĴ����ĵ�ַ
			0,										//0x01A8		״̬��Ϣ֡��8λ���żĴ����ĵ�ַ
			0,										//0x01A9		״̬��Ϣ֡��9λ���żĴ����ĵ�ַ
			0,										//0x01AA		״̬��Ϣ֡��10λ���żĴ����ĵ�ַ
			0,			    					//0x01AB		״̬��Ϣ֡��11λ���żĴ����ĵ�ַ
			0,										//0x01AC		״̬��Ϣ֡��12λ���żĴ����ĵ�ַ
			0,										//0x01AD		״̬��Ϣ֡��13λ���żĴ����ĵ�ַ
			0,										//0x01AE		
			0,										//0x01AF    
		},
		{	  // 0x017*�ֶ� ADC�궨����
			16384*0/70,					//0x0160    ADC0
			16384*0/70,      		//0x0161		POS0
			16384*1/70,					//0x0162		ADC1
			16384*1/70,					//0x0163		POS1
			16384*2/70,        	//0x0164		ADC2
			16384*2/70,					//0x0165		POS2
			16384*3/70,					//0x0166		ADC3
			16384*3/70,					//0x0167		POS3
			16384*4/70,					//0x0168		ADC4
			16384*4/70,					//0x0169		POS4
			16384*5/70,					//0x016A		ADC5
			16384*5/70,					//0x016B		POS5
			16384*6/70,					//0x016C		ADC6
			16384*6/70,					//0x016D		POS6
			16384*7/70,					//0x016E		ADC7
			16384*7/70,					//0x016F    POS7
		},
		{	  // 0x018*�ֶ� ADC�궨����
			16384*8/70,					//0x0170    ADC8
			16384*8/70,      		//0x0170		POS8
			16384*9/70,					//0x0172		ADC9
			16384*9/70,					//0x0173		POS9
			16384*10/70,        	//0x0174		ADC10
			16384*10/70,					//0x0175		POS10
			16384*11/70,					//0x0176		ADC11
			16384*11/70,					//0x0177		POS11
			16384*12/70,					//0x0178		ADC12
			16384*12/70,					//0x0179		POS12
			16384*13/70,					//0x017A		ADC13
			16384*13/70,					//0x017B		POS13
			16384*14/70,					//0x017C		ADC14
			16384*14/70,					//0x017D		POS14
			16384*15/70,					//0x017E		ADC15
			16384*15/70,				//0x017F    POS15
		},
		{	  // 0x019*�ֶ� ADC�궨����
			16384*16/70,					//0x0180    ADC16
			16384*16/70,      		//0x0181		POS16
			16384*17/70,					//0x0182		ADC17
			16384*17/70,					//0x0183		POS17
			16384*18/70,        	//0x0184		ADC18
			16384*18/70,					//0x0185		POS18
			16384*19/70,					//0x0186		ADC19
			16384*19/70,					//0x0187		POS19
			16384*20/70,					//0x0188		ADC20
			16384*20/70,					//0x0189		POS20
			16384*21/70,					//0x018A		ADC21
			16384*21/70,					//0x018B		POS21
			16384*22/70,					//0x018C		ADC22
			16384*22/70,					//0x018D		POS22
			16384*23/70,					//0x018E		ADC23
			16384*23/70,					//0x018F    POS23
		},
		{	  // 0x01A*�ֶ� ADC�궨����
			16384*24/70,					//0x0190    ADC24
			16384*24/70,      		//0x0191		POS24
			16384*25/70,					//0x0192		ADC25
			16384*25/70,					//0x0193		POS25
			16384*26/70,        	//0x0194		ADC26
			16384*26/70,					//0x0195		POS26
			16384*27/70,					//0x0196		ADC27
			16384*27/70,					//0x0197		POS27
			16384*28/70,					//0x0198		ADC28
			16384*28/70,					//0x0199		POS28
			16384*29/70,					//0x019A		ADC29
			16384*29/70,			    //0x019B		POS29
			16384*30/70,					//0x019C		ADC30
			16384*30/70,					//0x019D		POS30
			16384*31/70,					//0x019C		ADC30
			16384*31/70,					//0x019D		POS30 
		},
		{	  // 0x01B*�ֶ� ADC�궨����
			16384*32/70,					//0x0190    ADC24
			16384*32/70,      		//0x0191		POS24
			16384*33/70,					//0x0192		ADC25
			16384*33/70,					//0x0193		POS25
			16384*34/70,        	//0x0194		ADC26
			16384*34/70,					//0x0195		POS26
			16384*35/70,					//0x0196		ADC27
			16384*35/70,					//0x0197		POS27
			16384*36/70,					//0x0198		ADC28
			16384*36/70,					//0x0199		POS28
			16384*37/70,					//0x019A		ADC29
			16384*37/70,			    //0x019B		POS29
			16384*38/70,					//0x019C		ADC30
			16384*38/70,					//0x019D		POS30
			16384*39/70,					//0x019C		ADC30
			16384*39/70,					//0x019D		POS30 
		},
		{	  // 0x01C*�ֶ� ADC�궨����
			16384*40/70,					//0x0190    ADC24
			16384*40/70,      		//0x0191		POS24
			16384*41/70,					//0x0192		ADC25
			16384*41/70,					//0x0193		POS25
			16384*42/70,        	//0x0194		ADC26
			16384*42/70,					//0x0195		POS26
			16384*43/70,					//0x0196		ADC27
			16384*43/70,					//0x0197		POS27
			16384*44/70,					//0x0198		ADC28
			16384*44/70,					//0x0199		POS28
			16384*45/70,					//0x019A		ADC29
			16384*45/70,			    //0x019B		POS29
			16384*46/70,					//0x019C		ADC30
			16384*46/70,					//0x019D		POS30
			16384*47/70,					//0x019C		ADC30
			16384*47/70,					//0x019D		POS30 
		},
		{	  // 0x01D*�ֶ� ADC�궨����
			16384*48/70,					//0x0190    ADC24
			16384*48/70,      		//0x0191		POS24
			16384*49/70,					//0x0192		ADC25
			16384*49/70,					//0x0193		POS25
			16384*50/70,        	//0x0194		ADC26
			16384*50/70,					//0x0195		POS26
			16384*51/70,					//0x0196		ADC27
			16384*51/70,					//0x0197		POS27
			16384*52/70,					//0x0198		ADC28
			16384*52/70,					//0x0199		POS28
			16384*53/70,					//0x019A		ADC29
			16384*53/70,			    //0x019B		POS29
			16384*54/70,					//0x019C		ADC30
			16384*54/70,					//0x019D		POS30
			16384*55/70,					//0x019C		ADC30
			16384*55/70,					//0x019D		POS30 
		},
		{	  // 0x01E*�ֶ� ADC�궨����
			16384*56/70,					//0x0190    ADC24
			16384*56/70,      		//0x0191		POS24
			16384*57/70,					//0x0192		ADC25
			16384*57/70,					//0x0193		POS25
			16384*58/70,        	//0x0194		ADC26
			16384*58/70,					//0x0195		POS26
			16384*59/70,					//0x0196		ADC27
			16384*59/70,					//0x0197		POS27
			16384*60/70,					//0x0198		ADC28
			16384*60/70,					//0x0199		POS28
			16384*61/70,					//0x019A		ADC29
			16384*61/70,			    //0x019B		POS29
			16384*62/70,					//0x019C		ADC30
			16384*62/70,					//0x019D		POS30
			16384*63/70,					//0x019C		ADC30
			16384*63/70,					//0x019D		POS30 
		},
		{	  // 0x01F*�ֶ� ADC�궨����
			16384*64/70,					//0x0190    ADC24
			16384*64/70,      		//0x0191		POS24
			16384*65/70,					//0x0192		ADC25
			16384*65/70,					//0x0193		POS25
			16384*66/70,        	//0x0194		ADC26
			16384*66/70,					//0x0195		POS26
			16384*67/70,					//0x0196		ADC27
			16384*67/70,					//0x0197		POS27
			16384*68/70,					//0x0198		ADC28
			16384*68/70,					//0x0199		POS28
			16384*69/70,					//0x019A		ADC29
			16384*69/70,			    //0x019B		POS29
			16384*70/70,					//0x019C		ADC30
			16384*70/70,					//0x019D		POS30
			16384*71/71,					//0x019C		ADC30
			16384*71/71,					//0x019D		POS30 
		},

};      // �û����ƼĴ���
      // �û����ƼĴ���
s16 g_UserCmdMap_Default[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN] = 
{
	{// 0x000*�ֶ�
			0xAA55,								//0x0000     
			0,					      		//0x0001		�豸����
      VERSION_CODE,					//0x0002		�̼��汾
      0,										//0x0003		SN
      0,				          	//0x0004		SN
      0,										//0x0005		SN
			1,										//0x0006		ID
      BAUD_UART_115200,		  //0x0007		������
      0,		  							//0x0008		�������
      0,			    					//0x0009		��ͣ
      0,										//0x000A		��ͣ����
      0,			    					//0x000B		��ԭ����
      0,										//0x000C		��������
			0,										//0x000D		Ȩ����֤��
			80,										//0x000E		��������ֵ
			0,										//0x000F		��������ֵ
	},
	{// 0x001*�ֶ�
			16384,								//0x0010    ��������ֵ PU 
			16384,					      //0x0011		����������  PU 
      -16384,								//0x0012		���������� PU 
      16384,								//0x0013		�г�����ֵ PU 
      0,				          	//0x0014		�г�����ֵ PU 
      0,										//0x0015		���ط��� 0 ��ѹΪ�� ��0����Ϊ��
			0,										//0x0016		
      0,		  							//0x0017		
      0,		  							//0x0018		
      0,			    					//0x0019		
      0,										//0x001A		
      0,			    					//0x001B		
      0,										//0x001C		
			0,										//0x001D		
			0,										//0x001E		
			0,										//0x001F		
	},
	{// 0x002*�ֶ�
			0,								   	//0x0020    ����ģʽ 0��λ 1�ŷ� 2�ٶ� 3���� 4���� 5�ٶ�����
			0,					         	//0x0021		��������
      0,								   	//0x0022		��������
      16384,								//0x0023		�ٶ�����
      0,				          	//0x0024		λ������
      16384,								//0x0025		���ٶ�����
			0,										//0x0026		ʵ��λ��
      0,		  							//0x0027		ʱ�����
      0,		  							//0x0028		ʵ���ٶ�
      0,			    					//0x0029		ʵ������ֵ
      0,										//0x002A		������
      0,			    					//0x002B		�¶�
      0,										//0x002C		��ѹ
			0,										//0x002D		
			0,										//0x002E		
			0,										//0x002F		
	},
	{// 0x003*�ֶ�
			0,								   	//0x0030
			0,					         	//0x0031
      0,								   	//0x0032
      0,										//0x0033
      0,				          	//0x0034		
      0,										//0x0035		
			0,										//0x0036	
      0,		  							//0x0037		
      0,		  							//0x0038		
      0,			    					//0x0039		
      0,										//0x003A		
      0,			    					//0x003B		
      0,										//0x003C		
			0,										//0x003D		
			0,										//0x003E		
			0,										//0x003F		
	},
};




// �ڴ���Ʊ���дȨ��
u8 g_CmdMap_bWR[CMDMAP_INDLEN][CMDMAP_SUBLEN] = 
{
			{	  // 0x010*�ֶ�
			WRFLG_R,										//0x0100
			WRFLG_RW,					      		//0x0101		//�����Ƿ��ϵ�
      WRFLG_RW,										//0x0102		//����ģʽ
      WRFLG_RW,										//0x0103		//SN1
      WRFLG_RW,				          	//0x0104		//SN2
      WRFLG_RW,										//0x0105		//SN3
			WRFLG_RW,										//0x0106		//�������ݵ�FLASH
      WRFLG_RW,		  							//0x0107		//����ʵ��λ�ã�����ʹ�ã������޸��ۼ�ֵ
      WRFLG_RW,		  							//0x0108		//�������г��ڵ��α������б궨
      WRFLG_RW,			    					//0x0109		//���������ʶʹ�ܣ�1���������裬2���������
      WRFLG_RW,										//0x010A		//��բ�ɿ���ʱ
      WRFLG_RW,			    					//0x010B		//��բ������ʱ
      WRFLG_RW,										//0x010C		//MOS����
			WRFLG_RW,										//0x010D		//�������,�����������������
			WRFLG_RW,										//0x010E		//����ٶȻ�����ģʽ��0-�ٶȻ���������ת��1-�ٶȻ����ڼ���ֵ���������˶���2-�ٶȻ�ģʽ�������¶��˶���3-�ٶȻ�ģʽ����������˶�
			WRFLG_RW,										//0x010F		//��ԭ�궨����			
		},
		{	  // 0x011*�ֶ� ����ֵ
			WRFLG_R,										//0x0110    �������ۻ�׼ֵmA L_16Bit
			WRFLG_R,					      		//0x0111		�������ۻ�׼ֵmA H_16Bit
      WRFLG_R,										//0x0112		����ת�ٱ��ۻ�׼ֵPPS L_16Bit
      WRFLG_R,										//0x0113		����ת�ٱ��ۻ�׼ֵPPS H_16Bit
      WRFLG_R,				          	//0x0114		�Ƹ����ٶȱ��ۻ�׼ֵ L_16Bit
      WRFLG_R,										//0x0115		�Ƹ����ٶȱ��ۻ�׼ֵ H_16Bit
			WRFLG_R,										//0x0116		λ�ñ��ۻ�׼ֵuM L_16Bit
      WRFLG_R,		  							//0x0117		λ�ñ��ۻ�׼ֵuM H_16Bit
      WRFLG_R,		  							//0x0118		����ֵ���ۻ�׼ֵPPS L_16Bit
      WRFLG_R,			    					//0x0119		����ֵ���ۻ�׼ֵPPS H_16Bit
      WRFLG_R,										//0x011A		���ٶȱ��ۻ�׼ֵPPS L_16Bit um/s^2
      WRFLG_R,			    					//0x011B		���ٶȱ��ۻ�׼ֵPPS H_16Bit um/s^2
      WRFLG_R,										//0x011C		
			WRFLG_RW,										//0x011D		
			WRFLG_RW,										//0x011E		
			WRFLG_RW,										//0x011F		
		},
		{	  // 0x012*�ֶ� ��������Ϣ
			WRFLG_RW,										//0x0120    �������
			WRFLG_RW,					      		//0x0121		������
      WRFLG_RW,										//0x0122		������ѹ
      WRFLG_RW,										//0x0123		��������
      WRFLG_RW,				          	//0x0124		��������
      WRFLG_RW,										//0x0125		���������
			WRFLG_RW,										//0x0126		�����Ƕ�ƫ����
      WRFLG_RW,		  						//0x0127		�ݸ��ݾ� um
      WRFLG_RW,		  							//0x0128		�����ٱ� *256
      WRFLG_RW,			    					//0x0129		λ�ÿ������� 0�ݸ� 1�ؽڽ� 2����
      WRFLG_RW,										//0x012A		
      WRFLG_RW,			    					//0x012B		
      WRFLG_RW,										//0x012C		
			WRFLG_RW,										//0x012D		
			WRFLG_RW,										//0x012E		
			WRFLG_RW,										//0x012F	
		},
		{	  // 0x013*�ֶ� ����Ŀ��ֵ
			WRFLG_RW,										//0x0130    ����ģʽ��ռ�ձ�(0-100)
			WRFLG_RW,					      		//0x0131		Ŀ�����ֵ L16bit
      WRFLG_RW,										//0x0132		Ŀ�����ֵ H16bit
      WRFLG_RW,										//0x0133		Ŀ���ٶ�ֵ L16bit
      WRFLG_RW,				          	//0x0134		Ŀ���ٶ�ֵ H16bit
      WRFLG_RW,										//0x0135		Ŀ��λ�� 		L16bit
			WRFLG_RW,										//0x0136		Ŀ��λ�� 		H16bit
      WRFLG_RW,		  							//0x0137		Ŀ��ؽ� 		L16bit
      WRFLG_RW,		  							//0x0138		Ŀ��ؽ� 		H16bit
      WRFLG_RW,			    					//0x0139		
      WRFLG_RW,										//0x013A		
      WRFLG_RW,			    					//0x013B		
      WRFLG_RW,										//0x013C		
			WRFLG_RW,										//0x013D		
			WRFLG_RW,										//0x013E		
			WRFLG_RW,										//0x013F	
		},
		{	  // 0x014*�ֶ� ��������ֵ
			WRFLG_RW,										//0x0140    
			WRFLG_RW,					      		//0x0141		
      WRFLG_RW,										//0x0142		
      WRFLG_RW,										//0x0143		
      WRFLG_RW,				          	//0x0144		
      WRFLG_RW,										//0x0145		
			WRFLG_RW,										//0x0146		
      WRFLG_RW,		  							//0x0147		
      WRFLG_RW,		  							//0x0148		
      WRFLG_RW,			    					//0x0149		
      WRFLG_RW,										//0x014A		
      WRFLG_RW,			    					//0x014B		
      WRFLG_RW,										//0x014C		
			WRFLG_RW,										//0x014D		
			WRFLG_RW,										//0x014E		
			WRFLG_RW,										//0x014F	
		},
		{	  // 0x015*�ֶ� ���ջ���ز���
			WRFLG_RW,										//0x0150    ���ջ�����������־
			WRFLG_RW,					      		//0x0151		������P����
      WRFLG_RW,										//0x0152		������I����
      WRFLG_RW,										//0x0153		������D����
      WRFLG_RW,				          	//0x0154		�ٶȻ�P����
      WRFLG_RW,										//0x0155		�ٶȻ�I����
			WRFLG_RW,										//0x0156		�ٶȻ�D����
      WRFLG_RW,		  							//0x0157		�ٶȻ���������
      WRFLG_RW,		  							//0x0158		λ�û�P����
      WRFLG_RW,			    					//0x0159		λ�û�I����
      WRFLG_RW,										//0x015A		λ�û�D����
      WRFLG_RW,			    					//0x015B		λ�û���������
      WRFLG_RW,										//0x015C		���ػ�P����
			WRFLG_RW,										//0x015D		���ػ�I����
			WRFLG_RW,										//0x015E		���ػ�D����
			WRFLG_RW,										//0x015F		���ػ���������
		},
		{	  // 0x016*�ֶ� ADC�궨����
			WRFLG_RW,					//0x0160    ADC0
			WRFLG_RW,      		//0x0161		POS0
			WRFLG_RW,					//0x0162		ADC1
			WRFLG_RW,					//0x0163		POS1
			WRFLG_RW,        	//0x0164		ADC2
			WRFLG_RW,					//0x0165		POS2
			WRFLG_RW,					//0x0166		ADC3
			WRFLG_RW,					//0x0167		POS3
			WRFLG_RW,					//0x0168		ADC4
			WRFLG_RW,					//0x0169		POS4
			WRFLG_RW,					//0x016A		ADC5
			WRFLG_RW,					//0x016B		POS5
			WRFLG_RW,					//0x016C		ADC6
			WRFLG_RW,					//0x016D		POS6
			WRFLG_RW,					//0x016E		ADC7
			WRFLG_RW,					//0x016F    POS7
		},
		{	  // 0x017*�ֶ� ADC�궨����
			WRFLG_RW,					//0x0170    ADC8
			WRFLG_RW,      		//0x0171		POS8
			WRFLG_RW,					//0x0172		ADC9
			WRFLG_RW,					//0x0173		POS9
			WRFLG_RW,        	//0x0174		ADC10
			WRFLG_RW,					//0x0175		POS10
			WRFLG_RW,					//0x0176		ADC11
			WRFLG_RW,					//0x0177		POS11
			WRFLG_RW,					//0x0178		ADC12
			WRFLG_RW,					//0x0179		POS12
			WRFLG_RW,					//0x017A		ADC13
			WRFLG_RW,					//0x017B		POS13
			WRFLG_RW,					//0x017C		ADC14
			WRFLG_RW,					//0x017D		POS14
			WRFLG_RW,					//0x017E		ADC15
			WRFLG_RW,				//0x017F    POS15
		},
		{	  // 0x018*�ֶ� ADC�궨����
			WRFLG_RW,					//0x0180    ADC16
			WRFLG_RW,      		//0x0181		POS16
			WRFLG_RW,					//0x0182		ADC17
			WRFLG_RW,					//0x0183		POS17
			WRFLG_RW,        	//0x0184		ADC18
			WRFLG_RW,					//0x0185		POS18
			WRFLG_RW,					//0x0186		ADC19
			WRFLG_RW,					//0x0187		POS19
			WRFLG_RW,					//0x0188		ADC20
			WRFLG_RW,				//0x0189		POS20
			WRFLG_RW,					//0x018A		ADC21
			WRFLG_RW,					//0x018B		POS21
			WRFLG_RW,					//0x018C		ADC22
			WRFLG_RW,					//0x018D		POS22
			WRFLG_RW,					//0x018E		ADC23
			WRFLG_RW,					//0x018F    POS23
		},
		{	  // 0x019*�ֶ� ADC�궨����
			WRFLG_RW,					//0x0190    ADC24
			WRFLG_RW,      		//0x0191		POS24
			WRFLG_RW,					//0x0192		ADC25
			WRFLG_RW,					//0x0193		POS25
			WRFLG_RW,        	//0x0194		ADC26
			WRFLG_RW,					//0x0195		POS26
			WRFLG_RW,					//0x0196		ADC27
			WRFLG_RW,					//0x0197		POS27
			WRFLG_RW,					//0x0198		ADC28
			WRFLG_RW,					//0x0199		POS28
			WRFLG_RW,					//0x019A		ADC29
			WRFLG_RW,			    //0x019B		POS29
			WRFLG_RW,					//0x019C		ADC30
			WRFLG_RW,					//0x019D		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},
		{	  // 0x01A*�ֶ� ADC�궨����
			WRFLG_RW,					//0x01A0    ADC24
			WRFLG_RW,      		//0x01A1		POS24
			WRFLG_RW,					//0x01A2		ADC25
			WRFLG_RW,					//0x01A3		POS25
			WRFLG_RW,        	//0x01A4		ADC26
			WRFLG_RW,					//0x01A5		POS26
			WRFLG_RW,					//0x01A6		ADC27
			WRFLG_RW,					//0x01A7		POS27
			WRFLG_RW,					//0x01A8		ADC28
			WRFLG_RW,					//0x01A9		POS28
			WRFLG_RW,					//0x01AA		ADC29
			WRFLG_RW,			    //0x01AB		POS29
			WRFLG_RW,					//0x01AC		ADC30
			WRFLG_RW,					//0x01AD		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},	
		{	  // 0x01B*�ֶ� ADC�궨����
			WRFLG_RW,					//0x01A0    ADC24
			WRFLG_RW,      		//0x01A1		POS24
			WRFLG_RW,					//0x01A2		ADC25
			WRFLG_RW,					//0x01A3		POS25
			WRFLG_RW,        	//0x01A4		ADC26
			WRFLG_RW,					//0x01A5		POS26
			WRFLG_RW,					//0x01A6		ADC27
			WRFLG_RW,					//0x01A7		POS27
			WRFLG_RW,					//0x01A8		ADC28
			WRFLG_RW,					//0x01A9		POS28
			WRFLG_RW,					//0x01AA		ADC29
			WRFLG_RW,			    //0x01AB		POS29
			WRFLG_RW,					//0x01AC		ADC30
			WRFLG_RW,					//0x01AD		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},	
		{	  // 0x01C*�ֶ� ADC�궨����
			WRFLG_RW,					//0x01A0    ADC24
			WRFLG_RW,      		//0x01A1		POS24
			WRFLG_RW,					//0x01A2		ADC25
			WRFLG_RW,					//0x01A3		POS25
			WRFLG_RW,        	//0x01A4		ADC26
			WRFLG_RW,					//0x01A5		POS26
			WRFLG_RW,					//0x01A6		ADC27
			WRFLG_RW,					//0x01A7		POS27
			WRFLG_RW,					//0x01A8		ADC28
			WRFLG_RW,					//0x01A9		POS28
			WRFLG_RW,					//0x01AA		ADC29
			WRFLG_RW,			    //0x01AB		POS29
			WRFLG_RW,					//0x01AC		ADC30
			WRFLG_RW,					//0x01AD		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},	
		{	  // 0x01D*�ֶ� ADC�궨����
			WRFLG_RW,					//0x01A0    ADC24
			WRFLG_RW,      		//0x01A1		POS24
			WRFLG_RW,					//0x01A2		ADC25
			WRFLG_RW,					//0x01A3		POS25
			WRFLG_RW,        	//0x01A4		ADC26
			WRFLG_RW,					//0x01A5		POS26
			WRFLG_RW,					//0x01A6		ADC27
			WRFLG_RW,					//0x01A7		POS27
			WRFLG_RW,					//0x01A8		ADC28
			WRFLG_RW,					//0x01A9		POS28
			WRFLG_RW,					//0x01AA		ADC29
			WRFLG_RW,			    //0x01AB		POS29
			WRFLG_RW,					//0x01AC		ADC30
			WRFLG_RW,					//0x01AD		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},	
		{	  // 0x01E*�ֶ� ADC�궨����
			WRFLG_RW,					//0x01A0    ADC24
			WRFLG_RW,      		//0x01A1		POS24
			WRFLG_RW,					//0x01A2		ADC25
			WRFLG_RW,					//0x01A3		POS25
			WRFLG_RW,        	//0x01A4		ADC26
			WRFLG_RW,					//0x01A5		POS26
			WRFLG_RW,					//0x01A6		ADC27
			WRFLG_RW,					//0x01A7		POS27
			WRFLG_RW,					//0x01A8		ADC28
			WRFLG_RW,					//0x01A9		POS28
			WRFLG_RW,					//0x01AA		ADC29
			WRFLG_RW,			    //0x01AB		POS29
			WRFLG_RW,					//0x01AC		ADC30
			WRFLG_RW,					//0x01AD		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},	
		{	  // 0x01F*�ֶ� ADC�궨����
			WRFLG_RW,					//0x01A0    ADC24
			WRFLG_RW,      		//0x01A1		POS24
			WRFLG_RW,					//0x01A2		ADC25
			WRFLG_RW,					//0x01A3		POS25
			WRFLG_RW,        	//0x01A4		ADC26
			WRFLG_RW,					//0x01A5		POS26
			WRFLG_RW,					//0x01A6		ADC27
			WRFLG_RW,					//0x01A7		POS27
			WRFLG_RW,					//0x01A8		ADC28
			WRFLG_RW,					//0x01A9		POS28
			WRFLG_RW,					//0x01AA		ADC29
			WRFLG_RW,			    //0x01AB		POS29
			WRFLG_RW,					//0x01AC		ADC30
			WRFLG_RW,					//0x01AD		POS30
			WRFLG_RW,										//0x019E		
			WRFLG_RW,										//0x019F    
		},
		{	  // 0x020*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x021*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x022*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x023*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x024*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x025*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x026*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x027*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x028*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x029*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02A*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02B*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02C*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02D*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},	
		{	  // 0x02E*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02F*�ֶ� ADC�궨����
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},				
};

u8 g_CmdMap_User_bWR[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN] = 
{
	{// 0x000*�ֶ�
			WRFLG_R,								//0x0000     
			WRFLG_R,					     	//0x0001		�豸����
      WRFLG_R,								//0x0002		�̼��汾
      WRFLG_R,								//0x0003		SN
      WRFLG_R,				        //0x0004		SN
      WRFLG_R,								//0x0005		SN
			WRFLG_RW,								//0x0006		ID
      WRFLG_RW,		  					//0x0007		������
      WRFLG_RW,		  					//0x0008		�������
      WRFLG_RW,			    			//0x0009		��ͣ
      WRFLG_RW,								//0x000A		��ͣ����
      WRFLG_RW,			    			//0x000B		��ԭ����
      WRFLG_RW,								//0x000C		��������
			WRFLG_RW,								//0x000D		Ȩ����֤��
			WRFLG_RW,								//0x000E		��������ֵ
			WRFLG_RW,								//0x000F		��������ֵ
	},
	{// 0x001*�ֶ�
			WRFLG_RW,								//0x0010    ��������ֵ PU 
			WRFLG_RW,					      //0x0011		����������  PU 
      WRFLG_RW,								//0x0012		���������� PU 
      WRFLG_RW,								//0x0013		�г�����ֵ PU 
      WRFLG_RW,				          	//0x0014		�г�����ֵ PU 
      WRFLG_RW,										//0x0015		
			WRFLG_RW,										//0x0016		
      WRFLG_RW,		  							//0x0017		
      WRFLG_RW,		  							//0x0018		
      WRFLG_RW,			    					//0x0019		
      WRFLG_RW,										//0x001A		
      WRFLG_RW,			    					//0x001B		
      WRFLG_RW,										//0x001C		
			WRFLG_RW,										//0x001D		
			WRFLG_RW,										//0x001E		
			WRFLG_RW,										//0x001F		
	},
	{// 0x002*�ֶ�
			WRFLG_RW,								   	//0x0020    ����ģʽ 0��λ 1�ŷ� 2�ٶ� 3���� 4���� 5�ٶ�����
			WRFLG_RW,					         	//0x0021		��������
      WRFLG_RW,								   	//0x0022		��������
      WRFLG_RW,								//0x0023		�ٶ�����
      WRFLG_RW,				          	//0x0024		λ������
      WRFLG_RW,								//0x0025		���ٶ�����
			WRFLG_R,										//0x0026		ʵ��λ��
      WRFLG_R,		  							//0x0027		ʱ�����
      WRFLG_R,		  							//0x0028		ʵ���ٶ�
      WRFLG_R,			    					//0x0029		ʵ������ֵ
      WRFLG_R,										//0x002A		������
      WRFLG_R,			    					//0x002B		�¶�
      WRFLG_R,										//0x002C		��ѹ
			WRFLG_R,										//0x002D		
			WRFLG_R,										//0x002E		
			WRFLG_R,										//0x002F		
	},
	{// 0x003*�ֶ�
			WRFLG_RW,								   	//0x0030
			WRFLG_RW,					         	//0x0031
      WRFLG_RW,								   	//0x0032
      WRFLG_RW,										//0x0033
      WRFLG_RW,				          	//0x0034		
      WRFLG_RW,										//0x0035		
			WRFLG_RW,										//0x0036	
      WRFLG_RW,		  							//0x0037		
      WRFLG_RW,		  							//0x0038		
      WRFLG_RW,			    					//0x0039		
      WRFLG_RW,										//0x003A		
      WRFLG_RW,			    					//0x003B		
      WRFLG_RW,										//0x003C		
			WRFLG_RW,										//0x003D		
			WRFLG_RW,										//0x003E		
			WRFLG_RW,										//0x003F		
	},
		{// 0x004*�ֶ�
			WRFLG_RW,								   	//0x0030
			WRFLG_RW,					         	//0x0031
      WRFLG_RW,								   	//0x0032
      WRFLG_RW,										//0x0033
      WRFLG_RW,				          	//0x0034		
      WRFLG_RW,										//0x0035		
			WRFLG_RW,										//0x0036	
      WRFLG_RW,		  							//0x0037		
      WRFLG_RW,		  							//0x0038		
      WRFLG_RW,			    					//0x0039		
      WRFLG_RW,										//0x003A		
      WRFLG_RW,			    					//0x003B		
      WRFLG_RW,										//0x003C		
			WRFLG_RW,										//0x003D		
			WRFLG_RW,										//0x003E		
			WRFLG_RW,										//0x003F		
	},
			{// 0x005*�ֶ�
			WRFLG_RW,								   	//0x0030
			WRFLG_RW,					         	//0x0031
      WRFLG_RW,								   	//0x0032
      WRFLG_RW,										//0x0033
      WRFLG_RW,				          	//0x0034		
      WRFLG_RW,										//0x0035		
			WRFLG_RW,										//0x0036	
      WRFLG_RW,		  							//0x0037		
      WRFLG_RW,		  							//0x0038		
      WRFLG_RW,			    					//0x0039		
      WRFLG_RW,										//0x003A		
      WRFLG_RW,			    					//0x003B		
      WRFLG_RW,										//0x003C		
			WRFLG_RW,										//0x003D		
			WRFLG_RW,										//0x003E		
			WRFLG_RW,										//0x003F		
	},
};
s16 g_Sector_test = 0; 			        
s16 g_AngleAtSector = 0;
s16  g_tk = 0 ;
s16  g_tk_1 = 0;
s16  g_t0 = 0 ;


struct WAVE_SIN_GEN_TYPE waveGen;


/*
*********************************************************************************************************
*	�� �� ��: System_RunPer10ms
*	����˵��: �ú���ÿ��10ms��Systick�жϵ���1�Ρ���� systick.c�Ķ�ʱ�жϷ������һЩ��Ҫ�����Դ���
*			��������Է��ڴ˺��������磺����ɨ�衢���������п��Ƶȡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void System_RunPer10ms(void)
{
	//KeyScan();		/* ����ɨ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: System_RunPer1ms
*	����˵��: �ú���ÿ��1ms��Systick�жϵ���1�Ρ���� systick.c�Ķ�ʱ�жϷ������һЩ��Ҫ�����Դ�����
*			������Է��ڴ˺��������磺��������ɨ�衣
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void System_RunPer1ms(void)
{
	//TOUCH_Scan();	/* ����ɨ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: System_Idle
*	����˵��: ����ʱִ�еĺ�����
*			 ������ȱʡΪ�ղ������û���������ι��������CPU��������ģʽ�Ĺ��ܡ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void System_Idle(void)
{
	/* --- ι�� */

	/* --- ��CPU�������ߣ���Systick��ʱ�жϻ��ѻ��������жϻ��� */

	/* ���� emWin ͼ�ο⣬���Բ���ͼ�ο���Ҫ����ѯ���� */
	//GUI_Exec();

	/* ���� uIP Э��ʵ�֣����Բ���uip��ѯ���� */
}


