#include "global.h"

// 全局变量
volatile u8 Motor_forbidden = 0;        // 马达禁止输出标志
volatile u8 Motor_emergercy_stop;
volatile u8 Motor_pause_flag;
volatile s16 Zero_s16 = 0x0000;         // SPI产生CLK信号使用
volatile u8 g_BeginPIDFlag = 0;		      // 开始PID
volatile s16 g_SysStatus = 0; 		      // 驱动器状态字
volatile s32 g_Timer1CCR = 0;		        // 计时器最终的输出占空比
volatile s16 g_bEnableHB = 0;
s16 g_Temperature = 250; 			          // 温度
u16 g_PowVoltage = 4800;			          // 输入电压
double Spd_ratio =0;
s32 Encode_PPR =0;
s32 Encode_PP_Half_R = 0;
volatile s32 pos_nm_MU150 = 0;
uint8_t Flag_error_read_flash = 0; 		//flash 读取错误
uint8_t Flag_error_set_preDrive = 0;	//设置预驱动芯片参数错误
uint8_t Flag_error_sensor_comm = 0;		//传感器模块通讯错误
uint8_t Flag_error_current_clib = 0;	//电流校准失败
uint8_t Flag_error_MU_SPI = 0;				//MU SPI通讯故障
uint8_t Flag_error_MU_value_instable;		//MU 信号不稳定
uint8_t Flag_error_torsion_dect = 0;	//形变量检测错误，负载端和电机端的角度偏差太大

s32 range_icmu_vlaue_at_stroke;
s32 vlaue_icmu_offset_overturn;
s32 range_icmu_vlaue_need_addMax;


/*-------------------------Profile-----------------------------------*/
//标幺值
float pos_p_vel_float;
s32* g_pPos_ref_base_um = (s32*)&g_CmdMap[SYS_POS_REF_BASE_UM_L];//位置标幺值
s32* g_pVel_ref_base_PPS = (s32*)&g_CmdMap[SYS_SPD_REF_BASE_PPS_L];//速度标幺值
s32* g_pVel_ref_base_umPS = (s32*)&g_CmdMap[SYS_SPD_REF_BASE_UMPS_L];//速度标幺值
s32* g_pCur_ref_base_mA = (s32*)&g_CmdMap[SYS_CUR_REF_BASE_MA_L];//电流标幺值
s32* g_pAcc_ref_base_mmPS2 = (s32*)&g_CmdMap[SYS_ACC_REF_BASE_L];//加速度标幺值
s32* g_pForce_ref_base_mg = (s32*)&g_CmdMap[SYS_FORCE_REF_BASE_L];//受力标幺值
s32  g_Vel_Max_PosPU_pSec = 0; //最高线速度 pu(位置) /s
s32 g_servo_cmd_period_cnt = 0; //高频位置伺服下，命令周期计时
s32 g_servo_cmd_period = 0; //高频位置伺服下，命令周期
s32 g_servo_cmd_pos_cur = 0;//高频位置伺服下，当前的位置设定
s32 g_servo_cmd_pos_last = 0;//高频位置伺服下，上一周期的位置设定
s32 pos_set_current_frm_CMD_PU = 0;
s32 pos_set_last_frm_CMD_PU = 0;
u8 status_Mode_POSITION_FORCE = 0; //位置+力控模式下的状态机 0 Idle 1 Profile 2 Force_touch
/*-------------------------位置环-----------------------------------*/
s32* g_pMU_value_offset = (s32*)&g_CmdMap[SYS_MU_MA_OFFSET_L];
s32* g_pMU_0mm_offset = (s32*)&g_CmdMap[SYS_MU_MA_0MM_BASE_L];
s32 rod_pos_Muti_Encoder = 0;							// 编码器数值累计值，即单圈变多圈后的数据
s32 gPos_Kp;															// 当前的实时的校准后的位置环比例系数
s32 gPos_Ki;															// 当前的实时的校准后的位置环比例系数
s32 gPos_ds;      												// 当前的实时的校准后的位置环死区
volatile u8 g_PosReg_runing = 0;		      // 位置环状态 1运行0停止
volatile u8 g_PosReg_Enbale = 0;		      // 位置环使能命令
volatile u8 g_PosReg_Disabel = 0;		      // 位置环储能命令
/*-------------------------速度环-----------------------------------*/
s32 gSpd_Kp;															// 当前的实时的校准后的速度环比例系数
s32 gSpd_Ki;															// 当前的实时的校准后的速度环积分系数
double gVel_PPS = 0;															// 
double Vel_PPS_raw = 0;
s32 gSpd_ds;      												// 当前的实时的校准后的速度环死区
volatile u8 g_SpdReg_runing = 0;		      // 速度环状态 1运行0停止
volatile u8 g_SpdReg_Enbale = 0;		      // 速度环使能命令
volatile u8 g_SpdReg_Disabel = 0;		      // 速度环储能命令
/*-------------------------电流环-----------------------------------*/
s32 gCur_Kp;															// 当前的实时的校准后的电流环比例系数
s32 gCur_Ki;															// 当前的实时的校准后的电流环积分系数
volatile u8 g_CurReg_runing = 0;		      // 速度环状态 1运行0停止
volatile u8 g_CurReg_Enbale = 0;		      // 速度环使能命令
volatile u8 g_CurReg_Disabel = 0;		      // 速度环储能命�
/*---------------------------磁编角度相关---------------------------------*/
volatile s32 g_ElectricAngle_act = 0;	    // 真实电角度
volatile s32 g_ElectricAngle_sim = 0;	    // 仿真电角度
volatile s32 g_ElectricAngle_15bit = 0;		// 电角度_32768
volatile s32 g_ElectricAngle_15bit_Raw = 0;		// 电角度_32768
volatile s32 g_MechanicsAngle_15bit = 0;	// 机械角度_32768
volatile s32 g_MechanicsAngle_15bit_last = 0;	// 机械角度_32768
volatile s32 g_ElectricAngle = 0; 	      // 当前电角度


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
s32	Screw_um = 0;           //螺距
s32	Mot_Pairs = 0;          //电机极对数
// 电流标定变量
s32	g_ZeroCur_MotorA;    // 相电流零点标定值
s32	g_ZeroCur_MotorB;    // 相电流零点标定值
s32	g_ZeroCur_MotorC;	   // 相电流零点标定值

s16 Gripper_Move = 0;  //0 IDEL 1 夹取 2 松开
u8 Flag_contact = 0;
u8 Flag_grip_F_OK = 0;
u8 Flag_grip_P_OK = 0;
u8 Flag_rellease_P_OK = 0;
// 内存控制表32位变量指针


// 内存控制表32位变量指针

s32 g_PosOffset = 0;				// 标定偏移量
s32 g_PosSet_Calibrate = 0;

s16 s_count_50ms = 0;
s16 s_count_5ms = 0;
s16 s_count_1ms = 0;
u8 Flag_1ms = 0;
u8 Flag_50ms = 0;
// 内存控制表相关
s16 g_ThreeLoopParaBefLock[CMDMAP_SUBLEN] = {0};
volatile s16 g_CmdMap[CMDMAP_LEN+USERCMDMAP_LEN] = {0};
s16 g_CmdMap_Default[CMDMAP_INDLEN][CMDMAP_SUBLEN] = 
{
		{	  // 0x010*字段
			0xAA55,								//0x0100
			0,					      		//0x0101		//马达是否上电
      0,										//0x0102		//工作模式
      0xFFFF,								//0x0103		//SN1
      0xFFFF,		          	//0x0104		//SN2
      0xFFFF,								//0x0105		//SN3
			0,										//0x0106		//保存数据到FLASH
      0,		  							//0x0107		//设置实际位置，调试使用，用来修改累计值
      0,		  							//0x0108		//对整个行程内的形变量进行标定
      0,			    					//0x0109		//电机参数辨识使能，1：测量电阻，2：测量电感
      80,										//0x010A		//抱闸松开延时
      80,			    					//0x010B		//抱闸抱死延时
      0,										//0x010C		//MOS测试
			0,										//0x010D		//电机测试,电机与编码器方向测试
			0,										//0x010E		//电机速度环测试模式，0-速度环下连续旋转，1-速度环下在极限值两端来回运动，2-速度环模式下向最下端运动，3-速度环模式下向最大方向运动
			0,										//0x010F		//还原标定参数			
		},
		{	  // 0x011*字段 标幺值
			0,										//0x0110    电流标幺基准值mA L_16Bit
			0,					      		//0x0111		电流标幺基准值mA H_16Bit
      0,										//0x0112		马达转速标幺基准值PPS L_16Bit
      0,										//0x0113		马达转速标幺基准值PPS H_16Bit
      0,				          	//0x0114		推杆线速度标幺基准值 L_16Bit
      0,										//0x0115		推杆线速度标幺基准值 H_16Bit
			0,										//0x0116		位置标幺基准值uM L_16Bit
      0,		  							//0x0117		位置标幺基准值uM H_16Bit
      0,		  							//0x0118		受力值标幺基准值PPS L_16Bit
      0,			    					//0x0119		受力值标幺基准值PPS H_16Bit
      0,										//0x011A		加速度标幺基准值PPS L_16Bit um/s^2
      0,			    					//0x011B		加速度标幺基准值PPS H_16Bit um/s^2
      0,										//0x011C		
			1638,									//0x011D		//系统内部内部阻力对应的电流标幺值
			8192,									//0x011E		//100% 输出力 对应的电流标幺值
			0,										//0x011F		
		},
		{	  // 0x012*字段 电机相关信息
			0,										//0x0120    电机内阻
			0,					      		//0x0121		电机电感
      0,										//0x0122		电机额定电压
      0,										//0x0123		电机额定电流
      15,				          	//0x0124		码盘线速
      1,										//0x0125		电机极对数
			0,										//0x0126		电机电角度偏移量
      250,		  						//0x0127		螺杆螺距 um
      11,		  							//0x0128		减速速比 *256
      0,			    					//0x0129		位置控制类型 0螺杆 1关节角 2其他
      0,										//0x012A		
      0,			    					//0x012B		
      0,										//0x012C		
			0,										//0x012D		
			0,										//0x012E		
			0,										//0x012F	
		},
		{	  // 0x013*字段 控制目标值
			0,										//0x0130    开环模式下占空比(0-100)
			0,					      		//0x0131		目标电流值 L16bit
      0,										//0x0132		目标电流值 H16bit
      0,										//0x0133		目标速度值 L16bit
      0,				          	//0x0134		目标速度值 H16bit
      0,										//0x0135		目标位置 L16bit
			0,										//0x0136		目标位置 H16bit
      0,		  							//0x0137		目标关节 L16bit
      0,		  							//0x0138		目标关节 H16bit
      0,			    					//0x0139		
      0,										//0x013A		
      0,			    					//0x013B		
      0,										//0x013C		
			0,										//0x013D		
			0,										//0x013E		
			0,										//0x013F	
		},
		{	  // 0x014*字段 控制限制值
			16384,										//0x0140    电流控制上限 PU
			-16384,					      		//0x0141		电流控制下限 PU
      16384,										//0x0142		速度控制上限 PU
      -16384,										//0x0143		速度控制下限 PU
      16384,				          	//0x0144		力度控制上限 PU
      -16384,										//0x0145		力度控制下限 PU
			16384,										//0x0146		位置控制上限 PU
      0,		  									//0x0147		位置控制下限 PU
      16384,		  							//0x0148		加速度控制极限 PU
      16384,			    					//0x0149		附加速度调节的极限 PU
      0,												//0x014A		
			2200,											//0x014B		
			2000,											//0x014C		
			0,												//0x014D
			0,												//0x014E
			0,												//0x014F			
		},
		{	  // 0x015*字段 三闭环相关参数
			0,										//0x0150    三闭环参数锁定标志
			90,					      		//0x0151		电流环P参数
      300,										//0x0152		电流环I参数
      0,										//0x0153		电流环D参数
      200,				          	//0x0154		速度环P参数
      400,										//0x0155		速度环I参数
			0,										//0x0156		速度环D参数
      4,		  							//0x0157		速度环死区参数
      8000,		  							//0x0158		位置环P参数
      100,			    					//0x0159		位置环I参数
      0,										//0x015A		位置环D参数
      2,			    					//0x015B		位置环死区参数
      400,										//0x015C		//力控环P参数
			8,										//0x015D		//力控环I参数
			10,										//0x015E		//力控环D参数
			50,										//0x015F		//力控环死区
		},
		{	  // 0x016*字段 通讯设置，状态信息帧的设置
			6,					          //0x01A0    状态信息帧的长度
			CMD_POS_ACT_PU,      	//0x01A1		状态信息帧第1位所放寄存器的地址
			CMD_CUR_ACT_PU,				//0x01A2		状态信息帧第2位所放寄存器的地址
			CMD_FORCE_ACT_PU,			//0x01A3		状态信息帧第3位所放寄存器的地址
			CMD_SPD_ACT_PU,       //0x01A4		状态信息帧第4位所放寄存器的地址
			CMD_ERROR,						//0x01A5		状态信息帧第5位所放寄存器的地址
			CMD_TEMP,							//0x01A6		状态信息帧第6位所放寄存器的地址
			0,										//0x01A7		状态信息帧第7位所放寄存器的地址
			0,										//0x01A8		状态信息帧第8位所放寄存器的地址
			0,										//0x01A9		状态信息帧第9位所放寄存器的地址
			0,										//0x01AA		状态信息帧第10位所放寄存器的地址
			0,			    					//0x01AB		状态信息帧第11位所放寄存器的地址
			0,										//0x01AC		状态信息帧第12位所放寄存器的地址
			0,										//0x01AD		状态信息帧第13位所放寄存器的地址
			0,										//0x01AE		
			0,										//0x01AF    
		},
		{	  // 0x017*字段 ADC标定参数
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
		{	  // 0x018*字段 ADC标定参数
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
		{	  // 0x019*字段 ADC标定参数
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
		{	  // 0x01A*字段 ADC标定参数
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
		{	  // 0x01B*字段 ADC标定参数
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
		{	  // 0x01C*字段 ADC标定参数
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
		{	  // 0x01D*字段 ADC标定参数
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
		{	  // 0x01E*字段 ADC标定参数
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
		{	  // 0x01F*字段 ADC标定参数
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

};      // 用户控制寄存器
      // 用户控制寄存器
s16 g_UserCmdMap_Default[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN] = 
{
	{// 0x000*字段
			0xAA55,								//0x0000     
			0,					      		//0x0001		设备类型
      VERSION_CODE,					//0x0002		固件版本
      0,										//0x0003		SN
      0,				          	//0x0004		SN
      0,										//0x0005		SN
			1,										//0x0006		ID
      BAUD_UART_115200,		  //0x0007		波特率
      0,		  							//0x0008		清除故障
      0,			    					//0x0009		急停
      0,										//0x000A		暂停运行
      0,			    					//0x000B		还原参数
      0,										//0x000C		参数保存
			0,										//0x000D		权限验证码
			80,										//0x000E		过温设置值
			0,										//0x000F		低温设置值
	},
	{// 0x001*字段
			16384,								//0x0010    过流设置值 PU 
			16384,					      //0x0011		正向最大输出  PU 
      -16384,								//0x0012		反向最大输出 PU 
      16384,								//0x0013		行程上限值 PU 
      0,				          	//0x0014		行程下限值 PU 
      0,										//0x0015		力控方向 0 挤压为正 非0拉升为正
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
	{// 0x002*字段
			0,								   	//0x0020    控制模式 0定位 1伺服 2速度 3力控 4电流 5速度力控
			0,					         	//0x0021		电流设置
      0,								   	//0x0022		力控设置
      16384,								//0x0023		速度设置
      0,				          	//0x0024		位置设置
      16384,								//0x0025		加速度设置
			0,										//0x0026		实际位置
      0,		  							//0x0027		时间电流
      0,		  							//0x0028		实际速度
      0,			    					//0x0029		实际受力值
      0,										//0x002A		错误码
      0,			    					//0x002B		温度
      0,										//0x002C		电压
			0,										//0x002D		
			0,										//0x002E		
			0,										//0x002F		
	},
	{// 0x003*字段
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




// 内存控制表读写权限
u8 g_CmdMap_bWR[CMDMAP_INDLEN][CMDMAP_SUBLEN] = 
{
			{	  // 0x010*字段
			WRFLG_R,										//0x0100
			WRFLG_RW,					      		//0x0101		//马达是否上电
      WRFLG_RW,										//0x0102		//工作模式
      WRFLG_RW,										//0x0103		//SN1
      WRFLG_RW,				          	//0x0104		//SN2
      WRFLG_RW,										//0x0105		//SN3
			WRFLG_RW,										//0x0106		//保存数据到FLASH
      WRFLG_RW,		  							//0x0107		//设置实际位置，调试使用，用来修改累计值
      WRFLG_RW,		  							//0x0108		//对整个行程内的形变量进行标定
      WRFLG_RW,			    					//0x0109		//电机参数辨识使能，1：测量电阻，2：测量电感
      WRFLG_RW,										//0x010A		//抱闸松开延时
      WRFLG_RW,			    					//0x010B		//抱闸抱死延时
      WRFLG_RW,										//0x010C		//MOS测试
			WRFLG_RW,										//0x010D		//电机测试,电机与编码器方向测试
			WRFLG_RW,										//0x010E		//电机速度环测试模式，0-速度环下连续旋转，1-速度环下在极限值两端来回运动，2-速度环模式下向最下端运动，3-速度环模式下向最大方向运动
			WRFLG_RW,										//0x010F		//还原标定参数			
		},
		{	  // 0x011*字段 标幺值
			WRFLG_R,										//0x0110    电流标幺基准值mA L_16Bit
			WRFLG_R,					      		//0x0111		电流标幺基准值mA H_16Bit
      WRFLG_R,										//0x0112		马达转速标幺基准值PPS L_16Bit
      WRFLG_R,										//0x0113		马达转速标幺基准值PPS H_16Bit
      WRFLG_R,				          	//0x0114		推杆线速度标幺基准值 L_16Bit
      WRFLG_R,										//0x0115		推杆线速度标幺基准值 H_16Bit
			WRFLG_R,										//0x0116		位置标幺基准值uM L_16Bit
      WRFLG_R,		  							//0x0117		位置标幺基准值uM H_16Bit
      WRFLG_R,		  							//0x0118		受力值标幺基准值PPS L_16Bit
      WRFLG_R,			    					//0x0119		受力值标幺基准值PPS H_16Bit
      WRFLG_R,										//0x011A		加速度标幺基准值PPS L_16Bit um/s^2
      WRFLG_R,			    					//0x011B		加速度标幺基准值PPS H_16Bit um/s^2
      WRFLG_R,										//0x011C		
			WRFLG_RW,										//0x011D		
			WRFLG_RW,										//0x011E		
			WRFLG_RW,										//0x011F		
		},
		{	  // 0x012*字段 电机相关信息
			WRFLG_RW,										//0x0120    电机内阻
			WRFLG_RW,					      		//0x0121		电机电感
      WRFLG_RW,										//0x0122		电机额定电压
      WRFLG_RW,										//0x0123		电机额定电流
      WRFLG_RW,				          	//0x0124		码盘线速
      WRFLG_RW,										//0x0125		电机极对数
			WRFLG_RW,										//0x0126		电机电角度偏移量
      WRFLG_RW,		  						//0x0127		螺杆螺距 um
      WRFLG_RW,		  							//0x0128		减速速比 *256
      WRFLG_RW,			    					//0x0129		位置控制类型 0螺杆 1关节角 2其他
      WRFLG_RW,										//0x012A		
      WRFLG_RW,			    					//0x012B		
      WRFLG_RW,										//0x012C		
			WRFLG_RW,										//0x012D		
			WRFLG_RW,										//0x012E		
			WRFLG_RW,										//0x012F	
		},
		{	  // 0x013*字段 控制目标值
			WRFLG_RW,										//0x0130    开环模式下占空比(0-100)
			WRFLG_RW,					      		//0x0131		目标电流值 L16bit
      WRFLG_RW,										//0x0132		目标电流值 H16bit
      WRFLG_RW,										//0x0133		目标速度值 L16bit
      WRFLG_RW,				          	//0x0134		目标速度值 H16bit
      WRFLG_RW,										//0x0135		目标位置 		L16bit
			WRFLG_RW,										//0x0136		目标位置 		H16bit
      WRFLG_RW,		  							//0x0137		目标关节 		L16bit
      WRFLG_RW,		  							//0x0138		目标关节 		H16bit
      WRFLG_RW,			    					//0x0139		
      WRFLG_RW,										//0x013A		
      WRFLG_RW,			    					//0x013B		
      WRFLG_RW,										//0x013C		
			WRFLG_RW,										//0x013D		
			WRFLG_RW,										//0x013E		
			WRFLG_RW,										//0x013F	
		},
		{	  // 0x014*字段 控制限制值
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
		{	  // 0x015*字段 三闭环相关参数
			WRFLG_RW,										//0x0150    三闭环参数锁定标志
			WRFLG_RW,					      		//0x0151		电流环P参数
      WRFLG_RW,										//0x0152		电流环I参数
      WRFLG_RW,										//0x0153		电流环D参数
      WRFLG_RW,				          	//0x0154		速度环P参数
      WRFLG_RW,										//0x0155		速度环I参数
			WRFLG_RW,										//0x0156		速度环D参数
      WRFLG_RW,		  							//0x0157		速度环死区参数
      WRFLG_RW,		  							//0x0158		位置环P参数
      WRFLG_RW,			    					//0x0159		位置环I参数
      WRFLG_RW,										//0x015A		位置环D参数
      WRFLG_RW,			    					//0x015B		位置环死区参数
      WRFLG_RW,										//0x015C		力控环P参数
			WRFLG_RW,										//0x015D		力控环I参数
			WRFLG_RW,										//0x015E		力控环D参数
			WRFLG_RW,										//0x015F		力控环死区参数
		},
		{	  // 0x016*字段 ADC标定参数
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
		{	  // 0x017*字段 ADC标定参数
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
		{	  // 0x018*字段 ADC标定参数
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
		{	  // 0x019*字段 ADC标定参数
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
		{	  // 0x01A*字段 ADC标定参数
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
		{	  // 0x01B*字段 ADC标定参数
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
		{	  // 0x01C*字段 ADC标定参数
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
		{	  // 0x01D*字段 ADC标定参数
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
		{	  // 0x01E*字段 ADC标定参数
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
		{	  // 0x01F*字段 ADC标定参数
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
		{	  // 0x020*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x021*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x022*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x023*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x024*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x025*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x026*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x027*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x028*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x029*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02A*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02B*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02C*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02D*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},	
		{	  // 0x02E*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02F*字段 ADC标定参数
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},				
};

u8 g_CmdMap_User_bWR[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN] = 
{
	{// 0x000*字段
			WRFLG_R,								//0x0000     
			WRFLG_R,					     	//0x0001		设备类型
      WRFLG_R,								//0x0002		固件版本
      WRFLG_R,								//0x0003		SN
      WRFLG_R,				        //0x0004		SN
      WRFLG_R,								//0x0005		SN
			WRFLG_RW,								//0x0006		ID
      WRFLG_RW,		  					//0x0007		波特率
      WRFLG_RW,		  					//0x0008		清除故障
      WRFLG_RW,			    			//0x0009		急停
      WRFLG_RW,								//0x000A		暂停运行
      WRFLG_RW,			    			//0x000B		还原参数
      WRFLG_RW,								//0x000C		参数保存
			WRFLG_RW,								//0x000D		权限验证码
			WRFLG_RW,								//0x000E		过温设置值
			WRFLG_RW,								//0x000F		低温设置值
	},
	{// 0x001*字段
			WRFLG_RW,								//0x0010    过流设置值 PU 
			WRFLG_RW,					      //0x0011		正向最大输出  PU 
      WRFLG_RW,								//0x0012		反向最大输出 PU 
      WRFLG_RW,								//0x0013		行程上限值 PU 
      WRFLG_RW,				          	//0x0014		行程下限值 PU 
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
	{// 0x002*字段
			WRFLG_RW,								   	//0x0020    控制模式 0定位 1伺服 2速度 3力控 4电流 5速度力控
			WRFLG_RW,					         	//0x0021		电流设置
      WRFLG_RW,								   	//0x0022		力控设置
      WRFLG_RW,								//0x0023		速度设置
      WRFLG_RW,				          	//0x0024		位置设置
      WRFLG_RW,								//0x0025		加速度设置
			WRFLG_R,										//0x0026		实际位置
      WRFLG_R,		  							//0x0027		时间电流
      WRFLG_R,		  							//0x0028		实际速度
      WRFLG_R,			    					//0x0029		实际受力值
      WRFLG_R,										//0x002A		错误码
      WRFLG_R,			    					//0x002B		温度
      WRFLG_R,										//0x002C		电压
			WRFLG_R,										//0x002D		
			WRFLG_R,										//0x002E		
			WRFLG_R,										//0x002F		
	},
	{// 0x003*字段
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
		{// 0x004*字段
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
			{// 0x005*字段
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
*	函 数 名: System_RunPer10ms
*	功能说明: 该函数每隔10ms被Systick中断调用1次。详见 systick.c的定时中断服务程序。一些需要周期性处理
*			的事务可以放在此函数。比如：按键扫描、蜂鸣器鸣叫控制等。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void System_RunPer10ms(void)
{
	//KeyScan();		/* 按键扫描 */
}

/*
*********************************************************************************************************
*	函 数 名: System_RunPer1ms
*	功能说明: 该函数每隔1ms被Systick中断调用1次。详见 systick.c的定时中断服务程序。一些需要周期性处理的
*			事务可以放在此函数。比如：触摸坐标扫描。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void System_RunPer1ms(void)
{
	//TOUCH_Scan();	/* 触摸扫描 */
}

/*
*********************************************************************************************************
*	函 数 名: System_Idle
*	功能说明: 空闲时执行的函数。
*			 本函数缺省为空操作。用户可以添加喂狗、设置CPU进入休眠模式的功能。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void System_Idle(void)
{
	/* --- 喂狗 */

	/* --- 让CPU进入休眠，由Systick定时中断唤醒或者其他中断唤醒 */

	/* 对于 emWin 图形库，可以插入图形库需要的轮询函数 */
	//GUI_Exec();

	/* 对于 uIP 协议实现，可以插入uip轮询函数 */
}


