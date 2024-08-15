#include "global.h"

// È«¾Ö±äÁ¿
volatile u8 Motor_forbidden = 0;        // Âí´ï½ûÖ¹Êä³ö±êÖ¾
volatile u8 Motor_emergercy_stop;
volatile u8 Motor_pause_flag;
volatile s16 Zero_s16 = 0x0000;         // SPI²úÉúCLKÐÅºÅÊ¹ÓÃ
volatile u8 g_BeginPIDFlag = 0;		      // ¿ªÊ¼PID
volatile s16 g_SysStatus = 0; 		      // Çý¶¯Æ÷×´Ì¬×Ö
volatile s32 g_Timer1CCR = 0;		        // ¼ÆÊ±Æ÷×îÖÕµÄÊä³öÕ¼¿Õ±È
volatile s16 g_bEnableHB = 0;
s16 g_Temperature = 250; 			          // ÎÂ¶È
u16 g_PowVoltage = 4800;			          // ÊäÈëµçÑ¹
double Spd_ratio =0;
s32 Encode_PPR =0;
s32 Encode_PP_Half_R = 0;
volatile s32 pos_nm_MU150 = 0;
uint8_t Flag_error_read_flash = 0; 		//flash ¶ÁÈ¡´íÎó
uint8_t Flag_error_set_preDrive = 0;	//ÉèÖÃÔ¤Çý¶¯Ð¾Æ¬²ÎÊý´íÎó
uint8_t Flag_error_sensor_comm = 0;		//´«¸ÐÆ÷Ä£¿éÍ¨Ñ¶´íÎó
uint8_t Flag_error_current_clib = 0;	//µçÁ÷Ð£×¼Ê§°Ü
uint8_t Flag_error_MU_SPI = 0;				//MU SPIÍ¨Ñ¶¹ÊÕÏ
uint8_t Flag_error_MU_value_instable;		//MU ÐÅºÅ²»ÎÈ¶¨
uint8_t Flag_error_torsion_dect = 0;	//ÐÎ±äÁ¿¼ì²â´íÎó£¬¸ºÔØ¶ËºÍµç»ú¶ËµÄ½Ç¶ÈÆ«²îÌ«´ó

s32 range_icmu_vlaue_at_stroke;
s32 vlaue_icmu_offset_overturn;
s32 range_icmu_vlaue_need_addMax;


/*-------------------------Profile-----------------------------------*/
//±êçÛÖµ
float pos_p_vel_float;
s32* g_pPos_ref_base_um = (s32*)&g_CmdMap[SYS_POS_REF_BASE_UM_L];//Î»ÖÃ±êçÛÖµ
s32* g_pVel_ref_base_PPS = (s32*)&g_CmdMap[SYS_SPD_REF_BASE_PPS_L];//ËÙ¶È±êçÛÖµ
s32* g_pVel_ref_base_umPS = (s32*)&g_CmdMap[SYS_SPD_REF_BASE_UMPS_L];//ËÙ¶È±êçÛÖµ
s32* g_pCur_ref_base_mA = (s32*)&g_CmdMap[SYS_CUR_REF_BASE_MA_L];//µçÁ÷±êçÛÖµ
s32* g_pAcc_ref_base_mmPS2 = (s32*)&g_CmdMap[SYS_ACC_REF_BASE_L];//¼ÓËÙ¶È±êçÛÖµ
s32* g_pForce_ref_base_mg = (s32*)&g_CmdMap[SYS_FORCE_REF_BASE_L];//ÊÜÁ¦±êçÛÖµ
s32  g_Vel_Max_PosPU_pSec = 0; //×î¸ßÏßËÙ¶È pu(Î»ÖÃ) /s
s32 g_servo_cmd_period_cnt = 0; //¸ßÆµÎ»ÖÃËÅ·þÏÂ£¬ÃüÁîÖÜÆÚ¼ÆÊ±
s32 g_servo_cmd_period = 0; //¸ßÆµÎ»ÖÃËÅ·þÏÂ£¬ÃüÁîÖÜÆÚ
s32 g_servo_cmd_pos_cur = 0;//¸ßÆµÎ»ÖÃËÅ·þÏÂ£¬µ±Ç°µÄÎ»ÖÃÉè¶¨
s32 g_servo_cmd_pos_last = 0;//¸ßÆµÎ»ÖÃËÅ·þÏÂ£¬ÉÏÒ»ÖÜÆÚµÄÎ»ÖÃÉè¶¨
s32 pos_set_current_frm_CMD_PU = 0;
s32 pos_set_last_frm_CMD_PU = 0;
u8 status_Mode_POSITION_FORCE = 0; //Î»ÖÃ+Á¦¿ØÄ£Ê½ÏÂµÄ×´Ì¬»ú 0 Idle 1 Profile 2 Force_touch
/*-------------------------Î»ÖÃ»·-----------------------------------*/
s32* g_pMU_value_offset = (s32*)&g_CmdMap[SYS_MU_MA_OFFSET_L];
s32* g_pMU_0mm_offset = (s32*)&g_CmdMap[SYS_MU_MA_0MM_BASE_L];
s32 rod_pos_Muti_Encoder = 0;							// ±àÂëÆ÷ÊýÖµÀÛ¼ÆÖµ£¬¼´µ¥È¦±ä¶àÈ¦ºóµÄÊý¾Ý
s32 gPos_Kp;															// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄÎ»ÖÃ»·±ÈÀýÏµÊý
s32 gPos_Ki;															// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄÎ»ÖÃ»·±ÈÀýÏµÊý
s32 gPos_ds;      												// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄÎ»ÖÃ»·ËÀÇø
volatile u8 g_PosReg_runing = 0;		      // Î»ÖÃ»·×´Ì¬ 1ÔËÐÐ0Í£Ö¹
volatile u8 g_PosReg_Enbale = 0;		      // Î»ÖÃ»·Ê¹ÄÜÃüÁî
volatile u8 g_PosReg_Disabel = 0;		      // Î»ÖÃ»·´¢ÄÜÃüÁî
/*-------------------------ËÙ¶È»·-----------------------------------*/
s32 gSpd_Kp;															// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄËÙ¶È»·±ÈÀýÏµÊý
s32 gSpd_Ki;															// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄËÙ¶È»·»ý·ÖÏµÊý
double gVel_PPS = 0;															// 
double Vel_PPS_raw = 0;
s32 gSpd_ds;      												// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄËÙ¶È»·ËÀÇø
volatile u8 g_SpdReg_runing = 0;		      // ËÙ¶È»·×´Ì¬ 1ÔËÐÐ0Í£Ö¹
volatile u8 g_SpdReg_Enbale = 0;		      // ËÙ¶È»·Ê¹ÄÜÃüÁî
volatile u8 g_SpdReg_Disabel = 0;		      // ËÙ¶È»·´¢ÄÜÃüÁî
/*-------------------------µçÁ÷»·-----------------------------------*/
s32 gCur_Kp;															// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄµçÁ÷»·±ÈÀýÏµÊý
s32 gCur_Ki;															// µ±Ç°µÄÊµÊ±µÄÐ£×¼ºóµÄµçÁ÷»·»ý·ÖÏµÊý
volatile u8 g_CurReg_runing = 0;		      // ËÙ¶È»·×´Ì¬ 1ÔËÐÐ0Í£Ö¹
volatile u8 g_CurReg_Enbale = 0;		      // ËÙ¶È»·Ê¹ÄÜÃüÁî
volatile u8 g_CurReg_Disabel = 0;		      // ËÙ¶È»·´¢ÄÜÃüÁ
/*---------------------------´Å±à½Ç¶ÈÏà¹Ø---------------------------------*/
volatile s32 g_ElectricAngle_act = 0;	    // ÕæÊµµç½Ç¶È
volatile s32 g_ElectricAngle_sim = 0;	    // ·ÂÕæµç½Ç¶È
volatile s32 g_ElectricAngle_15bit = 0;		// µç½Ç¶È_32768
volatile s32 g_ElectricAngle_15bit_Raw = 0;		// µç½Ç¶È_32768
volatile s32 g_MechanicsAngle_15bit = 0;	// »úÐµ½Ç¶È_32768
volatile s32 g_MechanicsAngle_15bit_last = 0;	// »úÐµ½Ç¶È_32768
volatile s32 g_ElectricAngle = 0; 	      // µ±Ç°µç½Ç¶È


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
s32	Screw_um = 0;           //ÂÝ¾à
s32	Mot_Pairs = 0;          //µç»ú¼«¶ÔÊý
// µçÁ÷±ê¶¨±äÁ¿
s32	g_ZeroCur_MotorA;    // ÏàµçÁ÷Áãµã±ê¶¨Öµ
s32	g_ZeroCur_MotorB;    // ÏàµçÁ÷Áãµã±ê¶¨Öµ
s32	g_ZeroCur_MotorC;	   // ÏàµçÁ÷Áãµã±ê¶¨Öµ

s16 Gripper_Move = 0;  //0 IDEL 1 ¼ÐÈ¡ 2 ËÉ¿ª
u8 Flag_contact = 0;
u8 Flag_grip_F_OK = 0;
u8 Flag_grip_P_OK = 0;
u8 Flag_rellease_P_OK = 0;
// ÄÚ´æ¿ØÖÆ±í32Î»±äÁ¿Ö¸Õë


// ÄÚ´æ¿ØÖÆ±í32Î»±äÁ¿Ö¸Õë

s32 g_PosOffset = 0;				// ±ê¶¨Æ«ÒÆÁ¿
s32 g_PosSet_Calibrate = 0;

s16 s_count_50ms = 0;
s16 s_count_5ms = 0;
s16 s_count_1ms = 0;
u8 Flag_1ms = 0;
u8 Flag_50ms = 0;
// ÄÚ´æ¿ØÖÆ±íÏà¹Ø
s16 g_ThreeLoopParaBefLock[CMDMAP_SUBLEN] = {0};
volatile s16 g_CmdMap[CMDMAP_LEN+USERCMDMAP_LEN] = {0};
s16 g_CmdMap_Default[CMDMAP_INDLEN][CMDMAP_SUBLEN] = 
{
		{	  // 0x010*×Ö¶Î
			0xAA55,								//0x0100
			0,					      		//0x0101		//Âí´ïÊÇ·ñÉÏµç
      0,										//0x0102		//¹¤×÷Ä£Ê½
      0xFFFF,								//0x0103		//SN1
      0xFFFF,		          	//0x0104		//SN2
      0xFFFF,								//0x0105		//SN3
			0,										//0x0106		//±£´æÊý¾Ýµ½FLASH
      0,		  							//0x0107		//ÉèÖÃÊµ¼ÊÎ»ÖÃ£¬µ÷ÊÔÊ¹ÓÃ£¬ÓÃÀ´ÐÞ¸ÄÀÛ¼ÆÖµ
      0,		  							//0x0108		//¶ÔÕû¸öÐÐ³ÌÄÚµÄÐÎ±äÁ¿½øÐÐ±ê¶¨
      0,			    					//0x0109		//µç»ú²ÎÊý±æÊ¶Ê¹ÄÜ£¬1£º²âÁ¿µç×è£¬2£º²âÁ¿µç¸Ð
      80,										//0x010A		//±§Õ¢ËÉ¿ªÑÓÊ±
      80,			    					//0x010B		//±§Õ¢±§ËÀÑÓÊ±
      0,										//0x010C		//MOS²âÊÔ
			0,										//0x010D		//µç»ú²âÊÔ,µç»úÓë±àÂëÆ÷·½Ïò²âÊÔ
			0,										//0x010E		//µç»úËÙ¶È»·²âÊÔÄ£Ê½£¬0-ËÙ¶È»·ÏÂÁ¬ÐøÐý×ª£¬1-ËÙ¶È»·ÏÂÔÚ¼«ÏÞÖµÁ½¶ËÀ´»ØÔË¶¯£¬2-ËÙ¶È»·Ä£Ê½ÏÂÏò×îÏÂ¶ËÔË¶¯£¬3-ËÙ¶È»·Ä£Ê½ÏÂÏò×î´ó·½ÏòÔË¶¯
			0,										//0x010F		//»¹Ô­±ê¶¨²ÎÊý			
		},
		{	  // 0x011*×Ö¶Î ±êçÛÖµ
			0,										//0x0110    µçÁ÷±êçÛ»ù×¼ÖµmA L_16Bit
			0,					      		//0x0111		µçÁ÷±êçÛ»ù×¼ÖµmA H_16Bit
      0,										//0x0112		Âí´ï×ªËÙ±êçÛ»ù×¼ÖµPPS L_16Bit
      0,										//0x0113		Âí´ï×ªËÙ±êçÛ»ù×¼ÖµPPS H_16Bit
      0,				          	//0x0114		ÍÆ¸ËÏßËÙ¶È±êçÛ»ù×¼Öµ L_16Bit
      0,										//0x0115		ÍÆ¸ËÏßËÙ¶È±êçÛ»ù×¼Öµ H_16Bit
			0,										//0x0116		Î»ÖÃ±êçÛ»ù×¼ÖµuM L_16Bit
      0,		  							//0x0117		Î»ÖÃ±êçÛ»ù×¼ÖµuM H_16Bit
      0,		  							//0x0118		ÊÜÁ¦Öµ±êçÛ»ù×¼ÖµPPS L_16Bit
      0,			    					//0x0119		ÊÜÁ¦Öµ±êçÛ»ù×¼ÖµPPS H_16Bit
      0,										//0x011A		¼ÓËÙ¶È±êçÛ»ù×¼ÖµPPS L_16Bit um/s^2
      0,			    					//0x011B		¼ÓËÙ¶È±êçÛ»ù×¼ÖµPPS H_16Bit um/s^2
      0,										//0x011C		
			1638,									//0x011D		//ÏµÍ³ÄÚ²¿ÄÚ²¿×èÁ¦¶ÔÓ¦µÄµçÁ÷±êçÛÖµ
			8192,									//0x011E		//100% Êä³öÁ¦ ¶ÔÓ¦µÄµçÁ÷±êçÛÖµ
			0,										//0x011F		
		},
		{	  // 0x012*×Ö¶Î µç»úÏà¹ØÐÅÏ¢
			0,										//0x0120    µç»úÄÚ×è
			0,					      		//0x0121		µç»úµç¸Ð
      0,										//0x0122		µç»ú¶î¶¨µçÑ¹
      0,										//0x0123		µç»ú¶î¶¨µçÁ÷
      15,				          	//0x0124		ÂëÅÌÏßËÙ
      1,										//0x0125		µç»ú¼«¶ÔÊý
			0,										//0x0126		µç»úµç½Ç¶ÈÆ«ÒÆÁ¿
      250,		  						//0x0127		ÂÝ¸ËÂÝ¾à um
      11,		  							//0x0128		¼õËÙËÙ±È *256
      0,			    					//0x0129		Î»ÖÃ¿ØÖÆÀàÐÍ 0ÂÝ¸Ë 1¹Ø½Ú½Ç 2ÆäËû
      0,										//0x012A		
      0,			    					//0x012B		
      0,										//0x012C		
			0,										//0x012D		
			0,										//0x012E		
			0,										//0x012F	
		},
		{	  // 0x013*×Ö¶Î ¿ØÖÆÄ¿±êÖµ
			0,										//0x0130    ¿ª»·Ä£Ê½ÏÂÕ¼¿Õ±È(0-100)
			0,					      		//0x0131		Ä¿±êµçÁ÷Öµ L16bit
      0,										//0x0132		Ä¿±êµçÁ÷Öµ H16bit
      0,										//0x0133		Ä¿±êËÙ¶ÈÖµ L16bit
      0,				          	//0x0134		Ä¿±êËÙ¶ÈÖµ H16bit
      0,										//0x0135		Ä¿±êÎ»ÖÃ L16bit
			0,										//0x0136		Ä¿±êÎ»ÖÃ H16bit
      0,		  							//0x0137		Ä¿±ê¹Ø½Ú L16bit
      0,		  							//0x0138		Ä¿±ê¹Ø½Ú H16bit
      0,			    					//0x0139		
      0,										//0x013A		
      0,			    					//0x013B		
      0,										//0x013C		
			0,										//0x013D		
			0,										//0x013E		
			0,										//0x013F	
		},
		{	  // 0x014*×Ö¶Î ¿ØÖÆÏÞÖÆÖµ
			16384,										//0x0140    µçÁ÷¿ØÖÆÉÏÏÞ PU
			-16384,					      		//0x0141		µçÁ÷¿ØÖÆÏÂÏÞ PU
      16384,										//0x0142		ËÙ¶È¿ØÖÆÉÏÏÞ PU
      -16384,										//0x0143		ËÙ¶È¿ØÖÆÏÂÏÞ PU
      16384,				          	//0x0144		Á¦¶È¿ØÖÆÉÏÏÞ PU
      -16384,										//0x0145		Á¦¶È¿ØÖÆÏÂÏÞ PU
			16384,										//0x0146		Î»ÖÃ¿ØÖÆÉÏÏÞ PU
      0,		  									//0x0147		Î»ÖÃ¿ØÖÆÏÂÏÞ PU
      16384,		  							//0x0148		¼ÓËÙ¶È¿ØÖÆ¼«ÏÞ PU
      16384,			    					//0x0149		¸½¼ÓËÙ¶Èµ÷½ÚµÄ¼«ÏÞ PU
      0,												//0x014A		
			2200,											//0x014B		
			2000,											//0x014C		
			0,												//0x014D
			0,												//0x014E
			0,												//0x014F			
		},
		{	  // 0x015*×Ö¶Î Èý±Õ»·Ïà¹Ø²ÎÊý
			0,										//0x0150    Èý±Õ»·²ÎÊýËø¶¨±êÖ¾
			90,					      		//0x0151		µçÁ÷»·P²ÎÊý
      300,										//0x0152		µçÁ÷»·I²ÎÊý
      0,										//0x0153		µçÁ÷»·D²ÎÊý
      200,				          	//0x0154		ËÙ¶È»·P²ÎÊý
      400,										//0x0155		ËÙ¶È»·I²ÎÊý
			0,										//0x0156		ËÙ¶È»·D²ÎÊý
      4,		  							//0x0157		ËÙ¶È»·ËÀÇø²ÎÊý
      8000,		  							//0x0158		Î»ÖÃ»·P²ÎÊý
      100,			    					//0x0159		Î»ÖÃ»·I²ÎÊý
      0,										//0x015A		Î»ÖÃ»·D²ÎÊý
      2,			    					//0x015B		Î»ÖÃ»·ËÀÇø²ÎÊý
      400,										//0x015C		//Á¦¿Ø»·P²ÎÊý
			8,										//0x015D		//Á¦¿Ø»·I²ÎÊý
			10,										//0x015E		//Á¦¿Ø»·D²ÎÊý
			50,										//0x015F		//Á¦¿Ø»·ËÀÇø
		},
		{	  // 0x016*×Ö¶Î Í¨Ñ¶ÉèÖÃ£¬×´Ì¬ÐÅÏ¢Ö¡µÄÉèÖÃ
			6,					          //0x01A0    ×´Ì¬ÐÅÏ¢Ö¡µÄ³¤¶È
			CMD_POS_ACT_PU,      	//0x01A1		×´Ì¬ÐÅÏ¢Ö¡µÚ1Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			CMD_CUR_ACT_PU,				//0x01A2		×´Ì¬ÐÅÏ¢Ö¡µÚ2Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			CMD_FORCE_ACT_PU,			//0x01A3		×´Ì¬ÐÅÏ¢Ö¡µÚ3Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			CMD_SPD_ACT_PU,       //0x01A4		×´Ì¬ÐÅÏ¢Ö¡µÚ4Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			CMD_ERROR,						//0x01A5		×´Ì¬ÐÅÏ¢Ö¡µÚ5Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			CMD_TEMP,							//0x01A6		×´Ì¬ÐÅÏ¢Ö¡µÚ6Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01A7		×´Ì¬ÐÅÏ¢Ö¡µÚ7Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01A8		×´Ì¬ÐÅÏ¢Ö¡µÚ8Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01A9		×´Ì¬ÐÅÏ¢Ö¡µÚ9Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01AA		×´Ì¬ÐÅÏ¢Ö¡µÚ10Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,			    					//0x01AB		×´Ì¬ÐÅÏ¢Ö¡µÚ11Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01AC		×´Ì¬ÐÅÏ¢Ö¡µÚ12Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01AD		×´Ì¬ÐÅÏ¢Ö¡µÚ13Î»Ëù·Å¼Ä´æÆ÷µÄµØÖ·
			0,										//0x01AE		
			0,										//0x01AF    
		},
		{	  // 0x017*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x018*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x019*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01A*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01B*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01C*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01D*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01E*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01F*×Ö¶Î ADC±ê¶¨²ÎÊý
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

};      // ÓÃ»§¿ØÖÆ¼Ä´æÆ÷
      // ÓÃ»§¿ØÖÆ¼Ä´æÆ÷
s16 g_UserCmdMap_Default[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN] = 
{
	{// 0x000*×Ö¶Î
			0xAA55,								//0x0000     
			0,					      		//0x0001		Éè±¸ÀàÐÍ
      VERSION_CODE,					//0x0002		¹Ì¼þ°æ±¾
      0,										//0x0003		SN
      0,				          	//0x0004		SN
      0,										//0x0005		SN
			1,										//0x0006		ID
      BAUD_UART_115200,		  //0x0007		²¨ÌØÂÊ
      0,		  							//0x0008		Çå³ý¹ÊÕÏ
      0,			    					//0x0009		¼±Í£
      0,										//0x000A		ÔÝÍ£ÔËÐÐ
      0,			    					//0x000B		»¹Ô­²ÎÊý
      0,										//0x000C		²ÎÊý±£´æ
			0,										//0x000D		È¨ÏÞÑéÖ¤Âë
			80,										//0x000E		¹ýÎÂÉèÖÃÖµ
			0,										//0x000F		µÍÎÂÉèÖÃÖµ
	},
	{// 0x001*×Ö¶Î
			16384,								//0x0010    ¹ýÁ÷ÉèÖÃÖµ PU 
			16384,					      //0x0011		ÕýÏò×î´óÊä³ö  PU 
      -16384,								//0x0012		·´Ïò×î´óÊä³ö PU 
      16384,								//0x0013		ÐÐ³ÌÉÏÏÞÖµ PU 
      0,				          	//0x0014		ÐÐ³ÌÏÂÏÞÖµ PU 
      0,										//0x0015		Á¦¿Ø·½Ïò 0 ¼·Ñ¹ÎªÕý ·Ç0À­ÉýÎªÕý
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
	{// 0x002*×Ö¶Î
			0,								   	//0x0020    ¿ØÖÆÄ£Ê½ 0¶¨Î» 1ËÅ·þ 2ËÙ¶È 3Á¦¿Ø 4µçÁ÷ 5ËÙ¶ÈÁ¦¿Ø
			0,					         	//0x0021		µçÁ÷ÉèÖÃ
      0,								   	//0x0022		Á¦¿ØÉèÖÃ
      16384,								//0x0023		ËÙ¶ÈÉèÖÃ
      0,				          	//0x0024		Î»ÖÃÉèÖÃ
      16384,								//0x0025		¼ÓËÙ¶ÈÉèÖÃ
			0,										//0x0026		Êµ¼ÊÎ»ÖÃ
      0,		  							//0x0027		Ê±¼äµçÁ÷
      0,		  							//0x0028		Êµ¼ÊËÙ¶È
      0,			    					//0x0029		Êµ¼ÊÊÜÁ¦Öµ
      0,										//0x002A		´íÎóÂë
      0,			    					//0x002B		ÎÂ¶È
      0,										//0x002C		µçÑ¹
			0,										//0x002D		
			0,										//0x002E		
			0,										//0x002F		
	},
	{// 0x003*×Ö¶Î
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




// ÄÚ´æ¿ØÖÆ±í¶ÁÐ´È¨ÏÞ
u8 g_CmdMap_bWR[CMDMAP_INDLEN][CMDMAP_SUBLEN] = 
{
			{	  // 0x010*×Ö¶Î
			WRFLG_R,										//0x0100
			WRFLG_RW,					      		//0x0101		//Âí´ïÊÇ·ñÉÏµç
      WRFLG_RW,										//0x0102		//¹¤×÷Ä£Ê½
      WRFLG_RW,										//0x0103		//SN1
      WRFLG_RW,				          	//0x0104		//SN2
      WRFLG_RW,										//0x0105		//SN3
			WRFLG_RW,										//0x0106		//±£´æÊý¾Ýµ½FLASH
      WRFLG_RW,		  							//0x0107		//ÉèÖÃÊµ¼ÊÎ»ÖÃ£¬µ÷ÊÔÊ¹ÓÃ£¬ÓÃÀ´ÐÞ¸ÄÀÛ¼ÆÖµ
      WRFLG_RW,		  							//0x0108		//¶ÔÕû¸öÐÐ³ÌÄÚµÄÐÎ±äÁ¿½øÐÐ±ê¶¨
      WRFLG_RW,			    					//0x0109		//µç»ú²ÎÊý±æÊ¶Ê¹ÄÜ£¬1£º²âÁ¿µç×è£¬2£º²âÁ¿µç¸Ð
      WRFLG_RW,										//0x010A		//±§Õ¢ËÉ¿ªÑÓÊ±
      WRFLG_RW,			    					//0x010B		//±§Õ¢±§ËÀÑÓÊ±
      WRFLG_RW,										//0x010C		//MOS²âÊÔ
			WRFLG_RW,										//0x010D		//µç»ú²âÊÔ,µç»úÓë±àÂëÆ÷·½Ïò²âÊÔ
			WRFLG_RW,										//0x010E		//µç»úËÙ¶È»·²âÊÔÄ£Ê½£¬0-ËÙ¶È»·ÏÂÁ¬ÐøÐý×ª£¬1-ËÙ¶È»·ÏÂÔÚ¼«ÏÞÖµÁ½¶ËÀ´»ØÔË¶¯£¬2-ËÙ¶È»·Ä£Ê½ÏÂÏò×îÏÂ¶ËÔË¶¯£¬3-ËÙ¶È»·Ä£Ê½ÏÂÏò×î´ó·½ÏòÔË¶¯
			WRFLG_RW,										//0x010F		//»¹Ô­±ê¶¨²ÎÊý			
		},
		{	  // 0x011*×Ö¶Î ±êçÛÖµ
			WRFLG_R,										//0x0110    µçÁ÷±êçÛ»ù×¼ÖµmA L_16Bit
			WRFLG_R,					      		//0x0111		µçÁ÷±êçÛ»ù×¼ÖµmA H_16Bit
      WRFLG_R,										//0x0112		Âí´ï×ªËÙ±êçÛ»ù×¼ÖµPPS L_16Bit
      WRFLG_R,										//0x0113		Âí´ï×ªËÙ±êçÛ»ù×¼ÖµPPS H_16Bit
      WRFLG_R,				          	//0x0114		ÍÆ¸ËÏßËÙ¶È±êçÛ»ù×¼Öµ L_16Bit
      WRFLG_R,										//0x0115		ÍÆ¸ËÏßËÙ¶È±êçÛ»ù×¼Öµ H_16Bit
			WRFLG_R,										//0x0116		Î»ÖÃ±êçÛ»ù×¼ÖµuM L_16Bit
      WRFLG_R,		  							//0x0117		Î»ÖÃ±êçÛ»ù×¼ÖµuM H_16Bit
      WRFLG_R,		  							//0x0118		ÊÜÁ¦Öµ±êçÛ»ù×¼ÖµPPS L_16Bit
      WRFLG_R,			    					//0x0119		ÊÜÁ¦Öµ±êçÛ»ù×¼ÖµPPS H_16Bit
      WRFLG_R,										//0x011A		¼ÓËÙ¶È±êçÛ»ù×¼ÖµPPS L_16Bit um/s^2
      WRFLG_R,			    					//0x011B		¼ÓËÙ¶È±êçÛ»ù×¼ÖµPPS H_16Bit um/s^2
      WRFLG_R,										//0x011C		
			WRFLG_RW,										//0x011D		
			WRFLG_RW,										//0x011E		
			WRFLG_RW,										//0x011F		
		},
		{	  // 0x012*×Ö¶Î µç»úÏà¹ØÐÅÏ¢
			WRFLG_RW,										//0x0120    µç»úÄÚ×è
			WRFLG_RW,					      		//0x0121		µç»úµç¸Ð
      WRFLG_RW,										//0x0122		µç»ú¶î¶¨µçÑ¹
      WRFLG_RW,										//0x0123		µç»ú¶î¶¨µçÁ÷
      WRFLG_RW,				          	//0x0124		ÂëÅÌÏßËÙ
      WRFLG_RW,										//0x0125		µç»ú¼«¶ÔÊý
			WRFLG_RW,										//0x0126		µç»úµç½Ç¶ÈÆ«ÒÆÁ¿
      WRFLG_RW,		  						//0x0127		ÂÝ¸ËÂÝ¾à um
      WRFLG_RW,		  							//0x0128		¼õËÙËÙ±È *256
      WRFLG_RW,			    					//0x0129		Î»ÖÃ¿ØÖÆÀàÐÍ 0ÂÝ¸Ë 1¹Ø½Ú½Ç 2ÆäËû
      WRFLG_RW,										//0x012A		
      WRFLG_RW,			    					//0x012B		
      WRFLG_RW,										//0x012C		
			WRFLG_RW,										//0x012D		
			WRFLG_RW,										//0x012E		
			WRFLG_RW,										//0x012F	
		},
		{	  // 0x013*×Ö¶Î ¿ØÖÆÄ¿±êÖµ
			WRFLG_RW,										//0x0130    ¿ª»·Ä£Ê½ÏÂÕ¼¿Õ±È(0-100)
			WRFLG_RW,					      		//0x0131		Ä¿±êµçÁ÷Öµ L16bit
      WRFLG_RW,										//0x0132		Ä¿±êµçÁ÷Öµ H16bit
      WRFLG_RW,										//0x0133		Ä¿±êËÙ¶ÈÖµ L16bit
      WRFLG_RW,				          	//0x0134		Ä¿±êËÙ¶ÈÖµ H16bit
      WRFLG_RW,										//0x0135		Ä¿±êÎ»ÖÃ 		L16bit
			WRFLG_RW,										//0x0136		Ä¿±êÎ»ÖÃ 		H16bit
      WRFLG_RW,		  							//0x0137		Ä¿±ê¹Ø½Ú 		L16bit
      WRFLG_RW,		  							//0x0138		Ä¿±ê¹Ø½Ú 		H16bit
      WRFLG_RW,			    					//0x0139		
      WRFLG_RW,										//0x013A		
      WRFLG_RW,			    					//0x013B		
      WRFLG_RW,										//0x013C		
			WRFLG_RW,										//0x013D		
			WRFLG_RW,										//0x013E		
			WRFLG_RW,										//0x013F	
		},
		{	  // 0x014*×Ö¶Î ¿ØÖÆÏÞÖÆÖµ
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
		{	  // 0x015*×Ö¶Î Èý±Õ»·Ïà¹Ø²ÎÊý
			WRFLG_RW,										//0x0150    Èý±Õ»·²ÎÊýËø¶¨±êÖ¾
			WRFLG_RW,					      		//0x0151		µçÁ÷»·P²ÎÊý
      WRFLG_RW,										//0x0152		µçÁ÷»·I²ÎÊý
      WRFLG_RW,										//0x0153		µçÁ÷»·D²ÎÊý
      WRFLG_RW,				          	//0x0154		ËÙ¶È»·P²ÎÊý
      WRFLG_RW,										//0x0155		ËÙ¶È»·I²ÎÊý
			WRFLG_RW,										//0x0156		ËÙ¶È»·D²ÎÊý
      WRFLG_RW,		  							//0x0157		ËÙ¶È»·ËÀÇø²ÎÊý
      WRFLG_RW,		  							//0x0158		Î»ÖÃ»·P²ÎÊý
      WRFLG_RW,			    					//0x0159		Î»ÖÃ»·I²ÎÊý
      WRFLG_RW,										//0x015A		Î»ÖÃ»·D²ÎÊý
      WRFLG_RW,			    					//0x015B		Î»ÖÃ»·ËÀÇø²ÎÊý
      WRFLG_RW,										//0x015C		Á¦¿Ø»·P²ÎÊý
			WRFLG_RW,										//0x015D		Á¦¿Ø»·I²ÎÊý
			WRFLG_RW,										//0x015E		Á¦¿Ø»·D²ÎÊý
			WRFLG_RW,										//0x015F		Á¦¿Ø»·ËÀÇø²ÎÊý
		},
		{	  // 0x016*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x017*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x018*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x019*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01A*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01B*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01C*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01D*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01E*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x01F*×Ö¶Î ADC±ê¶¨²ÎÊý
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
		{	  // 0x020*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x021*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x022*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x023*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x024*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x025*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x026*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x027*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x028*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x029*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02A*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02B*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02C*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02D*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},	
		{	  // 0x02E*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},
		{	  // 0x02F*×Ö¶Î ADC±ê¶¨²ÎÊý
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,
			WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,WRFLG_RW,					 
		},				
};

u8 g_CmdMap_User_bWR[USERCMDMAP_INDLEN][USERCMDMAP_SUBLEN] = 
{
	{// 0x000*×Ö¶Î
			WRFLG_R,								//0x0000     
			WRFLG_R,					     	//0x0001		Éè±¸ÀàÐÍ
      WRFLG_R,								//0x0002		¹Ì¼þ°æ±¾
      WRFLG_R,								//0x0003		SN
      WRFLG_R,				        //0x0004		SN
      WRFLG_R,								//0x0005		SN
			WRFLG_RW,								//0x0006		ID
      WRFLG_RW,		  					//0x0007		²¨ÌØÂÊ
      WRFLG_RW,		  					//0x0008		Çå³ý¹ÊÕÏ
      WRFLG_RW,			    			//0x0009		¼±Í£
      WRFLG_RW,								//0x000A		ÔÝÍ£ÔËÐÐ
      WRFLG_RW,			    			//0x000B		»¹Ô­²ÎÊý
      WRFLG_RW,								//0x000C		²ÎÊý±£´æ
			WRFLG_RW,								//0x000D		È¨ÏÞÑéÖ¤Âë
			WRFLG_RW,								//0x000E		¹ýÎÂÉèÖÃÖµ
			WRFLG_RW,								//0x000F		µÍÎÂÉèÖÃÖµ
	},
	{// 0x001*×Ö¶Î
			WRFLG_RW,								//0x0010    ¹ýÁ÷ÉèÖÃÖµ PU 
			WRFLG_RW,					      //0x0011		ÕýÏò×î´óÊä³ö  PU 
      WRFLG_RW,								//0x0012		·´Ïò×î´óÊä³ö PU 
      WRFLG_RW,								//0x0013		ÐÐ³ÌÉÏÏÞÖµ PU 
      WRFLG_RW,				          	//0x0014		ÐÐ³ÌÏÂÏÞÖµ PU 
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
	{// 0x002*×Ö¶Î
			WRFLG_RW,								   	//0x0020    ¿ØÖÆÄ£Ê½ 0¶¨Î» 1ËÅ·þ 2ËÙ¶È 3Á¦¿Ø 4µçÁ÷ 5ËÙ¶ÈÁ¦¿Ø
			WRFLG_RW,					         	//0x0021		µçÁ÷ÉèÖÃ
      WRFLG_RW,								   	//0x0022		Á¦¿ØÉèÖÃ
      WRFLG_RW,								//0x0023		ËÙ¶ÈÉèÖÃ
      WRFLG_RW,				          	//0x0024		Î»ÖÃÉèÖÃ
      WRFLG_RW,								//0x0025		¼ÓËÙ¶ÈÉèÖÃ
			WRFLG_R,										//0x0026		Êµ¼ÊÎ»ÖÃ
      WRFLG_R,		  							//0x0027		Ê±¼äµçÁ÷
      WRFLG_R,		  							//0x0028		Êµ¼ÊËÙ¶È
      WRFLG_R,			    					//0x0029		Êµ¼ÊÊÜÁ¦Öµ
      WRFLG_R,										//0x002A		´íÎóÂë
      WRFLG_R,			    					//0x002B		ÎÂ¶È
      WRFLG_R,										//0x002C		µçÑ¹
			WRFLG_R,										//0x002D		
			WRFLG_R,										//0x002E		
			WRFLG_R,										//0x002F		
	},
	{// 0x003*×Ö¶Î
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
		{// 0x004*×Ö¶Î
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
			{// 0x005*×Ö¶Î
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
*	º¯ Êý Ãû: System_RunPer10ms
*	¹¦ÄÜËµÃ÷: ¸Ãº¯ÊýÃ¿¸ô10ms±»SystickÖÐ¶Ïµ÷ÓÃ1´Î¡£Ïê¼û systick.cµÄ¶¨Ê±ÖÐ¶Ï·þÎñ³ÌÐò¡£Ò»Ð©ÐèÒªÖÜÆÚÐÔ´¦Àí
*			µÄÊÂÎñ¿ÉÒÔ·ÅÔÚ´Ëº¯Êý¡£±ÈÈç£º°´¼üÉ¨Ãè¡¢·äÃùÆ÷Ãù½Ð¿ØÖÆµÈ¡£
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void System_RunPer10ms(void)
{
	//KeyScan();		/* °´¼üÉ¨Ãè */
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: System_RunPer1ms
*	¹¦ÄÜËµÃ÷: ¸Ãº¯ÊýÃ¿¸ô1ms±»SystickÖÐ¶Ïµ÷ÓÃ1´Î¡£Ïê¼û systick.cµÄ¶¨Ê±ÖÐ¶Ï·þÎñ³ÌÐò¡£Ò»Ð©ÐèÒªÖÜÆÚÐÔ´¦ÀíµÄ
*			ÊÂÎñ¿ÉÒÔ·ÅÔÚ´Ëº¯Êý¡£±ÈÈç£º´¥Ãþ×ø±êÉ¨Ãè¡£
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void System_RunPer1ms(void)
{
	//TOUCH_Scan();	/* ´¥ÃþÉ¨Ãè */
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: System_Idle
*	¹¦ÄÜËµÃ÷: ¿ÕÏÐÊ±Ö´ÐÐµÄº¯Êý¡£
*			 ±¾º¯ÊýÈ±Ê¡Îª¿Õ²Ù×÷¡£ÓÃ»§¿ÉÒÔÌí¼ÓÎ¹¹·¡¢ÉèÖÃCPU½øÈëÐÝÃßÄ£Ê½µÄ¹¦ÄÜ¡£
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void System_Idle(void)
{
	/* --- Î¹¹· */

	/* --- ÈÃCPU½øÈëÐÝÃß£¬ÓÉSystick¶¨Ê±ÖÐ¶Ï»½ÐÑ»òÕßÆäËûÖÐ¶Ï»½ÐÑ */

	/* ¶ÔÓÚ emWin Í¼ÐÎ¿â£¬¿ÉÒÔ²åÈëÍ¼ÐÎ¿âÐèÒªµÄÂÖÑ¯º¯Êý */
	//GUI_Exec();

	/* ¶ÔÓÚ uIP Ð­ÒéÊµÏÖ£¬¿ÉÒÔ²åÈëuipÂÖÑ¯º¯Êý */
}


