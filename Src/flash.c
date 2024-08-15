#include "flash.h"
#include "global.h"

//���㽫���õ�����ҳFlash��Size��16λ��Ϊ��λ��
static u32 Flash_PagesMask(u32 Size)
{
  u32 pagenumber = 0x0;
  u32 size = Size;

  Size >>= 1;
  if ((size % PAGE_SIZE) != 0)
  {
    pagenumber = (size / PAGE_SIZE) + 1;
  }
  else
  {
    pagenumber = size / PAGE_SIZE;
  }
  return pagenumber;
}

//׼��flash�ռ䣬������Ҫ��̵�ҳ
//AddressΪ��ʼ��ַ��LenΪ���ݳ��ȣ���16λ��Ϊ��λ
static s32 Flash_Prepared(u32 Address, u32 Len)
{
  u32 NbrOfPage = 0;
  u8 i;
	u32 PageError;
	
	FLASH_EraseInitTypeDef  FLASH_EraseInit;
	HAL_StatusTypeDef FLASHStatus = HAL_OK;
	
	HAL_FLASH_Unlock();
	
	FLASH_EraseInit.Banks = 0X01;
	FLASH_EraseInit.NbPages = 0X01;
	FLASH_EraseInit.Page = Address>>11;
	FLASH_EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;



	FLASHStatus = HAL_FLASHEx_Erase(&FLASH_EraseInit,&PageError); // ��ҳ����
	
	HAL_FLASH_Lock();
	
	if(FLASHStatus != HAL_OK)
	{
		return 0;
	}

  return 1;
}

//��16λ��Ϊ��λ������д��Flash
s32 Flash_Write(u32 Address, const u64* pData, u32 Len)
{
  u32 i;
  u32 FlashDestination = Address;
  u32 FlashSource;
  HAL_StatusTypeDef FLASHStatus = HAL_OK;
	uint32_t TypeProgram;
	
	TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD;

  if (!Flash_Prepared(Address,Len))
    return 0;
 
	
	HAL_FLASH_Unlock();
  //��ʼ���
  FlashSource = (u32)pData;
	
	
  for (i = 0; i<Len; i++)
  {
    FLASHStatus =  HAL_FLASH_Program(TypeProgram,FlashDestination, *(u64*)FlashSource);
    if (FLASHStatus != HAL_OK)
    {
      return 0;
    }
    if (*(u16*)FlashDestination != *(u16*)FlashSource)  // У��
    {
      return 0;
    }
    FlashDestination += 8;
    FlashSource += 8;
  }
 HAL_FLASH_Lock();
  return 1;
}
//��ȡFlash�е����ݣ�16λ��Ϊ��λ��
s32 Flash_Read(u32 Address, u16* Readbuff, u32 Len)
{
  u32 i;
  for (i=0; i<Len; i++)
  {
    Readbuff[i] = ((u16*)Address)[i];
  }
  return 1;
}
// ��ȡFlash�е��ڴ���Ʊ�����
s32 Flash_ReadCmdmap(void)
{
  Flash_Read(CMDMAP_ADD, (u16*)(&g_CmdMap[USERCMDMAP_LEN]), (CMDMAP_LEN));   //��ȡϵͳ����
	Flash_Read(USERCMD_MAP_ADD,(u16*)(&g_CmdMap[0]), (USERCMDMAP_LEN));//��ȡ�û����ݲ���
	g_CmdMap[CMD_SPD_SET_PU] = 0;
  g_CmdMap[CMD_CUR_SET_PU] = 0;
	g_CmdMap[CMD_POS_SET_PU] = g_CmdMap[CMD_POS_ACT_PU];
  g_CmdMap[TAG_OPEN_PWM] = 0;
  g_CmdMap[SYS_SAVE_TO_FLASH] = 0;
  g_CmdMap[SYS_REC_FORCE_ERROR_CURVE] = 0;
  g_CmdMap[CMD_CLEAR_ERROR] = 0;
  g_CmdMap[CMD_ERROR] = 0;
  g_CmdMap[SEV_PARAME_LOCKED] = 1; 
  return 1;
}

// �����ڴ���Ʊ���Flash
s32 Flash_SaveCmdmap(void)
{
  DISABLE_INT();      // �ر����ж�
  //if (bsp_WriteCpuFlash(CMDMAP_ADD, (u8*)g_CmdMap, (CMDMAP_LEN<<1)))
	if (Flash_Write(CMDMAP_ADD, (u64*)(&g_CmdMap[USERCMDMAP_LEN]), (CMDMAP_LEN>>2)))
  {
    ENABLE_INT();     // �������ж�
    return 0;
  }
  ENABLE_INT();
  return 1;
}
s32 Flash_SaveUserCmdmap(void)//�����û�����
{
  DISABLE_INT();      // �ر����ж�
  if (Flash_Write(USERCMD_MAP_ADD, (u64*)(&g_CmdMap[0]), (USERCMDMAP_LEN>>2)))
  {
    ENABLE_INT();     // �������ж�
    return 0;
  }
  ENABLE_INT();
  return 1;
}
// ��ʼ���ڴ���Ʊ�
s32 Flash_Init(void)
{
	s32 temp = 0;
	s32 result = 1;
	Flash_ReadCmdmap();
  if(g_CmdMap[USERCMDMAP_LEN] != g_CmdMap_Default[0][0])
  {
    memcpy((void*)(&g_CmdMap[USERCMDMAP_LEN]), g_CmdMap_Default, (CMDMAP_LEN<<1));		
		pos_linearity_set_default();
    g_CmdMap[SYS_SAVE_TO_FLASH] = 0;
    result = 0;
		Flag_error_read_flash = 1;
  }
	if(g_CmdMap[0] != g_UserCmdMap_Default[0][0])
  {
    memcpy((void*)g_CmdMap, g_UserCmdMap_Default, (USERCMDMAP_LEN<<1));
    g_CmdMap[SYS_SAVE_TO_FLASH] = 0;
    result = 0;
		Flag_error_read_flash = 1;
  }	
	range_icmu_vlaue_at_stroke = (int64_t)(LENGTH_UM_MAGNETIC_GRID)*(int64_t)(MAX_VALUE_ICMU)/(int64_t)DISTANCE_UM_ICMU;
	vlaue_icmu_offset_overturn = MAX_VALUE_ICMU - range_icmu_vlaue_at_stroke;
	range_icmu_vlaue_need_addMax = range_icmu_vlaue_at_stroke - (MAX_VALUE_ICMU - *g_pMU_value_offset);
	pos_linearity_ini();
  memcpy(g_ThreeLoopParaBefLock, (void*)(&g_CmdMap[SEV_PARAME_LOCKED]), (CMDMAP_SUBLEN<<1));
	g_CmdMap[CMD_ACCESS_CODE] = 0x0000;
	g_CmdMap[CMD_FW_VERSION] = VERSION_CODE;
	g_CmdMap[CMD_SN_PART1] = g_CmdMap[SYS_SN_PART1];
	g_CmdMap[CMD_SN_PART2] = g_CmdMap[SYS_SN_PART2];
	g_CmdMap[CMD_SN_PART3] = g_CmdMap[SYS_SN_PART3];	
	g_CmdMap[TAG_OPEN_PWM] = 0;
	if(g_CmdMap[MOT_ENC_LINES]<0 ||g_CmdMap[MOT_ENC_LINES]>25)
	{
		g_CmdMap[MOT_ENC_LINES] = 16;
	}
	Encode_PPR = 4095;//(0x00000001<<g_CmdMap[MOT_ENC_LINES])-1;
  Encode_PP_Half_R = 2048;//(0x00000001<<(g_CmdMap[MOT_ENC_LINES]-1));
	temp = (0x00000001<<g_CmdMap[MOT_ENC_LINES])-1;
  Spd_ratio =(60.0*(double)SPEED_TIM_FREQUENCY/((double)Encode_PPR));  //�����ٶȵ�ϵ��
	EN_0 = 	0;
	EN_30 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 30 / 360+0.5f);
	EN_45 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 45 / 360+0.5f);
	EN_60 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 60 / 360+0.5f);
	EN_90 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 90 / 360+0.5f);
	EN_120 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 120 / 360+0.5f);
	EN_150 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 150 / 360+0.5f);
	EN_180 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 180 / 360+0.5f);
	EN_210 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 210 / 360+0.5f);
	EN_240 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 240 / 360+0.5f);
	EN_270 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 270 / 360+0.5f);
	EN_300 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 300 / 360+0.5f);
	EN_330 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 330 / 360+0.5f);
	EN_360 = 	(s32)((float)temp / g_CmdMap[MOT_PAIRS] * 360 / 360+0.5f);	
	g_Encode_offset = (int)g_CmdMap[MOT_EANGLE_OFFSET];
	g_Encode_offset_EN = (g_Encode_offset*EN_360/360);
	if(g_CmdMap[TAG_WORK_MODE] == MODE_POSITION)
	{
		g_CmdMap[CMD_SPD_SET_PU] = 16384;
	}
	else
	{
		g_CmdMap[CMD_SPD_SET_PU] = 0;
	}
  g_CmdMap[CMD_CUR_SET_PU] = 0;
	g_CmdMap[CMD_OVER_TEMP_SET] = 80;
	g_CmdMap[SYS_MOT_TEST] = 0;
	g_CmdMap[SYS_MU_COMM_TO_PC] = 0;//Ĭ���ϵ��ȡMU����	
	g_CmdMap[SCREW_UM_RES] = 330;//�ݾ� 380um
	g_CmdMap[GEAR_RATIO_8BIT] = GEAR_8BIT;	
	g_CmdMap[CMD_E_STOP] =0;
	g_CmdMap[CMD_PAUSE] =0;
	g_CmdMap[SYS_SPD_ADD_LIMIT_PU] = 3276;
	g_CmdMap[CMD_ACC_SET_PU] = 8192;//4096;
	g_CmdMap[CMD_SPD_SOFT_SET_PU] = 819;//4096;
	*g_pPos_ref_base_um = STROKE_UM_RATED;//�г̻�׼ֵ10mm
	*g_pVel_ref_base_PPS = 23000*4096/60;//�ٶȱ���ֵ��׼ֵ
	*g_pVel_ref_base_umPS = (int64_t)(*g_pVel_ref_base_PPS)*g_CmdMap[SCREW_UM_RES]*256/(4096*g_CmdMap[GEAR_RATIO_8BIT]);//�ٶȱ���ֵ��׼ֵ,�ٱ�Ϊ11	
	*g_pAcc_ref_base_mmPS2 = 500;//500;//���ٶȱ���ֻ�׼ֵ� 500mm/s^2	
	*g_pCur_ref_base_mA = 5000;//��������ֵ��׼ֵ 1000mA
	*g_pForce_ref_base_mg = 20000;//��������ֵ��׼ֵ 20kg
	pos_p_vel_float = (float)((double)(*g_pPos_ref_base_um)/(double)(*g_pVel_ref_base_umPS));
	g_Vel_Max_PosPU_pSec = (float)PU_REFERENCE/pos_p_vel_float;
	g_CmdMap[SYS_SPEED_TEST] = 0;
	
	
	g_CmdMap[CMD_USEER_MODE] = CMD_USEER_MODE_0_POSITION;   // Ĭ�ϳ�ʼ��Ϊ��λģʽ


	PID_ini(&pid_force,F_SPD_REGULATOR_HZ);
	PID_ini(&pid_IQ,F_CUR_REGULATOR_HZ);
	PID_ini(&pid_ID,F_CUR_REGULATOR_HZ);
	PID_ini(&pid_spd,F_SPD_REGULATOR_HZ);
	PID_ini(&pid_pos,F_POS_REGULATOR_HZ);
	pid_force.Up_limit = g_CmdMap[SYS_CUR_UPPER_LIMIT_PU];
	pid_force.Low_limit = g_CmdMap[SYS_CUR_LOWER_LIMIT_PU];
	g_CmdMap[SYS_SELF_TUNING] = 0;
	//���Բ���������
	ini_waveSinGenrator(&waveGen,(float)(F_POS_REGULATOR_HZ/2));
  return result;
}
void clear_flash_erro(void)
{
	Flag_error_read_flash = 0;
}
