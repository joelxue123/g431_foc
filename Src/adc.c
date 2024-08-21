
#include "adc.h"
#include "global.h"
float bus_voltage_ = 0;
volatile s16 VOLT_ADC_Array[VOLT_SAMPNUM] = {0};								//��ѹģ���źŵ�ADCԭʼֵ
volatile s16 TEMP_ADC_Array[TEMP_SAMPNUM] = {0};								//�¶�ģ���źŵ�ADCԭʼֵ
volatile s16 VOLT_ADC_Array_Index = 0;
volatile s16 TEMP_ADC_Array_Index = 0;
volatile s16 VOLT_Filter = 0;																		//��ѹ(mV)
volatile s16 TEMP_Filter = 0;																		//�¶�(0.1��)
volatile u8 Flag_VOLT_ADC_Array_Filled = 0;											//��ѹADC���������
volatile u8 Flag_TEMP_ADC_Array_Filled = 0;											//�¶�ADC���������
uint32_t calibration_value[4] = {0};
#define NUM_POS_AFILTER 32
s32 Pos_ADC_Array[NUM_POS_AFILTER] = {0};                     //̼Ĥλ��ADC��ƽ���˲�����
s32 Index_Pos_ADC_Array = 0;                                  //̼Ĥλ��ADC��ƽ���˲�����
s32 sum_Pos_ADC_Array = 0;                                  //̼Ĥλ��ADC��ƽ���˲�����
volatile s32 MU_Value_base_0 = 0;
#define YBP_ARRAY_BIT 3 //5
#define YBP_ARRAY_SIZE (1<<YBP_ARRAY_BIT)
volatile uint16_t index_YBP_Array = 0;
volatile int16_t YBP_Array[YBP_ARRAY_SIZE] = {0};
volatile int32_t YBP_Sum = 0;

#define FORCE_ARRAY_BIT 0
#define FORCE_ARRAY_SIZE (1<<FORCE_ARRAY_BIT)
volatile uint16_t index_force_Array = 0;
volatile int16_t force_Array[FORCE_ARRAY_SIZE] = {0};
volatile int32_t force_Sum = 0;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;
volatile int16_t g_Rt_Map[71] = {11,18,23,29,34,12,43,48,52,57,62,68,73,79,86,93,101,110,120,130,141,153,167,181,196,212,230,249,268,290,312,336,360,387,414,443,473,505,537,571,607,643,681,721,761,803,845,889,35,981,1028,1076,1125,1175,1226,1278,1330,1384,1437,1492,1546,1602,1657,1713,1769,1825,1881,1937,1992,2048,2103
};

volatile uint16_t adc_measurements_[2] = { 0 };



/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
 void MX_ADC1_Init(void)
{

	int factor;
  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
	#if 1
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	#endif
  /** Configure Injected Channel
  */

	#if 1 
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
	
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
	#endif
	
//  if (HAL_ADCEx_EnableInjectedQueue(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	factor = HAL_ADCEx_Calibration_GetValue(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc1,ADC_SINGLE_ENDED,factor);
	
	
	
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC2_Init(void)
{
  int factor;

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

	
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  factor = HAL_ADCEx_Calibration_GetValue(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_SetValue(&hadc2, ADC_SINGLE_ENDED, factor);
	
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(adc_measurements_), 2);
}





void start_temperature_adc(void)
{
	HAL_ADC_Start(&hadc1);
	//g_CmdMap[CMD_TEMP] = get_temperature_value(hadc1.Instance->DR);
	float bus_voltage = ((float)(26.833f*3.3f*(hadc1.Instance->DR)))/4096.0f;
	bus_voltage_ += 0.1 * (bus_voltage - bus_voltage_);
	g_CmdMap[CMD_TEMP] = 0;
}







//ADC2->JDR3  g_CmdMap[CMD_YBP_ADC_12BIT]  g_CmdMap[CMD_FORCE_ACT_RAW_PU]  g_CmdMap[CMD_FORCE_ACT_PU]
void get_average_forceSensor_adc(void)
{
	s16 data_in=0,data_out=0;	
	data_in = ADC2->JDR2;
	data_out = YBP_Array[index_YBP_Array];
	YBP_Array[index_YBP_Array] = data_in;
	index_YBP_Array= (index_YBP_Array + 1)%YBP_ARRAY_SIZE;
	YBP_Sum = YBP_Sum - data_out + data_in;
	g_CmdMap[CMD_YBP_ADC_12BIT] = YBP_Sum>>YBP_ARRAY_BIT;	
}
void get_average_force_data(void)
{
	s16 force_act_raw_pu = 0,data_in=0,data_out=0;
	if(g_CmdMap[SYS_YBP_ADC_DIR] == 0)
	{
		force_act_raw_pu = ((int64_t)(g_CmdMap[CMD_YBP_ADC_12BIT] - g_CmdMap[SYS_YBP_BASE_VALUE_12BIT])*16384)/(int64_t)g_CmdMap[SYS_YBP_SLOP_12ADC_FORCEREF];
	}
	else
	{
		force_act_raw_pu = ((int64_t)(g_CmdMap[SYS_YBP_BASE_VALUE_12BIT] - g_CmdMap[CMD_YBP_ADC_12BIT])*16384)/(int64_t)g_CmdMap[SYS_YBP_SLOP_12ADC_FORCEREF];
	}
	//
	if(g_CmdMap[CMD_FORCE_DIR] == 0)
	{
		g_CmdMap[CMD_FORCE_ACT_RAW_PU] = force_act_raw_pu;
	}
	else
	{
		g_CmdMap[CMD_FORCE_ACT_RAW_PU] = 0-force_act_raw_pu;
	}	
	data_in = g_CmdMap[CMD_FORCE_ACT_RAW_PU];
	data_out = force_Array[index_force_Array];
	force_Array[index_force_Array] = data_in;
	index_force_Array= (index_force_Array + 1)%FORCE_ARRAY_SIZE;
	force_Sum = force_Sum - data_out + data_in;
	g_CmdMap[CMD_FORCE_ACT_PU] = force_Sum>>FORCE_ARRAY_BIT;
}
//��������ع�
//����������
//����ģ��������
void ADC1_2_IRQHandler(void)////ADC3_JEOC�ж�     ִ��Ƶ����PWMƵ����ͬ
{ 
  if( __HAL_ADC_GET_IT_SOURCE(&hadc1,ADC_IT_JEOS)!=RESET)
	 {
//		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
			ADC1->ISR |= (u32)ADC_FLAG_JEOC; 
			ServoPro_Fast();
//			get_average_forceSensor_adc();
//		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
	 }
}
s32 Get_VOLT_Filter(void) //Flag_VOLT_ADC_Array_Filled==1ʱִ��
{
	s16 i = 0;
	s32 sum = 0;
	Flag_TEMP_ADC_Array_Filled = 0;
	sum = 0;
	for(i=0;i<VOLT_SAMPNUM;i++)
	{
		sum = sum + VOLT_ADC_Array[i];
	}
	VOLT_Filter = (800*sum)/(VOLT_SAMPNUM*905);
	return VOLT_Filter;
}
//ADC ��������ϵ�����Ĭ�ϲ��� 700-3700 �� 0 - 16384���Զ�Ӧ
void pos_linearity_ini(void)
{
	s16 i = 0;
	for(i=0;i<NUM_LINEARITY_SEG;i++)
	{
		L_Slop_Array[i] = (((int32_t)(g_CmdMap[POS_LINEARITY_PU(i+1)] - g_CmdMap[POS_LINEARITY_PU(i)]))<<10)/(g_CmdMap[POS_LINEARITY_ADC(i+1)] - g_CmdMap[POS_LINEARITY_ADC(i)]);
	}
}
////ADC ��������ϵ�����Ĭ�ϲ��� 700-3700 �� 0 - 16384���Զ�Ӧ
//void pos_linearity_set_default(void)
//{
//	s16 i = 0;
//	for(i = 0;i < 31;i++)
//	{
//			g_CmdMap[POS_LINEARITY_ADC(i)]	= 700 + 90*i;	//̼ĤADC������ϵ㣬ADC��ֵ
//			g_CmdMap[POS_LINEARITY_PU(i)] 	= 0 + 16384*i/30;	//̼ĤADC������ϵ㣬ʵ���г�
//	}
//	for(i=0;i<=29;i++)
//	{
//		L_Slop_Array[i] = (((int32_t)(g_CmdMap[POS_LINEARITY_PU(i+1)] - g_CmdMap[POS_LINEARITY_PU(i)]))<<10)/(g_CmdMap[POS_LINEARITY_ADC(i+1)] - g_CmdMap[POS_LINEARITY_ADC(i)]);
//	}
//}
s16 Pos_Corrected_last = 0;
s16 MU_Value_error_cnt = 0;
s16 index_pos_array = 0;
s16 pos_array[128] = {0};
s32 pos_sum = 0;
s16 pos_filterd = 0;
s16 pos_valid = 0;
int32_t Pos_Correct(int32_t Pos_Org)
{
	s32 slopTotall = 0;
	s16 in_data,out_data;
	int32_t Pos_Corrected = 0;
	int32_t Index_A,Index_B = 0;
	s16 i = 0;
	if(Pos_Org>=g_CmdMap[POS_LINEARITY_ADC(0)] && Pos_Org<g_CmdMap[POS_LINEARITY_ADC(NUM_LINEARITY_SEG)])//��Ҫ�ж�������һ��
	{
		//���ַ�
		Index_A = 0;Index_B = NUM_LINEARITY_SEG;
		while(1)
		{
			if(Index_B-Index_A == 1)
			{
				break;
			}
			i = (Index_A + Index_B)/2;
			if(Pos_Org<=g_CmdMap[POS_LINEARITY_ADC(i)])
			{
				Index_B = i;
			}
			else
			{
				Index_A = i;
			}
		}
		Pos_Corrected = (((int32_t)(Pos_Org - g_CmdMap[POS_LINEARITY_ADC(Index_A)])* L_Slop_Array[Index_A])>>10) + g_CmdMap[POS_LINEARITY_PU(Index_A)];	
	}
	else if(Pos_Org<g_CmdMap[POS_LINEARITY_ADC(0)])
	{
		Pos_Corrected = Pos_Org-g_CmdMap[POS_LINEARITY_ADC(0)]; 
				if( Pos_Corrected < - 16384)
		{
			Pos_Corrected = -16384;
		}
	}	
	else
	{
			slopTotall = (((int32_t)(g_CmdMap[POS_LINEARITY_PU(NUM_LINEARITY_SEG)] - g_CmdMap[POS_LINEARITY_PU(0)]))<<10)/(g_CmdMap[POS_LINEARITY_ADC(NUM_LINEARITY_SEG)] - g_CmdMap[POS_LINEARITY_ADC(0)]);
			Pos_Corrected = (((int32_t)(Pos_Org - g_CmdMap[POS_LINEARITY_ADC(NUM_LINEARITY_SEG)])* slopTotall)>>10) + g_CmdMap[POS_LINEARITY_PU(NUM_LINEARITY_SEG)];
			if( Pos_Corrected> 32767 )
			{
				Pos_Corrected = 32767;
			}
		
	}
	in_data = Pos_Corrected;
	out_data = pos_array[index_pos_array];
	pos_array[index_pos_array] = in_data;
	index_pos_array= (index_pos_array + 1)%128;
	pos_sum = pos_sum - out_data + in_data;
	pos_filterd = pos_sum>>7;
	if(ABS(Pos_Corrected-pos_filterd)>8192)
	{
		
		MU_Value_error_cnt++;
	//	if(MU_Value_error_cnt>10) while(1);
		return pos_valid;
	}
	else
	{
		pos_valid = Pos_Corrected - g_CmdMap[SYS_POS_OFFSET];
		return pos_valid;
	}	
}
//�����Ƹ�ĩ��λ�ã���ϵ��ĩ�˴ű��ź� �� ̼Ĥλ��
void Get_Pos_Rod(void)
{
	s32 pos_noCorrect;
	if(g_CmdMap[SYS_MU_COMM_TO_PC] == 1)     //MU оƬ�궨ģʽ
	{	
		Encode_Single_to_Multi_R(0,0,g_MechanicsAngle_15bit,&rod_pos_Muti_Encoder,0x8000,0x40000);
		g_CmdMap[CMD_POS_ACT_PU] = ((int64_t)rod_pos_Muti_Encoder*g_CmdMap[SCREW_UM_RES]/(32767*(int64_t)g_CmdMap[GEAR_RATIO_8BIT]/256))*16384/(*g_pPos_ref_base_um);
	}
	else	
	{
		if(*g_pMU_value_offset > vlaue_icmu_offset_overturn)  
		{
			if(MU_Value<range_icmu_vlaue_need_addMax)//range_icmu_vlaue_at_stroke
			{
				MU_Value = MAX_VALUE_ICMU+MU_Value;
			}
			else
			{			
			}
		}
		else
		{
			MU_Value = MU_Value;
		}		
		
		#if 1
		MU_Value_base_0 = MU_Value-*g_pMU_value_offset;
//		MU_Value_base_0 = MU_Value - 12803800;
		pos_nm_MU150 = (DISTANCE_NM_ICMU*(int64_t)(MU_Value_base_0-*g_pMU_0mm_offset))>>MU_DATA_SPI_RES;		
//		pos_nm_MU150 = (DISTANCE_NM_ICMU*(int64_t)(MU_Value_base_0))>>MU_DATA_SPI_RES;		
		pos_noCorrect = ((int64_t)pos_nm_MU150<<14)/((int64_t)*g_pPos_ref_base_um*1000);
		
		g_CmdMap[CMD_POS_ACT_PU] = Pos_Correct(pos_noCorrect);
		
//		g_CmdMap[CMD_POS_ACT_PU] = pos_noCorrect;
		
		#endif
		
//		MU_Value_base_0 = MU_Value - 10003800;
//		pos_nm_MU150 = (DISTANCE_NM_ICMU*(int64_t)(MU_Value_base_0))>>MU_DATA_SPI_RES;		
//		pos_noCorrect = ((int64_t)pos_nm_MU150<<14)/((int64_t)*g_pPos_ref_base_um*1000);
//		g_CmdMap[CMD_POS_ACT_PU] = pos_noCorrect;
	}
	
	
//����λ�ô�����(IC_Hause_MU ��դ����)
		
	
//����̼ĤADC����	
//	g_CmdMap[CMD_POS_ADC_12BIT] = sum_Pos_ADC_Array>>5;//(ADC1->JDR2 & 0x0000FFFF);
//	g_CmdMap[CMD_POS_ACT_PU_ADC] = Pos_Correct(g_CmdMap[CMD_POS_ADC_12BIT]);	
	
	
//����Ȧ������	
//    �����Ƹ�ʵ���г�		
//		Encode_Single_to_Multi_R(0,0,g_MechanicsAngle_15bit,&rod_pos_Muti_Encoder,0x8000,0x40000);	
//		if(g_CmdMap[TAG_MOTOR_ENABLE] == 0 || PosAtDead_Flag == 1)
//		{
//			rod_pos_Muti_Encoder = 32767*((int64_t)(g_CmdMap[CMD_POS_ADC_12BIT_FILTER] - 300)*11*10000/3300)/g_CmdMap[SCREW_UM_RES];
//		}
//		g_CmdMap[CMD_POS_ACT_PU] = ((int64_t)rod_pos_Muti_Encoder*g_CmdMap[SCREW_UM_RES]/(32767*(int64_t)g_CmdMap[ANGLE_GEAR_RATIO_8BIT]/256))*16384/(*g_pPos_ref_base_um);
//	g_CmdMap[CMD_POS_ACT_PU] = g_CmdMap[CMD_POS_ACT_PU_ADC];	
}
s16 check_linearity(void)
{
	s16 i = 0;
	if(g_CmdMap[POS_LINEARITY_ADC(0)]<0 || g_CmdMap[POS_LINEARITY_ADC(0)]>19000)
	{
		return 0;
	}
	for(i = 1;i < NUM_LINEARITY_SEG;i++)
	{
			if(g_CmdMap[POS_LINEARITY_ADC(i)]<0|| g_CmdMap[POS_LINEARITY_ADC(i)]>19000)
			{
				return 0;
			}
			if(g_CmdMap[POS_LINEARITY_ADC(i-1)]>g_CmdMap[POS_LINEARITY_ADC(i)])
			{
				return 0;
			}
	}
	return 1;
}

//g_CmdMap[CMD_TEMP] = get_temperature_value(ADC1->JDR2)
int16_t get_temperature_value(int16_t para_val)
{
	int16_t index_A = 0,index_B = 70,index_Mid;
	para_val = 3300*para_val/NCT_VOLT;
	while(1)
	{
		index_Mid = (index_A+index_B)>>1;		
		if(para_val<=g_Rt_Map[index_Mid])
		{
			index_B = index_Mid;
		}
		else if(para_val>g_Rt_Map[index_Mid])
		{
			index_A = index_Mid;		
		}
		if(index_B == (index_A+1) || index_B == index_A)
		{
			break;
		}
	}
	return 2*index_A-40;
}
void pos_linearity_set_default(void)
{
	s16 i = 0;
	s32 inc_per_seg = 16384/NUM_LINEARITY_SEG;
	for(i = 0;i < NUM_LINEARITY_SEG+1;i++)
	{
			g_CmdMap[POS_LINEARITY_ADC(i)]	= inc_per_seg*i;	//
			g_CmdMap[POS_LINEARITY_PU(i)] 	= inc_per_seg*i;	//
	}
	for(i=0;i<NUM_LINEARITY_SEG;i++)
	{
		L_Slop_Array[i] = (((int32_t)(g_CmdMap[POS_LINEARITY_PU(i+1)] - g_CmdMap[POS_LINEARITY_PU(i)]))<<10)/(g_CmdMap[POS_LINEARITY_ADC(i+1)] - g_CmdMap[POS_LINEARITY_ADC(i)]);
	}
}

