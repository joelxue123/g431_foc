#include "global.h"
#include "spi_MEncoder.h"
#include "icmu.h"
#include"sensorless.h"

u16 EncoderAB_PulseWidth_Array[16] = {0};//�������������
s32 g_MEncode = 0;
volatile s32 g_Encode_AbsPos = 0;
volatile s32 g_Encode_AbsPos_Last = 0;
volatile s32 g_Encode_Inc = 0;
volatile s32 Num_Turns = 0;
volatile s32 g_Encode_offset = 0;
volatile s32 g_Encode_offset_EN = 0;
volatile s32 tim3_cnt = 0;
volatile s32 tim4_cnt = 0;

TIM_HandleTypeDef htim2;

float sincos_sample_s_ = 0.0f;
float sincos_sample_c_ = 0.0f;

extern DMA_HandleTypeDef hdma_spi1_rx;

extern DMA_HandleTypeDef hdma_spi1_tx;


/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
SPI_HandleTypeDef hspi3;


/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PA15     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
	
	else  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  //  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA1_Channel4;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */
//		__HAL_DMA_ENABLE(&hdma_spi1_rx);
//		__HAL_DMA_ENABLE(&hdma_spi1_tx);
//		
//		__HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC );
//		__HAL_DMA_ENABLE_IT(&hdma_spi1_tx, DMA_IT_TC );
//		
		
//		HAL_DMA_Start_IT(&hdma_spi1_rx,(uint32_t)(&(SPI1->DR)), (uint32_t)(&spi1_reg_buf[0]) ,4);

		
  /* USER CODE END SPI1_MspInit 1 */
  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PA15     ------> SPI3_NSS
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PB5     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }

}



void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

	__HAL_SPI_ENABLE(&hspi3);
}






/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 167;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period =0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
	
	TIM_TI1_SetConfig(htim2.Instance, sConfigIC.ICPolarity, TIM_ICSELECTION_TRC, sConfigIC.ICFilter);
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
	
  if (HAL_TIM_ConfigTI1Input(&htim2, TIM_TI1SELECTION_XORCOMBINATION) != HAL_OK)
  {
    Error_Handler();
  }
	
	htim2.Instance->SMCR |= TIM_TS_TI1F_ED; // select "ti1f_ed" input trigger 
	TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}


void hall_Configuration(void)
{
	
	MX_TIM2_Init();
	TIM2->CR1 |= TIM_AUTORELOAD_PRELOAD_ENABLE;

	 __HAL_TIM_ENABLE(&htim2);
}



volatile s32 Encoder_Current = 0;							// ��ǰ����������ֵ
volatile s32 Encoder_Last = 0;   							// ��ǰ����������ֵ
volatile s32 Encoder_Inc = 0;   							// ����������ֵ�仯��
volatile s32 s_SpeedTimeAdd1 = 0;   					// ����������ֵ�仯��
volatile s32 s_ForecastTimeAdd1 = 0;   				// ����������ֵ�仯��
volatile s32 speed_def[2] = {64,0};
volatile s32 speed_TCal = 0;									//T����õ��ٶ�
volatile s32 speed_MCal = 0;									//M������ٶ�
volatile s32 s_IntNum1 = 0; 									// 	��ֵ(����)����
volatile s32 s_ForecastCnt1 = 1;							//	Ԥ�����
volatile s32 g_CurrentSpeed = 0;

volatile s32 start_speed_sample = 0;

extern  volatile int speed1,speed2,speed3;
extern  volatile  int pid_hall_ture_value1;
extern  volatile  int pid_last_hall_ture_value1;

extern volatile int hall_speed_sample_timeout_cnt; //���㵽30ms
extern volatile int  hall_speed_sample_cnt;
extern volatile int hall_speed_sample_timeout;
extern volatile int hall_cnt;

extern volatile int hall_now_time;
extern volatile int hall_previous_1_time;
extern volatile int hall_previous_2_time;
extern volatile int hall_previous_3_time ;
extern volatile int64_t Vel_PPS_raw1 ;
void MTPro(void) //10KHz ִ��
{
	
	int64_t s_SpeedTimeAdd11=0;

	
	int speed;

		#if 0 
		
		//	g_CmdMap[CMD_SPDREG_REF_PU] = (s16)((((int64_t)Vel_PPS_raw1)<<22)/((int64_t)*g_pVel_ref_base_PPS));
		speed = (s16)((((int64_t)Vel_PPS_raw1)<<22)/((int64_t)*g_pVel_ref_base_PPS));


		g_CmdMap[CMD_SPD_ACT_PU] =speed; 
		
		#endif

			
}

float pll_estimated_phase = 0;
float  pll_estimated_spd = 0;

float  pll_pid_integ = 0;
float pll_pid_out = 0;

volatile int estimated_SPD_filterd = 0;
int g_measured_phase = 0;
#define  FILTER_SIZE   24
int temp_spd_index  = 0;
int temp_spd[FILTER_SIZE] ={0};
int temp_spd_filed[FILTER_SIZE] ={0};

int pll_pid_integ_int = 0;
int estimated_phase_integ_int = 0;
float estimated_phase_integ_int_1 = 0;

void Pll_phase(s16 measured_phase, float *estimated_phase, float *estimated_spd )
{
	float phase_erro;
	int estimated_spd_1 = 0;
	
	
	static int cnt  =0;
	cnt++;
	if(cnt ==4)
	{
		cnt = 0;
		
	if( measured_phase > 16384 )
	{
		measured_phase = measured_phase - 32768;
	}
		if( measured_phase < -16384 )
	{
		measured_phase = measured_phase + 32768;
	}

	

	
	phase_erro = measured_phase - *estimated_phase;
	
	if( phase_erro > 16384 )
	{
		phase_erro = phase_erro - 32768;
	}	
	if( phase_erro < -16384 )
	{
		phase_erro = phase_erro + 32768;
	}	
	pll_pid_integ_int = pll_pid_integ_int + phase_erro;
	
	
//	pll_pid_integ = pll_pid_integ + phase_erro *0.0001;
	pll_pid_integ = pll_pid_integ_int /10000;
	
	*estimated_spd =  ( 2000*phase_erro + 100000*pll_pid_integ);
	
	estimated_phase_integ_int = estimated_phase_integ_int + *estimated_spd ;
	
		if( estimated_phase_integ_int > 163840000 )
	{
		estimated_phase_integ_int =estimated_phase_integ_int - 327680000;
	}
		if( estimated_phase_integ_int < -163840000 )
	{
		estimated_phase_integ_int = estimated_phase_integ_int + 327680000;
	}

	
	*estimated_phase = estimated_phase_integ_int /10000;
//	*estimated_phase  = *estimated_phase  + *estimated_spd * 0.0001;
//	*estimated_phase = estimated_phase_integ_int *0.0001;

//	if( estimated_phase_integ_int_1 > 16384 )
//	{
//		estimated_phase_integ_int_1 =estimated_phase_integ_int_1 - 32768;
//	}
//		if( estimated_phase_integ_int_1 < -16384 )
//	{
//		estimated_phase_integ_int_1 = estimated_phase_integ_int_1 + 32768;
//	}

	
	if( (int)*estimated_phase > 16384 )
	{
		*estimated_phase = *estimated_phase - 32768;
	}
		if( (int)*estimated_phase < -16384 )
	{
		*estimated_phase = *estimated_phase + 32768;
	}
	
	if(g_CmdMap[TAG_MOTOR_ENABLE] == 1)
	{
		if(temp_spd_index < FILTER_SIZE)
		{
			temp_spd_filed[temp_spd_index] = estimated_SPD_filterd;
			temp_spd[temp_spd_index++] = (int)*estimated_spd;
		}
		else
			temp_spd_index = 0;
			
	}
	else
	{
		temp_spd_index = 0;
	}
	
	g_measured_phase = measured_phase;
	
///	estimated_spd_1 = ( (int)*estimated_spd ) / 2000;
	estimated_spd_1 = ( (int)*estimated_spd )* 30 /23000;
	estimated_SPD_filterd = ( (56*estimated_SPD_filterd) + ( 200*(int)(estimated_spd_1)) )>>8; ;
	

//	estimated_SPD_filterd =(s16)((((int64_t)estimated_SPD_filterd)<<10)/((int64_t)*g_pVel_ref_base_PPS));
//	g_CmdMap[CMD_SPD_ACT_PU] =  ( estimated_SPD_filterd /100) *100 ;
	g_CmdMap[CMD_SPD_ACT_PU] =  estimated_SPD_filterd  ;

//	
}
}



int g_ElectricAngle_15bit_Last = 0;
int s_Encoder_Inc = 0;

int g_ElectricAngle_add = 0;
void GetElectricAngle(void)//50us ִ������
{		
		static s32 count1 = 0;
		s32 inc__MechanicsAngle_15bit = 0;
		g_MechanicsAngle_15bit = (32768.f*(*motor_.phase_)/(2*M_PI));
		if(SPI3->DR == 0xFFFF)
		{
			if(count1<50000)
			{
				count1++;
			}
			else
			{
				g_CmdMap[CMD_ERROR] |= ERROR_MASK_ENCODER_FAULT;
			}
		}
		else
		{
			count1 = 0;
		}
//		g_MechanicsAngle_15bit += g_CmdMap[CMD_SPD_ACT_PU]*500/16384;
//		g_MechanicsAngle_15bit &= 0x7fff;
//		
		g_MechanicsAngle_15bit_last = g_MechanicsAngle_15bit;
		inc__MechanicsAngle_15bit = g_MechanicsAngle_15bit - g_MechanicsAngle_15bit_last;
		if(inc__MechanicsAngle_15bit > 16384)
		{
				inc__MechanicsAngle_15bit =  32768 - inc__MechanicsAngle_15bit;
		}
		else if(inc__MechanicsAngle_15bit < -16384)
		{
				inc__MechanicsAngle_15bit =  32768 + inc__MechanicsAngle_15bit;
		}
		if(inc__MechanicsAngle_15bit > 8192)//�������쳣
		{
			g_CmdMap[CMD_ERROR] |= ERROR_MASK_ENCODER_FAULT;
		}
		g_ElectricAngle_15bit_Raw = g_MechanicsAngle_15bit% EN_360;
		g_ElectricAngle_act = (g_MechanicsAngle_15bit )% EN_360;
		if(g_CmdMap[SYS_MOT_TEST] == 0)
		{										
			g_ElectricAngle = g_ElectricAngle_act;
		}
		else if(g_CmdMap[SYS_MOT_TEST] == 5)
		{
			g_ElectricAngle = (s16)(32768.f*(*motor_.phase_)/(2*M_PI));
		}
		else
		{
			g_ElectricAngle = g_ElectricAngle_sim;
		}
	  if(g_ElectricAngle>=EN_360)
		{
			g_ElectricAngle=g_ElectricAngle - EN_360;			
		}
		if(g_ElectricAngle<=EN_0)
		{
			g_ElectricAngle=EN_360 + g_ElectricAngle;
		}
		g_ElectricAngle_15bit = (g_ElectricAngle_act<<15)/EN_360;// ��Ƕ�_32768
		
		
		
		
		
//	s_cnt++;
		
//	if(s_cnt == 4)
//	{
//		s_cnt = 0;
//		s_Encoder_Inc = (g_ElectricAngle_15bit - g_ElectricAngle_15bit_Last); 
//		g_ElectricAngle_15bit_Last = g_ElectricAngle_15bit;
//		
//		if (s_Encoder_Inc > 16384)    // ����ֵ��Խ���߽߱紦��
//			s_Encoder_Inc -= 32768;
//		else if (s_Encoder_Inc < -16384)
//			s_Encoder_Inc += 32768;  
//		
//	//	speed_based_encoder = s_Encoder_Inc * 60*  1000000  /180   / 23000; 
//	}
//		
}
void sim_ElectricAngle(void)
{
		static s16 sys_mot_test_last = 0;
		static s32 cnt_temp = 0;
		if(sys_mot_test_last != g_CmdMap[SYS_MOT_TEST])
		{
			if(g_CmdMap[SYS_MOT_TEST] == 1)
			{
			}
			if(g_CmdMap[SYS_MOT_TEST] == 2)
			{
				cnt_temp = EN_60*2000/EN_360;
				g_ElectricAngle_sim = EN_60;
				g_Encode_offset_EN = EN_0;
				g_Encode_offset = g_Encode_offset_EN*360/EN_360;
				g_CmdMap[MOT_EANGLE_OFFSET] = g_Encode_offset;
			}
			if(g_CmdMap[SYS_MOT_TEST] == 3)
			{
				cnt_temp = EN_300*2000/EN_360;
				g_ElectricAngle_sim = EN_300;
				g_Encode_offset_EN = EN_0;
				g_Encode_offset = g_Encode_offset_EN*360/EN_360;
				g_CmdMap[MOT_EANGLE_OFFSET] = g_Encode_offset;
			}
		}
		if(g_CmdMap[SYS_MOT_TEST] == 1)//ģ�������ת��Ƕ�
		{
					cnt_temp = cnt_temp + 1;//1;
					g_ElectricAngle_sim = cnt_temp*EN_360/10000;//2s һȦ����ת�ٶ�
					if(g_ElectricAngle_sim>EN_360)
					{
						cnt_temp = 0;
					}				
					
		}
		else if(g_CmdMap[SYS_MOT_TEST] == 2)//���Ե�Ƕȣ���������ת(g_ElectricAngle_sim����)��g_ElectricAngle_sim = 0���¼��Ƕ�
		{
					if(g_ElectricAngle_sim < EN_30)
					{
						g_ElectricAngle_sim  = EN_0;
						
					}
					else
					{
						cnt_temp = cnt_temp + 1;
						g_ElectricAngle_sim = cnt_temp*EN_360/2000;//2s һȦ����ת�ٶ�
						if(g_ElectricAngle_sim>EN_360)
						{
							cnt_temp = 0;
						}
					}									
		}
		else if(g_CmdMap[SYS_MOT_TEST] == 3)//���Ե�Ƕȣ���������ת(g_ElectricAngle_sim����)��g_ElectricAngle_sim = 0���¼��Ƕ�
		{
					if(g_ElectricAngle_sim > EN_330)
					{
						g_ElectricAngle_sim  = EN_0;
					}
					else
					{
						cnt_temp = cnt_temp - 1;
						g_ElectricAngle_sim = cnt_temp*EN_360/2000;//2s һȦ����ת�ٶ�
						if(g_ElectricAngle_sim<EN_0)
						{
							cnt_temp = 0;
						}
					}	
		}
		else if(g_CmdMap[SYS_MOT_TEST] == 4 )//��� ����� ���еĲ���
		{
					g_ElectricAngle_sim = EN_30;//2s һȦ����ת�ٶ�
		}
		else if(g_CmdMap[SYS_MOT_TEST] == 5)//��� ����� ���еĲ���
		{
					g_ElectricAngle_sim = EN_30;//2s һȦ����ת�ٶ�
		}
		sys_mot_test_last = g_CmdMap[SYS_MOT_TEST];			
}

	volatile s32 flag_first_exe = 0;
void Encode_Single_to_Multi_R(u8 flag_reverse,u8 flag_cycle,s32 single_value_raw,s32* p_multi_value,s32 mod_single,s32 mod_multi)
{
	static s32 single_value_last_raw = 0;

	s32 single_value = 0;
	s32 single_value_last = 0;
	s32 inc = 0;
	s32 mod_half = 0; 
	if(flag_first_exe == 0)
	{
		flag_first_exe = 1;
		single_value_last_raw = single_value_raw;
		return;
	}
	if(flag_reverse == 1)
	{
		single_value = mod_single - single_value_raw;
		single_value_last = mod_single - single_value_last_raw;
	}
	else
	{
		single_value = single_value_raw;
		single_value_last = single_value_last_raw;	
	}
	inc = single_value - single_value_last;
	mod_half = (mod_single>>1)-1;
	if(inc>mod_half)//���� ��ת
	{
		inc = inc - mod_single;
	}
	if(inc<-mod_half)//���� ��ת
	{
		inc = inc + mod_single;
	}
	single_value_last = single_value;
	if(flag_reverse == 1)
	{
		single_value_last_raw = mod_single - single_value_last;
	}
	else
	{
		single_value_last_raw = single_value_last;	
	}
	if(flag_cycle == 0)
	{
		*p_multi_value = *p_multi_value+inc;
		return;
	}
	*p_multi_value = *p_multi_value%mod_multi;
}



