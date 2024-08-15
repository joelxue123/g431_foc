#include "PWM.h"
#include "global.h"
// λ�û����ٶȻ����ж����� EXTI_Line3

TIM_HandleTypeDef htim1;
volatile  s32 sample_invered = 0;



void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void SWI_EXTI_Config(void)
{
//	EXTI_HandleTypeDef  EXTI_Handle;
//	EXTI_ConfigTypeDef   EXTI_Config;
//	
//	
//	
//	EXTI_Handle.Line = EXTI_LINE_3;
//	
//	EXTI_Config.GPIOSel= EXTI_GPIOA;
//	EXTI_Config.Line =EXTI_LINE_3;
//	EXTI_Config.Mode = EXTI_MODE_INTERRUPT;
//	EXTI_Config.Trigger = EXTI_TRIGGER_RISING_FALLING;
//	
	EXTI->PR1 |= 0x0008;		// �����־
	EXTI->IMR1 |=  0x08;
	EXTI->EMR1 |=  0x08;
//	
//	HAL_EXTI_SetConfigLine(&EXTI_Handle, &EXTI_Config);
	
	
	HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}


/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
 void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = REAL_CCR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
	
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;//TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (TIM1->ARR)>>1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	sConfigOC.Pulse = (TIM1->ARR)>>2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
	sConfigOC.Pulse = (TIM1->ARR)>>1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = REAL_CCR-1;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
	
	
//	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_3);
//	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_4);
	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_4);
	
	__HAL_TIM_ENABLE(&htim1);
	
	HAL_ADCEx_InjectedStart_IT(&hadc1);
}





void PWM_Init(void)
{
	
	SWI_EXTI_Config();                                       //�������жϣ�ִ��λ�ú��ٶȻ�����
	MX_TIM1_Init();
}
void PWM_Enable(void)
{
	MP6540_ON();
  g_CmdMap[TAG_MOTOR_ENABLE] = 1;
  g_bEnableHB = 1;
  TIM1->BDTR|=(TIM_BDTR_MOE);
  PWM_OutPut_Open(TIM1,g_ElectricAngle,0);
}
void PWM_Disable(void)
{
	MP6540_OFF();
  g_CmdMap[TAG_MOTOR_ENABLE] = 0;
  g_bEnableHB = 0;
	
		TIM1->CCR3 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR1 = 0;
  TIM1->CCER = (u16)5000;
  TIM1->BDTR &= ~(TIM_BDTR_MOE);
}











	int hall_value = 1;
	int cnt = 0;

volatile int hall_a;
volatile int hall_b;
volatile int hall_c;
volatile  int hall_ture_value = 5;
volatile  int last_hall_ture_value = 0;
volatile  int tmp_hall_ture_value =0;
/**********
* hall_ture_value
* hall_ture_value = 5     B 
* hall_ture_value = 1     C 
 hall_ture_value = 3     C 
 hall_ture_value = 2     A 
 hall_ture_value = 6     A 
 hall_ture_value = 4     B 

*********/
volatile int G_hall_ture_value=0;
void Calculate_CCR(TIM_TypeDef* TIMx, s32 angle, s32 targetCCR)	  // Ŀ��CCR 0~MAX_CCR��Ӧ0~100ռ�ձ�
{
	static int p_cnt = 0;

  s32 t0, tk, tk_1;
  s32 CCR1 = 0;
  s32 CCR2 = 0;
  s32 CCR3 = 0;
	
	

	
	
  angle = angle<0?0:(angle<EN_360?angle:(EN_360-1));
  targetCCR = targetCCR>MAX_CCR?MAX_CCR:(targetCCR<0?0:targetCCR);
	
	
#if 0

	p_cnt++;
	if(p_cnt == 20000)
	{
		p_cnt =0 ;
		if(hall_ture_value == 5)
		{
			hall_ture_value = 1;
		}
		else if(hall_ture_value == 1)
		{
			hall_ture_value = 3;
		}
		else if(hall_ture_value == 3)
		{
			hall_ture_value = 2;
		}
		else if(hall_ture_value == 2)
		{
			hall_ture_value = 6;
		}
		else if(hall_ture_value == 6)
		{
			hall_ture_value = 4;
		}
		else if(hall_ture_value == 4)
		{
			hall_ture_value = 5;
		}
	}
	
#endif 
	

	
  if (angle >= 0 && angle < EN_60)
  {
//    tk = ( Math_Sin_EN360(EN_60-angle) * targetCCR ) >> 15;
//    tk_1 = ( Math_Sin_EN360(angle) * targetCCR ) >> 15;
    tk = Q15_MUL(Math_Sin_EN360(EN_60-angle),targetCCR);
    tk_1 = Q15_MUL(Math_Sin_EN360(angle),targetCCR);
    t0 = MAX_CCR - tk - tk_1;
    CCR3 = t0>>1;
    CCR2 = CCR3 + tk_1;
    CCR1 = CCR2 + tk;
		g_Sector_test = 1;
  }
  else if (angle >= EN_60 && angle < EN_120)
  {
    angle -= EN_60;
    tk = Q15_MUL(Math_Sin_EN360(EN_60-angle),targetCCR);
    tk_1 = Q15_MUL(Math_Sin_EN360(angle),targetCCR);
    t0 = MAX_CCR - tk - tk_1;
    CCR3 = t0>>1;
    CCR1 = CCR3 + tk;
    CCR2 = CCR1 + tk_1;
		g_Sector_test = 2;
  }
  else if (angle >= EN_120 && angle < EN_180)
  {
    angle -= EN_120;
    tk = Q15_MUL(Math_Sin_EN360(EN_60-angle),targetCCR);
    tk_1 = Q15_MUL(Math_Sin_EN360(angle),targetCCR);
    t0 = MAX_CCR - tk - tk_1;
    CCR1 = t0>>1;
    CCR3 = CCR1 + tk_1;
    CCR2 = CCR3 + tk;
		g_Sector_test = 3;
  }
  else if (angle >= EN_180 && angle < EN_240)
  {
    angle -= EN_180;
    tk = Q15_MUL(Math_Sin_EN360(EN_60-angle),targetCCR);
    tk_1 = Q15_MUL(Math_Sin_EN360(angle),targetCCR);
    t0 = MAX_CCR - tk - tk_1;
    CCR1 = t0>>1;
    CCR2 = CCR1 + tk;
    CCR3 = CCR2 + tk_1;
		g_Sector_test = 4;
  }
  else if (angle >= EN_240 && angle < EN_300)
  {
    angle -= EN_240;
    tk = Q15_MUL(Math_Sin_EN360(EN_60-angle),targetCCR);
    tk_1 = Q15_MUL(Math_Sin_EN360(angle),targetCCR);
    t0 = MAX_CCR - tk - tk_1;
    CCR2 = t0>>1;
    CCR1 = CCR2 + tk_1;
    CCR3 = CCR1 + tk;
		g_Sector_test = 5;

  }
  else if (angle >= EN_300 && angle < EN_360)
  {
    angle -= EN_300;
    tk = Q15_MUL(Math_Sin_EN360(EN_60-angle),targetCCR);
    tk_1 = Q15_MUL(Math_Sin_EN360(angle),targetCCR);
    t0 = MAX_CCR - tk - tk_1;
    CCR2 = t0>>1;
    CCR3 = CCR2 + tk;
    CCR1 = CCR3 + tk_1;
		g_Sector_test = 6;
  }
  TIMx->CCR3 = CCR3;
  TIMx->CCR2 = CCR2;
  TIMx->CCR1 = CCR1;
  //TIMx->CCER = (u16)0x5555;
	TIMx->CCER = (u16)PWM_OUTPUT_REG;
	
#if 0
	if(g_Sector_test == 1) 
	{

		ADC1->JSQR &=  ~(0x1f<<9);
		ADC1->JSQR |=  (0x03<<9);

	}
	else if(g_Sector_test == 2) 
	{
		ADC1->JSQR &=  ~(0x1f<<9);
		ADC1->JSQR |=  (0x03<<9);
		
	}
	else if(g_Sector_test == 3) 
	{
		ADC1->JSQR &=  ~(0x1f<<9);
		ADC1->JSQR |=  (0x01<<9);
	}
	else if(g_Sector_test == 4) 
	{
		ADC1->JSQR &=  ~(0x1f<<9);
		ADC1->JSQR |=  (0x01<<9);
	}
	else if(g_Sector_test == 5) 
	{

		ADC1->JSQR &=  ~(0x1f<<9);
		ADC1->JSQR |=  (0x01<<9);
	}
	else if(g_Sector_test == 6) 
	{
		ADC1->JSQR &=  ~(0x1f<<9);
		ADC1->JSQR |=  (0x03<<9);
	}

#endif
	
}



volatile int g_high_fre_pulse =1;

void PWM_OutPut_Open(TIM_TypeDef* TIMx, s32 ElectricAngle, s32 TimerCCR)
{
  s32 angle = 0;
	

	
//		TimerCCR += g_high_fre_pulse*500; 
//	g_high_fre_pulse = 0 - g_high_fre_pulse;
	
  //TimerCCR = TimerCCR>MAX_CCR?MAX_CCR:(TimerCCR<-MAX_CCR?-MAX_CCR:TimerCCR);
  TimerCCR = LIMIT_INT(TimerCCR, MAX_CCR,0);
  if (TimerCCR >= 0)
  {
    angle = ElectricAngle;
  }
  else
  {
    TimerCCR = -TimerCCR;
    angle = ElectricAngle + EN_180;
  }
  if (angle >= EN_360)
    angle -= EN_360;
  else if (angle < 0)
    angle += EN_360;
  Calculate_CCR(TIMx, angle, TimerCCR);
}

