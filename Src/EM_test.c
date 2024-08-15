/**
  ******************************************************************************
  * @file    EM_test.c 
  * @author  ZJ
  * @version V0.0.0
  * @date    5-January-2021
  * @brief   Electromagnetic characteristic test
  */
	
#if 0


#include "EM_test.h"	
#define SIGNLE_PHASE_SAMPLE_NUM (NUM_POINT*8)
s16 value_array_adc1[SIGNLE_PHASE_SAMPLE_NUM] = {0};	
/**
  * @brief  This function handles ADC configuration for single phase current fast acquisition.
  * @param  None
  * @retval None
  */
void ADC_Configuration_StartSinglePhase(void)
{
 

	DMA_InitTypeDef DMA_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_StructInit(&GPIO_InitStructure);
  // DMA配置
  //ADC_DMA_Configuration();
  // 时钟配置
	MX_ADC1_Init();
	MX_ADC2_Init();
	
  RCC_AHBPeriphClockCmd(RCC_AHBENR_ADC12EN|RCC_AHBENR_ADC34EN, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	
  // 端口配置  
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4;//GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);



	DMA_StructInit(&DMA_InitStructure);
	DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&value_array_adc1[0]);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = SIGNLE_PHASE_SAMPLE_NUM;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;     //单次采集并非循环采集
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);	
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE);	
//DMA 配置 SPI1_TX

//关闭4个通道的注入采集
	ADC_StopInjectedConversion(ADC1);
	ADC_StopInjectedConversion(ADC2);
	ADC_StopInjectedConversion(ADC3);
	ADC_StopInjectedConversion(ADC4);
	
	ADC_ExternalTriggerConfig(ADC1, ADC_ExternalTrigInjecConvEvent_0,ADC_ExternalTrigEventEdge_None); 
	ADC_ExternalTriggerConfig(ADC2, ADC_ExternalTrigInjecConvEvent_0,ADC_ExternalTrigEventEdge_None); 
	ADC_ExternalTriggerConfig(ADC3, ADC_ExternalTrigInjecConvEvent_0,ADC_ExternalTrigEventEdge_None); 
	ADC_ExternalTriggerConfig(ADC4, ADC_ExternalTrigInjecConvEvent_0,ADC_ExternalTrigEventEdge_None); 
	

  ADC_Cmd(ADC1, ENABLE);	
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	
	ADC_StopInjectedConversion(ADC1); 
	ADC_StopInjectedConversion(ADC2);
	ADC_StopInjectedConversion(ADC3);	
	ADC_StopInjectedConversion(ADC4);	
//	ADC_Cmd(ADC4, ENABLE);
	ADC_DMACmd(ADC1, ENABLE); 
	
	ADC_CommonInitStruct.ADC_Clock = ADC_Clock_SynClkModeDiv4;//ADC_Clock_SynClkModeDiv1;     //应该选择多少合适
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStruct.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;//ADC_Mode_InjSimul;//ADC_Mode_CombRegSimulInjSimul;  ADC_DMAMode_OneShot
	ADC_CommonInitStruct.ADC_TwoSamplingDelay  = 4;
	ADC_CommonInit(ADC1, &ADC_CommonInitStruct); 
	ADC_CommonInit(ADC2, &ADC_CommonInitStruct); 
	ADC_CommonInit(ADC4, &ADC_CommonInitStruct); 
		//ADC1 校准	
	ADC_StartConversion(ADC1);
		ADC_StartConversion(ADC2);
			ADC_StartConversion(ADC3);
//	重新开启4个通道注入采集
//	ADC_ExternalTriggerConfig(ADC1, ADC_ExternalTrigInjecConvEvent_0, ADC_ExternalTrigInjecEventEdge_RisingEdge); 
//	ADC_ExternalTriggerConfig(ADC2, ADC_ExternalTrigInjecConvEvent_0, ADC_ExternalTrigInjecEventEdge_RisingEdge); 
//	ADC_ExternalTriggerConfig(ADC3, ADC_ExternalTrigInjecConvEvent_0, ADC_ExternalTrigInjecEventEdge_RisingEdge); 
//	ADC_ExternalTriggerConfig(ADC4, ADC_ExternalTrigInjecConvEvent_0, ADC_ExternalTrigInjecEventEdge_RisingEdge); 

//	ADC_StartInjectedConversion(ADC1);
//	ADC_StartInjectedConversion(ADC2);
//	ADC_StartInjectedConversion(ADC3);
//	ADC_StartInjectedConversion(ADC4);	
}
void ADC_Configuration_StopSinglePhase(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_StructInit(&ADC_InitStructure);	
  RCC_AHBPeriphClockCmd(RCC_AHBENR_ADC12EN|RCC_AHBENR_ADC34EN, ENABLE);	
  // ADC1
//	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;     /*ENABLE:Continuous;DISABLE:Single mode*/
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;             /*ADC_resolution */ 
//  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigInjecConvEvent_0;//ADC_ExternalTrigInjecConvEvent_8;  
//  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None; //ADC_ExternalTrigEventEdge_RisingEdge;  
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right ;             
//  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;          /*ENABLE or DISABLE. 具体用途不理解*/
//  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;           	/*!< Enable/disable automatic injected group conversion after regular group conversion.*/
//  ADC_InitStructure.ADC_NbrOfRegChannel = 1;       											/*!< Specifies the number of ADC channels that will be converted using the sequencer for regular channel group.*/
//	ADC_Init(ADC1, &ADC_InitStructure);																							 																						 
//	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1, ADC_SampleTime_61Cycles5 );
//  // ADC2
//	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;     /*ENABLE:Continuous;DISABLE:Single mode*/
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;             /*ADC_resolution */ 
//  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigInjecConvEvent_0;  
//  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;  
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right ;             
//  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;           /*ENABLE or DISABLE. 具体用途不理解*/
//  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;             /*!< Enable/disable automatic injected group conversion after regular group conversion.*/
//  ADC_InitStructure.ADC_NbrOfRegChannel = 1;       											 /*!< Specifies the number of ADC channels that will be converted using the sequencer for regular channel group.*/
//	ADC_Init(ADC2, &ADC_InitStructure);		
//	ADC_RegularChannelConfig(ADC2,ADC_Channel_1,1, ADC_SampleTime_2Cycles5);	


//	// ADC3
//	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;     /*ENABLE:Continuous;DISABLE:Single mode*/
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;             /*ADC_resolution */ 
//  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigInjecConvEvent_0;  
//  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;  
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right ;             
//  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;           /*ENABLE or DISABLE. 具体用途不理解*/
//  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;           /*!< Enable/disable automatic injected group conversion after regular group conversion.*/
//  ADC_InitStructure.ADC_NbrOfRegChannel = 1;       /*!< Specifies the number of ADC channels that will be converted using the sequencer for regular channel group.*/
//	ADC_Init(ADC3, &ADC_InitStructure);																							 																						 
//	ADC_RegularChannelConfig(ADC3,ADC_Channel_1,1, ADC_SampleTime_2Cycles5);

//	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;     /*ENABLE:Continuous;DISABLE:Single mode*/
//  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;             /*ADC_resolution */ 
//  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigInjecConvEvent_0;  
//  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;  
//  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right ;             
//  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;           /*ENABLE or DISABLE. 具体用途不理解*/
//  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;           /*!< Enable/disable automatic injected group conversion after regular group conversion.*/
//  ADC_InitStructure.ADC_NbrOfRegChannel = 1;       /*!< Specifies the number of ADC channels that will be converted using the sequencer for regular channel group.*/
//	ADC_Init(ADC4, &ADC_InitStructure);																							 																						 
//	ADC_RegularChannelConfig(ADC4,ADC_Channel_1,1, ADC_SampleTime_2Cycles5);
	
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;
	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_InjectedChannel_1;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_InjectedChannel_2;
	ADC_InjectedInitStruct.ADC_NbrOfInjecChannel =2;
	ADC_InjectedInit(ADC1,&ADC_InjectedInitStruct);																				 
	ADC_InjectedChannelSampleTimeConfig(ADC1,ADC_InjectedChannel_1,ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC1,ADC_InjectedChannel_2,ADC_SampleTime_7Cycles5);
	

	
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;
	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_InjectedChannel_1;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_InjectedChannel_12;
	ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 2;
	ADC_InjectedInit(ADC2,&ADC_InjectedInitStruct);		
	

	ADC_InjectedChannelSampleTimeConfig(ADC2,ADC_InjectedChannel_1,ADC_SampleTime_7Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC2,ADC_InjectedChannel_12,ADC_SampleTime_61Cycles5);
	

	
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;
	
	
	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_InjectedChannel_1;


	ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 1;
	ADC_InjectedInit(ADC3,&ADC_InjectedInitStruct);																				 
	ADC_InjectedChannelSampleTimeConfig(ADC3,ADC_InjectedChannel_1,ADC_SampleTime_7Cycles5);

	ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
	ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_RisingEdge;
	ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_InjectedChannel_1;
	ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_InjectedChannel_1;

	ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 1;
	ADC_InjectedInit(ADC4,&ADC_InjectedInitStruct);																				 
	ADC_InjectedChannelSampleTimeConfig(ADC4,ADC_InjectedChannel_1,ADC_SampleTime_7Cycles5);

	ADC_CommonInitStruct.ADC_Clock = ADC_Clock_SynClkModeDiv4;//ADC_Clock_SynClkModeDiv1;     //应该选择多少合适
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_RegSimul;//ADC_Mode_InjSimul;//ADC_Mode_CombRegSimulInjSimul;  ADC_DMAMode_OneShot
	ADC_CommonInitStruct.ADC_TwoSamplingDelay  = 0;
	ADC_CommonInit(ADC1, &ADC_CommonInitStruct); 
	ADC_CommonInit(ADC2, &ADC_CommonInitStruct); 
	ADC_CommonInit(ADC4, &ADC_CommonInitStruct); 

//	//ADC3_JEOC中断
//	ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
//	//ADC_ITConfig(ADC2, ADC_IT_JEOC, ENABLE);	
//	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);	
//	//ADC1 校准
//	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
//  ADC_StartCalibration(ADC1);
//  while(ADC_GetCalibrationStatus(ADC1) != RESET );
//  calibration_value[0] = ADC_GetCalibrationValue(ADC1);
//	//ADC2 校准
//	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
//  ADC_StartCalibration(ADC2);
//  while(ADC_GetCalibrationStatus(ADC2) != RESET );
//  calibration_value[1] = ADC_GetCalibrationValue(ADC2);
//	//ADC3 校准
//	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
//  ADC_StartCalibration(ADC3);
//  while(ADC_GetCalibrationStatus(ADC3) != RESET );
//  calibration_value[2] = ADC_GetCalibrationValue(ADC3);
  // 允许ADC1
  ADC_Cmd(ADC1, ENABLE);	
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);

	ADC_StartInjectedConversion(ADC1);
	ADC_StartInjectedConversion(ADC2);
	ADC_StartInjectedConversion(ADC3);
	ADC_StartInjectedConversion(ADC4);

}

/**
  * @brief  Forced single-phase current output
  * @param  None
  * @retval None
  */
s16 j = 0;
void force_APhase_Open(TIM_TypeDef* TIMx)
{
	u32 i = 0;
	s16 temp_I = 0;
	PWM_Enable();
	ADC_ITConfig(ADC2, ADC_IT_JEOC, DISABLE);	
	for(i = 0;i < 0x2fffff;i++)
  __nop();
	ADC_Configuration_StartSinglePhase();
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_Active);
	TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//In
	TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);
	//TIMx->CCER = (u16)PWM_OUTPUT_REG;
  for(i = 0;i < 0x1fffff;i++)
  __nop(); 	
	ADC_DMACmd(ADC1, DISABLE);
	PWM_Disable();
	TIM_SelectOCxM(TIM1, TIM_Channel_1,TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_2,TIM_OCMode_PWM1);
	TIM_SelectOCxM(TIM1, TIM_Channel_3,TIM_OCMode_PWM1);
	struct_hFrame.internal_us = 2;
	for(j = 0;j < SIGNLE_PHASE_SAMPLE_NUM;j++)
	{
		temp_I = ((16*1444)*(s32)(value_array_adc1[j]-g_ZeroCur_MotorA))/(*g_pCur_ref_base_mA); // mA
		UART_PushFrame_HighFreqData(&struct_hFrame,200,temp_I);
	}
	g_CmdMap[SYS_SELF_TUNING] = 0;
	while(struct_hFrame.num_frame > 0)
	{
		for(i = 0;i < 0x1fff;i++)
		__nop(); 	
		UART_SendFrame_HighFreqData(&struct_hFrame);
	}	
	g_fast_debug = 0;
	ADC_Configuration_StopSinglePhase();
	ADC_ITConfig(ADC2, ADC_IT_JEOC, ENABLE);	
}




#endif
















