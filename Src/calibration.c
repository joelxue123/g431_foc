#include "calibration.h"
#include "global.h"

//�������У��������������ƫ�������ڼ������ʱ��ƫ��������
//ȫ�ֱ���g_ZeroCur_Motor1B,g_ZeroCur_Motor1C

void delay_ms( int cnt)
{
	volatile int i = 0 , j=0;
	
	for(j=0;j<cnt;j++)
		for(i =0; i< 10000;i++);
	
	
}

s32 CurrentDemarcate(void)
{
  u16 j;
  u16 temp1;
  u32 Cur1_B = 0;
  u32 Cur1_C = 0;
	u32 Cur1_A = 0;
  s32 result = 1;
  temp1 = TIM1->CCER;
	//bsp_DelayMS(100);
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCER = (u16)0x0000;   //1100,CC1E=0��ֹ,CC1P=0�ߵ�ƽ��Ч,CC1NE=1ʹ��,CC1NP=1�͵�ƽ��Ч
	__HAL_ADC_DISABLE_IT(&hadc1,ADC_FLAG_JEOS);
	MP6540_ON();
	TIM1->CCR3 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR1 = 0;
	//TIMx->CCER = (u16)0x5FFF;
	TIM1->CCER = (u16)PWM_OUTPUT_REG;
	delay_ms(10000);
	for (j=0; j<32; j++)
	{
//		FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
		while(__HAL_ADC_GET_FLAG(&hadc1,ADC_FLAG_JEOS)==RESET)
		{}
		__HAL_ADC_CLEAR_FLAG(&hadc1,ADC_FLAG_JEOS);
		//bsp_DelayMS(10);
		Cur1_A += hadc1.Instance->JDR1;
    Cur1_B += hadc2.Instance->JDR1;	
	}
	
	g_ZeroCur_MotorA = (Cur1_A)>>5;
  g_ZeroCur_MotorB = (Cur1_B)>>5;

  if (		g_ZeroCur_MotorA < (1848) || g_ZeroCur_MotorA > (2248) ||
					g_ZeroCur_MotorB < (1848) || g_ZeroCur_MotorB > (2248)  	)
	{
    result = 0;
  }	
	MP6540_OFF();
	__HAL_ADC_ENABLE_IT(&hadc1,ADC_FLAG_JEOS);
//  TIM1->CCER = temp1;
  return result;
}









