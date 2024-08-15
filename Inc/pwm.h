#ifndef __PWM_H__
#define __PWM_H__

#include "stm32g4xx_hal.h"

extern volatile int g_high_fre_pulse;

extern void PWM_Init(void);		//PWM��س�ʼ��
extern void PWM_Enable(void);
extern void PWM_Disable(void);
extern void PWM_OutPut_Open(TIM_TypeDef* TIMx, s32 ElectricAngle, s32 TimerCCR);
 void MX_TIM1_Init(void);
 void Calculate_CCR(TIM_TypeDef* TIMx, s32 angle, s32 targetCCR);	  // Ŀ��CCR 0~MAX_CCR��Ӧ0~100ռ�ձ�
 
#endif

