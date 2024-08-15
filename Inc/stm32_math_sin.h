/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_MATH_SIN_H__
#define __STM32_MATH_SIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
/** 
  * @brief  Trigonometrical functions type definition ���Ǻ���
  */
typedef struct
{
  int16_t hSin;
  int16_t hCos;
} Trig_Components;

extern Trig_Components g_sin_cos_value;

/**
  * @brief  This function returns cosine and sine functions of the angle fed in 
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Trig_Components Cos(angle) and Sin(angle) in Trig_Components format
  */
//Trig_Components Math_Trig_Functions(int16_t hAngle);
Trig_Components Math_Trig_Functions_512(int16_t hAngle);
  
s32 Math_Sin_EN360(u16 angle);
s32 Math_Cos_EN360(u16 angle);

#endif

