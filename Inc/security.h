#ifndef __SECURITY_H__
#define __SECURITY_H__

#include "stm32g4xx_hal.h"
extern void generate_errorCode(void);
extern s32 CheckTemperature(void);      // 温度检测
extern void ShortCurrentCheck(void);    // 过流检测
extern s32 CheckPowVoltage(void);       // 电源电压检测
extern s32 CheckAbsolutePos(void);      // 绝对编码器检测
extern void ClearError(void);           // 清除错误
extern void UpdateErrorStateLED(void);  // LED报警指示管理
extern void Stroke_protection(void);
#endif
