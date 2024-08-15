#ifndef __SECURITY_H__
#define __SECURITY_H__

#include "stm32g4xx_hal.h"
extern void generate_errorCode(void);
extern s32 CheckTemperature(void);      // �¶ȼ��
extern void ShortCurrentCheck(void);    // �������
extern s32 CheckPowVoltage(void);       // ��Դ��ѹ���
extern s32 CheckAbsolutePos(void);      // ���Ա��������
extern void ClearError(void);           // �������
extern void UpdateErrorStateLED(void);  // LED����ָʾ����
extern void Stroke_protection(void);
#endif
