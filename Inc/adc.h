#ifndef __ADC_H__
#define __ADC_H__


#include "stm32g4xx_hal.h"

#define ADC_REF 3300 															// ʹ�ø߾������ñ����
#define VOLT_SAMPNUM 			20 											//��ѹƽ���˲���������
#define TEMP_SAMPNUM 			20 											//�¶�ƽ���˲���������
#define OTHER_SAMPNUM 		20

extern volatile s16 VOLT_Filter;															
extern volatile s16 TEMP_Filter;
extern s16 MU_Value_error_cnt;
extern  volatile	s32 MU_Value_base_0;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


extern volatile uint16_t adc_measurements_[2];

extern void ADC_Configuration(void);  				// ADC��س�ʼ��
extern s32 Get_VOLT_Filter(void);
extern s32 Get_TEMP_Filter(void);
void pos_linearity_ini(void);
void pos_linearity_set_default(void);
int32_t Pos_Correct(int32_t Pos_Org);
void Get_Pos_Rod(void);
void Get_Temp_Filter(void);
s16 check_linearity(void);
int16_t get_temperature_value(int16_t para_val);
void pos_linearity_ini(void);
int32_t Pos_Correct(int32_t Pos_Org);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void start_temperature_adc(void);

#endif
