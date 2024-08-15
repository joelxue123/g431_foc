#ifndef __PERFORMANCE_H__
#define __PERFORMANCE_H__
#include "stm32g4xx_hal.h"
#include "global.h"
struct WAVE_SIN_GEN_TYPE{
	s32 peak,trough;
	s32 cycles_set;  //�������д���
	float freq_sin;  //��������Ҳ�Ƶ��
	float freq_exc;  //�ú�����ִ��Ƶ��
	s32 ampl_sin,core_sin;//��������Ҳ����������λ��
	s32 angle_p_exc; //һ��ִ�������ڿ�����ת�ĽǶ�
	s32 cycles_moved;  //�Ѿ����еĴ���
	s32 phase_ini;//��ʼ��λ
	s32 angle;  //�Ѿ����еĴ���
	u8 moving;
	s32 out_pos;
};
/**
  * @brief  �������Ҳ�����
  * @param  ptr_sinWave: �ṹ��ָ��
  * @param  para_frea_exc: ����ִ������   
  * @retval None
  */
void ini_waveSinGenrator(struct WAVE_SIN_GEN_TYPE * ptr_sinWave,float para_frea_exc);

/**
  * @brief  ���ɲ��εĲ������
  * @param  ptr_sinWave: �ṹ��ָ��
  * @param  para_freq_sin: ���Ҳ���Ƶ��
  * @param  para_peak: ���� 
  * @param  para_trough: ���� 
  * @param  para_cycles_se: ִ�����ڸ��� 
  * @param  start_pos: ��ʼλ�� 
  * @retval None
  */
void setPara_waveSinGenrator(struct WAVE_SIN_GEN_TYPE * ptr_sinWave,float para_freq_sin,s32 para_peak,s32 para_trough,s32 para_cycles_set,s32 start_pos);
/**
  * @brief  �������ɲ���
  * @param  ptr_sinWave: �ṹ��ָ��
  * @retval 
  */
void generate_posWaveSin(struct WAVE_SIN_GEN_TYPE * p_sinWave);
/**
  * @brief  �жϲ������ɼ���
  * @param  ptr_sinWave: �ṹ��ָ��
  * @retval 
  */
void stop_posWaveSin(struct WAVE_SIN_GEN_TYPE * p_sinWave);
#endif



