#ifndef __PERFORMANCE_H__
#define __PERFORMANCE_H__
#include "stm32g4xx_hal.h"
#include "global.h"
struct WAVE_SIN_GEN_TYPE{
	s32 peak,trough;
	s32 cycles_set;  //设置运行次数
	float freq_sin;  //输出的正弦波频率
	float freq_exc;  //该函数的执行频率
	s32 ampl_sin,core_sin;//输出的正弦波振幅和中心位置
	s32 angle_p_exc; //一次执行周期内可以旋转的角度
	s32 cycles_moved;  //已经运行的次数
	s32 phase_ini;//初始相位
	s32 angle;  //已经运行的次数
	u8 moving;
	s32 out_pos;
};
/**
  * @brief  生成正弦波数据
  * @param  ptr_sinWave: 结构体指针
  * @param  para_frea_exc: 计算执行周期   
  * @retval None
  */
void ini_waveSinGenrator(struct WAVE_SIN_GEN_TYPE * ptr_sinWave,float para_frea_exc);

/**
  * @brief  生成波形的参数设计
  * @param  ptr_sinWave: 结构体指针
  * @param  para_freq_sin: 正弦波的频率
  * @param  para_peak: 波峰 
  * @param  para_trough: 波谷 
  * @param  para_cycles_se: 执行周期个数 
  * @param  start_pos: 起始位置 
  * @retval None
  */
void setPara_waveSinGenrator(struct WAVE_SIN_GEN_TYPE * ptr_sinWave,float para_freq_sin,s32 para_peak,s32 para_trough,s32 para_cycles_set,s32 start_pos);
/**
  * @brief  计算生成波形
  * @param  ptr_sinWave: 结构体指针
  * @retval 
  */
void generate_posWaveSin(struct WAVE_SIN_GEN_TYPE * p_sinWave);
/**
  * @brief  中断波形生成计算
  * @param  ptr_sinWave: 结构体指针
  * @retval 
  */
void stop_posWaveSin(struct WAVE_SIN_GEN_TYPE * p_sinWave);
#endif



