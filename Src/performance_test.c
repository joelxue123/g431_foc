#include "performance_test.h"
#include "math.h"
/**
  * @brief  生成正弦波数据
  * @param  ptr_sinWave: 结构体指针
  * @param  para_frea_exc: 计算执行周期   
  * @retval None
  */
void ini_waveSinGenrator(struct WAVE_SIN_GEN_TYPE * ptr_sinWave,float para_frea_exc)
{
	ptr_sinWave->peak = 4096+8192;
	ptr_sinWave->trough = 4096;
	ptr_sinWave->cycles_set = 5;  //设置运行次数
	ptr_sinWave->freq_sin = 0.2f;  //输出的正弦波频率
	ptr_sinWave->freq_exc = para_frea_exc;  //该函数的执行频率
	ptr_sinWave->cycles_moved = 0;  //已经运行的次数
	ptr_sinWave->moving = 0;
}
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
void setPara_waveSinGenrator(struct WAVE_SIN_GEN_TYPE * ptr_sinWave,float para_freq_sin,s32 para_peak,s32 para_trough,s32 para_cycles_set,s32 start_pos)
{
	s32 temp_start_pos = 0;
	ptr_sinWave->peak = para_peak;
	ptr_sinWave->trough = para_trough;
	ptr_sinWave->cycles_set = para_cycles_set;  //设置运行次数
	ptr_sinWave->freq_sin = para_freq_sin;
	if(start_pos<para_trough )
	{
		temp_start_pos = para_trough;
	}
	else 	if(start_pos>para_peak)
	{
		temp_start_pos = para_peak;
	}
	else
	{
		temp_start_pos = start_pos;
	}

	ptr_sinWave->moving = 1;
	ptr_sinWave->cycles_moved = 0;  //已经运行的次数
	ptr_sinWave->angle_p_exc = (s32)((ptr_sinWave->freq_sin/ptr_sinWave->freq_exc)*(float)EN_360);
	ptr_sinWave->ampl_sin = (ptr_sinWave->peak-ptr_sinWave->trough)/2;
	ptr_sinWave->core_sin = (ptr_sinWave->peak+ptr_sinWave->trough)/2;
	ptr_sinWave->phase_ini = (s32)(asin((float)(temp_start_pos-ptr_sinWave->core_sin)/(float)ptr_sinWave->ampl_sin)*(float)EN_180/3.14159f);
	//ptr_sinWave->angle = ptr_sinWave->phase_ini;
}
/**
  * @brief  计算生成波形
  * @param  ptr_sinWave: 结构体指针
  * @retval 
  */
void generate_posWaveSin(struct WAVE_SIN_GEN_TYPE * ptr_sinWave)
{
	s32 angle_and_iniPhase = 0;
	if(ptr_sinWave->moving == 0)
	{
		return;
	}
	if(ptr_sinWave->cycles_moved>=ptr_sinWave->cycles_set)
	{
		ptr_sinWave->moving = 0;
		return;
	}
	ptr_sinWave->angle = ptr_sinWave->angle+ptr_sinWave->angle_p_exc;
	if(ptr_sinWave->angle > EN_360)
	{
		ptr_sinWave->cycles_moved++;
		ptr_sinWave->angle = ptr_sinWave->angle - EN_360;
	}
	angle_and_iniPhase =  (ptr_sinWave->angle+ptr_sinWave->phase_ini)%EN_360;
	s32 sin_Q15 = Math_Sin_EN360(angle_and_iniPhase);
	ptr_sinWave->out_pos = ((sin_Q15*ptr_sinWave->ampl_sin)>>15) + ptr_sinWave->core_sin; 
}
/**
  * @brief  中断波形生成计算
  * @param  ptr_sinWave: 结构体指针
  * @retval 
  */
void stop_posWaveSin(struct WAVE_SIN_GEN_TYPE * ptr_sinWave)
{
	ptr_sinWave->moving = 0;
}



