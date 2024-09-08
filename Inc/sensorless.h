#ifndef __SENSORLESS_H__
#define __SENSORLESS_H__

#include "stm32g4xx_hal.h"
#include "global.h"
#include <math.h>>
#include <float.h>

#define DT (0.00004f)
#define M_PI (3.14159265358979323846f)
#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))


static const float one_by_sqrt3 = 0.57735026919f;
static const float two_by_sqrt3 = 1.15470053838f;
static const float sqrt3_by_2 = 0.86602540378f;

typedef struct 
{
    short raw_phase;
    float phase_;
    float vel_estimate_erad_;
    float pll_bandwidth;
    float pll_pos_;
	float vel_estimate_valid_;
} encoder_sensor;

typedef struct 
{
	float V_alpha_beta_memory_[2];
    float flux_state_[2];
	float flux_state_memory_[2];
    float phase_;
    float vel_estimate_erad_;
    float pll_bandwidth;
    float pll_pos_;
	float vel_estimate_valid_;

}sensorless;



typedef struct  {
    float direction;
    float phase_inductance;
    float phase_resistance;
    float pm_flux_linkage;
    float pole_pairs;
    float observer_gain;
    float pll_bandwidth;
}sensorless_motor_control_config_t;

typedef struct  {
    float phA;
    float phB;
    float phC;
} sensorless_motor_control_meas_t;

typedef struct 
{
	float p_gain; // [V/A]
	float i_gain; // [V/As]
	float v_current_control_integral_d; // [V]
	float v_current_control_integral_q; // [V]
	float final_v_alpha;
	float final_v_beta;
} sensorless_current_control;

typedef struct 
{
	sensorless_motor_control_config_t config_;
	sensorless_motor_control_meas_t current_meas_;
	sensorless_current_control current_control_;
	sensorless sensorless_estimator_;
	float *vel_estimate_erad_;
	float *phase_;
	float vel_estimate_;
    float vel_estimate_filter;

} motor_control_reporting;

extern motor_control_reporting motor_;
extern encoder_sensor encoder_sensor_;

void init_motor(motor_control_reporting *motor);
void update_current_meas(motor_control_reporting *motor, float phA, float phB, float phC);
void update_current_control(motor_control_reporting *motor, float final_v_alpha, float final_v_beta);
int non_linear_flux_observer(void);
void encode_sample(void);
void sensorless_sensor_phase_switch(void);
#endif

