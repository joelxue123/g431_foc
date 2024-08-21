#include "sensorless.h"
#include "adc.h"
#define SQ(x)				((x) * (x))


encoder_sensor encoder_sensor_;

motor_control_reporting motor_;

float wrap_pm_pi(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
float fast_atan2(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = MACRO_MIN(abs_x, abs_y) / (MACRO_MAX(abs_x, abs_y) + 1e-20);
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x)
        r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f)
        r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f)
        r = -r;

    return r;
}


void init_motor(motor_control_reporting *motor) {


    motor->config_.direction = 1.0f;
    motor->config_.phase_inductance = 0.0009f;
    motor->config_.phase_resistance = 2.0f;
    motor->config_.pm_flux_linkage = 0.0066f;
    motor->config_.pole_pairs = 1.0f;
    motor->config_.observer_gain = 100.0f;
    motor->config_.pll_bandwidth = 1000.0f;


    motor->current_meas_.phA = 0.0f;
    motor->current_meas_.phB = 0.0f;
    motor->current_meas_.phC = 0.0f;

	motor->current_control_.p_gain = 2000.f * motor->config_.phase_inductance;
	motor->current_control_.i_gain = motor->current_control_.p_gain * (motor->config_.phase_resistance)/ motor->config_.phase_inductance;
    motor->current_control_.final_v_alpha = 0.0f;
    motor->current_control_.final_v_beta = 0.0f;


    motor->sensorless_estimator_.V_alpha_beta_memory_[0] = 0.0f;
    motor->sensorless_estimator_.V_alpha_beta_memory_[1] = 0.0f;
    motor->sensorless_estimator_.flux_state_[0] = 0.0f;
    motor->sensorless_estimator_.flux_state_[1] = 0.0f;
    motor->sensorless_estimator_.flux_state_memory_[0] = 0.0f;
    motor->sensorless_estimator_.flux_state_memory_[1] = 0.0f;
    motor->sensorless_estimator_.vel_estimate_valid_ = true;
    motor->sensorless_estimator_.pll_pos_ = 0.0f;
    motor->sensorless_estimator_.vel_estimate_erad_ = 0.0f;

    motor_.vel_estimate_erad_ = &encoder_sensor_.vel_estimate_erad_;
    motor_.phase_ = &encoder_sensor_.phase_;
    motor->vel_estimate_ = 0.0f;
    

    encoder_sensor_.pll_bandwidth =1000.f;

}

void update_current_meas(motor_control_reporting *motor, float phA, float phB, float phC)
{
	motor->current_meas_.phA = phA;
    motor->current_meas_.phB = phB;
    motor->current_meas_.phC = phC;
}
void update_current_control(motor_control_reporting *motor, float final_v_alpha, float final_v_beta)
{
	motor->current_control_.final_v_alpha = final_v_alpha;
    motor->current_control_.final_v_beta = final_v_beta;
}
int non_linear_flux_observer1(void) {
    // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
    // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    // In particular, equation 8 (and by extension eqn 4 and 6).

    // The V_alpha_beta applied immedietly prior to the current measurement associated with this cycle
    // is the one computed two cycles ago. To get the correct measurement, it was stored twice:
    // once by final_v_alpha/final_v_beta in the current control reporting, and once by V_alpha_beta_memory.

    // Clarke transform
    float I_alpha_beta[2] = {
        -motor_.current_meas_.phB - motor_.current_meas_.phC,
        one_by_sqrt3 * (motor_.current_meas_.phB - motor_.current_meas_.phC)};

    // Swap sign of I_beta if motor is reversed
    I_alpha_beta[1] *= motor_.config_.direction;

    // alpha-beta vector operations
    float eta[2];
    for (int i = 0; i <= 1; ++i) {
        // y is the total flux-driving voltage (see paper eqn 4)
        float y = -motor_.config_.phase_resistance * I_alpha_beta[i] + motor_.sensorless_estimator_.V_alpha_beta_memory_[i];
        // flux dynamics (prediction)
        float x_dot = y;
        // integrate prediction to current timestep
        motor_.sensorless_estimator_.flux_state_[i] += x_dot * DT;

        // eta is the estimated permanent magnet flux (see paper eqn 6)
        eta[i] = motor_.sensorless_estimator_.flux_state_[i] - motor_.config_.phase_inductance * I_alpha_beta[i];
    }

    // Non-linear observer (see paper eqn 8):
    float pm_flux_sqr = motor_.config_.pm_flux_linkage * motor_.config_.pm_flux_linkage;
    float est_pm_flux_sqr = eta[0] * eta[0] + eta[1] * eta[1];
    float bandwidth_factor = 1.0f / pm_flux_sqr;
	if(est_pm_flux_sqr > pm_flux_sqr )
	{
		//est_pm_flux_sqr = pm_flux_sqr;
	}
    float eta_factor = 0.5f * (motor_.config_.observer_gain * bandwidth_factor) * (pm_flux_sqr - est_pm_flux_sqr);

    // alpha-beta vector operations
    for (int i = 0; i <= 1; ++i) {
        // add observer action to flux estimate dynamics
        float x_dot = eta_factor * eta[i];
		
        // convert action to discrete-time
        motor_.sensorless_estimator_.flux_state_[i] += x_dot * DT;
        // update new eta
        eta[i] = motor_.sensorless_estimator_.flux_state_[i] - motor_.config_.phase_inductance * I_alpha_beta[i];
    }

    // Flux state estimation done, store V_alpha_beta for next timestep
    motor_.sensorless_estimator_.V_alpha_beta_memory_[0] = motor_.current_control_.final_v_alpha;
    motor_.sensorless_estimator_.V_alpha_beta_memory_[1] = motor_.current_control_.final_v_beta * motor_.config_.direction;

    // PLL
    // TODO: the PLL part has some code duplication with the encoder PLL
    // Pll gains as a function of bandwidth
    float pll_kp = 2.0f * motor_.config_.pll_bandwidth;
    // Critically damped
    float pll_ki = 0.25f * (pll_kp * pll_kp);
    // Check that we don't get problems with discrete time approximation
    if (!(DT * pll_kp < 1.0f)) {
        motor_.sensorless_estimator_.vel_estimate_valid_ = false;
        return false;
    }

    // predict PLL phase with velocity
    motor_.sensorless_estimator_.pll_pos_ = wrap_pm_pi(motor_.sensorless_estimator_.pll_pos_ + DT * motor_.sensorless_estimator_.vel_estimate_erad_);
    // update PLL phase with observer permanent magnet phase
    motor_.sensorless_estimator_.phase_ = fast_atan2(eta[1], eta[0]);//+ M_PI/2.0f;//- M_PI/2.0f;// + M_PI/2.0f ;
	motor_.sensorless_estimator_.phase_ = wrap_pm_pi(motor_.sensorless_estimator_.phase_ )   ;
	if(motor_.sensorless_estimator_.phase_ < 0) motor_.sensorless_estimator_.phase_ += 2.0f * M_PI;
    float delta_phase = wrap_pm_pi(motor_.sensorless_estimator_.phase_ - motor_.sensorless_estimator_.pll_pos_);
    motor_.sensorless_estimator_.pll_pos_ = wrap_pm_pi(motor_.sensorless_estimator_.pll_pos_ + DT * pll_kp * delta_phase);
    // update PLL velocity
    motor_.sensorless_estimator_.vel_estimate_erad_ += DT * pll_ki * delta_phase;
    // convert to mechanical turns/s for controller usage.
    motor_.sensorless_estimator_.vel_estimate_valid_ = true;
    return true;
};



int non_linear_flux_observer(void) {
    // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
    // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    // In particular, equation 8 (and by extension eqn 4 and 6).

    // The V_alpha_beta applied immedietly prior to the current measurement associated with this cycle
    // is the one computed two cycles ago. To get the correct measurement, it was stored twice:
    // once by final_v_alpha/final_v_beta in the current control reporting, and once by V_alpha_beta_memory.

    // Clarke transform
    float I_alpha_beta[2] = {
        -motor_.current_meas_.phB - motor_.current_meas_.phC,
        one_by_sqrt3 * (motor_.current_meas_.phB - motor_.current_meas_.phC)};

    // Swap sign of I_beta if motor is reversed
    I_alpha_beta[1] *= motor_.config_.direction;



	float L_ia =  motor_.config_.phase_inductance * I_alpha_beta[0];
	float L_ib =  motor_.config_.phase_inductance * I_alpha_beta[1];
	float R_ia = motor_.config_.phase_resistance *  I_alpha_beta[0];
	float R_ib = motor_.config_.phase_resistance *  I_alpha_beta[1];
	float pm_flux_sqr = motor_.config_.pm_flux_linkage * motor_.config_.pm_flux_linkage;
    float bandwidth_factor = 1.0f / pm_flux_sqr;
	const float gamma_half = 0.5f * (motor_.config_.observer_gain * bandwidth_factor);

	float err = SQ(motor_.config_.pm_flux_linkage) - (SQ(motor_.sensorless_estimator_.flux_state_[0] - L_ia) + SQ(motor_.sensorless_estimator_.flux_state_[1] - L_ib));
	if (err > 0.0) {
		//err = 0.0;
	}

	float x1_dot = motor_.sensorless_estimator_.V_alpha_beta_memory_[0] - R_ia + gamma_half * (motor_.sensorless_estimator_.flux_state_[0]  - L_ia) * err;
	float x2_dot = motor_.sensorless_estimator_.V_alpha_beta_memory_[1] - R_ib + gamma_half * (motor_.sensorless_estimator_.flux_state_[1]  - L_ib) * err;

	motor_.sensorless_estimator_.flux_state_[0]  += x1_dot * DT;
	motor_.sensorless_estimator_.flux_state_[1]  += x2_dot * DT;
	float eta[2];
	eta[0] = motor_.sensorless_estimator_.flux_state_[0] -  motor_.config_.phase_inductance * I_alpha_beta[0];
	eta[1] = motor_.sensorless_estimator_.flux_state_[1] -  motor_.config_.phase_inductance * I_alpha_beta[1];

    // Flux state estimation done, store V_alpha_beta for next timestep
    motor_.sensorless_estimator_.V_alpha_beta_memory_[0] = motor_.current_control_.final_v_alpha;
    motor_.sensorless_estimator_.V_alpha_beta_memory_[1] = motor_.current_control_.final_v_beta * motor_.config_.direction;

    // PLL
    // TODO: the PLL part has some code duplication with the encoder PLL
    // Pll gains as a function of bandwidth
    float pll_kp = 2.0f * motor_.config_.pll_bandwidth;
    // Critically damped
    float pll_ki = 0.25f * (pll_kp * pll_kp);
    // Check that we don't get problems with discrete time approximation
    if (!(DT * pll_kp < 1.0f)) {
        motor_.sensorless_estimator_.vel_estimate_valid_ = false;
        return false;
    }

    // predict PLL phase with velocity
    motor_.sensorless_estimator_.pll_pos_ = wrap_pm_pi(motor_.sensorless_estimator_.pll_pos_ + DT * motor_.sensorless_estimator_.vel_estimate_erad_);
    // update PLL phase with observer permanent magnet phase
    motor_.sensorless_estimator_.phase_ = fast_atan2(eta[1], eta[0]);//+ M_PI/2.0f;//- M_PI/2.0f;// + M_PI/2.0f ;
	motor_.sensorless_estimator_.phase_ = wrap_pm_pi(motor_.sensorless_estimator_.phase_ )   ;
	if(motor_.sensorless_estimator_.phase_ < 0) motor_.sensorless_estimator_.phase_ += 2.0f * M_PI;
    float delta_phase = wrap_pm_pi(motor_.sensorless_estimator_.phase_ - motor_.sensorless_estimator_.pll_pos_);
    motor_.sensorless_estimator_.pll_pos_ = wrap_pm_pi(motor_.sensorless_estimator_.pll_pos_ + DT * pll_kp * delta_phase);
    // update PLL velocity
    motor_.sensorless_estimator_.vel_estimate_erad_ += DT * pll_ki * delta_phase;
    // convert to mechanical turns/s for controller usage.
    motor_.sensorless_estimator_.vel_estimate_valid_ = true;
    return true;
};



void encode_sample(void)
{
	float sincos_sample_s_ = adc_measurements_[0] - 2064.f;
	float sincos_sample_c_ = adc_measurements_[1] -2064.f;

	float phase = fast_atan2(sincos_sample_c_, sincos_sample_s_) + M_PI*(float)g_Encode_offset/180.f - M_PI/2.0f;
    phase = wrap_pm_pi(phase);
    encoder_sensor_.phase_ = phase;
    if(encoder_sensor_.phase_ < 0) encoder_sensor_.phase_ += 2.0f * M_PI;
        float pll_kp = 2.0f * encoder_sensor_.pll_bandwidth;
    // Critically damped
    float pll_ki = 0.25f * (pll_kp * pll_kp);
    // Check that we don't get problems with discrete time approximation
    if (!(DT * pll_kp < 1.0f)) {
        encoder_sensor_.vel_estimate_valid_ = false;
        return false;
    }

    // predict PLL phase with velocity
    encoder_sensor_.pll_pos_ = wrap_pm_pi(encoder_sensor_.pll_pos_ + DT * encoder_sensor_.vel_estimate_erad_);
;
	
    float delta_phase = wrap_pm_pi(encoder_sensor_.phase_ - encoder_sensor_.pll_pos_);
    encoder_sensor_.pll_pos_ = wrap_pm_pi(encoder_sensor_.pll_pos_ + DT * pll_kp * delta_phase);
    // update PLL velocity
    encoder_sensor_.vel_estimate_erad_ += DT * pll_ki * delta_phase;
    // convert to mechanical turns/s for controller usage.
	
    encoder_sensor_.vel_estimate_valid_ = true;
}

void sensorless_sensor_phase_switch(void)
{
    if( fabsf(motor_.vel_estimate_) < 300.f)
    {
        motor_.vel_estimate_erad_ = &encoder_sensor_.vel_estimate_erad_;
        motor_.phase_ = &encoder_sensor_.phase_;

    }
    else if( fabsf(motor_.vel_estimate_ )> 600.f)
    {
        motor_.vel_estimate_erad_ = &motor_.sensorless_estimator_.vel_estimate_erad_;
         motor_.phase_ = &motor_.sensorless_estimator_.phase_;
    }
    else{}

    motor_.vel_estimate_ = 30.f*(*motor_.vel_estimate_erad_) / (motor_.config_.pole_pairs * 2.0f * M_PI);///23000.0f;
    g_CmdMap[CMD_SPD_ACT_PU] = motor_.vel_estimate_;
}