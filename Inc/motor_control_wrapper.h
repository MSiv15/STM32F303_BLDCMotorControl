/*
 * motor_control_wrapper.h
 *
 *  Created on: 2018/07/19
 *      Author: Shibasaki
 */

#ifndef MOTOR_CONTROL_WRAPPER_H_
#define MOTOR_CONTROL_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

extern uint16_t adc_data1, adc_data2, adc_data3, adc_data4;

// Maximum Value of PWM Counter
extern uint32_t period;

// Reference of Speed Control
extern float omega_ref;

// Total Calculation Time
extern uint32_t t_calc;

// Flag for Fault Detection
extern volatile bool is_warning_reported;

// Table for Debugging
#define N_TABLE 1000
extern volatile bool is_table_full;

extern float i_a_arr[N_TABLE];
extern float i_b_arr[N_TABLE];
extern float i_c_arr[N_TABLE];
extern float i_alpha_arr[N_TABLE];
extern float i_beta_arr[N_TABLE];
extern float i_d_arr[N_TABLE];
extern float i_q_arr[N_TABLE];
extern float v_d_arr[N_TABLE];
extern float v_q_arr[N_TABLE];
extern float e_d_arr[N_TABLE];
extern float e_q_arr[N_TABLE];
extern float e_abs_arr[N_TABLE];
extern float v_alpha_arr[N_TABLE];
extern float v_beta_arr[N_TABLE];
extern float duty_a_arr[N_TABLE];
extern float duty_b_arr[N_TABLE];
extern float duty_c_arr[N_TABLE];
extern float v_wind_d_arr[N_TABLE];
extern float v_wind_q_arr[N_TABLE];
extern float omega_arr[N_TABLE];
extern float theta_err_arr[N_TABLE];
extern uint32_t t_calc_arr[N_TABLE];

void FOC_EmergencyStop(void);
void FOC_LowFreqTask(void);
void FOC_HighFreqTask(void);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_CONTROL_WRAPPER_H_ */
