/*
 * motor_control_foc.cpp
 *
 *  Created on: 2018/07/19
 *      Author: Shibasaki
 */

#include "motor_control_foc.hpp"

void BLDCMotor::Update(void)
{
  // Calculation of Sine and Cosine Value of the Rotor Position
  MyDSP::SinCos(theta_e, &sin_val, &cos_val);

  // a-b-c to alpha-beta Conversion
  FOCUtil::Clarke_Dynamic(i_a, i_b, i_c, SV_sector, &i_alpha, &i_beta);

  // alpha-beta to d-q Conversion
  FOCUtil::Park(i_alpha, i_beta, &i_d, &i_q, sin_val, cos_val);

  float i_d_hat_prev = i_d_hat;
  float i_q_hat_prev = i_q_hat;

  // State Observer
  // i_d_hat, i_q_hatは1サンプル先の予想値
  //// x = (i_d i_q;d_d d_q)
  //// u = (v_d v_q)
  //// y = (i_d i_q)
  //// x_hat[n+1|n] = (A-LC B L) * (x_hat[n|n−1];u[n];y[n])
  Eigen::Matrix<float, OBS_NUM_STATE+OBS_NUM_INPUT+OBS_NUM_OUTPUT, OBS_NUM_CHANNEL> obs_in;
  obs_in << i_d_hat, i_q_hat, d_d_hat, d_q_hat, v_d, v_q, i_d, i_q;
  Eigen::Map<Eigen::Matrix<float, OBS_NUM_STATE, OBS_NUM_CHANNEL, Eigen::RowMajor>> obs_out(obs_out_data);
  obs_out.noalias() = obs_mat * obs_in;

  constexpr float K_XC = R*T_S*L_Q/L_D*std::exp(-R*T_S/L_D)/(1-std::exp(-R*T_S/L_D));
  e_d = d_d_hat + omega_e*K_XC*i_q_hat_prev;
  e_q = d_q_hat - omega_e*K_XC*i_d_hat_prev;

//  e_d = d_d_hat + omega_e*L_Q*i_q_hat_prev;
//  e_q = d_q_hat - omega_e*L_Q*i_d_hat_prev;

  e_abs = MyDSP::Hypot(e_d, e_q);

  // Damping Control
  Eigen::Vector2f hpf_e_dq_out = hpf_e_dq(Eigen::Map<Eigen::Vector2f>(e_dq));
  float i_damp_d = K_DAMP * hpf_e_dq_out[0];
  float i_damp_q = K_DAMP * hpf_e_dq_out[1];

  // Current PI Control with Anti-Windup
  switch (status)
  {
    case FOC_Status::STOP:
      pid_i_d.Clear();
      pid_i_q.Clear();
      v_d = 0;
      v_q = 0;
      break;
    case FOC_Status::IDENTIFICATION:
      pid_i_d.Clear();
      pid_i_q.Clear();
      v_d = v_d_ref;
      v_q = v_q_ref;
      break;
    case FOC_Status::POSITIONING:
      pid_i_d.SetOutput(pid_i_d.GetOutput() - K_WIND*v_wind_d);
      pid_i_q.SetOutput(pid_i_q.GetOutput() - K_WIND*v_wind_q);
      v_d = pid_i_d((i_pos_ref-i_damp_d)-i_d_hat);
      v_q = pid_i_q(-i_damp_q-i_q_hat);
      v_d += d_d_hat;
      v_q += d_q_hat;
      break;
    case FOC_Status::OPERATION_FEEDFORWARD:
      pid_i_d.SetOutput(pid_i_d.GetOutput() - K_WIND*v_wind_d);
      pid_i_q.SetOutput(pid_i_q.GetOutput() - K_WIND*v_wind_q);
      v_d = pid_i_d((i_d_ref_ff-i_damp_d)-i_d_hat);
      v_q = pid_i_q((i_q_ref_ff-i_damp_q)-i_q_hat);
      v_d += d_d_hat;
      v_q += d_q_hat;
      break;
    default:
      pid_i_d.SetOutput(pid_i_d.GetOutput() - K_WIND*v_wind_d);
      pid_i_q.SetOutput(pid_i_q.GetOutput() - K_WIND*v_wind_q);
      v_d = pid_i_d(i_d_ref-i_d_hat);
      v_q = pid_i_q(i_q_ref-i_q_hat);
      v_d += d_d_hat;
      v_q += d_q_hat;
      break;
  }

  // Voltage Saturation
  v_abs = MyDSP::Hypot(v_d, v_q);
  float v_max = v_dc*MyDSP::by_sqrt_three_f;
  if (v_abs > v_max)
  {
    float v_d_tmp = v_d;
    float v_q_tmp = v_q;
    if (v_d > v_max)
    {
      v_d = v_max;
      v_q = 0;
    }
    else if (v_d < -v_max)
    {
      v_d = -v_max;
      v_q = 0;
    }
    else if(v_q > 0)
    {
      v_q = MyDSP::Sqrt(v_max*v_max - v_d*v_d);
    }
    else if(v_q < 0)
    {
      v_q = -MyDSP::Sqrt(v_max*v_max - v_d*v_d);
    }
    v_wind_d = v_d_tmp - v_d;
    v_wind_q = v_q_tmp - v_q;
    v_abs = v_max;
  }
  else
  {
    v_wind_d = 0;
    v_wind_q = 0;
  }

  // Error of Position Estimation
  theta_err = MyDSP::Atan2(-MyDSP::Sign(omega_e)*e_d, MyDSP::Sign(omega_e)*e_q);

  // Position Calculation and State Transition
  CalculatePosition();

  // Calculation of Sine and Cosine Value of the Corrected Rotor Position
  float sin_val_v_dq, cos_val_v_dq;
  MyDSP::SinCos(theta_e+omega_e*0.5f*T_S*L_Q/L_D, &sin_val_v_dq, &cos_val_v_dq);

  // d-q to alpha-beta Conversion
  FOCUtil::InvPark(v_d, v_q, &v_alpha, &v_beta, sin_val_v_dq, cos_val_v_dq);

  // Duty Cycle Generation
  SV_sector = FOCUtil::SVPWM_3phase(v_alpha, v_beta, v_dc, &duty_a, &duty_b, &duty_c);

  // Estimation of Next Three-phase Currents
  float i_alpha_hat, i_beta_hat;
  float i_a_hat, i_b_hat, i_c_hat;
  FOCUtil::InvPark(i_d_hat, i_q_hat, &i_alpha_hat, &i_beta_hat, sin_val_v_dq, cos_val_v_dq);
  FOCUtil::InvClarke_abc(i_alpha_hat, i_beta_hat, &i_a_hat, &i_b_hat, &i_c_hat);

  // Dead Time Compensation
  float i_ripple_a, i_ripple_b, i_ripple_c;
  FOCUtil::CurrentRippleEstimate(duty_a, duty_b, duty_c, v_dc, F_PWM, (L_D+L_Q)/2, SV_sector, &i_ripple_a, &i_ripple_b, &i_ripple_c);
//  float i_dt = v_dc * 2 * C_OSS / T_DEAD;
//  duty_a = FOCUtil::DeadTimeCompensate_TTCM(duty_a, i_a+i_ripple_a, i_a-i_ripple_a, i_dt, F_PWM*T_DEAD);
//  duty_b = FOCUtil::DeadTimeCompensate_TTCM(duty_b, i_b+i_ripple_b, i_b-i_ripple_b, i_dt, F_PWM*T_DEAD);
//  duty_c = FOCUtil::DeadTimeCompensate_TTCM(duty_c, i_c+i_ripple_c, i_c-i_ripple_c, i_dt, F_PWM*T_DEAD);
  duty_a = FOCUtil::DeadTimeCompensate_3Level(duty_a, i_a_hat, i_ripple_a, F_PWM*T_DEAD);
  duty_b = FOCUtil::DeadTimeCompensate_3Level(duty_b, i_b_hat, i_ripple_b, F_PWM*T_DEAD);
  duty_c = FOCUtil::DeadTimeCompensate_3Level(duty_c, i_c_hat, i_ripple_c, F_PWM*T_DEAD);
}

void BLDCMotor::CalculatePosition(void)
{
  static uint32_t pos_count = 0;

  switch (status)
  {
    case FOC_Status::STOP:
    case FOC_Status::IDENTIFICATION:
      alpha_e = 0;
      omega_e = 0;
      break;
    case FOC_Status::POSITIONING:
      alpha_e = 0;
      omega_e = 0;
      if (pos_count++ > 1000)
      {
        pos_count = 0;
        status = FOC_Status::OPERATION_FEEDFORWARD;
      }
      break;
    case FOC_Status::OPERATION_FEEDFORWARD:
      if (omega_e > OMEGA_TH_H || omega_e < -OMEGA_TH_H)
      {
        pid_theta_est.Clear();
        status = FOC_Status::OPERATION_SENSORLESS;
      }
      else
      {
        alpha_e = alpha_ref;
        omega_e += alpha_e * T_S;
      }
      break;
    case FOC_Status::OPERATION_SENSORED_HALL:
      // alpha is externally calculated
      // omega and theta will be externally overwrited
      if (omega_e > OMEGA_TH_H || omega_e < -OMEGA_TH_H)
      {
        pid_theta_est.Clear();
        status = FOC_Status::OPERATION_SENSORLESS;
      }
      break;
    case FOC_Status::OPERATION_SENSORED_ENC:
      // alpha is not used
      // omega is externally calculated
      // theta will be externally overwrited
      break;
    case FOC_Status::OPERATION_SENSORLESS:
      alpha_e = pid_theta_est(theta_err);
      omega_e += alpha_e * T_S;
      if (e_abs < E_TH_L)
      {
        status = FOC_Status::OPERATION_FEEDFORWARD;
      }
      break;
    default:
      status = FOC_Status::FAULT;
      break;
  }

  theta_e += omega_e * T_S;
  if (theta_e >= MyDSP::pi_f)
  {
    theta_e -= 2*MyDSP::pi_f;
  }
  else if (theta_e < -MyDSP::pi_f)
  {
    theta_e += 2*MyDSP::pi_f;
  }
}

