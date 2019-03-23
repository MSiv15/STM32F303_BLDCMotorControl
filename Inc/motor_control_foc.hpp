/*
 * motor_control_foc.hpp
 *
 *  Created on: 2018/07/19
 *      Author: Shibasaki
 */

#ifndef MOTOR_CONTROL_FOC_HPP_
#define MOTOR_CONTROL_FOC_HPP_

#define EIGEN_NO_DEBUG

#include "Eigen/Core"
#include "foc_utility.hpp"

enum class FOC_Status
{
  UNINITIALIZED,
  FAULT,
  UVLO,
  OFFSET_CALIBRATION_START,
  OFFSET_CALIBRATION_ONGOING,
  OFFSET_CALIBRATION_FINISH,
  STOP,
  POSITIONING,
  OPERATION_FEEDFORWARD,
  OPERATION_SENSORED_HALL,
  OPERATION_SENSORED_ENC,
  OPERATION_SENSORLESS,
  IDENTIFICATION
};

class BLDCMotor {
public:
  // PWM Switching Frequency
  static constexpr float F_PWM = 20e3f;
  static constexpr float T_S = 1.0f/F_PWM;

  // Power Unit (ADCの1LSB = 1PU)
  static constexpr float V_BASE = 3.3f / 4096 * (1 + 10.0f / 1.0f) * 0.9913f;
  static constexpr float I_BASE = 3.3f / 4096 / (0.2e-3f * 40);

  // Parameters for Feed Forward Control
  static constexpr float OMEGA_TH_H = 200.0f*MyDSP::pi_f;
  static constexpr float E_TH_L = 1.0f/V_BASE;

  // Electric Constants of the Main Circuit
  static constexpr float T_DEAD = 0.50e-6f;
  static constexpr float C_OSS = 3375e-12f * V_BASE/I_BASE;

  // Electric Constants of the Motor
  static constexpr float L_D = 1.05e-5f * I_BASE/V_BASE;
  static constexpr float L_Q = 1.96e-5f * I_BASE/V_BASE;
  static constexpr float R = 2.76e-2f * I_BASE/V_BASE;

  // Mechanical Constants of the Motor
  static constexpr float J_M = 6.26e-5f;
  static constexpr uint32_t NUM_POLE_PAIR = 7;
  static constexpr uint32_t NUM_POLE = 2*NUM_POLE_PAIR;
  /* 2018/8/11: http://academy.maxonjapan.co.jp/faq#m3 に従い補正 */
  static constexpr float K_V = 268.4f; // Catalog: 270KV
  static constexpr float K_TAU = 5.0f * MyDSP::sqrt_three_f * 1.0f/K_V * I_BASE;

  // Control Parameters
  static constexpr float T_PID = 5e-4f;
  static constexpr float T_DOB = 5e-5f;
  static constexpr float K_WIND = 1.0f-std::exp(-R*T_S/L_D);
  static constexpr float K_DAMP = 0.25f * 1.0f/R;
  static constexpr float T_EST = 5e-3f; // T_EST >= 50*T_DOB にすること

  // DC-Link Voltage
  float v_dc;

  // Electric angle, frequency, acceleration
  float theta_e; // Electric angle [rad]
  float omega_e; // Electric angular frequency [rad/s]
  float alpha_e; // Electric angular acceleration [rad/s^2]
  float sin_val, cos_val;

  // Control References
  union
  {
    float v_dq_ref[2];
    struct
    {
      float v_d_ref, v_q_ref;
    };
  };
  union
  {
    float i_dq_ref[2];
    struct
    {
      float i_d_ref, i_q_ref;
    };
  };
  union
  {
    float i_dq_ref_ff[2];
    struct
    {
      float i_d_ref_ff, i_q_ref_ff;
    };
  };
  float i_pos_ref;

  float alpha_ref;

  // Electric State Variables
  union
  {
    float i_abc[3];
    struct
    {
      float i_a, i_b, i_c;
    };
  };
  union
  {
    float i_alphabeta[2];
    struct
    {
      float i_alpha, i_beta;
    };
  };
  union
  {
    float i_dq[2];
    struct
    {
      float i_d, i_q;
    };
  };
  union
  {
    float v_dq[2];
    struct
    {
      float v_d, v_q;
    };
  };
  float v_abs;
  union
  {
    float v_alphabeta[2];
    struct
    {
      float v_alpha, v_beta;
    };
  };
  union
  {
    float duty_abc[3];
    struct
    {
      float duty_a, duty_b, duty_c;
    };
  };

  // Estimated State
  union
  {
    float obs_out_data[4];
    struct
    {
      union
      {
        float i_dq_hat[2];
        struct
        {
          float i_d_hat, i_q_hat;
        };
      };
      union
      {
        float d_dq_hat[2];
        struct
        {
          float d_d_hat, d_q_hat;
        };
      };
    };
  };
  union
  {
    float e_dq[2];
    struct
    {
      float e_d, e_q;
    };
  };
  float e_abs;

  // Wind-up Voltage
  union
  {
    float v_wind_dq[2];
    struct
    {
      float v_wind_d, v_wind_q;
    };
  };

  // Error of Rotor Position Estimation
  float theta_err;

  // Operation Status
  volatile FOC_Status status;

  // Sector of Space Vector
  FOCUtil::sector_t SV_sector;

  // Constructor
  BLDCMotor()
  :
    v_dc(15.0f/V_BASE),
    theta_e(0),
    omega_e(0),
    alpha_e(0),
    sin_val(std::sin(theta_e)),
    cos_val(std::cos(theta_e)),
    v_d_ref(0),
    v_q_ref(0),
    i_d_ref(0),
//    i_q_ref(100),
    i_q_ref(0),
    i_d_ref_ff(100),
    i_q_ref_ff(0),
    i_pos_ref(100),
    alpha_ref(0),
    i_abc{},
    i_alphabeta{},
    i_dq{},
    v_dq{},
    v_abs(),
    v_alphabeta{},
    duty_abc{},
    i_dq_hat{},
    d_dq_hat{},
    e_dq{},
    e_abs(),
    v_wind_dq{},
    theta_err(),
    status(FOC_Status::UVLO),
    SV_sector(0),
    hpf_e_dq(hpf_e_dq_coeff),
    pid_i_d(pid_i_dq_coeff),
    pid_i_q(pid_i_dq_coeff),
    pid_theta_est(pid_theta_est_coeff)
  {
    // 同一次元オブザーバ
    constexpr float p1 = 0;
    constexpr float p2 = std::exp(-T_S/T_DOB);

    Eigen::Matrix2f obs_A;
    obs_A <<
        std::exp(-R*T_S/L_D), -(1-std::exp(-R*T_S/L_D))/R,
        0, 1;

    Eigen::Vector2f obs_B
    (
        (1-std::exp(-R*T_S/L_D))/R,
        0
    );

    Eigen::RowVector2f obs_C
    (
        1, 0
    );

    Eigen::Vector2f obs_L
    (
        1+std::exp(-R*T_S/L_D)-p1-p2,
        -R*(1-p1+p1*p2-p2)/(1-std::exp(-R*T_S/L_D))
    );

    /* obs_mat = (A-LC B L) */
    obs_mat.block<OBS_NUM_STATE,OBS_NUM_STATE>(0,0) = obs_A - obs_L * obs_C;
    obs_mat.block<OBS_NUM_STATE,OBS_NUM_INPUT>(0,OBS_NUM_STATE) = obs_B;
    obs_mat.block<OBS_NUM_STATE,OBS_NUM_OUTPUT>(0,OBS_NUM_STATE+OBS_NUM_INPUT) = obs_L;
  }

  void Update(void);
  void CalculatePosition(void);

protected:
  // EMF HPF for Damping Control
  // MATLAB Command:
  //// F_HPF = 10.0;
  //// [b,a] = butter(2,2*F_HPF/F_PWM,'high')
  static constexpr float hpf_e_dq_coeff[1][5] = {{0.997781024102941f, -1.99556204820588f, 0.997781024102941f, 1.99555712434579f, -0.995566972065975f}};
  MyDSP::IIRBiquadCascadeDF2T<Eigen::Vector2f,float,1> hpf_e_dq;

  // PID Controller for Current Control
  static constexpr float K_PID = T_S/T_PID;
  //static constexpr float K_PID = 1.0f-std::exp(-T_S/T_PID);
  static constexpr float pid_i_dq_coeff[3] = {K_PID*R*(1.0f/(1.0f-std::exp(-R*T_S/L_D))-1.0f), K_PID*R, 0};
  MyDSP::PIDController<float> pid_i_d;
  MyDSP::PIDController<float> pid_i_q;

  // PID Controller for Rotor Position Estimation
  static constexpr float pid_theta_est_coeff[3] = {3.0f/(T_EST*T_EST), 1.0f/(F_PWM*T_EST*T_EST*T_EST), 3.0f*F_PWM/T_EST};
  MyDSP::PIDController<float> pid_theta_est;

  // 同一次元オブザーバ
  static constexpr int OBS_NUM_STATE   = 2;
  static constexpr int OBS_NUM_INPUT   = 1;
  static constexpr int OBS_NUM_OUTPUT  = 1;
  static constexpr int OBS_NUM_CHANNEL = 2;
  Eigen::Matrix<float, OBS_NUM_STATE, OBS_NUM_STATE+OBS_NUM_INPUT+OBS_NUM_OUTPUT> obs_mat;
};


#endif /* MOTOR_CONTROL_FOC_HPP_ */
