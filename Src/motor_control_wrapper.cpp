/*
 * motor_control_wrapper.cpp
 *
 *  Created on: 2018/07/19
 *      Author: Shibasaki
 *
 * C++関数のC言語に対するインターフェースおよびハードウェア依存処理の抽象化
 */

#include "motor_control_foc.hpp"
#include "motor_control_wrapper.h"
#include "main.h"

#define DEBUG_ON

uint16_t adc_data1 = 0, adc_data2 = 0, adc_data3 = 0, adc_data4 = 0;

uint32_t period;

float omega_ref;

BLDCMotor M1;

constexpr float T_SC = 5e-3f;
constexpr float K_SC = BLDCMotor::J_M/(BLDCMotor::NUM_POLE_PAIR*BLDCMotor::K_TAU*T_SC);

constexpr float V_WAKE = 7.0f/BLDCMotor::V_BASE;
constexpr float V_UVLO = 5.0f/BLDCMotor::V_BASE;

// Voltage LPF for V_DC Detection
constexpr float T_VDC = 1e-2f;
constexpr float lpf_v_coeff[1][5]={{0, 1.0f-std::exp(-1.0f/(T_VDC*BLDCMotor::F_PWM)), 0, std::exp(-1.0f/(T_VDC*BLDCMotor::F_PWM)), 0}};
MyDSP::IIRBiquadCascadeDF2T<float,float,1> lpf_v_dc(lpf_v_coeff);

uint32_t t_calc;

volatile bool is_warning_reported = false;

volatile bool is_table_full = false;

//float i_a_arr[N_TABLE];
//float i_b_arr[N_TABLE];
//float i_c_arr[N_TABLE];
//float i_alpha_arr[N_TABLE];
//float i_beta_arr[N_TABLE];
float i_d_arr[N_TABLE] = {0};
float i_q_arr[N_TABLE] = {0};
float v_d_arr[N_TABLE] = {0};
float v_q_arr[N_TABLE] = {0};
float e_d_arr[N_TABLE] = {0};
float e_q_arr[N_TABLE] = {0};
//float e_abs_arr[N_TABLE] = {0};
//float v_alpha_arr[N_TABLE] = {0};
//float v_beta_arr[N_TABLE] = {0};
//float duty_a_arr[N_TABLE] = {0};
//float duty_b_arr[N_TABLE] = {0};
//float duty_c_arr[N_TABLE] = {0};
//float v_wind_d_arr[N_TABLE] = {0};
//float v_wind_q_arr[N_TABLE] = {0};
float omega_arr[N_TABLE] = {0};
float theta_err_arr[N_TABLE] = {0};
uint32_t t_calc_arr[N_TABLE] = {0};


// DRV8305 Configuration
//
// MOSFET: FDBL86561_F085
// Q_g=170nC, Q_gd=24nC, R_ds(ON)=0.85mΩ
// R_sense=0.2mΩ
// f_sw=20kHz
// Gate Drive RMS Current = Q_g * 6 * f_sw = 20.4mA
// MOSFET Slew Rate = Q_gd / IDRIVEP = 48ns
// Current Sensing Range = 1.65 / (R_sense * GAIN_CS) = 206.25A
// Overcurrent Trip = VDS_LVL / R_ds(ON) = 206A
namespace DRV8305Command
{
  constexpr uint16_t hs_gdc_w = 0x5<<11 | 0x2<<8 | 0xB<<4 | 0x9; // TDRIVEN=880ns, IDRIVEN_HS=1.25A, IDRIVEP_HS=0.50A
  constexpr uint16_t ls_gdc_w = 0x6<<11 | 0x2<<8 | 0xB<<4 | 0x9; // TDRIVEP=880ns, IDRIVEN_LS=1.25A, IDRIVEP_LS=0.50A
  constexpr uint16_t gdc_w    = 0x7<<11 | 0x1<<9 | 0x0<<7 | 0x0<<4 | 0x3<<2 | 0x3; // DEAD_TIME=35ns, TBLANK=7us, TVDS=7us
  constexpr uint16_t ic_op_w  = 0x9<<11 | 0x1<<7 | 0x020; // Enable Sense Amplifier Clamp
  constexpr uint16_t sac_w    = 0xA<<11 | 0x0<<8 | 0x0<<6 | 0x2<<4 | 0x2<<2 | 0x2; // CS_BLANK=0ns, GAIN_CS=40
  constexpr uint16_t vds_sc_w = 0xC<<11 | 0x09<<3 | 0x0; // VDS_LVL=0.175V
  constexpr uint16_t warn_r   = 0x1<<11 | 0x1<<15;
  constexpr uint16_t ov_flt_r = 0x2<<11 | 0x1<<15;
  constexpr uint16_t ic_flt_r = 0x3<<11 | 0x1<<15;
  constexpr uint16_t vg_flt_r = 0x4<<11 | 0x1<<15;
  constexpr uint16_t hs_gdc_r = 0x5<<11 | 0x1<<15;
  constexpr uint16_t ls_gdc_r = 0x6<<11 | 0x1<<15;
  constexpr uint16_t gdc_r    = 0x7<<11 | 0x1<<15;
  constexpr uint16_t ic_op_r  = 0x9<<11 | 0x1<<15;
  constexpr uint16_t sac_r    = 0xA<<11 | 0x1<<15;
  constexpr uint16_t vds_sc_r = 0xC<<11 | 0x1<<15;
}

void GateDriverInit(void)
{
  uint16_t spi_rx_buf;

  // Write HS Gate Drive Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::hs_gdc_w);
  delay_us(1);
  // Read HS Gate Drive Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::hs_gdc_r);
  xprintf("hs_gdc=%04X\n", spi_rx_buf);

  // Write LS Gate Drive Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::ls_gdc_w);
  delay_us(1);
  // Read LS Gate Drive Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::ls_gdc_r);
  xprintf("ls_gdc=%04X\n", spi_rx_buf);

  // Write Gate Drive Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::gdc_w);
  delay_us(1);
  // Read Gate Drive Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::gdc_r);
  xprintf("gdc=   %04X\n", spi_rx_buf);

  // Write IC Operation Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::ic_op_w);
  delay_us(1);
  // Read IC Operation Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::ic_op_r);
  xprintf("ic_op= %04X\n", spi_rx_buf);

  // Write Shunt Amplifier Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::sac_w);
  delay_us(1);
  // Read Shunt Amplifier Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::sac_r);
  xprintf("sac=   %04X\n", spi_rx_buf);

  // Write VDS Sense Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::vds_sc_w);
  delay_us(1);
  // Read VDS Sense Control Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::vds_sc_r);
  xprintf("vds_sc=%04X\n", spi_rx_buf);

  // Read Warning and Watchdog Reset Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::warn_r);
  xprintf("warn=  %04X\n", spi_rx_buf);

  // Enable Gate Drive
  LL_GPIO_SetOutputPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
  delay_ms(1);

  // Read Warning and Watchdog Reset Register of DRV8305
  spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::warn_r);
  xprintf("warn=  %04X\n", spi_rx_buf);
  if ((spi_rx_buf & 0x1<<10) == 0x0)
  {
    is_warning_reported = false;
  }
}

void FOC_EmergencyStop(void)
{
  LL_TIM_DisableAllOutputs(TIM1);
  LL_GPIO_ResetOutputPin(USER_LED_GPIO_Port,USER_LED_Pin);
  M1.status = FOC_Status::FAULT;
}

// ポーリングで回す処理
void FOC_LowFreqTask(void)
{
  while (M1.status == FOC_Status::UVLO)
  {
    LL_TIM_DisableAllOutputs(TIM1);
    LL_GPIO_ResetOutputPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
  }
  if (M1.status == FOC_Status::UNINITIALIZED)
  {
    GateDriverInit();

    // Start ADC Offset Calibration
    M1.status = FOC_Status::OFFSET_CALIBRATION_START;
    // Wait for End of Calibration
    while (M1.status == FOC_Status::OFFSET_CALIBRATION_START || M1.status == FOC_Status::OFFSET_CALIBRATION_ONGOING);
    if (M1.status != FOC_Status::OFFSET_CALIBRATION_FINISH)
    {
      return;
    }

    // Enable PWM Output
    LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(period * 0.5f));
    LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)(period * 0.5f));
    LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)(period * 0.5f));
    LL_TIM_EnableAllOutputs(TIM1);

    M1.status = FOC_Status::POSITIONING;
//    M1.status = FOC_Status::IDENTIFICATION;
  }
  if (is_warning_reported == true)
  {
    uint16_t spi_rx_buf;

    spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::warn_r);
    if ((spi_rx_buf & 0x1<<10) == 0x0)
    {
      is_warning_reported = false;
      LL_GPIO_ResetOutputPin(USER_LED_GPIO_Port,USER_LED_Pin);
    }
    else
    {
      M1.status = FOC_Status::FAULT;
      LL_TIM_DisableAllOutputs(TIM1);
      xprintf("DRV8305 Fault!\n");
      xprintf("0x1: %04X\n", spi_rx_buf);
      spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::ov_flt_r);
      xprintf("0x2: %04X\n", spi_rx_buf);
      spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::ic_flt_r);
      xprintf("0x3: %04X\n", spi_rx_buf);
      spi_rx_buf = SPI1_TransmitReceive16(DRV8305Command::vg_flt_r);
      xprintf("0x4: %04X\n", spi_rx_buf);
      LL_GPIO_ResetOutputPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
      LL_GPIO_SetOutputPin(USER_LED_GPIO_Port,USER_LED_Pin);

      while (LL_GPIO_IsInputPinSet(USER_SW_GPIO_Port,USER_SW_Pin));
      delay_ms(10);
      while (!LL_GPIO_IsInputPinSet(USER_SW_GPIO_Port,USER_SW_Pin));
      delay_ms(10);
      NVIC_SystemReset();
    }
  }
#ifndef DEBUG_ON
  delay_ms(500);
  xprintf("i_d=%4d, i_q=%4d, v_d=%3d, v_q=%3d, e_d=%3d, e_q=%3d, freq=%3d, theta_err=%3d, mode=%d\r\n",
    (int)M1.i_d, (int)M1.i_q, (int)(M1.v_d*100), (int)(M1.v_q*100), (int)(M1.e_d*100), (int)(M1.e_q*100),
    (int)(M1.omega_e*MyDSP::by_two_pi_f), (int)(M1.theta_err*MyDSP::one_eighty_by_pi_f), M1.status);
//  xprintf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//    adc_data1, adc_data2, adc_data3, (int)(M1.duty_a*1000), (int)(M1.duty_b*1000), (int)(M1.duty_c*1000), t_calc);

  if (LL_USART_IsActiveFlag_RXNE(USART1))
  {
    char uart_rx_buf = uart1_getc();
    int sign_omega = (omega_ref >= 0) ? 1 : -1;
    switch (uart_rx_buf)
    {
      case '0':
        omega_ref = 0;
        break;
      case '1':
        omega_ref = sign_omega * 100.0f * MyDSP::two_pi_f;
        break;
      case '2':
        omega_ref = sign_omega * 200.0f * MyDSP::two_pi_f;
        break;
      case '3':
        omega_ref = sign_omega * 300.0f * MyDSP::two_pi_f;
        break;
      case '4':
        omega_ref = sign_omega * 400.0f * MyDSP::two_pi_f;
        break;
      case '5':
        omega_ref = sign_omega * 500.0f * MyDSP::two_pi_f;
        break;
      case '6':
        omega_ref = sign_omega * 600.0f * MyDSP::two_pi_f;
        break;
      case '7':
        omega_ref = sign_omega * 700.0f * MyDSP::two_pi_f;
        break;
      case '8':
        omega_ref = sign_omega * 800.0f * MyDSP::two_pi_f;
        break;
      case '-':
        omega_ref = -omega_ref;
        break;
      default:
        break;
    }
  }
#else
  if (is_table_full == true)
  {
    M1.status = FOC_Status::FAULT;
    LL_GPIO_ResetOutputPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
    LL_TIM_DisableAllOutputs(TIM1);
    for (int i = 0; i < N_TABLE; i++)
    {
//      xprintf("%d\t%d\t%d\t%d\t%d\t%d\n",
//              (int)(M1.v_dc*10), (int)i_d_arr[i], (int)i_q_arr[i], (int)(v_d_arr[i]*10), (int)(v_q_arr[i]*10), t_calc_arr[i]);
      xprintf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
              (int)std::round(i_d_arr[i]), (int)std::round(i_q_arr[i]), (int)(std::round(v_d_arr[i]*10)), (int)(std::round(v_q_arr[i]*10)),
              (int)(e_d_arr[i]*10), (int)(e_q_arr[i]*10), (int)(theta_err_arr[i]*MyDSP::one_eighty_by_pi_f), (int)(omega_arr[i]*MyDSP::by_two_pi_f));
//      xprintf("%d\t%d\t%d\n",
//              (int)(duty_a_arr[i]*100), (int)(duty_b_arr[i]*100), (int)(duty_c_arr[i]*100));
    }
    is_table_full = false;
  }
#endif
}

// ADC割り込みで回す処理
void FOC_HighFreqTask(void)
{
  static uint_fast16_t offset_cal_count = 0;
  static float ad_offset1 = 0;
  static float ad_offset2 = 0;
  static float ad_offset3 = 0;
#ifdef DEBUG_ON
  static uint_fast16_t table_count = 0;
  static uint_fast16_t sub_count = 0;
#endif

  adc_data1 = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
  adc_data2 = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
  adc_data3 = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3);
  adc_data4 = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_4);
  M1.v_dc = lpf_v_dc(adc_data4);

  if (M1.status == FOC_Status::UVLO && M1.v_dc > V_WAKE)
  {
    M1.status = FOC_Status::UNINITIALIZED;
  }
  else if (M1.status != FOC_Status::FAULT && M1.v_dc < V_UVLO)
  {
    M1.status = FOC_Status::UVLO;
  }

  switch (M1.status)
  {
    case FOC_Status::FAULT:
    case FOC_Status::UVLO:
    case FOC_Status::UNINITIALIZED:
      LL_TIM_DisableAllOutputs(TIM1);
      break;
    case FOC_Status::OFFSET_CALIBRATION_START:
      LL_TIM_DisableAllOutputs(TIM1);
      offset_cal_count = 0;
      M1.v_d_ref = 0.625f/M1.V_BASE;
//      omega_ref = 500.0f*MyDSP::two_pi_f;
      M1.status = FOC_Status::OFFSET_CALIBRATION_ONGOING;
      /* no break */
    case FOC_Status::OFFSET_CALIBRATION_ONGOING:
      LL_TIM_DisableAllOutputs(TIM1);
      offset_cal_count++;

      ad_offset1 = (ad_offset1*(offset_cal_count-1) + adc_data1)/offset_cal_count;
      ad_offset2 = (ad_offset2*(offset_cal_count-1) + adc_data2)/offset_cal_count;
      ad_offset3 = (ad_offset3*(offset_cal_count-1) + adc_data3)/offset_cal_count;

      if (offset_cal_count >= 1000)
      {
        offset_cal_count = 0;
        M1.status = FOC_Status::OFFSET_CALIBRATION_FINISH;
      }
      break;
    case FOC_Status::OFFSET_CALIBRATION_FINISH:
      LL_TIM_DisableAllOutputs(TIM1);
      break;
    case FOC_Status::STOP:
    case FOC_Status::POSITIONING:
    case FOC_Status::OPERATION_FEEDFORWARD:
    case FOC_Status::OPERATION_SENSORED_HALL:
    case FOC_Status::OPERATION_SENSORED_ENC:
    case FOC_Status::OPERATION_SENSORLESS:
    case FOC_Status::IDENTIFICATION:
      M1.i_a = adc_data1 - ad_offset1;
      M1.i_b = adc_data2 - ad_offset2;
      M1.i_c = adc_data3 - ad_offset3;

      M1.Update();

      LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(period * M1.duty_a + 0.5f));
      LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)(period * M1.duty_b + 0.5f));
      LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)(period * M1.duty_c + 0.5f));

      // Speed Control
      M1.i_q_ref = K_SC * (omega_ref-M1.omega_e);
      if (M1.i_q_ref > 400) M1.i_q_ref = 400;
      else if (M1.i_q_ref < -400) M1.i_q_ref = -400;
      M1.i_q_ref_ff = M1.i_q_ref*1.2f;
      M1.alpha_ref = M1.i_q_ref * M1.K_TAU/M1.J_M * M1.NUM_POLE_PAIR*0.5f;

#ifdef DEBUG_ON
      if (table_count < N_TABLE && sub_count == 0 && is_table_full == false)
      {
        i_d_arr[table_count] = M1.i_d;
        i_q_arr[table_count] = M1.i_q;
//        i_alpha_arr[table_count] = M1.i_alpha;
//        i_beta_arr[table_count] = M1.i_beta;
//        i_a_arr[table_count] = M1.i_a;
//        i_b_arr[table_count] = M1.i_b;
//        i_c_arr[table_count] = M1.i_c;
        v_d_arr[table_count] = M1.v_d;
        v_q_arr[table_count] = M1.v_q;
        e_d_arr[table_count] = M1.e_d;
        e_q_arr[table_count] = M1.e_q;
//        e_abs_arr[table_count] = M1.e_abs;
//        v_alpha_arr[table_count] = M1.v_alpha;
//        v_beta_arr[table_count] = M1.v_beta;
//        duty_a_arr[table_count] = M1.duty_a;
//        duty_b_arr[table_count] = M1.duty_b;
//        duty_c_arr[table_count] = M1.duty_c;
//        v_wind_d_arr[table_count] = M1.v_wind_d;
//        v_wind_q_arr[table_count] = M1.v_wind_q;
        omega_arr[table_count] = M1.omega_e;
        theta_err_arr[table_count] = M1.theta_err;
        t_calc_arr[table_count] = t_calc;
        table_count++;
//        sub_count = 19;
        sub_count = 0;
        if (table_count == N_TABLE/2)
        {
          M1.v_d_ref += M1.v_d_ref;
//          M1.v_q_ref = 0.2f*M1.v_d_ref;
//          omega_ref = -omega_ref;
        }
        if (table_count >= N_TABLE)
        {
          is_table_full = true;
          M1.status = FOC_Status::FAULT;
        }
      }
      else if (table_count < N_TABLE)
      {
        sub_count--;
      }
#endif
      break;
    default:
      LL_TIM_DisableAllOutputs(TIM1);
      break;
  }
}

