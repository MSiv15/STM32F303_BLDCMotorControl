/*
 * foc_utility.hpp
 *
 *  Created on: 2018/09/01
 *      Author: Shibasaki
 *
 * ベクトル制御向け汎用関数群
 */

#ifndef FOC_UTILITY_HPP_
#define FOC_UTILITY_HPP_

#include "stm32f3xx.h"
#include "MyDSP/Math.hpp"
#include "MyDSP/Controller.hpp"
#include "MyDSP/Filter.hpp"
#include <stdint.h>

namespace FOCUtil
{
  // 空間ベクトルセクタ情報を格納する型
  using sector_t = uint_fast8_t;

  // Clarke変換 (3シャント)
  static inline void Clarke_abc(
    const float Ia,
    const float Ib,
    const float Ic,
    float * pIalpha,
    float * pIbeta)
  {
    /* pIalpha = 2/3 * Ia - 1/3 * Ib - 1/3 * Ic */
    *pIalpha = 2.0f/3.0f * Ia - 1.0f/3.0f * Ib - 1.0f/3.0f * Ic;

    /* pIbeta = 1/sqrt(3) * Ib - 1/sqrt(3) * Ic */
    *pIbeta = MyDSP::by_sqrt_three_f * Ib - MyDSP::by_sqrt_three_f * Ic;
  }

  // Clarke変換 (ab相2シャント)
  static inline void Clarke_ab(
    const float Ia,
    const float Ib,
    float * pIalpha,
    float * pIbeta)
  {
    /* pIalpha = Ia */
    *pIalpha = Ia;

    /* pIbeta = 1/sqrt(3) * Ia + 2/sqrt(3) * Ib */
    *pIbeta = MyDSP::by_sqrt_three_f * Ia + 2 * MyDSP::by_sqrt_three_f * Ib;
  }

  // Clarke変換 (ac相2シャント)
  static inline void Clarke_ac(
    const float Ia,
    const float Ic,
    float * pIalpha,
    float * pIbeta)
  {
    /* pIalpha = Ia */
    *pIalpha = Ia;

    /* pIbeta = -1/sqrt(3) * Ia - 2/sqrt(3) * Ic */
    *pIbeta = -MyDSP::by_sqrt_three_f * Ia - 2 * MyDSP::by_sqrt_three_f * Ic;
  }

  // Clarke変換 (bc相2シャント)
  static inline void Clarke_bc(
    const float Ib,
    const float Ic,
    float * pIalpha,
    float * pIbeta)
  {
    /* pIalpha = -Ib - I_c */
    *pIalpha = -Ib - Ic;

    /* pIbeta = 1/sqrt(3) * Ib - 1/sqrt(3) * Ic */
    *pIbeta = MyDSP::by_sqrt_three_f * Ib - MyDSP::by_sqrt_three_f * Ic;
  }

  // Clarke変換 (2/3シャント動的切り替え)
  static inline void Clarke_Dynamic(
    const float Ia,
    const float Ib,
    const float Ic,
    const sector_t sector,
    float * pIalpha,
    float * pIbeta)
  {
    switch (sector)
    {
      case 0:
      case 5:
        Clarke_bc(Ib,Ic,pIalpha,pIbeta);
        break;
      case 1:
      case 2:
        Clarke_ac(Ia,Ic,pIalpha,pIbeta);
        break;
      case 3:
      case 4:
        Clarke_ab(Ia,Ib,pIalpha,pIbeta);
        break;
      default: // 不正なセクタの場合，三相全ての電流値を使用する
        Clarke_abc(Ia,Ib,Ic,pIalpha,pIbeta);
        break;
    }
  }

  // 逆Clarke変換
  static inline void InvClarke_abc(
    const float Ialpha,
    const float Ibeta,
    float * pIa,
    float * pIb,
    float * pIc)
  {
    /* pIa = Ialpha */
    *pIa = Ialpha;

    /* pIb = -1/2 * Ialpha + sqrt(3)/2 * Ibeta */
    *pIb = -0.5f * Ialpha + 0.5f * MyDSP::sqrt_three_f * Ibeta;

    /* pIc = -1/2 * Ialpha - sqrt(3)/2 * Ibeta */
    *pIc = -0.5f * Ialpha - 0.5f * MyDSP::sqrt_three_f * Ibeta;
  }

  // Park変換
  static inline void Park(
    const float Ialpha,
    const float Ibeta,
    float * pId,
    float * pIq,
    const float sin_val,
    const float cos_val)
  {
    /* pId = Ialpha * cos_val + Ibeta * sin_val */
    *pId = Ialpha * cos_val + Ibeta * sin_val;

    /* pIq = - Ialpha * sin_val + Ibeta * cos_val */
    *pIq = -Ialpha * sin_val + Ibeta * cos_val;
  }

  // 逆Park変換
  static inline void InvPark(
    const float Id,
    const float Iq,
    float * pIalpha,
    float * pIbeta,
    const float sin_val,
    const float cos_val)
  {
    /* pIalpha = Id * cos_val - Iq * sin_val */
    *pIalpha = Id * cos_val - Iq * sin_val;

    /* pIbeta = Id * sin_val + Iq * cos_val */
    *pIbeta = Id * sin_val + Iq * cos_val;
  }

  // 空間ベクトルPWM (3相変調: 零ベクトル均等配分)
  // セクタは実行時に判定
  static inline sector_t SVPWM_3phase(
    const float Valpha,
    const float Vbeta,
    const float Vdc,
    float * p_duty_a,
    float * p_duty_b,
    float * p_duty_c)
  {
    sector_t sector;
    float Va,  Vb,  Vc;
    float Vab, Vbc, Vca;

    Vab = 1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta;
    Vbc = MyDSP::sqrt_three_f * Vbeta;
    Vca = -1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta;

    if (std::signbit(Vab) == std::signbit(Vbc))
    {
      Va = -0.5f * Vca;
      Vb =  0.5f * Vca + Vbc;
      Vc =  0.5f * Vca;
      if (Vca <= 0) // Va >= Vb >= Vc
      {
        sector = 0;
      }
      else // Vc >= Vb >= Va
      {
        sector = 3;
      }
    }
    else if (std::signbit(Vca) == std::signbit(Vab))
    {
      Va =  1.5f * Valpha;
      Vb =  0.5f * Vbc;
      Vc = -0.5f * Vbc;
      if (Vbc >= 0) // Vb > Va >= Vc
      {
        sector = 1;
      }
      else // Vc >= Va > Vb
      {
        sector = 4;
      }
    }
    else
    {
      Va =  0.5f * Vab;
      Vb = -0.5f * Vab;
      Vc =  0.5f * Vab + Vca;
      if (Vab <= 0) // Vb > Vc > Va
      {
        sector = 2;
      }
      else // Va > Vc > Vb
      {
        sector = 5;
      }
    }

    if (MyDSP::unlikely(!(Vdc > 0))) // ゼロ除算回避
    {
      *p_duty_a = 0.5f;
      *p_duty_b = 0.5f;
      *p_duty_c = 0.5f;
    }
    else
    {
      *p_duty_a = Va / Vdc + 0.5f;
      *p_duty_b = Vb / Vdc + 0.5f;
      *p_duty_c = Vc / Vdc + 0.5f;
    }
    return sector;
  }

  // 空間ベクトルPWM (3相変調: 零ベクトル均等配分)
  // セクタは事前に判定されたものを利用
  static inline void SVPWM_3phase(
    const float Valpha,
    const float Vbeta,
    const float Vdc,
    const sector_t sector,
    float * p_duty_a,
    float * p_duty_b,
    float * p_duty_c)
  {
    if (MyDSP::unlikely(!(Vdc > 0))) // ゼロ除算回避
    {
      *p_duty_a = 0.5f;
      *p_duty_b = 0.5f;
      *p_duty_c = 0.5f;
      return;
    }

    switch (sector)
    {
      case 0:
      case 3:
        *p_duty_a = ( 0.75f * Valpha + 0.25f * MyDSP::sqrt_three_f * Vbeta) / Vdc + 0.5f;
        *p_duty_b = (-0.75f * Valpha + 0.75f * MyDSP::sqrt_three_f * Vbeta) / Vdc + 0.5f;
        *p_duty_c = (-0.75f * Valpha - 0.25f * MyDSP::sqrt_three_f * Vbeta) / Vdc + 0.5f;
        break;
      case 1:
      case 4:
        *p_duty_a =  1.5f * Valpha / Vdc + 0.5f;
        *p_duty_b =  0.5f * MyDSP::sqrt_three_f * Vbeta / Vdc + 0.5f;
        *p_duty_c = -0.5f * MyDSP::sqrt_three_f * Vbeta / Vdc + 0.5f;
        break;
      case 2:
      case 5:
        *p_duty_a = ( 0.75f * Valpha - 0.25f * MyDSP::sqrt_three_f * Vbeta) / Vdc + 0.5f;
        *p_duty_b = (-0.75f * Valpha + 0.25f * MyDSP::sqrt_three_f * Vbeta) / Vdc + 0.5f;
        *p_duty_c = (-0.75f * Valpha - 0.75f * MyDSP::sqrt_three_f * Vbeta) / Vdc + 0.5f;
        break;
      default: // 不正なセクタの場合，出力を0Vにする
        *p_duty_a = 0.5f;
        *p_duty_b = 0.5f;
        *p_duty_c = 0.5f;
        break;
    }
  }

  // 空間ベクトルPWM (2相変調: 零ベクトル集中配分)
  // セクタは実行時に判定
  static inline sector_t SVPWM_2phase(
    const float Valpha,
    const float Vbeta,
    const float Vdc,
    float * p_duty_a,
    float * p_duty_b,
    float * p_duty_c)
  {
    sector_t sector;
    float Va,  Vb,  Vc;
    float Vab, Vbc, Vca;

    Vab = 1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta;
    Vbc = MyDSP::sqrt_three_f * Vbeta;
    Vca = -1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta;

    if (Vbc >= 0 && Vca <= 0) {
      Va = -Vca;
      Vb =  Vbc;
      Vc =  0;
      if (Vab >= 0) // Va >= Vb >= Vc
      {
        sector = 0;
      }
      else // Vb > Va >= Vc
      {
        sector = 1;
      }
    }
    else if(Vca >= 0 && Vab <= 0) {
      Va =  0;
      Vb = -Vab;
      Vc =  Vca;
      if (Vbc >= 0) // Vb >= Vc > Va
      {
        sector = 2;
      }
      else // Vc > Vb > Va
      {
        sector = 3;
      }
    }
    else {
      Va =  Vab;
      Vb =  0;
      Vc = -Vbc;
      if (Vca >= 0) // Vc >= Va > Vb
      {
        sector = 4;
      }
      else // Va > Vc > Vb
      {
        sector = 5;
      }
    }

    if (MyDSP::unlikely(!(Vdc > 0))) // ゼロ除算回避
    {
      *p_duty_a = 0;
      *p_duty_b = 0;
      *p_duty_c = 0;
    }
    else
    {
      *p_duty_a = Va / Vdc;
      *p_duty_b = Vb / Vdc;
      *p_duty_c = Vc / Vdc;
    }
    return sector;
  }

  // 空間ベクトルPWM (2相変調: 零ベクトル集中配分)
  // セクタは事前に判定されたものを利用
  static inline void SVPWM_2phase(
    const float Valpha,
    const float Vbeta,
    const float Vdc,
    const sector_t sector,
    float * p_duty_a,
    float * p_duty_b,
    float * p_duty_c)
  {
    if (MyDSP::unlikely(!(Vdc > 0))) // ゼロ除算回避
    {
      *p_duty_a = 0;
      *p_duty_b = 0;
      *p_duty_c = 0;
      return;
    }

    switch (sector)
    {
      case 0:
      case 1:
        *p_duty_a = (1.5f * Valpha + 0.5f * MyDSP::sqrt_three_f * Vbeta) / Vdc;
        *p_duty_b = MyDSP::sqrt_three_f * Vbeta / Vdc;
        *p_duty_c = 0;
        break;
      case 2:
      case 3:
        *p_duty_a = 0;
        *p_duty_b = (-1.5f * Valpha + 0.5f * MyDSP::sqrt_three_f * Vbeta) / Vdc;
        *p_duty_c = (-1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta) / Vdc;
        break;
      case 4:
      case 5:
        *p_duty_a = (1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta) / Vdc;
        *p_duty_b = 0;
        *p_duty_c = -MyDSP::sqrt_three_f * Vbeta / Vdc;
        break;
      default: // 不正なセクタの場合，出力を0Vにする
        *p_duty_a = 0;
        *p_duty_b = 0;
        *p_duty_c = 0;
        break;
    }
  }

  // デッドタイム補償 (2レベルACM)
  static inline float DeadTimeCompensate_2Level(
    const float duty_in,
    const float Iphase,
    const float dead_time_ratio)
  {
    return (duty_in <= 0) ? 0 :
           (duty_in >= 1) ? 1 :
                            duty_in + dead_time_ratio * MyDSP::Sign(Iphase) ;
  }

  // デッドタイム補償 (3レベルACM)
  static inline float DeadTimeCompensate_3Level(
    const float duty_in,
    const float Iphase,
    const float Ith,
    const float dead_time_ratio)
  {
    return (duty_in <= 0)  ? 0 :
           (duty_in >= 1)  ? 1 :
           (Iphase < -Ith) ? duty_in - dead_time_ratio :
           (Iphase >  Ith) ? duty_in + dead_time_ratio :
                             duty_in ;
  }

  // デッドタイム補償 (リニアACM)
  static inline float DeadTimeCompensate_Linear(
    const float duty_in,
    const float Iphase,
    const float Ith,
    const float dead_time_ratio)
  {
    return (duty_in <= 0)  ? 0 :
           (duty_in >= 1)  ? 1 :
           (Iphase < -Ith) ? duty_in - dead_time_ratio :
           (Iphase >  Ith) ? duty_in + dead_time_ratio :
                             duty_in + dead_time_ratio * Iphase/Ith ;
  }

  // デッドタイム補償 (TTCM)
  // https://doi.org/10.1541/ieejias.134.412
  // https://doi.org/10.1109/TPEL.2014.2352716
  // Ip = Iphase + Iripple
  // In = Iphase - Iripple
  // Ic = 2 * Coss * V_DC / T_DEAD
  static inline float DeadTimeCompensate_TTCM(
    const float duty_in,
    const float Ip,
    const float In,
    const float Ic,
    const float dead_time_ratio)
  {
    if      (duty_in <= 0) return 0 ;
    else if (duty_in >= 1) return 1 ;
    else
    {
      float DT_error_p, DT_error_n;

      DT_error_p = (Ip < 0)   ?  dead_time_ratio :
                   (Ip <= Ic) ?  dead_time_ratio * (1 - 0.5f*Ip/Ic) :
                                 dead_time_ratio * 0.5f*Ic/Ip ;

      DT_error_n = (In < -Ic) ?  dead_time_ratio * 0.5f*Ic/In :
                   (In <= 0)  ? -dead_time_ratio * (1 + 0.5f*In/Ic) :
                                -dead_time_ratio ;

      return duty_in - DT_error_p - DT_error_n;
    }
  }

  // 電流リプル推定 (三相 Center-aligned PWM)
  // セクタは実行時に判定
  // https://doi.org/10.1109/TPEL.2014.2352716
  static inline sector_t CurrentRippleEstimate(
    const float Da,
    const float Db,
    const float Dc,
    const float Vdc,
    const float F_PWM,
    const float L,
    float * pIra,
    float * pIrb,
    float * pIrc)
  {
    sector_t sector;
    float D1, D2, D3;
    float *pIr1, *pIr2, *pIr3;

    if (Db >= Dc && Da >= Dc)
    {
      D3 = Dc;
      pIr3 = pIrc;
      if (Da >= Db)
      {
        sector = 0;
        D1 = Da;
        D2 = Db;
        pIr1 = pIra;
        pIr2 = pIrb;
      }
      else
      {
        sector = 1;
        D1 = Db;
        D2 = Da;
        pIr1 = pIrb;
        pIr2 = pIra;
      }
    }
    else if (Dc >= Da && Db >= Da)
    {
      D3 = Da;
      pIr3 = pIra;
      if (Db >= Dc)
      {
        sector = 2;
        D1 = Db;
        D2 = Dc;
        pIr1 = pIrb;
        pIr2 = pIrc;
      }
      else
      {
        sector = 3;
        D1 = Dc;
        D2 = Db;
        pIr1 = pIrc;
        pIr2 = pIrb;
      }
    }
    else
    {
      D3 = Db;
      pIr3 = pIrb;
      if (Dc >= Da)
      {
        sector = 4;
        D1 = Dc;
        D2 = Da;
        pIr1 = pIrc;
        pIr2 = pIra;
      }
      else
      {
        sector = 5;
        D1 = Da;
        D2 = Dc;
        pIr1 = pIra;
        pIr2 = pIrc;
      }
    }

    *pIr1 = (2*D1-D2-D3-(2*D1-1)*(2*D1-D2-D3))*Vdc/(12*F_PWM*L);
    *pIr2 = (D1-D3-(2*D2-1)*(-D1+2*D2-D3))*Vdc/(12*F_PWM*L);
    *pIr3 = (D1+D2-2*D3-(2*D3-1)*(-D1-D2+2*D3))*Vdc/(12*F_PWM*L);

    return sector;
  }

  // 電流リプル推定 (三相 Center-aligned PWM)
  // セクタは事前に判定されたものを利用
  // https://doi.org/10.1109/TPEL.2014.2352716
  static inline void CurrentRippleEstimate(
    const float Da,
    const float Db,
    const float Dc,
    const float Vdc,
    const float F_PWM,
    const float L,
    const sector_t sector,
    float * pIra,
    float * pIrb,
    float * pIrc)
  {
    float D1, D2, D3;
    float *pIr1, *pIr2, *pIr3;

    switch (sector)
    {
      case 0:
        D1 = Da;
        D2 = Db;
        D3 = Dc;
        pIr1 = pIra;
        pIr2 = pIrb;
        pIr3 = pIrc;
        break;
      case 1:
        D1 = Db;
        D2 = Da;
        D3 = Dc;
        pIr1 = pIrb;
        pIr2 = pIra;
        pIr3 = pIrc;
        break;
      case 2:
        D1 = Db;
        D2 = Dc;
        D3 = Da;
        pIr1 = pIrb;
        pIr2 = pIrc;
        pIr3 = pIra;
        break;
      case 3:
        D1 = Dc;
        D2 = Db;
        D3 = Da;
        pIr1 = pIrc;
        pIr2 = pIrb;
        pIr3 = pIra;
        break;
      case 4:
        D1 = Dc;
        D2 = Da;
        D3 = Db;
        pIr1 = pIrc;
        pIr2 = pIra;
        pIr3 = pIrb;
        break;
      case 5:
        D1 = Da;
        D2 = Dc;
        D3 = Db;
        pIr1 = pIra;
        pIr2 = pIrc;
        pIr3 = pIrb;
        break;
      default: // 不正なセクタの場合，出力を0Aにする
        *pIra = 0;
        *pIrb = 0;
        *pIrc = 0;
        return;
    }

    *pIr1 = (2*D1-D2-D3-(2*D1-1)*(2*D1-D2-D3))*Vdc/(12*F_PWM*L);
    *pIr2 = (D1-D3-(2*D2-1)*(-D1+2*D2-D3))*Vdc/(12*F_PWM*L);
    *pIr3 = (D1+D2-2*D3-(2*D3-1)*(-D1-D2+2*D3))*Vdc/(12*F_PWM*L);
  }

  // 空間ベクトルセクタ判定
  static inline sector_t DecideSVSector(
    const float Valpha,
    const float Vbeta)
  {
    float Vab, Vbc, Vca;
    Vab = 1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta;
    Vbc = MyDSP::sqrt_three_f * Vbeta;
    Vca = -1.5f * Valpha - 0.5f * MyDSP::sqrt_three_f * Vbeta;

    if (Vbc >= 0) // 上半分
    {
      if (Vab >= 0) // Va >= Vb >= Vc
      {
        return 0;
      }
      else if (Vca < 0) // Vb > Va > Vc
      {
        return 1;
      }
      else // Vb > Vc >= Va
      {
        return 2;
      }
    }
    else // 下半分
    {
      if (Vab <= 0) // Vc > Vb >= Va
      {
        return 3;
      }
      else if (Vca > 0) // Vc > Va > Vb
      {
        return 4;
      }
      else // Va >= Vc > Vb
      {
        return 5;
      }
    }
  }
}

#endif /* FOC_UTILITY_HPP_ */
