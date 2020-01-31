/*!
 * @file    vb_psm_controller.hpp
 * @brief   速度制限付き(Velocity-bounded)PSM制御器
 *
 * @note    Obj-Proxy間にはPD制御をおこなう。
 */

#pragma once

#include "classical_control/util/math.hpp"

namespace ClassicalControl
{

  /*!
   * @brief 速度制限付き(Velocity-bounded)PSM制御器クラス
   */
  class VBPSMController
  {
  public:
    /*!
     * @brief   パラメータ
     */
    struct Params {
      Params() = default;
      Params(sec h, double kp, double kd, vel max_v, double max_f)
        : h(h), kp(kp), kd(kd),
          max_v(max_v), max_f(max_f)
      {
        runtime_assert(h > 0 and kp >= 0 and kd >= 0
                     and max_v > 0 and max_f > 0,
                     "Invalid parameters given");
      }

      sec h = 0.1f;        //!< Proxy-Dest間の収束時定数
      double kp = 0;     //!< Obj-Proxy間のPゲイン
      double kd = 0;     //!< Obj-Proxy間のDゲイン
      vel max_v = 0;       //!< 最大速度
      double max_f = 0;  //!< 最大出力
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec   制御周期[sec]
     */
    explicit VBPSMController(Params params, sec cycle_sec)
      : m_params(std::move(params)), m_cycle_sec(cycle_sec) {}

    /*!
     * @brief   積分項を0クリアする
     */
    void clearIntegral() { m_a0 = m_a1 = 0; }

    /*!
     * @brief   出力計算
     * @param   dest    目標位置
     * @param   pos     現在位置
     * @param   dest_v  目標速度
     * @param   v       現在速度
     */
    double calc(double dest, double pos, double dest_v, double v)
    {
      const auto& t = m_cycle_sec;
      const auto& p = m_params;
      const auto a_error = m_a0 - m_a1;

      const double u_star
        = (dest - pos + p.h * dest_v + t * v - a_error / t)
        / (p.h + t);
      const double u = clamp(u_star, -p.max_v, p.max_v);

      const double f_star = p.kp * a_error / t + (p.kp * t + p.kd) * (u - v);
      double f = clamp(f_star, -p.max_f, p.max_f);

      m_a1 = m_a0;
      m_a0 += (p.kd * a_error + t * t * f) / (p.kp * t + p.kd);

      return f;
    }

  private:
    const Params m_params;
    const sec m_cycle_sec;

    double m_a0 = 0;
    double m_a1 = 0;
  };

}  // namespace ClassicalControl
