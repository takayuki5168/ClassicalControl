/*!
 * @file    psm_controller.hpp
 * @brief   PSM制御器
 *
 * @note    Obj-Proxy間にはPID制御をおこなう。
 */

#pragma once

#include "classical_control/util/math.hpp"

namespace ClassicalControl
{

/*!
 * @brief   PSM制御器クラス
 */
class PSMController
{
public:
    /*!
     * @brief   パラメータ
     */
    struct Params {
        Params() = default;
        Params(sec h, double kp, double ki, double kd, double max_f)
            : h(h), kp(kp), ki(ki), kd(kd), max_f(max_f)
        {
            runtime_assert(h > 0 and kp >= 0 and ki >= 0 and kd >= 0
                             and max_f > 0,
                           "Invalid parameters given in PSMController");
        }

        sec h = 0.1f;        //!< Proxy-Dest間の収束時定数
        double kp = 0;     //!< Obj-Proxy間のPゲイン
        double ki = 0;     //!< Obj-Proxy間のIゲイン
        double kd = 0;     //!< Obj-Proxy間のDゲイン
        double max_f = 0;  //!< 最大出力
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec   制御周期[sec]
     */
    explicit PSMController(Params params, sec cycle_sec)
        : m_params(std::move(params)), m_cycle_sec(cycle_sec) {}

    /*!
     * @brief   積分項を0クリアする
     */
    void clearIntegral() { m_a0 = m_a1 = 0; }

    /*!
     * @brief   出力計算
     * @param   error            制御偏差(目標 - 現在値)
     * @param   error_pre        前周期の制御偏差
     */
    double calc(double error, double error_pre)
    {
        const auto& t = m_cycle_sec;
        const auto& p = m_params;

        const double sigma = error + p.h * (error - error_pre) / t;
        const double f_star
            = (p.kd + p.kp * t + p.ki * t * t) / (p.h + t) * sigma
              + (p.kp * p.h - p.kd + p.ki * t * (2 * p.h + t))
                    / (p.h + t) / t * m_a0
              - (p.kp * p.h - p.kd + p.ki * p.h * t)
                    / (p.h + t) / t * m_a1;

        double f = clamp(f_star, -p.max_f, p.max_f);

        const auto a2_tmp = m_a1;
        m_a1 = m_a0;
        m_a0 = ((2 * p.kd + p.kp * t) * m_a0 - p.kd * a2_tmp)
                   / (p.kd + p.kp * t + p.ki * t * t)
               + t * t / (p.kd + p.kp * t + p.ki * t * t) * f;

        return f;
    }

private:
    const Params m_params;
    const sec m_cycle_sec;

    double m_a0 = 0;
    double m_a1 = 0;
};

}  // namespace ClassicalControl
