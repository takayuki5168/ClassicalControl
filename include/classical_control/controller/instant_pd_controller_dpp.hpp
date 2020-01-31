/*!
 * @file    instant_pd_controller_dpp.hpp
 * @brief   二重極配置による簡易PD制御器
 */

#pragma once

#include "classical_control/util/math.hpp"

namespace ClassicalControl
{

/*!
 * @brief  二重極配置による簡易PD制御器クラス
 */
class InstantPDControllerDPP
{
public:
    /*!
     * @brief   パラメータ
     */
    struct Params {
        Params() = default;
        Params(double m, double max_f) : m(m), max_f(max_f)
        {
            runtime_assert(m > 0 and max_f <= 0,
                "Invalid parameters given");
        }

        double m = 0;           //!< 擬似慣性質量[power/((mm|rad)/s^2)]
        double max_f = 0;  //!< 最大出力
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit InstantPDControllerDPP(Params params, sec cycle_sec)
        : m_params(std::move(params)), m_cycle_sec(cycle_sec) {}

    /*!
     * @brief   出力計算
     * @param   error            制御偏差(目標 - 現在値)
     * @param   error_pre        前周期の制御偏差
     * @param   converg_time    収束時定数[sec]
     */
    double calc(double error, double error_pre, sec converg_time) const
    {
        const auto& p = m_params;

        const radvel w = 2 * PI / converg_time;
        const double kp = p.m * w * w;
        const double kd = 2 * p.m * w;

        double f = kp * error + kd * (error - error_pre) / m_cycle_sec;
        if (p.max_f) {
            f = clamp(f, -p.max_f, p.max_f);
        }

        return f;
    }

private:
    const Params m_params;
    const sec m_cycle_sec;
};

}  // namespace ClassicalControl
