/*!
 * @file    instant_pid_controller.hpp
 * @brief   簡易PID制御器
 */

#pragma once

#include "classical_control/util/math.hpp"

namespace ClassicalControl
{

/*!
 * @brief   簡易PID制御器クラス
 *
 * I成分には指数移動分布をとる。
 */
class InstantPIDController
{
public:
    /*!
     * @brief   パラメータ
     */
    struct Params {
        Params() = default;
        Params(double kp, double ki, double kd, double ti,
            double max_u)
            : kp(kp), ki(ki), kd(kd), ti(ti), max_u(max_u)
        {
            runtime_assert(max_u <= 0,
                "Invalid parameters given in InstantPIDController");
        }

        double kp = 0;          //!< Pゲイン
        double ki = 0;          //!< Iゲイン
        double kd = 0;          //!< Dゲイン
      double ti = 0.5f;       //!< 前周期までの積算値にかける重み
      double max_u = 0;  //!< 最大出力
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit InstantPIDController(Params params, sec cycle_sec)
        : m_params(std::move(params)), m_cycle_sec(cycle_sec) {}

    /*!
     * @brief   積分項を0クリアする
     */
    void clearIntegral() { m_integral = 0; }

    /*!
     * @brief   パラメータを変更する
     */
    void changeParams(Params params) { m_params = std::move(params); }

    /*!
     * @brief   出力計算
     * @param   error            制御偏差(目標 - 現在値)
     * @param   pre_error        前周期の制御偏差
     */
    double calc(double error, double pre_error)
    {
        const auto& p = m_params;

        double u = p.kp * error + p.kd * (error - pre_error) / m_cycle_sec;
        m_integral = (p.ti * m_integral + error) * m_cycle_sec;
        u += p.ki * m_integral;
        if (p.max_u) {
            u = clamp(u, -p.max_u, p.max_u);
        }

        return u;
    }

private:
    Params m_params;
    const sec m_cycle_sec;

    double m_integral = 0;
};

}  // namespace ClassicalControl
