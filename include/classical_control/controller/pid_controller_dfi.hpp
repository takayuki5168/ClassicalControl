/*!
 * @file    pid_controller_dfi.hpp
 * @brief   微分先行型PID制御器
 */

#pragma once

#include "classical_control/controller/abst_controller.hpp"
#include "classical_control/core/transfer_function.hpp"
#include "classical_control/util/math.hpp"

namespace ClassicalControl
{

/*!
 * @brief   微分先行型PID制御器クラス
 */
class PIDControllerDFI : AbstController
{
public:
    /*!
     * @brief   伝達関数群
     * @param  p_con P制御器
     * @param  i_con I制御器
     * @param  d_con D制御器
     *                           kd s                        ki
     * U(s) = (R(s) - (1 + --------------) * Y(s)) * (kp + -----)
     *                      1 + eta kd s                     s
     */
    struct Functions {
        Functions() = default;
        Functions(double kp, double ki, double kd, double eta, double cycle_sec)
            : p_con(TransferFunction(TransferFunction::SParams(1, 0, 0, kp, 0, 0),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)),
              i_con(TransferFunction(TransferFunction::SParams(0, 1, 0, ki, 0, 0),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)),
              d_con(TransferFunction(TransferFunction::SParams(1, eta * kd, 0, 0, kd, 0),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)) {}

        TransferFunction p_con;
        TransferFunction i_con;
        TransferFunction d_con;
    };

    /*!
     * @brief   パラメータを変更する
     */
    void changeFunctions(Functions functions) { m_functions = std::move(functions); }

    /*!
     * @brief   コンストラクタ
     * @param   functions   伝達関数群
     * @param   cycle_sec   制御周期[sec]
     */
    explicit PIDControllerDFI(Functions functions, double max_u)
        : m_functions(functions), m_max_u(max_u)
    {
      runtime_assert(max_u <= 0,
                     "Invalid parameters given");
    }

    /*!
     * @brief   出力計算
     * @param  ref   目標値
     * @param  y     出力
     */
    double calc(double ref, double y) override
    {
        auto& f = m_functions;

        auto e = ref - (y + f.d_con.calc(y));
        auto u = f.p_con.calc(e) + f.i_con.calc(e);
        if (m_max_u) {
            u = clamp(u, -m_max_u, m_max_u);
        }
        return u;
    }

private:
    Functions m_functions;      //!< 伝達関数群
    double m_max_u = 0;  //!< 最大出力
};
}
