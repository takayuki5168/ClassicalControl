/*!
 * @file    pid_controller.hpp
 * @brief   PID制御器。
 */

#pragma once

#include "classical_control/controller/abst_controller.hpp"
#include "classical_control/core/transfer_function.hpp"
#include "classical_control/util/math.hpp"

namespace ClassicalControl
{

/*!
 * @brief   PID制御器クラス。
 */
class PIDController : public AbstController
{
public:
    /*!
     * @brief   伝達関数群。
     */
    struct Functions {
        Functions() = default;
        /*!
         * @brief   コンストラクタ。
         * @param   p_con P制御器。
         * @param   i_con I制御器。
         * @param   d_con D制御器。
         *
         * U(s)          ki           kd s
         * ---- = kp + ------ + --------------
         * E(s)          s       1 + eta kd s
         */
        Functions(double kp, double ki, double kd, double eta, double cycle_sec)
            : p_con(TransferFunction(TransferFunction::SParams(1, 0, 0, kp, 0, 0),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)),
              i_con(TransferFunction(TransferFunction::SParams(0, 1, 0, ki, 0, 0),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)),
              d_con(TransferFunction(TransferFunction::SParams(1, eta * kd, 0, 0, kd, 0),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)) {}

        TransferFunction p_con;  //!< P制御器
        TransferFunction i_con;  //!< I制御器
        TransferFunction d_con;  //!< D制御器
    };

    /*!
     * @brief   パラメータを変更する。
     */
    void changeFunctions(Functions functions) { m_functions = std::move(functions); }

    /*!
     * @brief   コンストラクタ。
     * @param   functions   伝達関数群。
     * @param   max_u       最大入力値。
     */
    explicit PIDController(Functions functions, double max_u)
        : m_functions(functions), m_max_u(max_u)
    {
        runtime_assert(max_u <= 0,
            "Invalid parameters given in PIDController");
    }

    /*!
     * @brief   制御量計算。
     * @param   ref   目標値。
     * @param   y     出力。
     */
    double calc(double ref, double y) override
    {
        auto& f = m_functions;

        auto e = ref - y;
        auto u = f.p_con.calc(e) + f.i_con.calc(e) + f.d_con.calc(e);
        if (m_max_u) {
            u = ClassicalControl::clamp(u, -m_max_u, m_max_u);
        }
        return u;
    }

private:
    Functions m_functions;      //!< 伝達関数群
    double m_max_u = 0;  //!< 最大出力
};

}  // namespace ClassicalControl
