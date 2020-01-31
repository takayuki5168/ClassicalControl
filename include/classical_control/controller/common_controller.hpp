/*!
 * @file    common_controller.hpp
 * @brief   一般制御器。
 */

#pragma once

#include "classical_controller/controller/abst_controller.hpp"
#include "classical_controller/core/tranfer_function.hpp"

namespace ClassicalControl
{

/*!
 * @brief   一般制御器
 */
class CommonController : public AbstController
{
public:
    /*!
     * @brief   一般制御器クラス。
     */
    struct Functions {
        Functions() = default;
        /*!
         * @brief   コンストラクタ。
         * @param   deno0
         * @param   deno1
         * @param   deno2
         * @param   nume0
         * @param   nume1
         * @param   nume2
         *
         * U(s)     nume2 * s^2 + nume1 * s + nume0
         * ---- = -----------------------------------
         * E(s)     deno2 * s^2 + deno1 * s + deno0
         */
        Functions(double deno0, double deno1, double deno2,
            double nume0, double nume1, double nume2, sec cycle_sec)
            : con(TransferFunction(
                  TransferFunction::SParams(deno0, deno1, deno2, nume0, nume1, nume2),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)) {}
        /*!
         * @brief   コンストラクタ。
         * @param   deno
         * @param   nume
         *
         * U(s)     nume.at(2) * s^2 + nume.at(1) * s + nume.at(0)
         * ---- = -------------------------------------------------
         * E(s)     deno.at(2) * s^2 + deno.at(1) * s + deno.at(0)
         */
        Functions(std::array<double, 3> deno, std::array<double, 3> nume)
            : con(TransferFunction(TransferFunction::SParams(deno, nume),
                  TransferFunction::ApproxMethod::Backward, cycle_sec)) {}

        TransferFunction con;
    };

    /*!
     * @brief   コンストラクタ。
     * @param   functionss   伝達関数群。
     * @param   max_u        最大入力値。
     */
    explicit CommonController(Functions functions, double max_u)
        : m_functions(functions), m_max_u(max_u)
    {
      runtime_assert(max_u <= 0,
                     "Invalid parameters given");
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
        auto u = f.con.calc(e);
        if (m_max_u) {
            u = clamp(u, -m_max_u, m_max_u);
        }
        return u;
    }

private:
    Functions m_functions;      //!< 伝達関数
    double m_max_u = 0;  //!< 最大出力
};

}  // namespace ClassicalControl
