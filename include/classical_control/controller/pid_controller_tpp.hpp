/*!
 * @file    pid_controller_tpp.hpp
 * @brief   三重極配置PID制御器
 */

#pragma once

#include "classical_control/controller/pid_controller.hpp"
#include "classical_control/core/plant.hpp"

namespace ClassicalControl
{
/*!
 * @brief   三重極配置PID制御器クラス
 */
class PIDControllerTPP : PIDController
{
    /*!
     * @brief   パラメータ
     *          Y(s)        1
     *          ---- = -------------
     *          R(s)    m(s - p)^3
     */

    struct Params {
        Params() = default;
        Params(double pole, double eta, Plant plant, sec cycle_sec)
            : pole(pole), eta(eta), plant(plant), cycle_sec(cycle_sec) {}

        double pole;       //!< 三重極
        double eta;        //!< 疑似微分
        Plant plant;         //!< プラント
        double cycle_sec;  //!< 制御周期
    };


    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit PIDControllerTPP(Fractions fractions, double max_u)
    {
        double kp, ki, kd;
        if (plant == Plant::Model::Inertia) {
            if (eta == 0) {
            } else {
            }
        } else if (plant == Plant::Model::Viscosity) {
            if (eta == 0) {
                runtime_assert(true, "Invalid parameters given in PIDControllerTPP");
            } else {
            }
        }
        PIDController(PIDController::Fractions(kp, kd, ki, eta, cycle_sec), max_u);
    }

private:
    const Params params;  //!<
};
}  // namespace of ClassicalControl
