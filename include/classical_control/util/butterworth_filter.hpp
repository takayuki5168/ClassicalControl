/*!
 * @file    butterworth_filtor.hpp
 * @brief   ButterworthFilter
 */

#pragma once

#include "classical_control/core/transfer_function.hpp"

namespace ClassicalControl
{
  /*!
   * @brief   ButterworthFilterクラス
   */
  class ButterworthFilter : public TransferFunction
  {
  public:
    /*!
     * @brief   パラメータ
     *          Y(s)               omg_c^2
     *          ---- = -------------------------------- * constant
     *          X(s)    s^2 + sqrt(2) omg_c s + omg_c^2
     */
    struct Params {
      Params() = default;
      Params(double omg_c, double constant = 1)
        : omg_c(omg_c), constant(constant) {}

      double omg_c = 0;     //!< カットオフ周波数
      double constant = 1;  //!< 定数倍
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit ButterworthFilter(Params params, double cycle_sec)
      : TransferFunction(
                         TransferFunction::SParams(
                                                   params.omg_c * params.omg_c, std::sqrt(2.0) * params.omg_c, 1,
                                                   params.omg_c * params.omg_c, 0, 0),
                         TransferFunction::ApproxMethod::Backward,
                         cycle_sec) {}
  };

}  // namespace ClassicalControl
