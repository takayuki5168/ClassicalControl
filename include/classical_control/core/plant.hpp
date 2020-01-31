/*!
 * @file    plant.hpp
 * @brief   プラントのモデル
 */

#pragma once

#include "classical_control/core/transfer_function.hpp"

namespace ClassicalControl
{

  /*!
   * @brief   慣性項のみのプラントクラス
   */
  class Inertia : public TransferFunction
  {
  public:
    /*!
     * @brief   パラメータ
     * @param   m   慣性質量
     *          Y(s)      1
     *          ---- = --------
     *          X(s)    m s^2
     */
    struct Params {
      Params() = default;
      Params(double m) : m(m) {}
      double m = 1;  //!< 慣性質量
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit Inertia(Params params, double cycle_sec)
      : TransferFunction(
                         TransferFunction::SParams(0.0, 0.0, params.m, 1.0, 0.0, 0.0),
                         TransferFunction::ApproxMethod::Backward,
                         cycle_sec) {}
  };

  /*!
   * @brief   粘性項のみのプラントクラス
   */
  class Viscosity : public TransferFunction
  {
  public:
    /*!
     * @brief   パラメータ
     * @param   m   粘性質量
     *          Y(s)     1
     *          ---- = ------
     *          X(s)    c s
     */
    struct Params {
      Params() = default;
      Params(double c) : c(c) {}
      double c = 1;  //!< 粘性質量
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit Viscosity(Params params, double cycle_sec)
      : TransferFunction(
                         TransferFunction::SParams(0.0, params.c, 0.0, 1.0, 0.0, 0.0),
                         TransferFunction::ApproxMethod::Backward,
                         cycle_sec) {}
  };

  /*!
   * @brief   比例項のみのプラントクラス
   */
  class Proportion : public TransferFunction
  {
  public:
    /*!
     * @brief   パラメータ
     * @param   m   比例質量
     *          Y(s)    1
     *          ---- = ----
     *          X(s)    k
     */
    struct Params {
      Params() = default;
      Params(double k) : k(k) {}
      double k = 1;  //!< 比例質量
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit Proportion(Params params, double cycle_sec)
      : TransferFunction(
                         TransferFunction::SParams(params.k, 0.0, 0.0, 1.0, 0.0, 0.0),
                         TransferFunction::ApproxMethod::Backward,
                         cycle_sec) {}
  };

  /*!
   * @brief   バネマスダンパ系のプラントクラス
   */
  class SpringMassDamper : public TransferFunction
  {
  public:
    /*!
     * @brief   パラメータ
     * @param   m   慣性質量
     * @param   c   粘性質量
     * @param   k   比例質量
     *          Y(s)           1
     *          ---- = -----------------
     *          X(s)    m s^2 + c s + k
     */
    struct Params {
      Params() = default;
      Params(double m, double c, double k) : m(m), c(c), k(k) {}
      double m = 1;  //!< 慣性質量
      double c = 1;  //!< 粘性質量
      double k = 1;  //!< 比例質量
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     * @param   cycle_sec  制御周期[sec]
     */
    explicit SpringMassDamper(Params params, double cycle_sec)
      : TransferFunction(
                         TransferFunction::SParams(params.k, params.c, params.m, 1.0, 0.0, 0.0),
                         TransferFunction::ApproxMethod::Backward,
                         cycle_sec) {}
  };

}  // namespace ClassicalControl
