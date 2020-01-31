/*!
 * @file    disturbance_observer_control.hpp
 * @brief   外乱オブザーバと制御器を組み合わせた系
 */

#pragma once

#include "classical_control/util/disturbance_observer.hpp"
#include "classical_control/controller/abst_controller.hpp"
//#include "core/transfer_function.hpp"

namespace ClassicalControl
{

  /*!
   * @brief   外乱オブザーバと制御器を組み合わせた系のクラス
   */
  class DOBController
  {
  public:
    /*!
     * @brief   コンストラクタ
     * @param   controller   制御器
     * @param   dob          外乱オブザーバ
     */
    explicit DOBController(AbstController& controller, DOB dob)
      : m_controller(controller), m_dob(dob) {}

    /*!
     * @brief   外乱を考慮した制御量計算
     * @param   ref   目標値
     * @param   y     出力
     */
    double calc(double ref, double y)
    {
      auto u = m_controller.calc(ref, y);
      d = m_dob.calc(u - d, y);
      return u - d;
    }

  private:
    AbstController& m_controller;  //!< 制御器
    DOB m_dob;                     //!< 外乱オブザーバ

    double d = 0;  //!< 外乱
  };

}  // namespace ClassicalControl
