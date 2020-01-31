/*!
 * @file    disturbance_observer.hpp
 * @brief   外乱オブザーバ
 */

#pragma once

#include "classical_control/core/transfer_function.hpp"

namespace ClassicalControl
{

/*!
 * @brief   外乱オブザーバクラス
 */
class DOB
{
public:
    /*!
     * @brief   パラメータ
     */
    struct Params {
        Params() = default;
        Params(TransferFunction plant, TransferFunction filter, double gain)
          : filter(filter), filter_plantinv(filter * plant.inverse()), gain(gain)
      {}

        TransferFunction filter;           //!< フィルタ
        TransferFunction filter_plantinv;  //!< フィルタとプラントの逆数の積
        double gain;                     //!< ゲイン
    };

    /*!
     * @brief   コンストラクタ
     * @param   params      パラメータ
     */
    explicit DOB(Params params)
      : m_params(params) {}

    /*!
     * @brief   出力計算
     * @param   u   制御量
     * @param   y   出力
     */
    double calc(double u, double y)
    {
        auto d = m_params.gain * (m_params.filter_plantinv.calc(y) - m_params.filter.calc(u));
        return d;
    }

private:
    Params m_params;
};

}  // namespace ClassicalControl
