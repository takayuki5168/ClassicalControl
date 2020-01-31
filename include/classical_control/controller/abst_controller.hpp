/*!
 * @file    abst_controller.hpp
 * @brief   制御器の抽象クラス
 */
#pragma once

namespace ClassicalControl
{

/*!
 * @brief   制御器クラス
 */
class AbstController
{
public:
    explicit AbstController() = default;
    /*!
     * @brief   制御量計算
     * @param   ref 目標値
     * @param   y   出力
     */
    virtual double calc(double /*ref*/, double /*y*/) = 0;
};

}  // namespace ClassicalControl
