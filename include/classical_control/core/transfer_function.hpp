/*!
 * @file    transfer_function.hpp
 * @brief   伝達関数クラス。
 */

#pragma once

#include <array>
#include "classical_control/util/assert.hpp"

namespace ClassicalControl
{

  /*!
   * @brief   伝達関数クラス。
   */
  class TransferFunction
  {
  public:
    struct SParams {
      SParams() = default;
      SParams(double deno0, double deno1, double deno2,
              double nume0, double nume1, double nume2)
        : deno(std::array<double, 3>{deno0, deno1, deno2}),
          nume(std::array<double, 3>{nume0, nume1, nume2}) {}
      SParams(std::array<double, 3> deno, std::array<double, 3> nume)
        : deno(std::move(deno)), nume(std::move(nume)) {}

      std::array<double, 3> deno = std::array<double, 3>{1, 1, 1};  //!< 分母の係数
      std::array<double, 3> nume = std::array<double, 3>{1, 1, 1};  //!< 分子の係数
    };

    struct ZParams {
      ZParams() = default;
      ZParams(double deno0, double deno1, double deno2,
              double nume0, double nume1, double nume2)
        : deno(std::array<double, 3>{deno0, deno1, deno2}),
          nume(std::array<double, 3>{nume0, nume1, nume2}) {}
      ZParams(std::array<double, 3> deno, std::array<double, 3> nume)
        : deno(std::move(deno)), nume(std::move(nume)) {}

      std::array<double, 3> deno = std::array<double, 3>{1, 1, 1};  //!< 分母の係数
      std::array<double, 3> nume = std::array<double, 3>{1, 1, 1};  //!< 分子の係数
    };

    /*!
     * @brief   Z変換に使う近似方法。
     */
    enum class ApproxMethod : uint8_t {
      Forward,   //<! 前進差分近似
        Backward,  //<! 後退差分近似
        Tustin,    //<! Tustin近似
        };

    explicit TransferFunction(SParams sparams, ApproxMethod approx_method, double cycle_sec)
      : m_sparams(sparams), m_approx_method(approx_method), m_cycle_sec(cycle_sec)
    {
      m_input = {0, 0, 0};
      m_output = {0, 0, 0};


      switch (m_approx_method) {
      case ApproxMethod::Forward:
        break;
      case ApproxMethod::Tustin: {
        const auto& sd = m_sparams.deno;
        const auto& sn = m_sparams.nume;
        const auto& t = m_cycle_sec;

        const auto zd0 = 4.0 * sd.at(2) - 2.0 * sd.at(1) * t + sd.at(0) * t * t;
        const auto zd1 = -8.0 * sd.at(2) + 2.0 * sd.at(0) * t * t;
        const auto zd2 = 4.0 * sd.at(2) + 2.0 * sd.at(1) * t + sd.at(0) * t * t;
        const auto zn0 = 4.0 * sn.at(2) - 2.0 * sn.at(1) * t + sn.at(0) * t * t;
        const auto zn1 = -8.0 * sn.at(2) + 2.0 * sn.at(0) * t * t;
        const auto zn2 = 4.0 * sn.at(2) + 2.0 * sn.at(1) * t + sn.at(0) * t * t;

        m_zparams = ZParams{zd0, zd1, zd2, zn0, zn1, zn2};
        break;
      }
      case ApproxMethod::Backward:
      default: {
        const auto& sd = m_sparams.deno;
        const auto& sn = m_sparams.nume;
        const auto& t = m_cycle_sec;

        const auto zd0 = sd.at(2);
        const auto zd1 = -2.0 * sd.at(2) - sd.at(1) * t;
        const auto zd2 = sd.at(2) + sd.at(1) * t + sd.at(0) * t * t;
        const auto zn0 = sn.at(2);
        const auto zn1 = -2.0 * sn.at(2) - sn.at(1) * t;
        const auto zn2 = sn.at(2) + sn.at(1) * t + sn.at(0) * t * t;

        m_zparams = ZParams{zd0, zd1, zd2, zn0, zn1, zn2};
        break;
      }
      }
    }

    explicit TransferFunction(ZParams zparams)
      : m_zparams(zparams)
    {
      m_input = {0, 0, 0};
      m_output = {0, 0, 0};
    }

    explicit TransferFunction() {}

    /*!
     * @brief   出力計算。
     * @param   input   入力信号。
     */
    double calc(double input)
    {
      m_input.at(2) = m_input.at(1);
      m_input.at(1) = m_input.at(0);
      m_input.at(0) = input;
      m_output.at(2) = m_output.at(1);
      m_output.at(1) = m_output.at(0);
      const auto& z_deno = m_zparams.deno;
      const auto& z_nume = m_zparams.nume;


      double output = (z_nume.at(2) * m_input.at(0)
                       + z_nume.at(1) * m_input.at(1)
                       + z_nume.at(0) * m_input.at(2)
                       - z_deno.at(1) * m_output.at(1)
                       - z_deno.at(0) * m_output.at(2))
        / z_deno.at(2);
      m_output.at(0) = output;
      return output;
    }

    /*!
     * @brief   逆数を返す関数。
     */
    TransferFunction inverse()
    {
      return TransferFunction(SParams{m_sparams.nume, m_sparams.deno}, m_approx_method, m_cycle_sec);
    }

    /*!
     * @brief   積のオペレータ。
     */
    TransferFunction operator*(const TransferFunction& func)
    {
      const auto& this_sd = m_sparams.deno;
      const auto& this_sn = m_sparams.nume;
      const auto& other_sp = func.getSParams();
      const auto& other_sd = other_sp.deno;
      const auto& other_sn = other_sp.nume;

      runtime_assert(not (this_sd.at(2) * other_sd.at(2) == 0
                          and this_sd.at(2) * other_sd.at(1) + this_sd.at(1) * other_sd.at(2) == 0
                          and this_sn.at(2) * other_sn.at(2) == 0
                          and this_sn.at(2) * other_sn.at(1) + this_sn.at(1) * other_sn.at(2) == 0),
                     "Invalid parameters given in TransferFunction");

      auto sdeno = std::array<double, 3>{
        this_sd.at(0) * other_sd.at(0),
        this_sd.at(1) * other_sd.at(0) + this_sd.at(0) * other_sd.at(1),
        this_sd.at(2) * other_sd.at(0) + this_sd.at(1) * other_sd.at(1) + this_sd.at(0) * other_sd.at(2)};
      auto snume = std::array<double, 3>{
        this_sn.at(0) * other_sn.at(0),
        this_sn.at(1) * other_sn.at(0) + this_sn.at(0) * other_sn.at(1),
        this_sn.at(2) * other_sn.at(0) + this_sn.at(1) * other_sn.at(1) + this_sn.at(0) * other_sn.at(2)};
      return TransferFunction(SParams{sdeno, snume}, m_approx_method, m_cycle_sec);
    }

    /*!
     * @brief   m_sparamsを返す関数。
     */
    SParams getSParams() const
    {
      return m_sparams;
    }

  private:
    SParams m_sparams;
    ZParams m_zparams;
    ApproxMethod m_approx_method;
    double m_cycle_sec;

    std::array<double, 3> m_input;
    std::array<double, 3> m_output;
  };

}  // namespace ClassicalControl
