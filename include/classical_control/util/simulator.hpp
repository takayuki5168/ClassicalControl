/*!
 * @file    simulator.hpp
 * @brief   シミュレーション
 */

#pragma once

#include <Eigen/Core>
#include <fstream>

namespace ClassicalControl
{

template <int Dim>
using Mat = Eigen::Matrix<double, Dim, 1>;
// TODO using Mat = Eigen::Matrix<double, Dim, 2>;

/*!
 * @brief    プラントを表すクラス
 */
template <int DenoDegree>
class Simulator
{
public:
    /*!
     * @brief    コンストラクタ
     * @param    deno                  伝達関数の分母\sum_{i=1}^{DenoDegree-1} deno.at(i)*s^{i}
     * @param    nume                  伝達関数の分子
     * @param    ini_state             初期状態
     * @param    disturb_input_func    時間を引数にとり外乱やノイズ入力を返す関数
     * @param    control_input_func    時間を引数にとり制御入力を返す関数
     */
    explicit Simulator(std::array<double, DenoDegree> deno,
        std::array<double, DenoDegree - 1> nume,
        Mat<DenoDegree - 1> ini_state,
        std::function<double(double)> disturb_input_func,
        std::function<double(double, Mat<DenoDegree - 1>)> control_input_func)
        : m_deno(deno), m_nume(nume),
          m_ini_state(ini_state),
          m_disturb_input_func(disturb_input_func), m_control_input_func(control_input_func)
    {
        m_output_log.open("output_log.log", std::ofstream::out);
        m_control_input_log.open("control_input_log.log", std::ofstream::out);
        m_disturb_input_log.open("disturb_input_log.log", std::ofstream::out);
    }

    void execute()
    {
        double now_time = 0.0;
        double step_time = 0.001;
        Mat<DenoDegree - 1> state = m_ini_state;  //getInitState();
        //int vel_flag = 0;

        while (now_time < 10) {
            m_disturb_input = m_disturb_input_func(now_time);
            m_control_input = m_control_input_func(now_time, state);

            if (state[1] > 0) {
                m_control_input += -1;
            } else if (state[1] < 0) {
                m_control_input += 1;
            }


            Mat<DenoDegree - 1> k1 = m_calc(now_time, state);
            Mat<DenoDegree - 1> k2 = m_calc(now_time + step_time / 2.0, state + step_time / 2.0 * k1);
            Mat<DenoDegree - 1> k3 = m_calc(now_time + step_time / 2.0, state + step_time / 2.0 * k2);
            Mat<DenoDegree - 1> k4 = m_calc(now_time + step_time, state + step_time * k3);

            m_output_log << now_time << " " << state[0] << std::endl;
            m_disturb_input_log << now_time << " " << m_disturb_input << std::endl;
            m_control_input_log << now_time << " " << m_control_input << std::endl;


            now_time += step_time;
            state += step_time / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
    }


private:
    std::array<double, DenoDegree> m_deno;
    std::array<double, DenoDegree - 1> m_nume;
    Mat<DenoDegree - 1> m_ini_state;
    std::function<double(double)> m_disturb_input_func;
    std::function<double(double, Mat<DenoDegree - 1>)> m_control_input_func;

    double m_control_input;
    double m_disturb_input;

    std::fstream m_output_log;
    std::fstream m_control_input_log;
    std::fstream m_disturb_input_log;

    /*
     * @brief   与えられた初期状態をルンゲクッタの初期状態に変換
     */
    Mat<DenoDegree - 1> getInitState()
    {
        Mat<DenoDegree - 1> state;

        Eigen::Matrix<double, DenoDegree - 1, DenoDegree - 1> po;
        for (int i = 0; i < DenoDegree - 1; i++) {
            for (int j = 0; j < DenoDegree - 1; j++) {
                if (i > j) {
                    po(i, j) = 0;
                } else {
                    po(i, j) = m_nume[j - i];
                }
            }
        }
        return po.inverse() * m_ini_state;
    }

    /*
     * @brief   ルンゲクッタの更新関数
     */
    std::function<Mat<DenoDegree - 1>(double, Mat<DenoDegree - 1>)> m_calc
        = [this](double /*time*/, Mat<DenoDegree - 1> state) {
              Mat<DenoDegree - 1> next;
              double input_sum = 0;

              for (int i = 0; i < DenoDegree - 1; i++) {
                  if (i != DenoDegree - 2) {
                      next[i] = state[i + 1];
                      input_sum += -m_deno[i] / m_deno[DenoDegree - 1] * state[i];
                  } else if (i == DenoDegree - 2) {
                      next[i] = input_sum - m_deno[i] / m_deno[DenoDegree - 1] * state[i]
                                + m_disturb_input / m_deno[DenoDegree - 1] + m_control_input / m_deno[DenoDegree - 1];
                  }
              }

              double output_sum = 0;
              for (int i = 0; i < DenoDegree - 1; i++) {
                  output_sum += next[i] * m_nume[i];
              }

              return next;
          };
};

}  // namespace of ClassicalControl
