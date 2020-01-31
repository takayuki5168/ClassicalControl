#include <iostream>

#include "classical_control/controller/pid_controller.hpp"
#include "classical_control/controller/pid_controller_dfi.hpp"
#include "classical_control/util/butterworth_filter.hpp"
#include "classical_control/util/disturbance_observer.hpp"
#include "classical_control/util/disturbance_observer_controller.hpp"
#include "classical_control/core/plant.hpp"
#include "classical_control/util/simulator.hpp"
#include "classical_control/util/math.hpp"

int main()
{
  using namespace ClassicalControl;

  constexpr int END_TIME = 10;
  constexpr double CONTROL_CYCLE_SEC = 0.001;

  constexpr int DEGREE_OF_DENO = 3;
  constexpr double m = 0.175;

//  try {
    /*
     * プラントの初期化
     */
    //!< プラントの伝達関数の分母の係数 deno[0] + deno[1] * s + deno[2] * s^2 + ...
    std::array<double, DEGREE_OF_DENO> deno = {0, 0, m};
    //!< プラントの伝達関数の分子の係数 nume[0] + nume[1] * s + nume[2] * s^2 + ...
    std::array<double, DEGREE_OF_DENO - 1> nume = {1, 0};
    //!< プラントの初期状態 (x, dx/dt, d^2x/dt^2 ...)
    Mat<DEGREE_OF_DENO - 1> ini_state = {0, 0};
    //!< 外乱入力
    auto disturb_func = [](double time) {
      if (6 < time and time < 6.1) {
        return impulse(1);
      } else {
        return zero();
      }
    };

    //!< 制御器
    auto inertia = Inertia(Inertia::Params{m}, CONTROL_CYCLE_SEC);
    auto butterworth_filter = ButterworthFilter(ButterworthFilter::Params{100.0}, CONTROL_CYCLE_SEC);
    auto dob = DOB(DOB::Params{inertia, butterworth_filter, 0});

    auto pole = -10;
    auto pid_controller = PIDController(PIDController::Functions{m * (pole * pole), 0, -m * pole * 2, 0, CONTROL_CYCLE_SEC}, 4.0);
    auto pid_controller_dfi = PIDControllerDFI(PIDControllerDFI::Functions{70, 0, 10.0 / 70, 0, CONTROL_CYCLE_SEC}, 4.75);
    auto dob_controller = DOBController(pid_controller, dob);

    std::function<double(double, Mat<DEGREE_OF_DENO - 1>)>
      control_func
      = [&](double time, Mat<DEGREE_OF_DENO - 1> state) {
      double dest, cur;
      if (time < 1) {
        dest = 6.28 * 0.1;  // - std::exp(-1 * time);
      } else if (time < 2) {
        dest = 6.28 * 0.3;  // * 3;
      } else {
        dest = 6.28 * 0.6;
      }

      //cur = pid_controller.calc(dest, state[0]);
      cur = dob_controller.calc(dest, state[0]);
      //cur = truncateAbs(cur, 4.0); TODO

      return cur;
    };

    //!< シミュレータの初期化
    auto sim = Simulator<DEGREE_OF_DENO>(deno,          //!< プラントの伝達関数の分母の極
                                         nume,          //!< プラントの伝達関数の分子の極
                                         ini_state,     //!< プラントの出力状態
                                         disturb_func,  //!< 外乱入力
                                         control_func   //!< 制御器
                                         );

    sim.execute();  //!< プラントシミュレータの計算、更新
/*
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception" << std::endl;
  }
*/
  return 0;
}
