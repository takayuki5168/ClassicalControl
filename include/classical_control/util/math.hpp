/*!
 * @file    disturb_input.hpp
 * @brief   外乱入力関数
 */

#pragma once

#include <cmath>

namespace ClassicalControl
{

  /*!
   * @brief   零を返す関数
   */
  double zero()
  {
    return 0;
  }

  /*!
   * @brief    周期的にインパルスを発生させる
   * @param    time        現在時刻
   * @param    mag         インパルスの大きさ
   * @param    omg         インパルスが入る周期
   * @param    width       インパルスを発生させる時間
   */
  double impulse(double time, double mag,
                 double omg = 0, double width = 0)
  {
    constexpr double epsilon = 0.1;
    if (std::fabs(omg) < epsilon) {
      return 0.0;
    } else {
      auto po = std::floor(time / omg);
      if (time - omg * po < width) {
        return mag;
      } else {
        return 0.0;
      }
    }
    return 0.0;
  }

  /*!
   * @brief    インパルスを発生させる
   * @param    mag         インパルスの大きさ
   */
  double impulse(double mag)
  {
    return mag;
  }

  /*!
   * @brief   step関数
   */
  double step(double time)
  {
    if (time > 0.0) {
      return 1.0;
    } else {
      return 0.0;
    }
  }

  /*!
   * @brief   ramp関数
   */
  double ramp(double time)
  {
    if (time > 0.0) {
      return time;
    } else {
      return 0.0;
    }
  }

  /*!
   * @brief   振幅と周波数と初期角度を指定できるsin関数
   */
  double sin(double time,
             double amp, double omg, double theta)
  {
    if (time > 0.0) {
      return amp * std::sin(omg * time + theta);
    } else {
      return 0.0;
    }
  }

  double clamp(double val, double min, double max) {
    if (val < min) { val = min; }
    if (val > max) { val = max; }
    return val;
  }

}  // namespace of ClassicalControl
