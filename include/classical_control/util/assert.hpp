/*!
 * @file    assert.hpp
 * @brief   
 */

#pragma once

#include <string>
#include <iostream>

namespace ClassicalControl
{

  /*!
   * @brief   零を返す関数
   */
  void runtime_assert(bool condition, std::string sentence)
  {
    if (condition) {
      std::cout << sentence << std::endl;
      std::exit(0);
    }
  }

}  // namespace of ClassicalControl
