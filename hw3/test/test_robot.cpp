/* test_robot.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>

#include "robot.h"

int main(int argc, char** argv) {
  // StateValue start_value = {0, 0, 0, 1, 0.1, 0, 0};
  StateValue start_value = {26.003429, -36.271821, -0.953800, -17.707797, -8.044582, -1.770780, -0.804458};
  State start(start_value, 10);
  // Test simulation
  auto traj = robot_dynamic::forward_simulation(start, 10, 1);
  for (auto state : traj) {
    auto value = state.get_value();
    std::cout << "Time: " << state.get_time() << std::endl;
    for (int i=0; i < value.size(); ++i) {
      std::cout << value[i] << ", ";
    }
    std::cout << std::endl;
  }
  return 0;
}
