/* test_problem.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <memory>
#include <vector>
#include <string>


#include "problem.h"
#include "robot.h"

int main(int argc, char** argv) {
  std::string obstacles_file =
    "../data/obstacles.txt";

  std::string robot_file =
    "../data/H3_robot.txt";

  // Create problem
  std::shared_ptr<Problem> problem_ptr =
    std::make_shared<Problem>(obstacles_file, robot_file);

  StateValue state1_value = {0, 0, 0, 1, 0.1, 0, 0};
  State state1(state1_value, 0.0);
  StateValue state2_value = {0, 5, 0, 1, 0.1, 0, 0};
  State state2(state2_value, 0.0);
  StateValue state3_value = {0, -30, 0, 1, 0.1, 0, 0};
  State state3(state3_value, 0.0);

  std::vector<State> trajectory;
  trajectory.push_back(state1);
  trajectory.push_back(state2);

  // Test trajectory collision
  if (problem_ptr->check_collision(trajectory)) {
    std::cout << "Collide in state1 and state2" << std::endl;
  }

  // State 3 should collide
  trajectory.push_back(state3);

  if (problem_ptr->check_collision(trajectory)) {
    std::cout << "Collide in state3" << std::endl;
  }

  return 0;
}



