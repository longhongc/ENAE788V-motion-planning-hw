/* main.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <filesystem>
#include <string>
#include <memory>

#include "problem.h"
#include "solver.h"

int main(int argc, char** argv) {
  if (argc != 9) {
      std::cout << "Error: requires problem number, start x, start y, start theta"
       "goal x, goal y, goal tolerance radius, RRT epsilon" << std::endl;
      return -1;
  }

  // Load arguments
  std::string problem_number = argv[1];
  Coord2D start_coord = std::make_pair(std::stod(argv[2]), std::stod(argv[3]));
  double start_theta = std::stod(argv[4]);
  Coord2D goal_coord = std::make_pair(std::stod(argv[5]), std::stod(argv[6]));
  double goal_tolerance = std::stod(argv[7]);
  double rrt_epsilon = std::stod(argv[8]);

  std::string obstacles_file =
    "./data/obstacles.txt";

  std::string robot_file =
    "../data/H3_robot.txt";

  // Create problem
  std::shared_ptr<Problem> problem_ptr =
    std::make_shared<Problem>(obstacles_file, robot_file);

  std::cout << "Solving problem " << problem_number << ": " << std::endl;
  std::cout << "\033[32mstart:\033[0m " <<
    "(" << start_coord.first << ", " << start_coord.second << ")" << std::endl;
  std::cout << "\033[33mgoal:\033[0m " <<
    "(" << goal_coord.first << ", " << goal_coord.second << ")" << std::endl;
  std::cout << "goal_tolerance: " << goal_tolerance << std::endl;
  std::cout << "rrt_epsilon: " << rrt_epsilon << std::endl;

  std::cout << "=====" << std::endl;

  // Set up start state
  auto [x, y] = start_coord;
  StateValue start_state_value = {x, y, start_theta, 0, 0, 0, 0};
  State start_state(start_state_value, 0.0);

  // Create solver
  Solver solver(start_state, goal_coord,
      goal_tolerance, rrt_epsilon, problem_ptr);

  auto path = solver.solve();
  solver.print_path();

  // Store results
  std::string result_dir = "results/Problem" + problem_number;
  std::filesystem::create_directories(result_dir);
  std::string path_file =
    result_dir + "/Problem" + problem_number + "_path.txt";
  solver.save_path_to_file(path_file);

  std::string search_tree_file =
    result_dir + "/Problem" + problem_number + "_search_tree.txt";
  solver.save_search_tree_to_file(search_tree_file);

  std::string nodes_file =
    result_dir + "/Problem" + problem_number + "_nodes.txt";
  solver.save_nodes_to_file(nodes_file);
}

