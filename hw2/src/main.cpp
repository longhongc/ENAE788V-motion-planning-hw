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
  if (argc != 8) {
      std::cout << "Error: requires problem number, start x, start y, "
       "goal x, goal y, goal tolerance radius, RRT epsilon" << std::endl;
      return -1;
  }

  std::string problem_number = argv[1];
  Coord2D start_coord = std::make_pair(std::stod(argv[2]), std::stod(argv[3]));
  Coord2D goal_coord = std::make_pair(std::stod(argv[4]), std::stod(argv[5]));
  double goal_tolerance = std::stod(argv[6]);
  double rrt_epsilon = std::stod(argv[7]);

  std::string obstacles_file =
    "./data/obstacles.txt";

  // Create problem
  std::shared_ptr<Problem> problem_ptr =
    std::make_shared<Problem>(obstacles_file);

  std::cout << "Solving problem " << problem_number << ": " << std::endl;
  std::cout << "\033[32mstart:\033[0m " <<
    "(" << start_coord.first << ", " << start_coord.second << ")" << std::endl;
  std::cout << "\033[33mgoal:\033[0m " <<
    "(" << goal_coord.first << ", " << goal_coord.second << ")" << std::endl;
  std::cout << "goal_tolerance: " << goal_tolerance << std::endl;
  std::cout << "rrt_epsilon: " << rrt_epsilon << std::endl;

  std::cout << "=====" << std::endl;

  // Create solver
  Solver solver(start_coord, goal_coord,
      goal_tolerance, rrt_epsilon, problem_ptr);

  auto path = solver.solve();
  solver.print_path();

  std::string result_dir = "results/Problem" + problem_number;
  std::filesystem::create_directories(result_dir);
  std::string path_file =
    result_dir + "/Problem" + problem_number + "_path.txt";
  solver.save_path_to_file(path_file);

  std::string search_tree_file =
    result_dir + "/Problem" + problem_number + "_search_tree.txt";
  solver.save_search_tree_to_file(search_tree_file);
}

