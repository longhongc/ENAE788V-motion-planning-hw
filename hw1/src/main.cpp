#include <iostream>
#include <filesystem>
#include <string>
#include <memory>

#include "problem.h"
#include "solver.h"

int main(int argc, char** argv) {
  if (argc != 4) {
      std::cout << "Error: requires problem number, start id, and goal_id" << std::endl;
      return -1;
  }
  std::string problem_number = argv[1];
  int start_id = std::stoi(argv[2]);
  int goal_id = std::stoi(argv[3]);

  std::string nodes_file = 
    "./data/nodes_" + problem_number + ".txt";
  std::string edges_file = 
    "./data/edges_with_costs_" + problem_number + ".txt";

  std::shared_ptr<Problem> problem1_ptr = 
    std::make_shared<Problem>(nodes_file, edges_file);

  std::cout << "\033[4mSolving problem " << problem_number <<
    " from start_id \033[4;32m" << start_id <<
    "\033[0m\033[4m to goal_id \033[33m" <<
    goal_id << "\033[0m" << std::endl;

  Solver solver(problem1_ptr, start_id, goal_id);
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

  return 0;
}
