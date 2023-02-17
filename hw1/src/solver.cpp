/* solver.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <fstream>
#include <queue>
#include <cmath>
#include <memory>

#include "solver.h"

Solver::Solver(std::shared_ptr<Problem> problem_ptr,
  ID start_id, ID goal_id) :
  problem_ptr_{problem_ptr},
  start_id_{start_id},
  goal_id_{goal_id} {
  // Insert start node into priority queue
  node_queue_.emplace(std::make_pair(0.0, start_id));
  Node start_node{start_id, 0, 0.0};
  node_map_[start_id] = start_node;
}

std::vector<ID> Solver::solve() {
  while (!node_queue_.empty()) {
    // Pop out the node with least cost
    auto [curr_total_cost, curr_id] = node_queue_.top();
    auto curr_cost_to_start = node_map_[curr_id].cost_to_start;
    node_queue_.pop();
    // Check if the node is already visited
    if (visited_.find(curr_id) != visited_.end()) {
      continue;
    }
    visited_.insert(curr_id);

    // If it is the goal, then return path
    if (curr_id == goal_id_) {
      std::cout << "\033[32mFound solution\033[0m" << std::endl;
      std::cout << "Total cost: " << node_map_[curr_id].cost_to_start << std::endl;
      return this->generate_path(curr_id);
    }

    // Expand to child nodes
    for (auto [cost, child_id] : problem_ptr_->get_neighbors(curr_id)) {
      auto cost_to_start = curr_cost_to_start + cost;

      // If node already visited and has higher cost then before,
      // then ignore it
      if (node_map_.find(child_id) != node_map_.end()) {
        if (cost_to_start > node_map_[child_id].cost_to_start) {
          continue;
        }

      }

      node_map_[child_id] = Node{child_id, curr_id, cost_to_start};

      // Calculate the total cost by
      // cost_to_start + cost_to_goal(heuristic)
      auto total_cost =
        cost_to_start + this->heuristic(child_id, goal_id_);

      node_queue_.emplace(std::make_pair(total_cost, child_id));
    }

  }

  std::cout << "\033[31mSolution not found\033[0m" << std::endl;

  return {};
}

double Solver::heuristic(ID node_1, ID node_2) {
  auto [x1, y1] = problem_ptr_->get_coord(node_1);
  auto [x2, y2] = problem_ptr_->get_coord(node_2);

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

std::vector<ID> Solver::generate_path(ID node_id) {
  auto curr_id = node_id;
  std::vector<ID> path;
  path.push_back(curr_id);
  // Backtracking from current node
  while (curr_id != start_id_) {
    curr_id = node_map_[curr_id].parent_id;
    path.push_back(curr_id);
  }

  std::reverse(path.begin(), path.end());
  path_ = path;
  return path;
}

void Solver::print_path() {
  std::string content;
  for (auto id : path_) {
    auto [x, y] = problem_ptr_->get_coord(id);
    std::string line =
      "ID \033[36m" + std::to_string(id) + "\033[0m at " +
      "(" + std::to_string(x) + ", " +
      std::to_string(y) + ")";

    line += "\n";
    content += line;
  }
  std::cout << content << std::endl;
}

void Solver::save_path_to_file(std::string file_name) {
  std::ofstream file{file_name};
  std::string content;
  for (auto id : path_) {
    auto [x, y] = problem_ptr_->get_coord(id);
    std::string line =
      std::to_string(id) + ", " +
      std::to_string(x) + ", " +
      std::to_string(y);

    line += "\n";
    content += line;
  }
  file << content;
  file.close();
}

void Solver::save_search_tree_to_file(std::string file_name) {
  std::ofstream file{file_name};
  std::string content;
  for (auto [id, node] : node_map_) {
    if (id == start_id_) {
      continue;
    }

    auto [x1, y1] = problem_ptr_->get_coord(id);
    auto [x2, y2] = problem_ptr_->get_coord(node.parent_id);
    std::string line =
      std::to_string(id) + ", " +
      std::to_string(x1) + ", " +
      std::to_string(y1) + ", " +
      std::to_string(node.parent_id) + ", " +
      std::to_string(x2) + ", " +
      std::to_string(y2);

    line += "\n";
    content += line;
  }
  file << content;
  file.close();
}
