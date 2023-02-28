#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <random>

#include "solver.h"

Node::Node(Coord2D c, std::shared_ptr<Node> p):
    coord{c},
    parent{p} {
}

Solver::Solver(Coord2D start, Coord2D goal,
    double goal_tolerance, double rrt_epsilon,
    std::shared_ptr<Problem> problem_ptr):
  start_coord_{start},
  goal_coord_{goal},
  goal_tolerance_{goal_tolerance},
  rrt_epsilon_{rrt_epsilon},
  problem_ptr_{problem_ptr} {
  // Check if start coord is valid
  // Checking for goal is not required
  // because there is a tolerance value for the goal
  if (!problem_ptr_->valid_coord(start_coord_)) {
    throw std::invalid_argument("Start coordinate out of bound");
  }

  if (problem_ptr_->check_collision(start_coord_)) {
    throw std::invalid_argument("Start coordinate in obstacle");
  }

  std::shared_ptr<Node> start_node =
    std::make_shared<Node>(start_coord_, nullptr);
  nodes_.push_back(start_node);
}

std::vector<Coord2D> Solver::solve() {
  auto start_time = std::chrono::steady_clock::now();
  // Total duration of RRT
  std::chrono::seconds duration_s{0};

  // Everytime duration passes the threshold
  // print out a time message
  std::chrono::seconds duration_threshold{1};

  while (duration_s < RRT_TIME_LIMIT) {
    auto curr_node = this->sample_node();
    if (curr_node == nullptr) {
      continue;
    }

    if (euclidean_distance(curr_node->coord, goal_coord_) <= goal_tolerance_) {
      std::cout << "\033[32mFound solution\033[0m" << std::endl;
      return this->generate_path(curr_node);
    }

    duration_s = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time);

    if (duration_s > duration_threshold) {
      std::cout << "Search time: [" << duration_s.count() << "s]" << std::endl;
      duration_threshold++;
    }
  }
  std::cout << "\033[31mSolution not found in time limit\033[0m" << std::endl;
  return {};
}

std::shared_ptr<Node> Solver::find_nearest_node(Coord2D coord) {
  // This function can be more efficient using KD-tree
  std::shared_ptr<Node> nearest_node_ptr;
  double min_distance = std::numeric_limits<double>::max();

  for (auto node_ptr : nodes_) {
    auto curr_distance = euclidean_distance(node_ptr->coord, coord);

    if (curr_distance < min_distance) {
      min_distance = curr_distance;
      nearest_node_ptr = node_ptr;
    }
  }

  return nearest_node_ptr;
}

std::shared_ptr<Node> Solver::sample_node() {
  // Create uniform distribution
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<double> uniform_sample_x(
      problem_ptr_->X_MIN, problem_ptr_->X_MAX);
  static std::uniform_real_distribution<double> uniform_sample_y(
      problem_ptr_->Y_MIN, problem_ptr_->Y_MAX);

  // Sample a random coordinate
  Coord2D random_coord = std::make_pair(
      uniform_sample_x(gen), uniform_sample_y(gen));

  auto nearest_node_ptr = this->find_nearest_node(random_coord);
  auto nearest_coord = nearest_node_ptr->coord;
  auto curr_distance = euclidean_distance(nearest_coord, random_coord);

  // If the distance is too long,
  // cut it into epsilon length
  if (curr_distance > rrt_epsilon_) {
    auto& [x1, y1] = nearest_coord;
    auto& [x2, y2] = random_coord;
    double cut_ratio = rrt_epsilon_ / curr_distance;
    double new_x = x1 + (x2 - x1) * cut_ratio;
    double new_y = y1 + (y2 - y1) * cut_ratio;
    random_coord = std::make_pair(new_x, new_y);
  }

  // Return nullptr if this new sampled coordinate is invalid
  if (problem_ptr_->check_collision(nearest_coord, random_coord)) {
    return nullptr;
  }

  std::shared_ptr<Node> new_node =
    std::make_shared<Node>(random_coord, nearest_node_ptr);

  nodes_.push_back(new_node);
  edges_.push_back(std::make_pair(nearest_coord, random_coord));

  return new_node;
}

std::vector<Coord2D> Solver::generate_path(std::shared_ptr<Node> node_ptr) {
  auto curr_node_ptr = node_ptr;
  std::vector<Coord2D> path;
  // Backtracking from current node
  while (curr_node_ptr != nullptr) {
    path.push_back(curr_node_ptr->coord);
    curr_node_ptr = curr_node_ptr->parent;
  }

  std::reverse(path.begin(), path.end());
  path_ = path;
  return path;
}

void Solver::print_path() {
  std::string content;
  for (auto& [x, y] : path_) {
    std::string line =
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
  for (auto& [x, y] : path_) {
    std::string line =
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
  for (auto [coord1, coord2] : edges_) {
    auto& [x1, y1] = coord1;
    auto& [x2, y2] = coord2;
    std::string line =
      std::to_string(x1) + ", " +
      std::to_string(y1) + ", " +
      std::to_string(x2) + ", " +
      std::to_string(y2);

    line += "\n";
    content += line;
  }
  file << content;
  file.close();
}
