/* solver.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <random>

#include "solver.h"

Node::Node(State s, std::vector<State> wp, std::shared_ptr<Node> p):
    state{s},
    waypoints{wp},
    parent{p} {
  coord = state.get_coord();
}

Solver::Solver(State start, Coord2D goal,
    double goal_tolerance, double rrt_epsilon,
    std::shared_ptr<Problem> problem_ptr):
  start_state_{start},
  goal_coord_{goal},
  goal_tolerance_{goal_tolerance},
  rrt_epsilon_{rrt_epsilon},
  problem_ptr_{problem_ptr} {
  // Check if start coord is valid
  // Checking for goal is not required
  // because there is a tolerance value for the goal
  start_coord_ = start_state_.get_coord();
  if (!problem_ptr_->valid_coord(start_coord_)) {
    throw std::invalid_argument("Start coordinate out of bound");
  }

  if (problem_ptr_->check_collision(start_coord_)) {
    throw std::invalid_argument("Start coordinate in obstacle");
  }

  std::shared_ptr<Node> start_node =
    std::make_shared<Node>(start_state_, std::vector<State>(), nullptr);
  nodes_.push_back(start_node);
}

std::vector<State> Solver::solve() {
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

  // Create uniform distribution for actions
  // The actions are linear accleration and angular accleration
  static std::uniform_real_distribution<double> uniform_sample_a(
      robot_dynamic::ACC_MIN, robot_dynamic::ACC_MAX);
  static std::uniform_real_distribution<double> uniform_sample_r(
      robot_dynamic::GAMMA_MIN, robot_dynamic::GAMMA_MAX);

  // Add action to the current state
  StateValue new_value = nearest_node_ptr->state.get_value();
  new_value[5] = uniform_sample_a(gen);
  new_value[6] = uniform_sample_r(gen);

  double nearest_node_time = nearest_node_ptr->state.get_time();
  State nearest_node_add_control{new_value, nearest_node_time};

  // Simulate starting from current state
  auto trajectory = robot_dynamic::forward_simulation(
      nearest_node_add_control,
      rrt_epsilon_,
      RRT_DELTA);

  if (trajectory.empty()) {
    return nullptr;
  }

  // Check if any waypoints in the trajectory is already in goal
  // and cut the trajectory to that waypoint
  int cut_index = -1;
  for (int i = 0; i < trajectory.size(); ++i) {
    auto curr_coord = trajectory[i].get_coord();
    if (euclidean_distance(curr_coord, goal_coord_) <= goal_tolerance_) {
      std::cout << "test" << std::endl;
      cut_index = i + 1;
      break;
    }
  }

  if (cut_index > 0 &&
      cut_index < trajectory.size()) {
    trajectory.erase(trajectory.begin() + cut_index, trajectory.end());
  }


  // Do collision checking for waypoints in the trajectory
  if (problem_ptr_->check_collision(trajectory)) {
    return nullptr;
  }

  auto new_state = trajectory.back();
  std::shared_ptr<Node> new_node =
    std::make_shared<Node>(new_state, trajectory, nearest_node_ptr);

  nodes_.push_back(new_node);

  // Store every edge including local trajectory into the edges
  auto prev_coord = nearest_node_ptr->coord;
  for (int i = 0; i < trajectory.size(); ++i) {
    auto curr_coord = trajectory[i].get_coord();
    edges_.push_back(std::make_pair(
        prev_coord, curr_coord));
    prev_coord = curr_coord;
  }

  return new_node;
}

std::vector<State> Solver::generate_path(std::shared_ptr<Node> node_ptr) {
  auto curr_node_ptr = node_ptr;
  std::vector<State> path;
  // Backtracking from current node
  while (curr_node_ptr != nullptr) {
    for (int i=curr_node_ptr->waypoints.size() - 1;
        i >= 0; --i) {
      path.push_back(curr_node_ptr->waypoints[i]);
    }
    curr_node_ptr = curr_node_ptr->parent;
  }

  std::reverse(path.begin(), path.end());
  path_ = path;
  return path;
}

void Solver::print_path() {
  std::string content;
  for (auto& state : path_) {
    double x = state.get_value()[0];
    double y = state.get_value()[1];
    double theta = state.get_value()[2];
    double v = state.get_value()[3];
    double w = state.get_value()[4];
    double a = state.get_value()[5];
    double r = state.get_value()[6];

    double t = state.get_time();

    std::string line =
      "Time: " + std::to_string(t) + ", " +
      "(" + std::to_string(x) + ", " +
      std::to_string(y) + ", " +
      std::to_string(theta) + ", " +
      std::to_string(v) + ", " +
      std::to_string(w) + ", " +
      std::to_string(a) + ", " +
      std::to_string(r) + ")";

    line += "\n";
    content += line;
  }
  std::cout << content << std::endl;
}

void Solver::save_path_to_file(std::string file_name) {
  std::ofstream file{file_name};
  std::string content;
  for (auto& state : path_) {
    double x = state.get_value()[0];
    double y = state.get_value()[1];
    double theta = state.get_value()[2];
    double v = state.get_value()[3];
    double w = state.get_value()[4];
    double a = state.get_value()[5];
    double r = state.get_value()[6];

    double t = state.get_time();

    std::string line =
      std::to_string(t) + ", " +
      std::to_string(x) + ", " +
      std::to_string(y) + ", " +
      std::to_string(theta) + ", " +
      std::to_string(v) + ", " +
      std::to_string(w) + ", " +
      std::to_string(a) + ", " +
      std::to_string(r);

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

void Solver::save_nodes_to_file(std::string file_name) {
  std::ofstream file{file_name};
  std::string content;
  for (auto node_ptr : nodes_) {
    auto& [x, y] = node_ptr->coord;
    std::string line =
      std::to_string(x) + ", " +
      std::to_string(y);

    line += "\n";
    content += line;
  }
  file << content;
  file.close();
}
