/* problem.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "problem.h"

Problem::Problem(std::string nodes_file,
  std::string edges_file) {
  this->load_nodes(nodes_file);
  this->load_edges(edges_file);
  std::cout << "\033[3mLoading complete\033[0m" << std::endl;
}

void Problem::load_nodes(std::string& nodes_file) {
  std::cout << "\033[3mLoading " <<
    nodes_file << "\033[0m" << std::endl;
  std::ifstream file{nodes_file};
  std::string line;
  int line_count = 0;

  if (file.is_open()) {
    while (std::getline(file, line)) {
      line_count++;
      // The first line is the total number of nodes
      if (line_count == 1) {
        continue;
      }

      std::stringstream ss{line};
      std::string item;
      int item_count = 0;
      ID id = 0;
      double x = 0.0;
      double y = 0.0;
      // Every line has the form
      // ID, x, y
      while (std::getline(ss, item, ',')) {
        switch (item_count) {
          case 0:
            id = stoi(item);
            break;
          case 1:
            x = stod(item);
            break;
          case 2:
            y = stod(item);
            break;
          default:
            break;
        }

        item_count++;
      }

      nodes_.push_back(id);
      nodes_coord_[id] = std::make_pair(x, y);
    }

  } else {
    throw std::invalid_argument("Bad node file name");
  }
}

void Problem::load_edges(std::string& edges_file) {
  std::cout << "\033[3mLoading " <<
    edges_file << "\033[0m" << std::endl;
  std::ifstream file{edges_file};
  std::string line;
  int line_count = 0;

  if (file.is_open()) {
    while (std::getline(file, line)) {
      line_count++;
      // The first line is the total number of edges
      if (line_count == 1) {
        continue;
      }

      std::stringstream ss{line};
      std::string item;
      int item_count = 0;
      ID parent_id = 0;
      ID child_id = 0;
      double cost = 0.0;
      // Every line has the form
      // parent_node, child_node, cost
      while (std::getline(ss, item, ',')) {
        switch (item_count) {
          case 0:
            parent_id = stoi(item);
            break;
          case 1:
            child_id = stoi(item);
            break;
          case 2:
            cost = stod(item);
            break;
          default:
            break;
        }

        item_count++;
      }
      neighbors_map_[parent_id].push_back(
          std::make_pair(cost, child_id));
    }

  } else {
    throw std::invalid_argument("Bad node file name");
  }
}

bool Problem::node_valid(ID node_id) {
  try {
    this->get_coord(node_id);

  } catch (...) {
    return false;

  }

  return true;
}

Coord2D Problem::get_coord(ID node_id) {
  if (nodes_coord_.find(node_id) != nodes_coord_.end()) {
    return nodes_coord_[node_id];

  } else {
    throw std::invalid_argument("Bad node_id");
  }

  return {};
}

std::vector<CostAndID> Problem::get_neighbors(ID node_id) {
  if (neighbors_map_.find(node_id) != neighbors_map_.end()) {
    return neighbors_map_[node_id];

  } else {
    throw std::invalid_argument("Bad node_id");
  }

  return {};
}
