/* solver.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <functional>
#include <queue>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "problem.h"

/**
 * @Brief  Structure for representing a node
 */
struct Node {
  ID id = 0;
  ID parent_id = 0;
  double cost_to_start = 0.0;
};

/**
 * @Brief  A solver class to solve a graph serach problem with A*
 */
class Solver {
  public:
    Solver(std::shared_ptr<Problem> problem_ptr,
           ID start_id, ID goal_id);

    /**
     * @Brief  Run A* algorithm
     *
     * @Returns A path of node_id
     */
    std::vector<ID> solve();

    /**
     * @Brief  Print out the path
     */
    void print_path();

    /**
     * @Brief  Save the path to a file
     *
     * @Param file_name
     */
    void save_path_to_file(std::string file_name);

    /**
     * @Brief  Save the search tree to a file
     *
     * @Param file_name
     */
    void save_search_tree_to_file(std::string file_name);

  private:
    /**
     * @Brief  Euclidean heuristic for A*
     *
     * @Param node_1
     * @Param node_2
     *
     * @Returns The euclidean distance between node_1 and node_2
     */
    double heuristic(ID node_1, ID node_2);

    /**
     * @Brief  Generate the path by backtracking from the given node_id
     *
     * @Param node_id
     *
     * @Returns A path containig nodes id that start from start_id to given node_id
     */
    std::vector<ID> generate_path(ID node_id);

    /**
     * @Brief  For accessing Problem specific data
     */
    std::shared_ptr<Problem> problem_ptr_;
    ID start_id_ = 0;
    ID goal_id_ = 0;

    /**
     * @Brief  A priority queue for searching in A*
     *         This is a min priority queue
     */
    std::priority_queue<CostAndID,
      std::vector<CostAndID>,
      std::greater<CostAndID>> node_queue_;

    /**
     * @Brief  A dictionary to find node by node id
     */
    std::unordered_map<ID, Node> node_map_;

    /**
     * @Brief  Visited node
     */
    std::unordered_set<ID> visited_;

    /**
     * @Brief  The path last generated
     */
    std::vector<ID> path_;
};
