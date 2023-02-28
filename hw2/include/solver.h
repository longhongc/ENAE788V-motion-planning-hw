/* solver.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "problem.h"

/**
 * @Brief  Storing state information
 */
struct Node {
  Node(Coord2D c, std::shared_ptr<Node> p);

  /**
   * @Brief  The coordinate of this node
   */
  Coord2D coord{0.0, 0.0};

  /**
   * @Brief  The parent node of this node
   */
  std::shared_ptr<Node> parent;
};

/**
 * @Brief  A solver class to solve path planning problem using RRT
 */
class Solver {
  public:
    Solver(Coord2D start, Coord2D goal,
        double goal_tolerance, double rrt_epsilon,
        std::shared_ptr<Problem> problem_ptr);

    /**
     * @Brief  Run RRT algorithm
     *
     * @Returns  A path from start to goal
     */
    std::vector<Coord2D> solve();

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

    /**
     * @Brief  Time limit for RRT algorithm.
     *         The algorithm fails after this limit.
     */
    const std::chrono::seconds RRT_TIME_LIMIT{5};

  private:
    /**
     * @Brief  Find the nearest node in the tree to the target coordordinate
     *
     * @Param coord The target coordinate
     *
     * @Returns  The nearest node
     */
    std::shared_ptr<Node> find_nearest_node(Coord2D coord);

    /**
     * @Brief  Sample a random coordinate in the workspace and
     *         create a node with it if the coordinate is collision-free
     *
     * @Returns  A new node with a collision-free coordinate
     */
    std::shared_ptr<Node> sample_node();

    /**
     * @Brief  Generate the path by backtracking from the given node
     *
     * @Param node_ptr
     *
     * @Returns A path containig 2D coordinates from start to node
     */
    std::vector<Coord2D> generate_path(std::shared_ptr<Node> node_ptr);

    /**
     * @Brief  The start coordinate
     */
    Coord2D start_coord_{0.0, 0.0};

    /**
     * @Brief  The goal coordinate
     */
    Coord2D goal_coord_{0.0, 0.0};

    /**
     * @Brief  Tolerance to the goal coordinate
     *         Node will be treated as arrived-to-goal
     *         if having a distance smaller than this tolerance
     */
    double goal_tolerance_ = 0.0;

    /**
     * @Brief  The epsilon value for RRT algorithm
     */
    double rrt_epsilon_ = 0.0;

    /**
     * @Brief  For accessing workspace information such as collision check
     */
    std::shared_ptr<Problem> problem_ptr_;

    /**
     * @Brief  Searched nodes (The RRT itself)
     */
    std::vector<std::shared_ptr<Node>> nodes_;

    /**
     * @Brief  Edges in the RRT
     */
    std::vector<std::pair<Coord2D, Coord2D>> edges_;

    /**
     * @Brief  The path from start to goal
     */
    std::vector<Coord2D> path_;
};
