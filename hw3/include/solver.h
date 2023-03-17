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
#include "robot.h"

/**
 * @Brief  Storing state information
 */
struct Node {
  Node(State s, std::vector<State> wp, std::shared_ptr<Node> p);

  /**
   * @Brief  The state of this node
   *         State is (x, y, theta, v, w, a, r)
   */
  State state;

  /**
   * @Brief  The waypoints between the parent node and this node
   *         The first waypoint is the next state after parent
   *         The last waypoint is the state of this node
   */
  std::vector<State> waypoints;

  /**
   * @Brief  The parent node of this node
   */
  std::shared_ptr<Node> parent;

  /**
   * @Brief  The coordinate of this node
   */
  Coord2D coord{0.0, 0.0};
};

/**
 * @Brief  A solver class to solve path planning problem using RRT
 */
class Solver {
  public:
    Solver(State start, Coord2D goal,
        double goal_tolerance, double rrt_epsilon,
        std::shared_ptr<Problem> problem_ptr);

    /**
     * @Brief  Run RRT algorithm
     *
     * @Returns  A path from start to goal
     */
    std::vector<State> solve();

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
     *         The search tree contains local trajectory
     *         between nodes
     *
     * @Param file_name
     */
    void save_search_tree_to_file(std::string file_name);

    /**
     * @Brief  Save the rrt nodes to a file
     *
     * @Param file_name
     */
    void save_nodes_to_file(std::string file_name);

    /**
     * @Brief  Time limit for RRT algorithm.
     *         The algorithm fails after this limit.
     */
    const std::chrono::seconds RRT_TIME_LIMIT{30};

    /**
     * @Brief  The resolution length for collision check
     */
    const double RRT_DELTA{0.5};

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
     *         create a node with it if the edge to the node is collision-free
     *
     * @Returns  A new node with collision-free coordinate and waypoints
     */
    std::shared_ptr<Node> sample_node();

    /**
     * @Brief  Generate the path by backtracking from the given node
     *
     * @Param node_ptr
     *
     * @Returns A path containig states from start to node
     */
    std::vector<State> generate_path(std::shared_ptr<Node> node_ptr);

    /**
     * @Brief  The start coordinate
     */
    Coord2D start_coord_{0.0, 0.0};

    /**
     * @Brief  The start state (x, y, theta, v, w, a, r)
     */
    State start_state_;

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
     *         Contains all edges including local trajectory between rrt nodes
     */
    std::vector<std::pair<Coord2D, Coord2D>> edges_;

    /**
     * @Brief  The path from start to goal
     */
    std::vector<State> path_;
};
