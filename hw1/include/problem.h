/* problem.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <unordered_map>
#include <vector>

using ID = int;
using CostAndID = std::pair<double, int>;
using Coord2D = std::pair<double, double>;

/**
 * @Brief  A class for translating a graph search problem into data structure
 */
class Problem {
  public:
    Problem(std::string nodes_file_name,
            std::string edges_file_name);

    /**
     * @Brief  Check if an ID is a valid ID in the problem
     *
     * @Param node_id
     *
     * @Returns True if node_id is valid
     */
    bool node_valid(ID node_id);

    /**
     * @Brief  Get x, y coordinate of a node
     *
     * @Param node_id
     *
     * @Returns returns a pair(x, y)
     */
    Coord2D get_coord(ID node_id);

    /**
     * @Brief  Get the neighbor nodes of a node
     *
     * @Param node_id
     *
     * @Returns  a vector of pair(cost_to_neighbor, neighbor_id)
     */
    std::vector<CostAndID> get_neighbors(ID node_id);

  private:
    /**
     * @Brief  Load the nodes file into program
     *
     * @Param nodes_file_name
     */
    void load_nodes(std::string& nodes_file_name);

    /**
     * @Brief  Load the edges file into program
     *
     * @Param edges_file_name
     */
    void load_edges(std::string& edges_file_name);

    /**
     * @Brief  All nodes id
     */
    std::vector<ID> nodes_;

    /**
     * @Brief  The 2D coordinate of each node
     */
    std::unordered_map<ID, Coord2D> nodes_coord_;

    /**
     * @Brief  The neighbors info of a node
     *         A neighbor info include (cost_to_neighbor, neighbor_id)
     */
    std::unordered_map<ID, std::vector<CostAndID>> neighbors_map_;
};
