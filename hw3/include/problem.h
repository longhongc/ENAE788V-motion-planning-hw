/* problem.h
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "robot.h"

using Radius = double;
using Coord2D = std::pair<double, double>;
using Circle = std::pair<Coord2D, Radius>;

/**
 * @Brief  A class for constucting the workspace with obstacle
 */
class Problem {
  public:
    Problem(std::string obstacles_file, std::string robot_file);

    /**
     * @Brief  Check whether the coord is in an obstacle
     *
     * @Param coord The target coordinate
     *
     * @Returns  True if coord is collide with obstacle
     */
    bool check_collision(Coord2D coord, double margin = 0);

    /**
     * @Brief  Check whether a line collides with an obstacle
     *
     * @Param coord1 One end of the line
     * @Param coord2 The other end of the line
     * @Param total_points
     *        The number of points that will be checked for obstacle collision
     *
     * @Returns  True if the line is collide with obstacle
     */
    bool check_collision(Coord2D coord1, Coord2D coord2, int total_points = 10);

    /**
     * @Brief  Check whether a trajectory collides with an obstacle
     *
     * @Param trajectory A vector of state. Each state is a waypoint in the trajectory
     *
     * @Returns  True if any waypoints in the trajectory collides with obstacle
     */
    bool check_collision(std::vector<State> trajectory);

    /**
     * @Brief  Check if a coordinate is within the boundary of the workspace
     *
     * @Param coord The target coordinate
     *
     * @Returns  True if the coordinate is within the boundary of the workspace
     */
    bool valid_coord(Coord2D coord);

    /**
     * @Brief  Max limit of workspace x axis
     */
    const double X_MAX = 50;

    /**
     * @Brief  Min limit of workspace x axis
     */
    const double X_MIN = -50;

    /**
     * @Brief  Max limit of workspacew y axis
     */
    const double Y_MAX = 50;

    /**
     * @Brief  Min limit of workspace y axis
     */
    const double Y_MIN = -50;

  private:
    /**
     * @Brief  Load the obstacles from file
     *
     * @Param obstacles_file
     */
    void load_workspace(std::string obstacles_file);

    /**
     * @Brief  Load the robot shape from file
     *
     * @Param robot_file
     */
    void load_robot(std::string robot_file);

    /**
     * @Brief  Stores the center and radius of circular obstacles
     */
    std::vector<Circle> circular_obstacles_;

    /**
     * @Brief  All the points that form the shape of the robot
     *         Every point is stored using homogenious coordinate (x, y, 1)
     *         in a row of this matrix
     */
    Eigen::Matrix<double, Eigen::Dynamic, 3> robot_;


    /**
     * @Brief The maximum length between points in a robot and its center
     */
    double robot_radius_;
};

/**
 * @Brief  Calculate euclidean distance between two 2D coordinate
 *
 * @Param coord1
 * @Param coord2
 *
 * @Returns  Euclidean distance between coord1 and coord2
 */
inline double euclidean_distance(Coord2D coord1, Coord2D coord2) {
  auto& [x1, y1] = coord1;
  auto& [x2, y2] = coord2;
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
