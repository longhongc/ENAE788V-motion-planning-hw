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

#include "problem.h"

Problem::Problem(
    std::string obstacles_file,
    std::string robot_file) {
  this->load_workspace(obstacles_file);
  this->load_robot(robot_file);
  std::cout << "\033[3mLoading complete\033[0m" << std::endl;
}

void Problem::load_workspace(std::string obstacles_file) {
  std::cout << "\033[3mLoading " <<
    obstacles_file << "\033[0m" << std::endl;

  std::ifstream file{obstacles_file};
  std::string line;
  int line_count = 0;

  if (file.is_open()) {
    while (std::getline(file, line)) {
      line_count++;
      // The first line is the total number of obstacles
      if (line_count == 1) {
        continue;
      }

      std::stringstream ss{line};
      std::string item;
      int item_count = 0;
      double x = 0.0;
      double y = 0.0;
      Radius radius = 0.0;
      // Every line has the form
      // x, y, radius
      while (std::getline(ss, item, ',')) {
        switch (item_count) {
          case 0:
            x = stod(item);
            break;
          case 1:
            y = stod(item);
            break;
          case 2:
            radius = stod(item);
            break;
          default:
            break;
        }

        item_count++;
      }

      Circle circle{std::make_pair(x, y), radius};

      circular_obstacles_.push_back(circle);
    }

  } else {
    throw std::invalid_argument("Bad node file name");
  }
}

void Problem::load_robot(std::string robot_file) {
  std::cout << "\033[3mLoading " <<
    robot_file << "\033[0m" << std::endl;

  std::ifstream file{robot_file};
  std::string line;
  std::vector<Coord2D> robot_vec;

  if (file.is_open()) {
    while (std::getline(file, line)) {
      std::stringstream ss{line};
      std::string item;
      int item_count = 0;
      double x = 0.0;
      double y = 0.0;
      // Every line has the form
      // x, y
      while (std::getline(ss, item, ',')) {
        switch (item_count) {
          case 0:
            x = stod(item);
            break;
          case 1:
            y = stod(item);
            break;
          default:
            break;
        }

        item_count++;
      }

      robot_vec.emplace_back(std::make_pair(x, y));
    }

    // Resize the matrix
    robot_.resize(robot_vec.size(), 3);

    double max_radius = 0.0;
    Coord2D center{0.0, 0.0};
    for (size_t i=0; i < robot_vec.size(); ++i) {
      auto [x, y] = robot_vec[i];

      Coord2D curr_coord{x, y};
      // Calculate the maximum radius of the robot
      double dist = euclidean_distance(center, curr_coord);
      if (dist > max_radius) {
        max_radius = dist;
      }

      robot_.row(i) = Eigen::RowVector3d(x, y, 1);
    }

    robot_radius_ = max_radius;

  } else {
    throw std::invalid_argument("Bad node file name");
  }
}

bool Problem::check_collision(Coord2D coord, double margin) {
  for (auto& obstacle : circular_obstacles_) {
    auto& [center, r] = obstacle;
    if (euclidean_distance(coord, center) <= r + margin) {
      return true;
    }
  }

  return false;
}

bool Problem::check_collision(
        Coord2D coord1, Coord2D coord2, int total_points) {
  if (total_points < 2) {
    throw std::invalid_argument(
        "Line collision checking requires at least 2 points");
  }

  auto& [x1, y1] = coord1;
  auto& [x2, y2] = coord2;

  // Do interpolation between coord1 and coord2
  double x_step = (x2 - x1) / (total_points - 1);
  double y_step = (y2 - y1) / (total_points - 1);

  for (int i=0; i < total_points; ++i) {
    Coord2D curr_coord =
      std::make_pair(x1 + x_step * i, y1 + y_step * i);
    // Check each of the interpolated coordinates
    if (this->check_collision(curr_coord)) {
      return true;
    }
  }

  return false;
}

bool Problem::check_collision(std::vector<State> trajectory) {
  for (auto& state : trajectory) {
    double x = state.get_value()[0];
    double y = state.get_value()[1];
    double theta = state.get_value()[2];

    // If the center of the robot is far away from all obstacles with some margin,
    // then only the center will be checked.
    // Otherwise, all robot points in all waypoints inside the trajectory will have
    // collsion check.
    Coord2D center{x, y};
    if (!this->check_collision(center, robot_radius_ * 1.2)) {
      continue;
    }
    Eigen::Translation2d translation(x, y);
    Eigen::Rotation2D<double> rotation(theta);

    Eigen::Transform<double, 2, Eigen::Affine> transform;
    transform = translation * rotation;

    Eigen::MatrixXd transformed_robot =
      (transform.matrix() * robot_.transpose()).transpose();

    for (int i = 0; i < robot_.rows(); ++i) {
      double tranformed_x = transformed_robot.row(i)(0);
      double tranformed_y = transformed_robot.row(i)(1);
      Coord2D transformed_coord{tranformed_x, tranformed_y};
      if (this->check_collision(transformed_coord)) {
        return true;
      }
    }
  }

  return false;
}

bool Problem::valid_coord(Coord2D coord) {
  auto& [x, y] = coord;
  if (x < X_MIN || x > X_MAX ||
      y < Y_MIN || y > Y_MAX) {
    return false;
  }

  return true;
}
