/* test_eigen.cpp
 *
 * Author: Chang-Hong Chen
 * Email: longhongc@gmail.com
 */

#include <iostream>

#include <Eigen/Geometry>

int main(int argc, char** argv) {
  Eigen::Translation2d translation(3, 4);
  Eigen::Rotation2D<double> rotation(1.57);

  Eigen::Transform<double, 2, Eigen::Affine> transform;
  // First rotate then translate
  transform = translation * rotation;

  Eigen::Matrix<double, 2, 3> test_data;
  test_data << 1, 2, 1,
               5, 5, 1;
  std::cout << "Transformation matrix" << std::endl;
  std::cout << transform.matrix() << std::endl;
  std::cout << std::endl;

  std::cout << "Test data" << std::endl;
  std::cout << test_data << std::endl;

  // Test transformation matrix
  Eigen::MatrixXd transformed_data =
    transform.matrix() * test_data.transpose();

  std::cout << std::endl;
  std::cout << "Test data after transform" << std::endl;
  std::cout << transformed_data << std::endl;
}
