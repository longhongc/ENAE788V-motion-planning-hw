cmake_minimum_required(VERSION 3.10)
project(hw3_RRT_with_dynamic)
set(CMAKE_CXX_STANDARD 17)

find_package(drake CONFIG REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(myLibs
  src/problem.cpp
  src/solver.cpp
)
target_link_libraries(myLibs Eigen3::Eigen)

add_library(robotLibs
  src/robot.cpp
)
target_link_libraries(robotLibs 
  drake::drake
)

target_link_libraries(myLibs robotLibs)

add_executable(solution src/main.cpp)
target_link_libraries(solution 
  myLibs
)

add_executable(test_robot test/test_robot.cpp)
target_link_libraries(test_robot 
  robotLibs
)

add_executable(test_problem test/test_problem.cpp)
target_link_libraries(test_problem 
  myLibs
)

add_executable(test_eigen test/test_eigen.cpp)
target_link_libraries(test_eigen 
  Eigen3::Eigen
)

set_target_properties(
  test_robot
  test_problem
  test_eigen
  PROPERTIES 
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

configure_file(scripts/run_problems.sh run_problems.sh COPYONLY)
configure_file(scripts/draw_rrt.py draw_rrt.py COPYONLY)
configure_file(scripts/draw_rrt_movie.py draw_rrt_movie.py COPYONLY)
file(COPY data DESTINATION ${CMAKE_BINARY_DIR})
