cmake_minimum_required(VERSION 3.10)
project(hw1_A-Star)
set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(myLibs
  src/problem.cpp
  src/solver.cpp
)

add_executable(solution src/main.cpp)
target_link_libraries(solution 
  myLibs
)

configure_file(scripts/run_problems.sh run_problems.sh COPYONLY)
file(COPY data DESTINATION ${CMAKE_BINARY_DIR})
