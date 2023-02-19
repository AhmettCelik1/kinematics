cmake_minimum_required(VERSION 3.0)
project(robot_kinematics)

# Add the Eigen library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Add the tf library
find_package(tf REQUIRED)
include_directories(${tf_INCLUDE_DIRS})

# Add the executable
add_executable(robot_kinematics main.cpp)

# Link the libraries
target_link_libraries(robot_kinematics ${tf_LIBRARIES})
