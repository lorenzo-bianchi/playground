cmake_minimum_required(VERSION 3.8)
project(voronoi_planner)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(params_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/voronoi_planner/params.yaml"
  OUT_FILE "init_parameters.cpp")

add_executable(voronoi_planner
  src/voronoi_planner/voronoi_planner.cpp
  src/voronoi_planner/voronoi_planner_utils.cpp
  init_parameters.cpp
  src/voronoi_planner_app.cpp)

target_link_libraries(voronoi_planner
  Eigen3::Eigen
  qhullcpp
  qhull_r)

target_include_directories(voronoi_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  voronoi_planner
  dua_app_management
  dua_node
  dua_qos
  Eigen3
  params_manager
  rclcpp
  rclcpp_components)

# voronoi_planner app
install(TARGETS voronoi_planner
  DESTINATION lib/${PROJECT_NAME})

# Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
