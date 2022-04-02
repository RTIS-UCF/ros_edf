cmake_minimum_required(VERSION 3.5)
project(priority_executor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(simple_timer REQUIRED)

add_library(priority_executor src/priority_executor.cpp)
target_include_directories(priority_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(priority_executor
  rmw
  rclcpp
  rcl
  simple_timer 
)

add_library(test_nodes src/test_nodes.cpp)
target_include_directories(test_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(test_nodes
  rmw
  rclcpp
  rcl
  simple_timer
  std_msgs
)
target_link_libraries(test_nodes
  primes_workload
)

add_library(primes_workload src/primes_workload.cpp)
target_include_directories(primes_workload PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(primes_workload
  simple_timer
)


add_executable(test_publisher src/test_publisher.cpp)
target_include_directories(test_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(test_publisher
  priority_executor
  test_nodes
  default_executor
)
ament_target_dependencies(
  test_publisher
  rclcpp
  std_msgs
  std_srvs
  simple_timer 
)

install(TARGETS test_publisher priority_executor
  DESTINATION lib/${PROJECT_NAME})

add_library(default_executor src/default_executor.cpp)
target_include_directories(default_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(default_executor
  rclcpp
  simple_timer
)
target_link_libraries(default_executor
  primes_workload
)

add_executable(f1tenth_publisher src/f1tenth_test.cpp)
target_include_directories(test_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(f1tenth_publisher
  priority_executor
  test_nodes
  default_executor
)
ament_target_dependencies(f1tenth_publisher
  rclcpp
  std_msgs
  std_srvs
  simple_timer 
)

add_executable(usage_example src/usage_example.cpp)
target_include_directories(usage_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(usage_example
  priority_executor
  test_nodes
  default_executor
)
ament_target_dependencies(usage_example
  rclcpp
  std_msgs
  std_srvs
  simple_timer 
)

install(TARGETS f1tenth_publisher priority_executor usage_example
  DESTINATION lib/${PROJECT_NAME})

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
