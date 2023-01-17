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

add_library(priority_executor  src/priority_executor.cpp src/priority_memory_strategy.cpp)
target_include_directories(priority_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# target_link_libraries(priority_executor
#  simple_timer
# )
ament_target_dependencies(priority_executor
  rmw
  rclcpp
  rcl
  simple_timer
)

add_executable(usage_example src/usage_example.cpp)
target_include_directories(usage_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(usage_example
  priority_executor
)
ament_target_dependencies(usage_example
  rclcpp
  std_msgs
  std_srvs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS usage_example
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS priority_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
  
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

ament_export_include_directories(include)
ament_export_libraries(priority_executor)
ament_package()
