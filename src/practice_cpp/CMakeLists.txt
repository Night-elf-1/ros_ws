cmake_minimum_required(VERSION 3.8)
project(practice_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)                  # 添加此功能包下的Include文件夹为头文件目录

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(node01 src/node01.cpp)
ament_target_dependencies(node01 rclcpp)

install(TARGETS
  node01
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(node02 src/node02.cpp)
ament_target_dependencies(node02 rclcpp)

install(TARGETS
  node02
  DESTINATION lib/${PROJECT_NAME}
)