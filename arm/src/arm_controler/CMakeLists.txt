cmake_minimum_required(VERSION 3.8)
project(arm_controler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


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

# 自动查找所有的.cpp文件
file(GLOB_RECURSE SOURCES
  src/*.cpp
)
# 添加头文件搜索路径
include_directories(include)

add_executable(arm_controler ${SOURCES})
ament_target_dependencies(arm_controler 
  rclcpp
  sensor_msgs
  trajectory_msgs
  geometry_msgs
)

# 包含Eigen头文件
target_include_directories(arm_controler PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  arm_controler
  DESTINATION lib/${PROJECT_NAME})

ament_package()
