cmake_minimum_required(VERSION 3.10)
project(arm)

## 2、设置c++编译器的配置 编译器的标准和行为
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL2 REQUIRED)

# 添加 include 目录
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${SDL2_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY(.  SRC_LIST)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src  SRC_LIST)
message(${SRC_LIST})

add_executable(main ${SRC_LIST})
target_link_libraries(main ${SDL2_LIBRARIES})
