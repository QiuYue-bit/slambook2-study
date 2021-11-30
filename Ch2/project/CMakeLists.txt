# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(HelloSlam)

# 设置编译模式
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# 寻找库文件
find_package(hello REQUIRED)
include_directories(${HELLO_INCLUDE_DIR})

# 添加可执行文件
add_executable(HelloSlam useHello.cpp)
target_link_libraries(${PROJECT_NAME}   ${HELLO_LIBRARY})


message("include dir:${HELLO_INCLUDE_DIR}  lib dir:${HELLO_LIBRARY} ")