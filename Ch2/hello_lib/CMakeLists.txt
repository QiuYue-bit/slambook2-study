# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(hello)

# 设置编译模式
set(CMAKE_BUILD_TYPE Release)

############################################################
# Create a library
############################################################
# 静态库
# 添加hello库
# add_library(hello STATIC 
#     ${PROJECT_SOURCE_DIR}/src/libHelloSLAM.cpp
# )

# target_include_directories(hello
#     PUBLIC 
#     ${PROJECT_SOURCE_DIR}/inc
# )

# 动态库
#Generate the shared library from the library sources
add_library(hello SHARED 
    src/libHelloSLAM.cpp
)

target_include_directories(hello
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/inc
)

############################################################
# Install
############################################################
# Library
# Note: may not work on windows
install (TARGETS hello
    LIBRARY DESTINATION lib)

# Header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/inc/ 
    DESTINATION include)
