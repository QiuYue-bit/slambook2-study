cmake_minimum_required(VERSION 3.1)
project(Ch6)

############################################################
# Build Type
############################################################
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
############################################################
#  C/C++ Flags
############################################################
# -O3 编译等级优化为3上 开启了这个就不能用debug,会丢失信息了
# -Wall 开启所有警告
# -march=native, which automatically detects the features your CPU supports and sets the options appropriately.
if(CMAKE_BUILD_TYPE MATCHES Release)
   MESSAGE("开启编译器优化")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
endif()
########################################s####################
# Check C++14
############################################################
# 设置指定的C++编译器版本是必须的，如果不设置，或者为OFF，则指定版本不可用时，会使用上一版本。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定为C++14 版本
set(CMAKE_CXX_STANDARD 14)
############################################################
# Cmake Moudule Path
############################################################
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(${CMAKE_MODULE_PATH})

############################################################
# 寻找库并包含路径
############################################################
#Eigen
include_directories(/home/divenire/Divenire_ws/libsStore/EIGEN/eigen-3.3.8/)

# OPENCV
set(OpenCV_DIR "/home/divenire/Divenire_ws/libsStore/OPENCV/4.2.0/opencv-4.2.0/build")
find_package(OpenCV  REQUIRED)

# 添加Pangolin依赖
find_package( Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS} )
message("Pangolin_INCLUDE_DIRS dir is " ${Pangolin_INCLUDE_DIRS})

# Sophus依赖
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Ceres依赖
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# g2o依赖
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
message("g2o is "${G2O_INCLUDE_DIRS})

############################################################
# 输出可执行文件
############################################################
## 二进制文件输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/exe)

add_executable(GN gaussNewton.cpp)
target_link_libraries(GN ${OpenCV_LIBS})


add_executable(ceresCurveFittingAutoDiff ceresCurveFittingAutoDiff.cpp)
target_link_libraries(ceresCurveFittingAutoDiff ${OpenCV_LIBS} Ceres::ceres)

add_executable(ceresCurveFittingAnalyticDiff ceresCurveFittingAnalyticDiff.cpp)
target_link_libraries(ceresCurveFittingAnalyticDiff ${OpenCV_LIBS} Ceres::ceres)

add_executable(ceresCurveFittingNumericDiff ceresCurveFittingNumericDiff.cpp)
target_link_libraries(ceresCurveFittingNumericDiff ${OpenCV_LIBS} Ceres::ceres)

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o::core g2o::solver_eigen)