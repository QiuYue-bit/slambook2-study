cmake_minimum_required(VERSION 3.1)
project(Ch7)

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

# 开启指令集
set(CMAKE_CXX_FLAGS " ${SSE_FLAGS} -msse4")

# 使用传统的OPENGL 防止出警告
if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
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


find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
############################################################
# 输出可执行文件
############################################################
## 二进制文件输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/exe)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(triangulation_cv triangulation_cv.cpp)
target_link_libraries(triangulation_cv  ${OpenCV_LIBS})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d  ${OpenCV_LIBS} g2o::core Sophus::Sophus)

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d  ${OpenCV_LIBS} g2o::core Sophus::Sophus)

add_executable(g2o_ba_example g2o_ba_example.cpp)
target_link_libraries(g2o_ba_example 
                g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
                ${OpenCV_LIBS}  
                ${CHOLMOD_LIBRARIES}
)
