cmake_minimum_required(VERSION 3.2)
project(main  LANGUAGES CXX)


############################################################
# Build Type
############################################################
IF(NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
############################################################
#  C/C++ Flags -march=native -march=native
############################################################
# 开启SSE指令集 、开启所有警告、本地优化
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4  -Wall  -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse4  -Wall  -march=native")

message(${CMAKE_CXX_FLAGS})


# -O3 编译等级优化为3上 开启了这个就不能用debug,会丢失信息了
# -Wall 开启所有警告
# -march=native, which automatically detects the features your CPU supports and sets the options appropriately.
if(CMAKE_BUILD_TYPE MATCHES Release)
   MESSAGE("开启编译器优化")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()
########################################s####################
# Check C++14
############################################################
# 设置指定的C++编译器版本是必须的，如果不设置，或者为OFF，则指定版本不可用时，会使用上一版本。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定为C++14 版本
set(CMAKE_CXX_STANDARD 14)
############################################################
# POLICY 参考g2o源码的Cmakelists
############################################################
set(OpenGL_GL_PREFERENCE "GLVND")

set (G2O_LIB_TYPE SHARED)
############################################################
# Cmake Moudule Path
############################################################
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(${CMAKE_MODULE_PATH})

############################################################
# 寻找库并包含路径
############################################################

# Helloslam lib
find_package(hello)
include_directories(${HELLO_INCLUDE_DIR})

# Eigen
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# g2o
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

SET(G2O_LIBS
g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff 
g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse 
)

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_LIBRARIES})

# CSparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# QGLViewer For building the GUI
find_package(QGLViewer)

# OPENGL
find_package(OpenGL)
set(G2O_USE_OPENGL ON CACHE BOOL "Build g2o with OpenGL support for visualization")
if (OPENGL_FOUND AND G2O_USE_OPENGL)
  if (TARGET OpenGL::GL)
    set(G2O_OPENGL_TARGET "OpenGL::GL;OpenGL::GLU")
  else()
    set(G2O_OPENGL_TARGET "${OPENGL_LIBRARIES}")
    include_directories(${OPENGL_INCLUDE_DIR})
  endif()
  set (G2O_HAVE_OPENGL 1)
  message(STATUS "Compiling with OpenGL support")
  #message(WARNING G2O_OPENGL_TARGET=${G2O_OPENGL_TARGET})
endif()

# OPENCV
set(OpenCV_DIR /home/divenire/Divenire_ws/libstore/OPENCV/opencv-3.2.0/build)
find_package(OpenCV REQUIRED)

# Pangolin
find_package(Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS} )

# Sophus依赖
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


############################################################
# 加入子文件夹
############################################################
#add_subdirectory(Ch2)
#add_subdirectory(Ch3)
#add_subdirectory(Ch4)
#add_subdirectory(Ch5)
#add_subdirectory(Ch6)

#TODO
 add_subdirectory(Ch7)
# add_subdirectory(Ch8)
# add_subdirectory(Ch9)
# add_subdirectory(Ch10)

