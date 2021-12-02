############################################################
# 输出可执行文件目录
############################################################
# 二进制文件输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
# 设定库文件编译路径
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

############################################################
# 生成执行文件并链接
############################################################
add_executable(GnCurveFitting manual/GnCurveFitting.cpp)
target_link_libraries(GnCurveFitting ${OpenCV_LIBS})


add_executable(ceresCurveFittingAutoDiff ceres/ceresCurveFittingAutoDiff.cpp)
target_link_libraries(ceresCurveFittingAutoDiff ${OpenCV_LIBS} Ceres::ceres)

add_executable(ceresCurveFittingAnalyticDiff ceres/ceresCurveFittingAnalyticDiff.cpp)
target_link_libraries(ceresCurveFittingAnalyticDiff ${OpenCV_LIBS} Ceres::ceres)

add_executable(ceresCurveFittingNumericDiff ceres/ceresCurveFittingNumericDiff.cpp)
target_link_libraries(ceresCurveFittingNumericDiff ${OpenCV_LIBS} Ceres::ceres)

add_executable(g2oCurveFitting g2o/g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} g2o::core g2o::solver_eigen g2o_stuff)