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
add_executable(pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp)
target_link_libraries(pose_graph_g2o_lie
        ${G2O_LIBS}
        ${CHOLMOD_LIBRARIES}
        Sophus::Sophus
        )

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
        ${G2O_LIBS}
        ${CHOLMOD_LIBRARIES}
        Sophus::Sophus
        )
