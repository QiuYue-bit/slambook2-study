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
add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method  ${OpenCV_LIBS}  Sophus::Sophus)


add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow  ${OpenCV_LIBS})


add_executable(direct_sparse_g2o hw_04/direct_sparse_g2o.cpp)
target_link_libraries(direct_sparse_g2o  ${OpenCV_LIBS}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
        Sophus::Sophus)

add_executable(direct_semidense_g2o hw_04/direct_semidense_g2o.cpp)
target_link_libraries(direct_semidense_g2o  ${OpenCV_LIBS}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
        Sophus::Sophus)