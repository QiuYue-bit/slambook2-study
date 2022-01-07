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

add_library(bal_common
        common/common.cpp)

add_executable(BA_Ceres bundle_adjustment_ceres.cpp)
target_link_libraries(BA_Ceres
        ${CERES_LIBRARIES}
        Sophus::Sophus
        bal_common)

add_executable(BA_G2o bundle_adjustment_g2o.cpp)
target_link_libraries(BA_G2o
        ${G2O_LIBS}
        Sophus::Sophus
        bal_common)