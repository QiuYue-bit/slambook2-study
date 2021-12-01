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

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError Sophus::Sophus ${Pangolin_LIBRARIES})
