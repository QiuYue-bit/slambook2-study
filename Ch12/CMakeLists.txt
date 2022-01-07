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
add_executable(dense_mono dense_mono/dense_mapping.cpp)
target_link_libraries(dense_mono
        Sophus::Sophus
        ${OpenCV_LIBS}
        )

add_executable(surfel_mapping dense_rgbd/surfel_mapping.cpp)
target_link_libraries(surfel_mapping
        ${OpenCV_LIBS} ${PCL_LIBRARIES}
        )

add_executable(pointcloud_mapping dense_rgbd/pointcloud_mapping.cpp)
target_link_libraries(pointcloud_mapping
        ${OpenCV_LIBS} ${PCL_LIBRARIES}
)

add_executable(octomap_mapping dense_rgbd/octomap_mapping.cpp)
target_link_libraries(octomap_mapping
        ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES}
)