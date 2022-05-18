add_executable(imageBasics imageBasics.cpp)
# 链接OpenCV库
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})

add_executable(undistortFullImage undistortFullImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortFullImage ${OpenCV_LIBS})

add_executable(image_copy image_copy.cpp)
# 链接OpenCV库
target_link_libraries(image_copy ${OpenCV_LIBS})
