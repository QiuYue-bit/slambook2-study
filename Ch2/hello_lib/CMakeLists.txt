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

###########################################################
# Create a library
############################################################
# 静态库
# 添加hello库
# add_library(hello STATIC 
#     ${PROJECT_SOURCE_DIR}/src/libHelloSLAM.cpp
# )

# target_include_directories(hello
#     PUBLIC 
#     ${PROJECT_SOURCE_DIR}/inc
# )

# 动态库
#Generate the shared library from the library sources
add_library(hello SHARED 
    src/libHelloSLAM.cpp
)

target_include_directories(hello
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/inc
)

############################################################
# Install
############################################################
# Library
# Note: may not work on windows
install (TARGETS hello
    LIBRARY DESTINATION lib)

# Header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/inc/ 
    DESTINATION include)
