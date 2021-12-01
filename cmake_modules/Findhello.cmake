# 在指定目录下寻找头文件和动态库文件的位置，可以指定多个目标路径
# Find the header files
FIND_PATH(HELLO_INCLUDE_DIR 
NAMES libHelloSLAM.h
PATHS /usr/local/include
      /usr/include
)

# Find lib 
FIND_LIBRARY(HELLO_LIBRARY
NAMES hello
PATHS /usr/local/lib
      /usr/lib
)

if (HELLO_INCLUDE_DIR AND HELLO_LIBRARY)
    set(HELLO_FOUND TRUE)
endif (HELLO_INCLUDE_DIR AND HELLO_LIBRARY)