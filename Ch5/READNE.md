---
lastmod: '2021-12-01T09:08:40.928Z'
---
相机与图像章节的程序

- imageBasics
    - imageBasics.cpp
        - 图片的显示，对图片区域进行遍历。
        - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211201213458295.png" alt="image-20211201213458295" style="zoom:33%;" />
    - undistortImage.cpp
        - 根据内参计算像素(u,v)经过畸变后的点的坐标，再根据畸变后的点的值通过双线性插值算法赋值给(u,v)处的值。
        - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211201215853623.png" alt="image-20211201215853623" style="zoom:33%;" />
    - undistortFullImage.cpp
        - 换一种思路，之前的办法可能会导致原图像相比于真实图像有一定的缩小，损失了图片的信息。因此对原畸变图像的每一个点，求取其去畸变后的坐标，再通过插值算法填补空缺后，得到新的去畸变图像，这样不存在信息损失。
        - - [ ] 中间有一些像素未被填充，考虑使用插值算法进行填充。
        - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211201220002541.png" alt="image-20211201220002541" style="zoom:33%;" />
- rgbd
    - joinMap.cpp
        - 通过给定的5张图片及对应的深度图和图片之间的位姿变换关系，对于每张图片计算像素对应的3D点后，根据位姿转换到统一坐标系下，使用Pangolin绘制点云。
        - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211201221802716.png" alt="image-20211201221802716" style="zoom:33%;" />
- stereo
    - stereoVision.cpp
        - OPENCV库函数SGBM算法生成图像之间的视察，从而根据视察计算出图片公共区域的深度，从而得到三维点云信息，利用pangolin绘制点云。
        - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211201222302392.png" alt="image-20211201222302392" style="zoom: 50%;" />

