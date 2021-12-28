视觉里程计part1 章节程序


- orb_cv.cpp


- 调用Opencv自带的ORB特征检测器，对图片进行特征提取后，使用Hamming距离进行暴力匹配。


    - ![image-20211225195452566](https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211225195452566.png)



- orb_self.cpp


    - Opencv提取Fast角点，利用orbPartten计算质心与角度，使用Hamming距离进行暴力匹配。


    - ![image-20211225195722638](https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211225195722638.png)



- pose_estimation_2d2d.cpp  本质矩阵和单应矩阵估计位姿


    - Opencv提取特征，Hamming距离暴力匹配。采用匹配好的坐标计算本质矩阵以及单应矩阵求解出`R,t`。利用求解出的`R,t`，对对极约束的方程进行验证，绘制本质矩阵`E`(旋转和平移)的误差大于`0.003`匹配结果。


    - 可以看出来误差大于`0.003`的匹配结果的匹配关系也是基本正确的，不存在明显的误匹配。因此可以认为对极约束的精度在`10-3`量级。 


    - 将本质矩阵*1000后分解得到的r,t仍然是正确的。也就是尺度不变性。


    - ![image-20211226104443788](https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211226104443788.png)



- triangulation_cv.cpp 三角化计算特征点的深度。


- pose_estimation_3d2d.cpp 使用pnp及优化算法求解位姿。


    - 首先对两张图像进行特征匹配，得到3D-2D匹配关系，一共67组3D-2D匹配点。分别调用OPENCV的`Epnp`,高斯牛顿pose BA,以及g2o进行求解。


    - 高斯牛顿法迭代5次收敛，耗时0.07ms。


    - OpenCV调用Epnp算法，耗时0.2816ms.


    - g2o使用高斯牛顿求解ba问题，耗时0.26ms.


    - 最后得到的结果几乎相同，都在1ms内，说明3D-2D位姿估计并不是很耗时。



- pose_estimation_3d3d.cpp 使用ICP及优化算法求解位姿。


    - 特征匹配得到79组3D-3D特征点。通过ICP求解位姿估计问题以及pose-BA求解。


    - ICP求解耗时0.1ms，BA求解迭代8次耗时0.4ms。



  - g2o_ba_example.cpp


    - 根据两张图片以及对应的特征匹配关系，得到初始三维地图点(深度未知)。用三维地图点和两帧图像的位姿做一个Full BA.并查看内点数量。
    
      ![image-20211225193051782](https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211225193051782.png)



    - 匹配得到150组特征点，因此有300条边，以优化后的重投影误差1个像素为边界，最终得到243个内点。通过重投影误差可以很容易的剔除掉明显的误匹配。






- [ ] 从几何上理解对极约束的误差。
- [ ] 本质矩阵和系数矩阵需要进一步的学习。比如求解E的时候，系数矩阵满秩，特征点处于什么情况下退化，不满秩，等等。
