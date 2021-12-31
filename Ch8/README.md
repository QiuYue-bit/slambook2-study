## 视觉里程计part2 章节程序

### 书上例程

#### 光流法

##### 正向光流

从结果可以看出OPENCV的光流实现效果最好，其次是多层光流和单层光流。

跟踪229个特征点所花费的时间，在2ms左右。

同时可以发现，金字塔的层数与缩放倍率对光流的跟踪效果有影响。

实验结果从好到坏: `4-0.5`>`8-0.5`>`8-0.8`>`4-0.8`.

由于金字塔的Coarse to fine，如果缩放的太厉害，可能会将离得比较远的相似点给匹配上。如果缩放力度不够的话，容易被周围的像素给带偏了，得到错误的结果。

- 多层光流
  - 8层，缩放倍数为0.8
  - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229200941864.png" alt="image-20211229200941864" style="zoom: 50%;" />
  - 4层，缩放倍数为0.8
  - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229201259475.png" alt="image-20211229201259475" style="zoom:50%;" />
  - 4层，缩放倍数为0.5
  - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229201352932.png" alt="image-20211229201352932" style="zoom:50%;" />
  - 8层，缩放倍数为0.5
  - <img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229201538460.png" alt="image-20211229201538460" style="zoom:50%;" />

- 单层光流

<img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229200331344.png" alt="image-20211229200331344" style="zoom:50%;" />

- OPENCV实现的光流

<img src="https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229200312178.png" alt="image-20211229200312178" style="zoom:50%;" />

##### 反向光流

效果也还行，就不对比了。

#### 直接法

单层结果明显不如多层。

- 单层光流

  ![image-20211229212338183](https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229212338183.png)

- 多层光流

  ![image-20211229212254301](https://tuchuang-1998.oss-accelerate.aliyuncs.com/Picgo/image-20211229212254301.png)

### 课后习题



## 参考资料

[opencv::parallel_for_使用说明 - penuel - 博客园 (cnblogs.com)](https://www.cnblogs.com/penuel/p/13410924.html) ps:自己博客的内容也可以参考这个人的~

【Check】把整本书的各个流程计算耗时统计在一张表格上，好对比。不然用特征匹配计算位姿和光流匹配和直接法匹配耗时都没有一个直观的理解。
