---
lastmod: '2021-12-02T07:09:09.175Z'
---
非线性优化章节的程序

- g2o
    - g2oCurveFitting.cpp
        - 使用g2o进行曲线拟合,查看不同的迭代算法与鲁棒核函数对结果的影响。
        
            #不采用鲁棒核函数
            LM算法：
            solve time cost = 0.35 ms.
            true model: a:1 b:2     c:1
            estimated model: a:0.890912     b:2.1719        c:0.943629
            GN算法：
            solve time cost = 0.27 ms.
            true model: a:1 b:2     c:1
            estimated model: a:0.890912     b:2.1719        c:0.943629
            Dog-leg算法：
            solve time cost = 0.18 ms. 
            true model: a:1 b:2     c:1
            estimated model: a:0.890912     b:2.1719        c:0.943629
            
            #采用Huber核函数
            true model: a:1	b:2	c:1
            estimated model: a:0.948334	b:2.10234	c:0.961749
        
        - 可以看出，鲁棒核函数在这种凸的问题上依然有比较好的作用，但是不同的迭代算法结果都一样，对于凸优化问题能力都可以，但是LM算法比Dog-leg算法耗时更久。

- ceres
    - ceresCurveFittingAnalyticDiff.cpp 解析求导
    
      - 耗时0.3ms。`estimated a,b,c = 0.890908 2.1719 0.943628`
    
    - ceresCurveFittingAutoDiff.cpp 自动求导
    
      - 耗时0.28ms。`estimated a,b,c = 0.890908 2.1719 0.943628 `
    
    - ceresCurveFittingNumericDiff.cpp 数值求导
    
      - 耗时0.4ms。`estimated a,b,c = 0.890908 2.1719 0.943628`
    
      对于Ceres来说，问题比较简单，没有使用鲁棒核函数，得到的结果与g2o相近。但是耗时比g2o少了许多。
    
- manual
    - GnCurveFitting.cpp
        - 解析给出导数表达式，使用高斯牛顿算法迭代。
        - 耗时 0.13ms,`estimated model: a:0.890912	b:2.1719	c:0.943629`.

结论基本是手动>g2o>ceres。

