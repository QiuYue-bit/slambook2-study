/*
 * @Author: Xiang Gao
 * @Date: 2021-10-19 14:59:29
 * @LastEditors: Divenire
 * @LastEditTime: 2021-10-20 13:49:48
 * @Description: 在这个程序中，我们读取两张图像，进行特征匹配。然后根据匹配得到的特征，计算相机运动以及特征点的位置。
 *               这是一个典型的Bundle Adjustment，我们用g2o进行优化。
 *              参考 https://github.com/gaoxiang12/g2o_ba_example
 *                  https://www.cnblogs.com/gaoxiang12/p/5304272.html
 *
 */


// for std
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


using namespace std;
using namespace cv;

// 寻找两个图像中的对应点，像素坐标系
// 输入：img1, img2 两张图像
// 输出：points1, points2, 两组对应的2D点
int findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 );

// 相机内参
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;


string img1_path = "../data/1.png";
string img2_path = "../data/2.png";

int main( int argc, char** argv )
{

    // 读取图像
    cv::Mat img1 = cv::imread( img1_path ); 
    cv::Mat img2 = cv::imread( img2_path ); 
    
    // 寻找两幅图的特征点与匹配关系
    vector<cv::Point2f> pts1, pts2;
    if ( findCorrespondingPoints( img1, img2, pts1, pts2 ) == false )
    {
        cout<<"匹配点不够！"<<endl;
        return 0;
    }else

    cout<<"找到了"<<pts1.size()<<"组对应特征点。"<<endl;

    // 构造g2o中的图
    // 使用Cholmod中的线性方程求解器
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver (new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ());

    // 矩阵块求解器
    std::unique_ptr<g2o::BlockSolver_6_3> block_solver(new g2o::BlockSolver_6_3 (std::move(linearSolver)));

    // L-M 下降
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( std::move(block_solver));

    // 求解器
    g2o::SparseOptimizer    optimizer;
    optimizer.setAlgorithm( algorithm );
    // 打开可视化
    optimizer.setVerbose( true );
    
    // 添加节点
    // 两个位姿节点
    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
    
    // 很多个特征点的节点
    // 以第一帧为准
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );

        // mark as landmark
        v->setMarginalized(true);

        // mappoint initial value
        // 由于深度不知道，只能把深度设置为1了
        double z = 1;
        double x = ( pts1[i].x - cx ) * z / fx; 
        double y = ( pts1[i].y - cy ) * z / fy; 

        v->setEstimate( Eigen::Vector3d(x,y,z) );
        optimizer.addVertex( v );
    }
    
    // 计算edge误差的时候，需要用到相机参数，现在这儿定义了
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );
    
    // 准备边
    // 第一帧
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        // i:
        // type
        // vertex id
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );

        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );

        edge->setInformation( Eigen::Matrix2d::Identity() );

        edge->setParameterId(0, 0);

        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    
    // 第二帧
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    
    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout<<"优化完毕"<<endl;
    
    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;
    
    // 以及所有特征点的位置
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }
    
    // 估计inlier的个数
    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 )
        {
            cout<<"error = "<<e->chi2()<<endl;
        }
        else 
        {
            inliers++;
        }
    }
    
    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
    optimizer.save("ba.g2o");
    return 0;
}


int  findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 )
{

    //-- 初始化
    vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desp1, desp2;

    Ptr<FeatureDetector> detector = ORB::create(); //ORB检测器
    Ptr<DescriptorExtractor> descriptor = ORB::create(); //ORB描述子

    // 计时开始
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img1, kp1);
    detector->detect(img2, kp2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子并存储在对应的容器中
    descriptor->compute(img1, kp1, desp1);
    descriptor->compute(img2, kp2, desp2);


    cout<<"分别找到了"<<kp1.size()<<"和"<<kp2.size()<<"个特征点"<<endl;
    
    cv::Ptr<cv::DescriptorMatcher>  matcher = cv::DescriptorMatcher::create( "BruteForce-Hamming");
    
    double knn_match_ratio=0.8;
    vector< vector<cv::DMatch> > matches_knn;
    matcher->knnMatch( desp1, desp2, matches_knn, 2 );
    vector< cv::DMatch > matches;
    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
            matches.push_back( matches_knn[i][0] );
    }
    
    if (matches.size() <= 20) //匹配点太少
        return false;
    
    for ( auto m:matches )
    {
        points1.push_back( kp1[m.queryIdx].pt );
        points2.push_back( kp2[m.trainIdx].pt );
    }

    cv::Mat outImg;
    cv::drawMatches(img1,kp1,img2,kp2,matches,outImg,Scalar(255, 255, 255),Scalar(0, 0, 255));
    cv::imshow("matches",outImg);
    cv::waitKey();
    return true;
}