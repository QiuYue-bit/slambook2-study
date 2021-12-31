//
// Created by divenire on 12/31/21.
//

#include "types.h"
#include "common.h"



bool poseEstimationDirect ( const vector< Measurement >& measurements,  cv::Mat* gray, const Eigen::Matrix3d& K, Sophus::SE3d& Tcw );

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: useLK path_to_dataset"<<endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";

    ifstream fin ( associate_file );

    string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    vector<Measurement> measurements;

    Eigen::Matrix3d K;
    K<<fx,0.f,cx,0.f,fy,cy,0.f,0.f,1.0f;

    // 第二帧的位姿
    Sophus::SE3d Tcw = Sophus::SE3d();

    cv::Mat prev_color;
    for ( int index=0; index<10; index++ )
    {
        /// 读取源文件，将彩色图片转换为灰度图
        cout<<"*********** loop "<<index<<" ************"<<endl;
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        color = cv::imread ( path_to_dataset+"/"+rgb_file );
        depth = cv::imread ( path_to_dataset+"/"+depth_file, -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            continue;
        cv::cvtColor ( color, gray, cv::COLOR_BGR2GRAY );

        /// 以第一帧为参考帧，对第一帧图像进行FAST角点提取，并根据深度图转换为对应的3维空间点坐标
        if(index == 0)
        {

            // select the pixels with high gradiants
            // 防止选取到边缘处的点
            for ( int x=10; x<gray.cols-10; x++ )
                for ( int y=10; y<gray.rows-10; y++ )
                {
                    // 简单的采用相邻像素点的亮度差异作为梯度
                    Eigen::Vector2d delta (
                            gray.ptr<uchar>(y)[x+1] - gray.ptr<uchar>(y)[x-1],
                            gray.ptr<uchar>(y+1)[x] - gray.ptr<uchar>(y-1)[x]
                    );
                    // 只对梯度大于50的点进行计算
                    if ( delta.norm() < 50 )
                        continue;

                    ushort d = depth.ptr<ushort> (y)[x];
                    if ( d==0 )
                        continue;
                    Eigen::Vector3d p3d = project2Dto3D ( x, y, d, fx, fy, cx, cy, depth_scale );
                    auto grayscale = float ( gray.ptr<uchar> (y) [x] );

                    // 记录梯度大于50的像素点的3D空间坐标
                    measurements.emplace_back( p3d, grayscale );
                }
            prev_color = color.clone();
            cout<<"add total "<<measurements.size()<<" measurements."<<endl;

        }

        /// 使用直接法计算相机运动
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        poseEstimationDirect ( measurements, &gray, K, Tcw );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
        cout<<"direct method costs time: "<<time_used.count() <<" seconds."<<endl;
        cout<<"Tcw="<<Tcw.matrix() <<endl;

        /// 绘制匹配关系
        cv::Mat img_show ( color.rows, color.cols*2, CV_8UC3 );
        prev_color.copyTo ( img_show ( cv::Rect ( 0,0,color.cols, color.rows ) ) );
        color.copyTo ( img_show ( cv::Rect ( color.cols,0,color.cols, color.rows ) ) );
        for (auto m : measurements)
        {
//            DrawRelevantPointsWithLines(m,img_show,Tcw);
            DrawRelevantPointsWithoutLines(m,img_show,Tcw);
        }

        cv::imshow ( "result", img_show );
        cv::waitKey ( 0 );
    }
    return 0;
}


bool poseEstimationDirect ( const vector< Measurement >& measurements,  cv::Mat* gray, const Eigen::Matrix3d& K, Sophus::SE3d& Tcw )
{
    /// 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    std::unique_ptr< DirectBlock::LinearSolverType> linearSolver ( new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ());

    /// 矩阵块求解器
    std::unique_ptr<DirectBlock> solver_ptr(new DirectBlock ( std::move(linearSolver) ));
    auto* solver(new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) ));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( true );

    /// 添加节点
    auto pose = new VertexSE3();
//    pose->setEstimate(Sophus::SE3d());
    pose->setEstimate(Tcw);
    pose->setId(0);
    optimizer.addVertex(pose);

    /// 添加边
    for (const auto & measurement : measurements)
    {
        auto edge = new EdgeProjection(
                measurement.Get3dcorrdinate(),
                K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ),
                gray
        );
        edge->setVertex(0,pose);
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
        edge->setMeasurement(measurement.Getgrayscale());
        optimizer.addEdge(edge);
    }
    /// 构造矩阵并开始求解
    cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );

    auto *v = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
    Tcw = v->estimate();
    return true;
}
