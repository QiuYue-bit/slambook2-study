#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

// 一般编程中都需要先检查一个条件才进入等待环节，因此在中间有一个检查时段，检查条件的时候是不安全的，需要lock
namespace myslam
{

    Backend::Backend()
    {
        backend_running_.store(true);

        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this)); //类成员函数需要绑定该类的指针
        // bind函数的用法和详细参考： https://www.cnblogs.com/jialin0x7c9/p/12219239.html
        // this指针的用法和详解参考： http://c.biancheng.net/view/2226.html
    }

    void Backend::UpdateMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex_); //没有defer_lock的话创建就会自动上锁了
        // TODO 这句话的作用？
        map_update_.notify_one(); //随机唤醒一个wait的线程
    }

    void Backend::Stop()
    {
        backend_running_.store(false); // replace the contained value with "parameter" 这里的parameter就是false
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop()
    {
        // backend_running_是一个原子类型
        // load就是读取整个原子类型的值
        while (backend_running_.load())
        {
            // 上锁
            std::unique_lock<std::mutex> lock(data_mutex_);

            // TODO 这个wait怎么用？ 查一下

            // wait():一般编程中都需要先检查一个条件才进入等待环节，因此在中间有一个检查时段，检查条件的时候是不安全的，需要lock
            //被notify_one唤醒后，wait() 函数也会自动调用 data_mutex_.lock()，使得data_mutex_恢复到上锁状态
            // 是不是幻想一次,一次继续上锁?
            // !不太懂这个锁的操作
            map_update_.wait(lock);

            // 后端仅优化激活的Frames和Landmarks
            // 把当前窗口内的地图点和关键帧取出来
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();

            // 对当前窗口内的地图点和关键帧进行优化
            Optimize(active_kfs, active_landmarks);
        }
    }

    void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
    {

        // 块求解器 6维pose 3维landmark
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> block; //对于二元边来说，这里的6,3是两个顶点的维度

        // TODO LinearSolverCSparse和Dense 什么情况下用什么?
        // 增量方程Hx=b的求解方法 Dense cholesky分解法
        typedef g2o::LinearSolverCSparse<block::PoseMatrixType> LinearSolverType;
        // 使用LM算法
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<block>(g2o::make_unique<LinearSolverType>()));
        //创建稀疏优化器
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 调试信息输出到命令行
        optimizer.setVerbose(true);

        // Pose顶点
        std::map<unsigned long, VertexPose *> vertices;

        // Landmark顶点
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        // 每个特征点对应的误差边
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        // 存储最大的关键帧ID号
        // ? 虽然我认为没有这个必要,如果仅仅是为了给g2o的话
        // 也可能是调试用的
        unsigned long max_kf_id = 0;

        // 把滑动窗口里的关键帧取出来
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;

            // 位姿节点初始化,初值和ID号
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());

            // 求解器中添加关键帧的位姿顶点
            optimizer.addVertex(vertex_pose);

            if (kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }

            // 建立关键帧ID号和位姿节点的对应关系
            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 内参和左右目的位姿
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // 误差边的ID号
        int index = 1;

        // 卡方检验阈值95%置信度,2自由度
        double chi2_th = 5.991;

        // 对于每一个地图点
        for (auto &landmark : landmarks)
        {
            // 如果这个地图点是外点
            // ? 在哪里判断??
            if (landmark.second->is_outlier_)
                continue;

            unsigned long landmark_id = landmark.second->id_;

            // 把地图点的观测取出来,也就是特征点
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations)
            {
                // ?会发生这样的事情?
                // ?mappoint.cpp 第43行的erase有没有这个效果?
                if (obs.lock() == nullptr)
                {
                    continue;
                    LOG(ERROR) << "obs is nullptr";
                }

                // 拿到一个特征点的指针
                auto feat = obs.lock();

                // ? 为什么会发生这样的事情
                // 这个外点都在哪些地方设置了
                // 判断一下这个特征是不是外点 且对应的帧指针是不是空
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr)
                    continue;

                // 把特征所处的frame指针也拿出来
                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;

                // 误差边
                if (feat->is_on_left_image_)
                {
                    edge = new EdgeProjection(K, left_ext);
                }
                else
                {
                    edge = new EdgeProjection(K, right_ext);
                }

                // 建立地图点顶点
                if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ();
                    v->setEstimate(landmark.second->Pos());

                    // g2o要求,路标点的ID要大于Pose的ID,后续构建H矩阵会用到
                    // 这里就看出max_kf_id有啥用了
                    v->setId(landmark_id + max_kf_id + 1);

                    // 边缘化
                    v->setMarginalized(true);

                    // 建立路标点ID号和路标点顶点的对应关系
                    // 通知能够防止重复添加顶点
                    // 因为一个地图点对应了多个图像上的特征
                    vertices_landmarks.insert({landmark_id, v});

                    // 求解器中添加地图点顶点
                    optimizer.addVertex(v);
                }

                edge->setId(index);
                // 一个地图点,对应了多帧上的特征点
                // 一条边关联着地图点和对应特征的帧的位姿

                // TODO 这儿的数据成员访问方法,写个程序
                edge->setVertex(0, vertices.at(frame->keyframe_id_));   // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark

                // 特征点的坐标就是测量值
                edge->setMeasurement(toVec2(feat->position_.pt));

                edge->setInformation(Mat22::Identity()); // e转置*信息矩阵*e,所以由此可以看出误差向量为n×1,则信息矩阵为n×n
                auto rk = new g2o::RobustKernelHuber();

                //设置鲁棒核函数，之所以要设置鲁棒核函数是为了平衡误差，不让二范数的误差增加的过快。
                // 鲁棒核函数里要自己设置delta值，
                // 这个delta值是，当误差的绝对值小于等于它的时候，误差函数不变。否则误差函数根据相应的鲁棒核函数发生变化。
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);

                // 建立边和特征的对应关系
                edges_and_features.insert({edge, feat});

                // 求解器中添加边
                optimizer.addEdge(edge);

                index++;
            }
        }

        // 开始优化
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;


        // 调整阈值
        // TODO 看一下这个函数的作用
        // while (iteration < 5)
        // {
        //     cnt_outlier = 0;
        //     cnt_inlier = 0;

        //     for (auto &ef : edges_and_features)
        //     {

        //         if (ef.first->chi2() > chi2_th)
        //         {
        //             cnt_outlier++;
        //         }
        //         else
        //         {
        //             cnt_inlier++;
        //         }
        //     }

        //     double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);

        //     if (inlier_ratio > 0.5)
        //     {
        //         break;
        //     }
        //     else
        //     {
        //         chi2_th *= 2;
        //         iteration++;
        //     }
        // }

        // 根据重投影误差的大小来剔除外点
        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            }
            else
            {
                ef.second->is_outlier_ = false;
            }
        }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier;

        // 优化结束,赋值优化的结果
        for (auto &v : vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }

}
