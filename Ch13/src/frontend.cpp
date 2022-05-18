#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h" //
#include "myslam/g2o_types.h"
#include "myslam/map.h" //
#include "myslam/viewer.h"
#include "myslam/frontend.h"

using namespace std;

namespace myslam
{

    Frontend::Frontend()
    {
        // 创建特征提取器 关键点强度阈值0.01 关键点之间的最小距离为20
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);

        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    bool Frontend::AddFrame(myslam::Frame::Ptr frame)
    {
		// 当前帧赋值
        current_frame_ = frame;

        switch (status_) // status_是前端Frontend的类成员，所以这里判断的是整个前端的状态
        {
        case FrontendStatus::INITING:
            StereoInit(); //这里的StereoInit应该是一个bool函数
            break;

		// 这而不管是GOOD还是BAD都会继续执行track()函数
		// 	
        case FrontendStatus::TRACKING_GOOD:

        //不加break就不会跳出switch结构，不管后面case的条件是否符合都将会执行，
        // 直到遇到第一个break才会跳出switch结构
        //此处的TRACKING_GOOD和TRACKING_BAD并非跟踪失败或者跟踪成功的含义，这里的good和bad只是跟踪时条件好坏的区别
        //当跟踪到的特征点数目充足时就是good，当特征点数目不足时就是bad，但不论good还是bad，都只是一个条件恶劣与否的问题，并不涉及失败
        //当特征点少到不行的时候，就已经不是good和bad的问题了，太少的时候我们就认为跟踪丢了，设置为LOST，这个lost可以理解为跟踪失败
        //所以，lost之后需要reset系统
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
        }

        // 每处理了一帧，就把当前帧赋给上一帧。
        last_frame_ = current_frame_;
        return true;
    }

    //根据上面的Addframe函数，我们应当在后续实现StereoInit，Track和Reset各个函数

    //先来StereoInit
    bool Frontend::StereoInit()
    {

        //一个frame其实就是一个时间点，里面同时含有左，右目的图像。这一步在提取左目特征,
        int num_features_left = DetectFeatures();

        //左目提取到的特征在右目进行匹配，返回匹配上的特征数量
        //因为后续匹配上的特征点需要进行三角化，创建初始地图
        int num_coor_features = FindFeaturesInRight();

        if (num_coor_features < num_features_init_)
        { //对应数目不足，无法初始化
            return false;
        }

        bool build_map_success = BuildInitMap(); //初始化成功，则开始建立初始的地图

        //初始地图建立成功
        if (build_map_success)
        {
            // 初始化完毕，前端状态变成TRACKING_GOOD
            status_ = FrontendStatus::TRACKING_GOOD;

            // 如果开启了可视化功能
            if (viewer_)
            {
                // 把当前帧赋予给Viewer的私有成员变量currentframe
                viewer_->AddCurrentFrame(current_frame_);

                // 把当前激活的地图点和路标点拿出来，赋予给显示线程
                viewer_->UpdateMap();

                //?为什么Track()函数里面只有AddCurrentFrame(current_frame_)，没有UpdateMap()呢
                //?还需后续读一读AddCurrentFrame(current_frame_)和UpdateMap()的具体源码实现。
            }

            return true;
        }

        //如果初始地图没有建立成功，则会跳过上述if段落，则还是没能完成初始化工作，返回false
        return false;
    }

    //假如初始化成功后，此时前端状态已经变为case FrontendStatus::TRACKING_GOOD:，再来一帧之后，则会执行Track()

    bool Frontend::Track()
    {
        // 如果上一帧不是空指针,使用匀速模型预测当前帧的位姿
		// ? 一个保护措施吧，很少遇到
		// 良好的习惯
        if (last_frame_) 
        {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose()); //当前帧pose等于上一帧pose加上一个帧间pose
            // world----last_frame * last_frame------current_frame = world----current_frame
        }else
		{
			LOG(ERROR)<<"last_frame_ is nullptr";
		}

		
        //从上一帧跟踪到当前帧，跟踪到了多少个特征。
		// 同时建立特征与地图点的单向联系 featrue -> mappoint
        int num_track_last = TrackLastFrame();

        // 使用g2o进行poseOnly优化，并剔除外点，返回内点的数量
        tracking_inliers_ = EstimateCurrentPose();

        //(重投影)，同时根据这个函数的名字，我猜测后面还有对当前帧pose的精化。
        //接下来根据跟踪到的内点的匹配数目，可以分类进行后续操作

        // 设定跟踪状态
        // 大于50个点 good
        // 大于20个点 bad
        // 否则 gg

        //输出一下现在跟踪到的点的数量
        // cout << "tracking_inliers_ is " << tracking_inliers_ << endl;

        if (tracking_inliers_ > num_features_tracking_)
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_)
        {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            // lost
            status_ = FrontendStatus::LOST;
        }

        // 根据当前的tracking_inliers_判断其是否为关键帧,
        InsertKeyframe();

        // 计算一下当前帧与上一帧的相对运动
        // 用于匀速模型来计算下一帧的初值
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        // 可视化
        // 普通真不需要更新地图，只需要绘制一下当前帧的相机框框(绿色)
        // 在InsertKeyframe函数中，如果是关键帧，那就要更新一下地图
        // 因为关键帧会新创建处地图点
        if (viewer_)
            viewer_->AddCurrentFrame(current_frame_); //可视化

        return true;
    }

    //针对前面的status_,已经将INITING，GOOD，BAD三种情况下对应的Stereoinit和Track函数都说明了，接下来说明Reset函数
    bool Frontend::Reset()
    {
        LOG(INFO) << "Reset is not implemented. ";
        // //前端状态重置
        status_ = FrontendStatus::INITING;

        return true;
    }

    //三个上层级函数已经实现，接下来对stereoInit，Track，Reset三个函数中的一些细节函数再作以补充实现。
    //首先对StereoInit函数中的DetectFeatures()，FindFeaturesInRight()，BuildInitMap()三个函数做实现，可视化模块放在后面统一实现
    int Frontend::DetectFeatures()
    {

        //掩膜，灰度图，同时可以看出，DetectFeatures是对左目图像的操作
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);

        //        cv::imshow("init mask",mask);

        //即在这个矩形区域中不提取特征了，保持均匀性，并避免重复
        // auto语法讲解：auto可以根据初始化值进行自动类型推断，在这里auto&定义了feat，初始化值就是后面的current_frame_->features_left_
        // auto&在自动类型推断完成定义的同时构成了引用类型，也就是说feat的改变将同步影响current_frame_->features_left_中的元素
        //如果单单用一个auto就不会有这种同步效果
        for (auto &feat : current_frame_->features_left_)
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED); //在已有的特征附近一个矩形区域内将掩膜值设为0
        }

        // for debugger
        //        if(current_frame_->features_left_.empty())
        //        {
        //            cv::imshow("this is empty",mask);
        //        } else
        //        {
        //            cv::imshow("this is not empty",mask);
        //        }

        std::vector<cv::KeyPoint> keypoints; //关键点容器

        // detect函数，第三个参数是用来指定特征点选取区域的，一个和原图像同尺寸的掩膜，其中非0区域代表detect函数感兴趣的提取区域，相当于为
        // detect函数明确了提取的大致位置
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0; //检测到的特征计数

        // 尽量用auto &
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    //找到左目图像的feature之后，就在右目里面找对应值
    int Frontend::FindFeaturesInRight()
    {
        std::vector<cv::Point2f> kps_left, kps_right; //定义两个存储关键点坐标的vector容器

        //遍历左目特征的关键点
        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);

            // 把左目特征点对应的地图点拿出来，根据位姿投影到右目下，作为初值【光流的初值？】
            auto mp = kp->map_point_.lock(); //通过weak_ptr的lock()函数实现对地图点shared_ptr智能指针的复制，并赋予mp
            if (mp)
            {
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->Pose());

                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            { 
                //如果指针为空则执行else语段
                // 如果左目的特征还没有和地图点有关联，就使用相同的坐标。
                kps_right.push_back(kp->position_.pt);
            }
        }

        //进行光流跟踪，从这条opencv光流跟踪语句我们就可以知道，前面遍历左目特征关键点是为了给光流跟踪提供一个右目初始值
        std::vector<uchar> status; //光流跟踪成功与否的状态向量（无符号字符），成功则为1,否则为0
        Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_,
            kps_left, kps_right, status,
            error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);
        // OPTFLOW_USE_INITIAL_FLOW使用初始估计，存储在nextPts中;如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计。

        int num_good_pts = 0; //右目中光流跟踪成功的点
        for (size_t i = 0; i < status.size(); ++i)
        {

            // 成功跟踪到了这个特征
            if (status[i])
            {
                // 这个7是特征点的大小，OPENCV绘制特征点那个圆的半径
                cv::KeyPoint kp(kps_right[i], 7);

                // 将这个特征认为是右目上的特征
                Feature::Ptr feat(new Feature(current_frame_, kp));
                

                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                //光流跟踪没找到的特征，就在features_right_里面填空指针
                current_frame_->features_right_.push_back(nullptr); 
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    //现在左目图像的特征提取出来了，并根据左目图像的特征对右目图像做了特征的光流跟踪，找到了对应值，当对应数目满足阈值条件时，我们可以开始建立
    //初始地图
    bool Frontend::BuildInitMap()
    {

        // 构造一个存储SE3的vector，里面初始化就放两个pose，
        // 这个pose存储着两个相机的外参
        // T_C0C1和T_C0C2
        // C0是虚拟相机坐标系和左目重合
        // C1是左目 C2是右目
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};

        size_t cnt_init_landmarks = 0; //初始化的路标数目

        // 对于左目中提取到的所有特征，进行遍历
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            //对于左目的每一个feature，我们必须在右目找到对应的feature，才能继续三角化，不然就跳过
            //对于右目feature容器，如果是成功跟踪到了就是一个指向Feature的指针，否则就是个空指针，我们需要跳过跟踪失败的空指针
            if (current_frame_->features_right_[i] == nullptr)
                continue;

            //将左目和右目匹配好的特征点根据相机的内参转换到归一化平面上
            // （u，v,1） -> (x,y,1)
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),

                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))

            };

            //建立一个存储3D世界坐标的VEC3，3D向量
            Vec3 pworld = Vec3::Zero();

            // 正式三角化
            // 先执行triangulation,然后再用结果去判断pworld的深度大于0
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {

                // 创建一个MapPoint类对象用来承载三角化出的世界坐标pworld
                // 同时给这个地图一个ID号
                auto new_map_point = MapPoint::CreateNewMappoint();
                // 给这个新创建的地图点一个坐标
                new_map_point->SetPos(pworld);

                // * 建立地图点和特征坐标之间的双向联系
                // 记录一下这个地图点被哪些相机帧给观测到了
                // 将地图点与这个帧对应的特征点进行关联   地图点->特征 的单向联系
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);

                // 上两句是为地图点添加观测，这两句就是为特征类Feature对象填写地图点成员
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;

                //初始化成功的landmark或者说地图点数目加1
                cnt_init_landmarks++;

                //对Map类对象来说，地图里面应当多了一个地图点，所以要将这个地图点加到地图中去
                map_->InsertMapPoint(new_map_point);
            }
        }

        // 当前帧能够进入初始化说明已经满足了初始化所需的帧特征数量，作为初始化帧.可看做开始的第一帧，所以应当是一个关键帧
        // 为关键帧赋予关键帧的ID号
        current_frame_->SetKeyFrame();

        //对Map类对象来说，地图里面应当多了一个关键帧，所以要将这个关键帧加到地图中去
        map_->InsertKeyFrame(current_frame_);

        //关键帧插入，后端需要对新纳入的关键帧进行优化处理
        backend_->UpdateMap();

        //向日志输入消息
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    // TrackLastFrame()从上一帧跟踪到当前帧
    int Frontend::TrackLastFrame()
    {
        // 上一帧的特征点坐标 当前帧的特征点坐标
        std::vector<cv::Point2f> kps_last, kps_current;

        // Step 1 提供光流的粗值
        // * 如果特征有地图点，根据投影关系确定粗值，否则直接用上一帧的坐标
        //  对于上一帧的左目特征点
        for (auto &kp : last_frame_->features_left_)
        {
            //如果上一帧的特征有对应的地图点
            if (kp->map_point_.lock())
            {
                // 如果有，就根据当前帧的粗位姿(匀速运动模型得到)投影到当前帧下得到的像素坐标
                // 认为是上一帧的特征点在当前帧的位置
                auto mp = kp->map_point_.lock();
                auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->Pose());

                // 保存一下特征点关系
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                //没有MapPoint就没有初始化猜测值，那么光流搜索的起点就是上一帧中点的像素位置
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat err;
        // 光流跟踪
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_,
            kps_last, kps_current, status,
            err, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            // status[i]=true则说明跟踪成功有对应点，false则跟踪失败没找到对应点
            if (status[i])
            {
                // 7是关键点的直径
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));

                // 对于成功跟踪上的特征点，如果在上一帧中对应了地图点，那么这一帧的特征点也和该地图点建立联系
                // 如果没有对应的地图点，这儿赋值一个空指针
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;

                // if( !feature->map_point_.lock())
                //     LOG(ERROR)<<"this is a nullptr";


                // 把匹配上的特征，作为当前帧的特征点存储到特征管理器中。
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    //跟踪成功之后可以求解精准的Current——Pose

    // EstimateCurrentPose()求解精准的CurrentPose

    int Frontend::EstimateCurrentPose()
    {
        // 块求解器 6维pose 3维landmark
        typedef g2o::BlockSolver_6_3 BlockSolverType;

        // 增量方程Hx=b的求解方法 Dense cholesky分解法
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;

        // 使用LM算法
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));

        // 创建稀疏优化器
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 只有一个顶点->当前的帧的位姿SE3
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);                            // 顶点的id
        vertex_pose->setEstimate(current_frame_->Pose()); //设定初值
        optimizer.addVertex(vertex_pose);                 //添加顶点

        // K
        Mat33 K = camera_left_->K(); //左目内参矩阵

        int index = 1; //边的ID号

        // 建立边的容器，边类型为EdgeProjectionPoseOnly
        // 一元边、仅优化位姿不优化地图点 PnP问题
        std::vector<EdgeProjectionPoseOnly *> edges;

        //建立一个特征容器，存储左目带有地图点的特征，也就是参加优化的那些特征
        std::vector<Feature::Ptr> features;

        //建立Pnp
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {

            auto mp = current_frame_->features_left_[i]->map_point_.lock();

            // 如果左目特征关联了一个地图点，可以得到重投影误差，建立一个边。
            // 常规操作，没啥好说的
            if (mp)
            {
                features.push_back(current_frame_->features_left_[i]);

                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);

                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->position_.pt));

                edge->setInformation(Eigen::Matrix2d::Identity());

                // 鲁棒核
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose the determine the outliers
        const double chi2_th = 5.991;
        int cnt_outlier = 0;

        // *4轮大优化 每轮大优化进行10次小优化
        // 每轮大优化后，根据重投影误差的大小，剔除掉误差明显的边，也就去去除掉外点，然后进行进一步的优化。
        for (int iteration = 0; iteration < 4; ++iteration)
        {
            // 每次优化的初值都设定为current_frame_->Pose()，
            // 在优化的过程中并不会改变这个pose
            // 也就是说，重复优化的目的就是去除外点，选更好的数据进行估计
            // 但每次涉及的特征都不一样，所以每次的重投影误差都不一样，
            // 就有可能发现新的outlier，这是一个不断筛查,删除,精化的过程
            vertex_pose->setEstimate(current_frame_->Pose());

            // 套路
            optimizer.initializeOptimization();
            optimizer.optimize(10);

            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];

                // 一开始优化，在optimize中会对所有的边计算误差
                // 由于初始值偏移的比较远以及外点的影响，这个误差一轮优化还降低不下来
                // 会设置误差比较大的点为外点，但是这些外点里可能还是有一些内点被误认为是外点了
                // 所以每次优化后，还要在计算这些边的误差，能救一个点是一个点。
                // 防止误判
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }

                // 这边是一个卡方检验的误差剔除
                // 假设误差在u和v方向上的标准差是1个像素，那么重投影误差符合自由度为2卡方分布
                // 取置信概率为95%，也就是95%的概率是外点。
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    // g2o只会处理level=0的边
                    // 如果确定某个边的重投影误差过大，则把level设置为1，也就是舍弃这个边对于整个优化的影响
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                // 只有前三次迭代使用鲁棒核来降低外点的影响
                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier;

        // 优化的结果赋值给当前帧
        current_frame_->SetPose(vertex_pose->estimate());
        // 输出当前帧的位姿
        LOG(INFO) << "Current Pose = \n"
                  << current_frame_->Pose().matrix();

        // 如果在优化的过程中，发现了外点
        // 把这个外点对应的地图点给删掉
        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                //弱指针自带的操作函数reset，作用是将指针置空
                feat->map_point_.reset(); 

                // feat是一个vector类型的容器，不好删除
                // 把这个特对应的地图点删除，这样后面就不会再用到这个特征了。
                feat->is_outlier_ = false;
            }
        }
        return features.size() - cnt_outlier; // inliers
    }

    //在完成前后帧的跟踪和pose估计后，我们需要对新来的每一帧进行关键帧判别，看它是不是一个关键帧，这里就需要用到InsertKeyframe函数
    bool Frontend::InsertKeyframe()
    {
        // 如果当前成功跟踪到的特征点大于80个
        // 就认为新增的信息还不够多，不是关键帧
        if (tracking_inliers_ >= num_features_needed_for_keyframe_)
        {
            return false;
        }

        // 设置当前帧为关键帧
        // 在地图里插入关键帧
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;

        // 在之前的TrackLastFrame函数中，设定了特征->地图点的对应关系
        // 这个对地图点的观测进行更新
        // 增加地图点->特征的匹配关系以及更新地图点的被观测次数
        SetObservationsForKeyFrame();

        // 在左目中提取一些新的特征点
        DetectFeatures();

        // 左目特征点寻找在右目匹配的关键点
        FindFeaturesInRight();

        // 为关键帧中新提取的特征点，进行三角化
        TriangulateNewPoints();

        // 告诉后端，我们有一个新的关键帧来了
        backend_->UpdateMap();

        // 关键帧更新了，告诉一下显示线程，你的显示也要更新一下咯
        if (viewer_)
            viewer_->UpdateMap();

        return true;
    }

    //在InsertKeyFrame函数中出现了一个三角化步骤，这是因为当一个新的关键帧到来后，我们势必需要补充一系列新的特征点，
    // 此时则需要像建立初始地图一样，对这些新加入的特征点进行三角化，求其3D位置
    int Frontend::TriangulateNewPoints()
    {
        //这个函数其实与BuildInitMap差不多
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse(); // current_frame_->Pose()是从世界到相机,逆就是从相机到世界

        // 这波新生成的地图点的数量
        int cnt_triangulated_pts = 0;

        // 对于左目中提取到的所有特征，进行遍历
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            // 如果当前左目特征对应的地图点被注销了
            // 且在右目找到了对应的特征
            // 就可以进行一波三角化
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {

                //将左目和右目匹配好的特征点根据相机的内参转换到归一化平面上
                // （u，v,1） -> (x,y,1)
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};

                //建立一个存储3D世界坐标的VEC3，3D向量
                Vec3 pworld = Vec3::Zero();

                // 正式三角化
                // 先执行triangulation,然后再用结果去判断pworld的深度大于0
                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {

                    // 将之前在相机坐标系下三角化得到的地图点，转回到世界坐标系下
                    pworld = current_pose_Twc * pworld; //从相机坐标系转到世界坐标系

                    // 创建一个新的地图点，并赋予地图点的坐标
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    new_map_point->SetPos(pworld);

                    // * 建立地图点和特征坐标之间的双向联系
                    // 记录一下这个地图点被哪些相机帧给观测到了
                    // 将地图点与这个帧对应的特征点进行关联   地图点->特征 的单向联系
                    new_map_point->AddObservation(current_frame_->features_left_[i]);
                    new_map_point->AddObservation(current_frame_->features_right_[i]);

                    // 上两句是为地图点添加观测，这两句就是为特征类Feature对象填写地图点成员
                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;

                    //对Map类对象来说，地图里面应当多了一个地图点，所以要将这个地图点加到地图中去
                    map_->InsertMapPoint(new_map_point);

                    // 新创建的地图点数量++
                    cnt_triangulated_pts++;
                }
            }
        }

        // 输出一下这波新建关键帧的同时整出了多少新的地图点
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    void Frontend::SetObservationsForKeyFrame()
    { //查找当前帧中的特征，看是否对应已有的地图点，若对应则为地图点添加当前帧内的特征观测
        //若不对应则不做操作，跳过即可
        for (auto &feat : current_frame_->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->AddObservation(feat);
        }
    }

}
