#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam
{

    Viewer::Viewer()
    {
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    }

    void Viewer::Close()
    {
        // 停止线程的基本操作
        // 在while循环中如果这个为fasle
        // 线程就会退出while，进入join，结束线程
        viewer_running_ = false;
        viewer_thread_.join();
    }

    // 在显示线程中更新当前图像帧
    void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
    {

        // 为什么这儿需要锁？
        // TODO
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);

        // 注意current_frame_ 这个是Viewer的私有成员函数
        current_frame_ = current_frame;
    }

    // 在图像中的每一个特征点上，绘制一个远点
    cv::Mat Viewer::PlotFrameImage()
    {
        cv::Mat img_out;
        cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.lock())
            {
                auto feat = current_frame_->features_left_[i];
                // 在特征点的坐标上画圆
                // TODO opencv的常见函数
                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                           2);
            }
        }
        return img_out;
    }

    void Viewer::UpdateMap()
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);

        // 再次断言判断一下这个Viewer中的map_有没有被初始化成功
        assert(map_ != nullptr);

        // 需要显示所有的地图点和关键帧
        // 这个active_keyframes_和active_landmarks_是Viewer类的私有成员
        active_keyframes_ = map_->GetAllKeyFrames();
        active_landmarks_ = map_->GetAllMapPoints();

        // active_keyframes_ = map_->GetActiveKeyFrames();
        // active_landmarks_ = map_->GetActiveMapPoints();
    }

    void Viewer::ThreadLoop()
    {

        //创建一个GUI界面
        pangolin::CreateWindowAndBind("MySLAM", 1024, 768);

        // 启用深度测试
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        // 在OpenGL中使用颜色混合
        // glEnable和glBlendFunc可以参照：https://blog.csdn.net/ZhaDeNianQu/article/details/103926074
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        /**
         * @description: Define Camera Render Object (for view / scene browsing)
         *               在视窗中放置一个相机，给出相机自身的位置和相机观察的位置，以及相机本身哪个轴朝上。
         * @param {
         * OpenGlMatrix& projection_matrix:定义相机投影模型：
         *                                  ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
         *                                  参数依次为观察相机的图像高度、宽度、焦距fu,fy和漂移cx,cy以及最近和最远视距
         * OpenGlMatrix& modelview_matrix:定义观测方位向量：
         *                                  ModelViewLookAt( x,  y,  z,  lx,  ly,  lz, AxisDirection up);
         *                                  观测点位置(相机所在位置)：(x y z)
         *                                  观测目标位置(相机所看的视点位置)：(lx, ly, lz)
         *                                  观测的方位向量：(0.0,-1.0, 0.0)
         * }
         **/
        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 10000),
            pangolin::ModelViewLookAt(0, -100, -0.1, 0, 0, 0, 0.0, -1.0, 0.0));

        //在先前创建的GUI界面中添加创建一个视口，或者说是一个可视化窗口，可以理解为从GUI界面中划分出特定大小的一部分窗口用于显示特定内容
        pangolin::View &vis_display =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(vis_camera));

        const float blue[3] = {0, 0, 1};
        const float green[3] = {0, 1, 0};

        // 开始绘画
        // 循环刷新，两个终止条件，第一个就是叫停Pangolin，第二个就是VO失败，可视化停止。
        while (!pangolin::ShouldQuit() && viewer_running_)
        {

            // 清空颜色缓存和深度缓存，即清除屏幕
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            // 使用白色背景
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            //将视口vis_display根据vis_camera这个相机视角激活为一个交互式视图（可以用鼠标进行视角旋转）
            vis_display.Activate(vis_camera);

            // 由于current_frame_以及
            // TODO 为什么要锁
            std::unique_lock<std::mutex> lock(viewer_data_mutex_);

            //如果当前帧不为空指针
            if (current_frame_)
            {
                // 当前帧用绿色画出来
                DrawFrame(current_frame_, green);

                // 可视化的3D观察视角跟随相机的位姿
                // FollowCurrentFrame(vis_camera);

                // 把特征点在图像上画出来
                cv::Mat img = PlotFrameImage(); 

                cv::imshow("image", img);
                cv::waitKey(1); //停1ms
            }

            //如果map_不为空则执行后续,map_这个类成员变量通过.h文件中定义的SetMap()函数来赋值，
            // 在其它线程中完成地图更新后可以用SetMap()函数为接口传入可视化器Viewer类对象.
            if (map_)
            {

                // 绘制关键帧和地图点（红色）
                DrawMapPoints(); 
            }

            pangolin::FinishFrame();
            usleep(5000); // usleep功能把进程挂起一段时间， 单位是微秒（百万分之一秒）；
        }

        LOG(INFO) << "Stop viewer";
    }

    void Viewer::DrawFrame(Frame::Ptr frame, const float *color)
    {

        SE3 Twc = frame->Pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        //关于glPushMatrix()和glPopMatrix()操作网上有一个很有趣的解释，希望能帮助你理解：
        //有时候在经过一些变换后我们想回到原来的状态，就像我们谈恋爱一样，换来换去还是感觉初恋好
        //参考资料：https://blog.csdn.net/passtome/article/details/7768379
        glPushMatrix(); //压栈操作

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        //将原有的世界坐标系变换到当前坐标系下，便于直接画出相机模型
        glMultMatrixf((GLfloat *)m.data()); 


        // 没有指定画什么颜色，就画红色
        if (color == nullptr)
        { 
            glColor3f(1, 0, 0);
        }
        else
            glColor3f(color[0], color[1], color[2]);

        //这里要在Pangolin里面画出相机模型了，如果你实际运行了这个VO，应该会记得Pangolin里面画出的那个小相机是由8条边构成的
        glLineWidth(line_width); //设置线宽
        glBegin(GL_LINES);       //开始画线
                                 
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();       //结束画线
        glPopMatrix(); //弹出操作，或者说出栈操作，恢复到这个变换以前的状态
    }

    void Viewer::DrawMapPoints()
    {
        const float red[3] = {1.0, 0, 0};

        for (auto &kf : active_keyframes_)
        { //将所有的窗口内激活帧都画成红色
            DrawFrame(kf.second, red);
        }

        glPointSize(2);     //确定点云尺寸
        glBegin(GL_POINTS); //开始画点
        for (auto &landmark : active_landmarks_)
        {
            auto pos = landmark.second->Pos();  //获得地图路标点的世界坐标，存储在pos中
            glColor3f(red[0], red[1], red[2]);  //确定点云颜色为红
            glVertex3d(pos[0], pos[1], pos[2]); //给出点云位置，画出点云
        }
        glEnd(); //结束画点云
    }

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
    {
        // 当前帧的位姿SE3
        SE3 Twc = current_frame_->Pose().inverse();

        //依据Pangolin形式设定变换矩阵
        pangolin::OpenGlMatrix m(Twc.matrix());

        // 交互视图的视角，或者说Pangolin观察相机的相机视角跟着这个变换矩阵进行随动，完成对当前帧的视角跟随
        vis_camera.Follow(m, true);
    }

}
