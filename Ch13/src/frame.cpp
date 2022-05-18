/*** 
 * @Author       : Yue
 * @Date         : 2022-05-12 15:50:57
 * @LastEditTime : 2022-05-17 21:11:05
 * @LastEditors  : Yue
 * @Description  : 
 * @FilePath     : /Ch13/src/frame.cpp
 * @佛祖保佑 BUG FREE
 */
///针对frame.h中涉及的各类函数进行定义实现


#include "myslam/frame.h"

namespace myslam {

    // 静态成员函数的定义
    Frame::Ptr Frame::CreateFrame() {
        // 帧ID号
        static long factory_id = 0;

        // Frame::Ptr new_frame(new Frame);
        Frame::Ptr new_frame = std::make_shared<Frame>();
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    void Frame::SetKeyFrame() {
        // 关键帧的ID号
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

}
