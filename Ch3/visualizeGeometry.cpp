/*
 * @Author: Divenire
 * @Date: 2021-09-13 20:57:01
 * @LastEditors: Divenire
 * @LastEditTime: 2021-09-22 17:09:16
 * @Description: 添加注释与修改
 */
#include <iostream>
#include <iomanip>

using namespace std;


#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace Eigen;

#include "pangolin/pangolin.h"

struct RotationMatrix {
  Matrix3d matrix = Matrix3d::Identity();
};

//第二个形参是引用是因为希望避免复制实参，常量是因为通常不会修改打印内容

/**
 * @description: 打印旋转矩阵的值
 * @param {*}
 * @return {*}
 */
ostream &operator<<(ostream &out, const RotationMatrix &r) {
    
// 不使用科学计数法
  out.setf(ios::fixed);
//保留小数点后2位，默认是6位。
  out.precision(2);

//打印数据
  Matrix3d matrix = r.matrix;
  out << '=';
  out << "[" << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],"
      << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],"
      << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
  return out;
}


istream &operator>>(istream &in, RotationMatrix &r) {
  return in;
}

/**
 * @description: 平移向量
 * @param {*}
 * @return {*}
 */
struct TranslationVector {
  Vector3d trans = Vector3d(0, 0, 0);
};

/**
 * @description: 输出平移向量
 * @param {*}
 * @return {*}
 */
ostream &operator<<(ostream &out, const TranslationVector &t) {
  out << "=[" << t.trans(0) << ',' << t.trans(1) << ',' << t.trans(2) << "]";
  return out;
}


istream &operator>>(istream &in, TranslationVector &t) {
  return in;
}


struct QuaternionDraw {
  Quaterniond q;
};
/**
 * @description: 输出四元数
 * @param {*}
 * @return {*}
 */
ostream &operator<<(ostream &out, const QuaternionDraw quat) {
  auto c = quat.q.coeffs();
  out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
  return out;
}

istream &operator>>(istream &in, const QuaternionDraw quat) {
  return in;
}

int main(int argc, char **argv) {

  //创建pangolin窗口
  pangolin::CreateWindowAndBind("visualize geometry", 640, 480);

  //开启深度测试
  glEnable(GL_DEPTH_TEST);
  
  //摆放一个相机
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
    pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
  );

  
  const int UI_WIDTH = 500;

  //创建交互试图用于显示立方体 
  pangolin::View &d_cam = pangolin::CreateDisplay().
    SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f).
    SetHandler(new pangolin::Handler3D(s_cam));

  // ui
  pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
  pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
  pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
  pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));


  while (!pangolin::ShouldQuit()) {
    //清除颜色缓存
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);

    //获得相机的位姿T
    pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
    Matrix<double, 4, 4> m = matrix;

    // 提取旋转的部分
    RotationMatrix R;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        R.matrix(i, j) = m(j, i);
        
    // 重载ostream用于显示旋转矩阵到ui上
    rotation_matrix = R;


    // 平移向量
    TranslationVector t;
    t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));
    t.trans = -R.matrix * t.trans;
    translation_vector = t;

    // 欧拉角
    TranslationVector euler;
    euler.trans = R.matrix.eulerAngles(2, 1, 0);
    euler_angles = euler;


    // 转换得到四元数
    QuaternionDraw quat;
    quat.q = Quaterniond(R.matrix);
    quaternion = quat;


    // 绘制立方体
    glColor3f(1.0, 1.0, 1.0);

    pangolin::glDrawColouredCube();
    
    // draw the original axis
    // 原点的坐标轴
    // 绘制线宽
    glLineWidth(3);
    // 设置画笔颜色
    glColor3f(0.8f, 0.f, 0.f);
    glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(10, 0, 0);
      glColor3f(0.f, 0.8f, 0.f);
      glVertex3f(0, 0, 0);
      glVertex3f(0, 10, 0);
      glColor3f(0.2f, 0.2f, 1.f);
      glVertex3f(0, 0, 0);
      glVertex3f(0, 0, 10);
    glEnd();

    pangolin::FinishFrame();
  }
}
