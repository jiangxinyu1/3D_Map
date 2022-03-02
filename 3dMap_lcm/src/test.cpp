/*
 * @Author: your name
 * @Date: 2022-03-02 18:05:37
 * @LastEditTime: 2022-03-02 18:24:04
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /3dMap_lcm/src/test.cpp
 */
#include "lcmHandler.h"



static void setRotationMat1(Eigen::Matrix3f &RotationMat_ , const float &angleX_, const float &angleY_, const float &angleZ_)
{
    Eigen::Matrix3f Rotation_Y = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rotation_Z = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rotation_X = Eigen::Matrix3f::Identity();
    float cosAngleZ = std::cos(DEG2RAD(angleZ_));
    float sinAngleZ = std::sin(DEG2RAD(angleZ_));
    Rotation_Z << cosAngleZ , -sinAngleZ, 0 ,
                             sinAngleZ, cosAngleZ,0,
                             0,0,1;
    RotationMat_ = Rotation_Z * Rotation_Y * Rotation_X;   
}

static inline int64_t getTime1()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec*1000.0+ std::round(t.tv_nsec/1000000.0);
}

int main(int argc, char **argv)
{
  Eigen::Vector3f t1 (0.f,0.f,0.f);
  Eigen::Matrix3f m1;
  setRotationMat1(m1,0.f,0.f,1.32f);
  auto time1 = getTime1();
  for (int i = 0 ; i < 200000; i++) 
  {
    Eigen::Vector3f p1((float)i+2.121f,(float)i+4.123f,(float)i+1.2f);
    Eigen::Vector3f m= m1*p1+t1;
    m[0] = 1.0;
    m[1] = 1.0;
    m[2] = 1.0;
  }
  auto time2 = getTime1();
  std::cout << "[TEST] : " << time2-time1 << "\n";
  return 0;
} 