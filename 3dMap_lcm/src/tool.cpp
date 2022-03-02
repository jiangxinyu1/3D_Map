/*
 * @Author: your name
 * @Date: 2022-02-11 17:20:21
 * @LastEditTime: 2022-03-02 16:47:21
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /3dMap_lcm/src/include/tool/tool.cpp
 */

#include "tool.h"

// EIGEN_NO_DEBUG和NDEBUG

#ifndef EIGEN_NO_DEBUG
#define EIGEN_NO_DEBUG
#endif


#ifndef NDEBUG
#define NDEBUG
#endif

struct Point3f {
  Point3f(float x_ ,float y_ ,float z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }
  float x;
  float y;
  float z;
};

struct Point3d {
  Point3d(double x_ ,double y_ ,double z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }
  double x;
  double y;
  double z;
};

void debugPrint(const std::string &position)
{
#if DEBUG_POSITION_PRINT
  std::cout << "[DEBUG]: +++++++++++++++++++ POSITION  "<< position << "  ++++++++\n" ;
#endif
}



