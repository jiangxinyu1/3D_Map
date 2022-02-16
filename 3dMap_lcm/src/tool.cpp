/*
 * @Author: your name
 * @Date: 2022-02-11 17:20:21
 * @LastEditTime: 2022-02-11 18:52:36
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /3dMap_lcm/src/include/tool/tool.cpp
 */

#include "tool.h"


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


int64_t getTime()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec*1000.0+ std::round(t.tv_nsec/1000000.0);
}


