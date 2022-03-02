/*
 * @Author: your name
 * @Date: 2022-02-11 17:20:54
 * @LastEditTime: 2022-03-01 20:51:21
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /3dMap_lcm/src/include/tool/tool.h
 */


#ifndef _TOOL_H_
#define _TOOL_H_


#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>

#ifndef DEBUG_POSITION_PRINT
#define DEBUG_POSITION_PRINT 1
#endif

#define point_z_filtter


#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

void debugPrint(const std::string &position);

inline int64_t getTime()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec*1000.0+ std::round(t.tv_nsec/1000000.0);
}


#endif // !_TOOL_H_

