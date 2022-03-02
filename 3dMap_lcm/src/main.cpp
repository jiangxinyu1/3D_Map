/*
 * @Author: your name
 * @Date: 2022-01-24 18:42:33
 * @LastEditTime: 2022-03-02 18:05:47
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /test_lcm/test_lcm.cpp
 */

#include "lcmHandler.h"


int main(int argc, char **argv)
{
  lcmHandler handlerObj;
  handlerObj.run();
  handlerObj.exit();
  return 0;
} 


