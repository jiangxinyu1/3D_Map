#!/bin/bash
###
 # @Author: jiangxinyu
 # @Date: 2021-12-13 16:30:10
 # @LastEditTime: 2022-03-02 17:04:49
 # @LastEditors: Please set LastEditors
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /my_rkaiq_3A_server/build.sh
### 

echo "Configuring and building  ..."
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
# cmake ..
cmake -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_TOOLCHAIN_FILE=~/.Toolchain/arm_toolchain.cmake
make -j8
date
