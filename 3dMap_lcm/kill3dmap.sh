#!/bin/sh
###
 # @Author: your name
 # @Date: 2022-02-25 10:10:20
 # @LastEditTime: 2022-02-25 10:15:44
 # @LastEditors: Please set LastEditors
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /3dMap_lcm/kill3dmap.sh
### 

kill -9 $(pidof sy_camerademo_lcm)

echo "kill sy_camerademo_lcm done"

kill -9 $(pidof cape_lcm)

echo "kill cape_lcm done"

kill -9 $(pidof skiMap_node)

echo "kill skiMap_node done"

