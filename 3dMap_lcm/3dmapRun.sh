#!/bin/sh
###
 # @Author: your name
 # @Date: 2022-01-22 14:32:01
 # @LastEditTime: 2022-02-25 10:12:20
 # @LastEditors: Please set LastEditors
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /camera_lcm_demo_SY/tools/run.sh
### 

mkdir -p /userdata/sunnytest/capture/
rm -rf /userdata/sunnytest/capture/*
export LD_LIBRARY_PATH="/userdata/sunnytest/libs:${LD_LIBRARY_PATH}"

v4l2-ctl -d /dev/video0 --set-fmt-video=width=224,height=2193,pixelformat=BG12 --set-crop=top=0,left=0,width=224,height=2193 --stream-mmap=4 --stream-count=1

chmod 777 /userdata/sunnytest/sy_camerademo
cd /userdata/sunnytest/
./sy_camerademo_lcm  > /dev/null  &

cd /userdata/
./cape_lcm > /dev/null  &

cd /userdata/sunnytest/
./skiMap_node > /dev/null &