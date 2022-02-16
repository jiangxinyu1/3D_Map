/*
 * @Author: your name
 * @Date: 2022-01-24 18:29:14
 * @LastEditTime: 2022-02-14 10:44:04
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /test_lcm/src/lcmHandler.h
 */

#ifndef LCM_HANDLER_H
#define LCM_HANDLER_H

#include <iostream>
#include <string>
#include <queue>
#include <map>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>

#include "tf/Vector3.h"

#include "lcm/lcm-cpp.hpp"
#include "lcm_ros/lcm_utils.h"
#include "lcm_std_msgs/Int8.hpp"
#include "lcm_std_msgs/Int16.hpp"
#include "lcm_std_msgs/Float32MultiArray.hpp"
#include "lcm_msgs/lcm_sensor_msgs/PointCloud.hpp"
#include "lcm_msgs/lcm_geometry_msgs/Point32.hpp"
#include "lcm_msgs/lcm_visualization_msgs/Marker.hpp"
#include "lcm_msgs/lcm_visualization_msgs/MarkerArray.hpp"

#include "lcm_ros/time.h"

#include "opencv2/highgui.hpp"

#include "tool.h"

class lcmHandler
{
public:

  lcmHandler();
  ~lcmHandler();

  void cmdCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcm_std_msgs::Float32MultiArray* cmd_in);
  void pointCloudCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const lcm_sensor_msgs::PointCloud* cloud);
  void skiMapBuilderThread();
  void run();
  void exit();

  std::queue<lcm_sensor_msgs::PointCloud> pointCloudBuffer;
  // std::mutex *pointcloud_buffer_mutex;
  const int pointCloudBufferMaxSize = 5;
  std::mutex *pointcloud_buffer_mutex;
  bool pointcloud_buffer_have=false;
  std::thread *skiMap_builder_thread;
  std::atomic<int> thread_exit_flag;

private:
    lcm::LCM *node_;
    
};


// void integrateMeasurement1(const std::vector<std::vector<int16_t>>& map_points_index, 
//                                                    std::vector<VoxelDataColor>& voxels, 
//                                                    SKIMAP *&map,
//                                                    const std::vector<int16_t>& map_camera_index );

// inline int64_t getTime();

#endif // !LCM_HANDLER_H
