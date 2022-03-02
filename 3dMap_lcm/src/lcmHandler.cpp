/*
 * @Author: your name
 * @Date: 2022-01-24 18:28:58
 * @LastEditTime: 2022-03-02 17:44:24
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /test_lcm/src/lcmHandler.cpp
 */

#include "lcmHandler.h"

// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>

// skimap
typedef skimap::VoxelDataRGBW<uint16_t, float> VoxelDataColor;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float> SKIMAP;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float>::Tiles2D Tiles2D;

SKIMAP *map;

struct MapParameters 
{
  float ground_level;
  float agent_height;
  float map_resolution;
  int min_voxel_weight;
  bool enable_chisel;
  bool height_color;
  int chisel_step;
} mapParameters;

struct CameraParameters {
  double fx, fy, cx, cy;
  int cols, rows;
  double min_distance;
  double max_distance;
  int point_cloud_downscale;
} camera;

struct ColorPoint {
  cv::Point3f point;
  cv::Vec4b color;
  int w;
};

enum VisualizationType {
  POINT_CLOUD,
  VOXEL_MAP,
  VOXEL_GRID,
};


struct SensorMeasurement 
{
  lcm_ros::Time stamp;
  std::vector<ColorPoint> points;
  std::vector<ColorPoint> chisel_points;

  void addChiselPoints(CameraParameters &camera, float resolution) 
  {
    chisel_points.clear();

#pragma omp parallel
    {
      std::vector<ColorPoint> new_points;

#pragma omp for nowait
      for (int i = 0; i < points.size(); i++) 
      {
        cv::Point3f dir = points[i].point - cv::Point3f(0, 0, 0);
        dir = dir * (1 / cv::norm(dir));
        for (float dz = camera.min_distance; dz < points[i].point.z;dz += resolution) 
             {
          cv::Point3f dp = dir * dz;
          ColorPoint colorPoint;
          colorPoint.point = dp;
          colorPoint.w = -mapParameters.chisel_step;
          new_points.emplace_back(colorPoint);
        }
      }

#pragma omp critical
      points.insert(points.end(), new_points.begin(), new_points.end());
    }
  }
};

struct IntegrationParameters 
{
  std::vector<VoxelDataColor> voxels_to_integrate;
  // std::vector<tf::Vector3> poses_to_integrate;
  std::vector<bool> tiles_mask;
  int integration_counter;

  IntegrationParameters() { integration_counter = 0; }
} integrationParameters;




void setRotationMat(Eigen::Matrix3f &RotationMat_ , const float &angleX_, const float &angleY_, const float &angleZ_)
{
    Eigen::Matrix3f Rotation_Y = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rotation_Z = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rotation_X = Eigen::Matrix3f::Identity();

    // float cosAngleX = std::cos(DEG2RAD(angleX_));
    // float sinAngleX = std::sin(DEG2RAD(angleX_));
    // Rotation_X << 1,0,0,
    //                           0,cosAngleX,-sinAngleX,
    //                           0,sinAngleX,cosAngleX;

    
    float cosAngleZ = std::cos(DEG2RAD(angleZ_));
    float sinAngleZ = std::sin(DEG2RAD(angleZ_));
    Rotation_Z << cosAngleZ , -sinAngleZ, 0 ,
                             sinAngleZ, cosAngleZ,0,
                             0,0,1;
    RotationMat_ = Rotation_Z * Rotation_Y * Rotation_X;   
}



/**
 * @brief 
 * 
 * @param map_points_index 存储所有点云在map系中栅格坐标，按顺序存储
 * @param voxels 
 * @param map skiMap
 * @param map_camera_index 相机在map下的栅格化位置
 */
void integrateMeasurement1(const std::vector<std::vector<int16_t>>& map_points_index, 
                                                   VoxelDataColor &voxelInit,
                                                   SKIMAP *&map,
                                                   const std::vector<int16_t>& map_camera_index)
{
  map->enableConcurrencyAccess(true);

  // 单个index的值对应多个 map_point_index
  std::unordered_map<int , std::vector<std::vector<int16_t>>> voxel_index;

  // std::vector<std::pair<int , std::vector<int16_t>>> voxel_index;
  // std::vector<voxel_index_node> voxel_index;

  // 遍历 map_points_index，更新 hit point 的概率值
  auto updateHitStartTime = getTime();
  
  // 遍历所有点云的栅格坐标，
  for (int i = 0; i < map_points_index.size(); i++) 
  {
    // 将每一个点插入到map中，如果对应的skipmap(x,y,z)为空，插入voxel，否则对应的data+1
    bool newP = false;
    map->integrateVoxel(map_points_index[i], &voxelInit , newP);
    if(newP)
    {

#if 1  // for update updataMissVoxel 1    
      voxel_index[(map_points_index[i][0] ) * 2*Max_Index_Value + map_points_index[i][1]].emplace_back(map_points_index[i]);
#endif 

#if 0 // for update updataMissVoxel 2
      std::pair<int ,std::vector<int16_t>> pp((map_points_index[i][0] ) * Max_Index_Value + map_points_index[i][1] ,map_points_index[i]);
      voxel_index.emplace_back(pp);
#endif

#if 0 // for update updataMissVoxel 3
      voxel_index_node tmp;
      tmp.index = (map_points_index[i][0] ) * Max_Index_Value + map_points_index[i][1];
      tmp.map_index = map_points_index[i];
      voxel_index.emplace_back(tmp);
#endif

    }
  } //for 

  auto updateHitEndTime = getTime();
  std::cout << "[integrateMeasurement1] : update hit time = " << updateHitEndTime - updateHitStartTime << "\n";

  // printf("map_points_index.size = %i \n" , (int)map_points_index.size());
  printf("voxel_index.size = %i \n" , (int)voxel_index.size());

  auto updateFreeStartTime = getTime();
  
  map->updataMissVoxel1(map_camera_index, voxel_index);
  // map->updataMissVoxel2(map_camera_index, voxel_index);
  // map->updataMissVoxel3(map_camera_index, voxel_index);
  
  auto updateFreeEndTime = getTime();
  std::cout << "[integrateMeasurement1] : update free time = " << updateFreeEndTime - updateFreeStartTime << "\n";


  // timings.startTimer("clear");

  map->clearVoxelsUpdateFlag();

  // timings.printTime("clear");
  // integrationParameters.integration_counter++;
}



/**
 * @brief Get the Measure Points From Point Cloud object
 * 
 * @param msg 
 * @param RobotCameraRotationMatrix 
 * @param RobotCameraTransvec 
 * @param camera_points 
 * @param map_points_index 
 * @param depthThr 
 */
void getMeasurePointsFromPointCloud(const lcm_sensor_msgs::PointCloud &msg,
                                                                     const Eigen::Matrix3f &RobotCameraRotationMatrix,
                                                                     const Eigen::Vector3f &RobotCameraTransvec,
                                                                     const Eigen::Matrix3f &MapRobotRotationMatrix,
                                                                     const Eigen::Vector3f &MapRobotTransvec,
                                                                     std::vector<ColorPoint> &camera_points,
                                                                     std::vector<std::vector<int16_t>> &map_points_index,
                                                                     const float & maxDepthThrCamera,
                                                                     const float & heightThrMap,
                                                                     const float &minDepthThrCamera)
{
  map_points_index.reserve(msg.points.size());
  camera_points.reserve(msg.points.size());
  // 多多已转换到robot系下，这里无序转换
  Eigen::Matrix3f  MapCameraRotationMatrix = MapRobotRotationMatrix;
  Eigen::Vector3f MapCameraTransvec = MapRobotTransvec;
  // Eigen::Matrix3f  MapCameraRotationMatrix = MapRobotRotationMatrix*RobotCameraRotationMatrix;
  // Eigen::Vector3f MapCameraTransvec = MapRobotRotationMatrix*RobotCameraTransvec + MapRobotTransvec;  

  for (int i = 0 ;  i < msg.points.size(); i++)
  {
    //  1 : 过滤掉在相机系下没一定深度值内的点
    if ( msg.points[i].z < minDepthThrCamera || msg.points[i].z > maxDepthThrCamera)
    {
      continue;
    }
    // 2 ：将单个点云转换到map系下
    // Eigen::Vector3f camera_point(float(msg.points[i].x),float(msg.points[i].y),float(msg.points[i].z));
    // camera 和 robot 的坐标系方向不同
    static Eigen::Vector3f camera_point;
    camera_point << float(msg.points[i].z),-float(msg.points[i].x),-float(msg.points[i].y);

    static Eigen::Vector3f map_point;
    map_point = MapCameraRotationMatrix*camera_point + MapCameraTransvec;

    // 3 : 过滤掉map下过高的点及地面以下的点
    if (map_point[2] > heightThrMap || map_point[2] < 0 )
    {
      continue;
    }

    // 4 ：turn to  map index
    int16_t ix, iy ,iz;
    if(map->integrateVoxelWithTable(int(map_point[0]*1000), 
                                                              int(map_point[1]*1000),
                                                              int(map_point[2]*1000),
                                                               ix, iy, iz))
    {
      std::vector<int16_t> data{ix , iy , iz};
      map_points_index.emplace_back(data);
      ColorPoint cp;
      cp.point.x = msg.points[i].x;
      cp.point.y = msg.points[i].y;
      cp.point.z = msg.points[i].z;
      camera_points.emplace_back(cp);
    }
  }// for

}

lcm_visualization_msgs::Marker createVisualizationMarker(std::string frame_id,
                                                                                                        lcm_ros::Time time, 
                                                                                                        int id,
                                                                                                       VisualizationType type) {
  /**
   * Creating Visualization Marker
   */
  lcm_visualization_msgs::Marker marker;

  marker.action = lcm_visualization_msgs::Marker::ADD;
  marker.type = lcm_visualization_msgs::Marker::SPHERE_LIST;
  marker.header.frame_id = frame_id;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  if (type == VisualizationType::POINT_CLOUD) {
    marker.type = lcm_visualization_msgs::Marker::POINTS;
    marker.scale.x = mapParameters.map_resolution;
    marker.scale.y = mapParameters.map_resolution;
    marker.scale.z = mapParameters.map_resolution;
  } else if (type == VisualizationType::VOXEL_MAP) {
    marker.type = lcm_visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = mapParameters.map_resolution;
    marker.scale.y = mapParameters.map_resolution;
    marker.scale.z = mapParameters.map_resolution;
  } else if (type == VisualizationType::VOXEL_GRID) {
    marker.type = lcm_visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = mapParameters.map_resolution;
    marker.scale.y = mapParameters.map_resolution;
    marker.scale.z = mapParameters.map_resolution;
  }
  return marker;
}

void fillVisualizationMarkerWithVoxels( lcm_visualization_msgs::Marker &voxels_marker, 
                                                                     std::vector<Voxel3D> &voxels,
                                                                     int min_weight_th) 
{
  for (int i = 0; i < voxels.size(); i++)
  {
    // if(ValueToProbability(voxels[i].data->tableValue) < 0.7)
    //   continue;

    lcm_geometry_msgs::Point point;
    point.x = voxels[i].x;
    point.y = voxels[i].y;
    point.z = voxels[i].z;

    voxels_marker.points.emplace_back(point);
  
    lcm_std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 255;
    color.b = 0;
    color.a = voxels[i].data->tableValue;
    voxels_marker.colors.emplace_back(color);
  }

  voxels_marker.size = voxels_marker.points.size();
  voxels_marker.pose.position.x = 0;
  voxels_marker.pose.position.y = 0;
  voxels_marker.pose.position.z = 0;

  voxels_marker.pose.orientation.x = 0;
  voxels_marker.pose.orientation.y = 0;
  voxels_marker.pose.orientation.z = 0;
  voxels_marker.pose.orientation.w =0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
lcmHandler::lcmHandler()
{
  node_ = new lcm::LCM(getUdpmAddr(true), true);

  if(!node_->good())
  {
    std::cout << "[ERROR] : LCM node is not good ! \n";
  }
  // 在构造函数中注册回调函数，如果需要触发式的发布，则在callback中进行发布
  // node_->subscribe("slam_cmd", &lcmHandler::cmdCallback, this);
  node_->subscribe("pointcloud_obst",&lcmHandler::pointCloudCallback,this);
  node_->subscribe("good_odom",&lcmHandler::getRobotPoseCallback,this);
}

lcmHandler::~lcmHandler()
{
  delete node_;
  std::cout << "[~lcmHandler] : delete lcm node "<< "\n";
}

void lcmHandler::cmdCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcm_std_msgs::Float32MultiArray* cmd_in)
{
  printf("[cmdCallback] : in ... \n");
  if( int(cmd_in->data[1]) == 1666 )
  {
    printf("[test]:  Get SLAM_CMD 1666. \n");
  }
  if( int(cmd_in->data[1]) == 1667 )
  {
    printf("[test]:  Get SLAM_CMD 1667. \n");
  }
  printf("[cmdCallback] : out ... \n");
}


/**
 * @brief 
 * 
 * @param rbuf 
 * @param chan 
 * @param cloud 
 */
void lcmHandler::pointCloudCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const lcm_sensor_msgs::PointCloud* cloud)
{
  // std::cout << "[pointCloudCallback]: The number of points = " << cloud->n_points << "\n";
  lcm_sensor_msgs::PointCloud info = *cloud;
  std::unique_lock<std::mutex> lk(*pointcloud_buffer_mutex);
  
  if (pointCloudBuffer.size() > pointCloudBufferMaxSize )
  {
      pointCloudBuffer.pop();
  }
  pointCloudBuffer.push(info);

  // std::cout << "[pointCloudCallback] : time stamp = " << info.header.stamp.sec << "\n";
  // std::cout << "[pointCloudCallback] : buffer size = " << pointCloudBuffer.size() << "\n";
}



void lcmHandler::getRobotPoseCallback(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const lcm_nav_msgs::Odometry* msg)
{
  // std::cout << "[getRobotPoseCallback]: The msg  child_frame_id = " << msg->child_frame_id << "\n";
  static lcm_nav_msgs::Odometry  robot_pose = *msg;
  std::unique_lock<std::mutex> lock_(*pointcloud_buffer_mutex);
  if (robotPoseBuffer.size() > robotPoseBufferMaxSize )
  {
      robotPoseBuffer.pop();
  }
  robotPoseBuffer.push(robot_pose);
}


void lcmHandler::run()
{
  pointcloud_buffer_mutex = new std::mutex();
  skiMap_builder_thread = new std::thread(&lcmHandler::skiMapBuilderThread,this);
  thread_exit_flag=0;

  while (true)
  {
    auto res = node_->handleTimeout(0);
    if (res < 0 )
    {
      printf("[handleTimeout return] = %i \n",res);
      break;
    }
  }//while
}

void lcmHandler::exit()
{
  thread_exit_flag = 1;
  skiMap_builder_thread->join();
  delete skiMap_builder_thread;
  delete pointcloud_buffer_mutex;
}

/**
 * @brief skiMap建图线程
 * 
 */
void lcmHandler::skiMapBuilderThread()
{
  // set mapParameters
  mapParameters.map_resolution = Map_Resolution;
  mapParameters.ground_level = 0.05f;
  mapParameters.min_voxel_weight = 50;
  mapParameters.enable_chisel = false;
  mapParameters.height_color = false;
  mapParameters.chisel_step = 30;
  mapParameters.agent_height = 1.5f;

  map = new SKIMAP(mapParameters.map_resolution, mapParameters.ground_level);

  std::cout << "[mapParameters]: " << mapParameters.map_resolution << "\n";
  int calCount = 0 ;

  while(thread_exit_flag == 0 )
  {
    /*
     * @brief （1）从pointCloudBuffer中获取点云 
     * 
     */
    std::unique_lock<std::mutex> lk1(*pointcloud_buffer_mutex);
    if ( pointCloudBuffer.empty() || robotPoseBuffer.empty() )
    {
      usleep(20);
      continue;
    }
    std::cout << "\n[skiMapBuilderThread]: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   Begin a new frame  ...  \n" 
                    << "count= " << calCount << "\n";

    auto startTime_ = getTime();
    // 从pointCloudBiffer中取出点云
    lcm_sensor_msgs::PointCloud curCloud =  pointCloudBuffer.front();
    pointCloudBuffer.pop();
    lcm_nav_msgs::Odometry curPose =  robotPoseBuffer.front();
    robotPoseBuffer.pop();
    lk1.unlock();
    auto getPointCloudEndTime = getTime();
    std::cout << "[skiMapBuilderThread]: get PointCloud time = " << getPointCloudEndTime - startTime_ << "\n";
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*
     * @brief （2）获取用于建图的点云数据measurement，滤点，用外参将点云转到map系下
     * 
     */
    auto makeMapPointsStartTime = getTime();

    SensorMeasurement measurement;
    std::vector<std::vector<int16_t>> map_points_index;

    // 相机外参
    Eigen::Matrix3f RobotCameraRotationMatrix = Eigen::Matrix3f::Identity(); // camera  在 map 下 pose 
    Eigen::Vector3f RobotCameraTransvec(0.0f,0.046f,0.0f);
    // 机器人Pose
    Eigen::Matrix3f MapRobotRotationMatrix = Eigen::Matrix3f::Identity();
    Eigen::Vector3f MapRobotTransvec (curPose.pose.pose.position.x, 
                                                                  curPose.pose.pose.position.y, 
                                                                  0.0);
    setRotationMat(MapRobotRotationMatrix,0.f,0.f,(float)curPose.pose.pose.position.z);

    auto makeMapPointsPosition1Time = getTime();

    const float maxDepthThrCamera = 0.5;
    const float heightThrMap = maxDepthThrCamera*0.8;
    const float minDepthThrCamera = 0.03;
    getMeasurePointsFromPointCloud(curCloud,
                                                                RobotCameraRotationMatrix,
                                                                RobotCameraTransvec,
                                                                MapRobotRotationMatrix,
                                                                MapRobotTransvec,
                                                                measurement.points,
                                                                map_points_index,
                                                                maxDepthThrCamera,
                                                                heightThrMap,
                                                                minDepthThrCamera);

    measurement.stamp = curCloud.header.stamp;

    auto makeMapPointsPosition2Time = getTime();

    // 将camera在map系下的原点位置存为整数
    int16_t ix, iy, iz;
    map->integrateVoxelWithTable(int(MapRobotTransvec[0]*1000), 
                                                           int(MapRobotTransvec[1]*1000), 
                                                           0, 
                                                           ix, iy, iz);
    std::vector<int16_t> map_camera_index{ix , iy , iz};

    auto makeMapPointsEndTime = getTime();


    std::cout << "[skiMapBuilderThread]: make map points time =  "<<  makeMapPointsEndTime - makeMapPointsStartTime <<" , ";
    std::cout << "[SensorMeasurement] :" << measurement.points.size() << "\n";
    std::cout << "[make point time ]:  position 1 =  "<<  makeMapPointsPosition1Time - makeMapPointsStartTime <<" \n";
    std::cout << "[make point time ]:  position 2 =  "<<  makeMapPointsPosition2Time - makeMapPointsPosition1Time <<" \n";
    std::cout << "[make point time ]:  position 3 =  "<<  makeMapPointsEndTime - makeMapPointsPosition2Time <<" \n";
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
     * @brief （3）update map
     * 
     */
    auto mapIntegrationStartTime_ = getTime();
    // 默认体素的初始值为0，权重为1 
    VoxelDataColor voxelInit (0,255,0,1.0);
    integrateMeasurement1(map_points_index, voxelInit, map, map_camera_index);

    auto mapIntegrationEndTime_ = getTime();
    std::cout << "[skiMapBuilderThread]: integrateMeasurement  time =  "<<  mapIntegrationEndTime_ - mapIntegrationStartTime_ <<" \n";

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
     * @brief （4）make voxels
     * 
     */
    auto mapBuilderStartTime_ = getTime();
    std::vector<Voxel3D> voxels_new; 
    // map->fetchVoxels(voxels1); // voxels存储所有map中的体素
    map->fetchUpdateVoxelsOnly(voxels_new);
    auto mapBuilderEndTime_ = getTime();
    std::cout << "[skiMapBuilderThread]: make voxels time =  "<<  mapBuilderEndTime_ - mapBuilderStartTime_ <<" \n";


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    auto endTime_ = getTime();
    auto mapPublisherStartTime_ = getTime();
    
    if (voxels_new.size() > 0 )
    {
      std::string base_frame_name = "map";
      lcm_visualization_msgs::Marker map_marker = createVisualizationMarker(base_frame_name, 
                                                                                                                                      measurement.stamp,
                                                                                                                                      1,
                                                                                                                                      VisualizationType::VOXEL_MAP);    
      fillVisualizationMarkerWithVoxels(map_marker, 
                                                                 voxels_new,
                                                                 mapParameters.min_voxel_weight);
      node_->publish("map_3d",&map_marker);
    }
    
    auto mapPublisherEndTime_ = getTime();
    std::cout << "[skiMapBuilderThread]: publisher map time =  "<<  mapPublisherEndTime_ - mapPublisherStartTime_ <<" \n";
    std::cout << "[skiMapBuilderThread]: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   Frame handle time =  "<<  endTime_ - startTime_ <<" \n";
    std::cout << "\n";
    calCount ++;
  }//while
  std::cout  << "[skiMapBuilderThread] skiMapBuilderThread  Exit";
}