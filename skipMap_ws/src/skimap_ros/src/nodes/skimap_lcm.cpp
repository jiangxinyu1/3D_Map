#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// LCM
#include "lcm/lcm-cpp.hpp"
#include "lcm_sensor_msgs/LaserScan.hpp"

// OPENCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>

#define point_z_filtter

// skimap
typedef skimap::VoxelDataRGBW<uint16_t, float> VoxelDataColor;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float> SKIMAP;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float>::Tiles2D Tiles2D;
SKIMAP *map;

// Ros
ros::NodeHandle *nh;
tf::TransformListener *tf_listener;
ros::Publisher cloud_publisher;
ros::Publisher map_publisher;
ros::Publisher map_2d_publisher;

// Live Params
std::string base_frame_name = "slam_map";
std::string camera_frame_name = "camera";

/**
 */
struct MapParameters {
  float ground_level;
  float agent_height;
  float map_resolution;
  int min_voxel_weight;
  bool enable_chisel;
  bool height_color;
  int chisel_step;
} mapParameters;

/**
 */
struct CameraParameters {
  double fx, fy, cx, cy;
  int cols, rows;
  double min_distance;
  double max_distance;
  int point_cloud_downscale;
} camera;

/**
 */
struct ColorPoint {
  cv::Point3f point;
  cv::Vec4b color;
  int w;
};

/**
 */
struct SensorMeasurement {
  ros::Time stamp;
  std::vector<ColorPoint> points;
  std::vector<ColorPoint> chisel_points;

  void addChiselPoints(CameraParameters &camera, float resolution) {
    chisel_points.clear();

#pragma omp parallel
    {
      std::vector<ColorPoint> new_points;

#pragma omp for nowait
      for (int i = 0; i < points.size(); i++) {
        cv::Point3f dir = points[i].point - cv::Point3f(0, 0, 0);
        dir = dir * (1 / cv::norm(dir));
        for (float dz = camera.min_distance; dz < points[i].point.z;
             dz += resolution) {
          cv::Point3f dp = dir * dz;
          ColorPoint colorPoint;
          colorPoint.point = dp;
          colorPoint.w = -mapParameters.chisel_step;
          new_points.push_back(colorPoint);
        }
      }

#pragma omp critical
      points.insert(points.end(), new_points.begin(), new_points.end());
    }

    //        points.insert(points.end(), chisel_points.begin(),
    //        chisel_points.end());
  }
};

std::queue<SensorMeasurement> measurement_queue;
int measurement_queue_max_size = 2;

inline int64_t getTime(){
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec*1000.0+round(t.tv_nsec/1000000.0);
}

/**
 */
struct Timings {
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;
  typedef std::chrono::microseconds us;
  typedef std::chrono::duration<float> fsec;

  std::map<std::string, std::chrono::time_point<std::chrono::system_clock>>
      times;

  void startTimer(std::string name) {
    times[name] = Time::now(); // IS NOT ROS TIME!
  }

  us elapsedMicroseconds(std::string name) {
    fsec elaps = Time::now() - times[name];
    return std::chrono::duration_cast<us>(elaps);
  }

  ms elapsedMilliseconds(std::string name) {
    fsec elaps = Time::now() - times[name];
    return std::chrono::duration_cast<ms>(elaps);
  }

  void printTime(std::string name) {
    ROS_INFO("Time for %s: %f ms", name.c_str(),
             float(elapsedMicroseconds(name).count()) / 1000.0f);
  }
} timings;

/**
 * Extracts point cloud from RGB-D Frame
 * @param rgb RGB image
 * @param depth Depth image
 * @param camera Camera parameters
 * @param sample_jumps downsample factor
 * @param output_points OUTPUT vector containing points
 */
void extractPointCloud(cv::Mat rgb, cv::Mat depth, CameraParameters camera,
                       int sample_jumps,
                       std::vector<ColorPoint> &output_points) {
  sample_jumps = sample_jumps > 1 ? sample_jumps : 1;

  output_points.clear();

  for (float y = 0; y < depth.rows; y += sample_jumps) {
    for (float x = 0; x < depth.cols; x += sample_jumps) {
      float d = depth.at<float>(y, x);
      ColorPoint cp;
      cp.point.x = (d / camera.fx) * (x - camera.cx);
      cp.point.y = (d / camera.fy) * (y - camera.cy);
      cp.point.z = d;
      if (cp.point.z != cp.point.z)
        continue;
      if(cp.point.y > 1) continue;
      if (cp.point.z >= camera.min_distance &&
          cp.point.z <= camera.max_distance) {
        cp.color = rgb.at<cv::Vec4b>(y, x);
        cp.w = 1;
        output_points.push_back(cp);
      }
    }
  }
}

/**
 * Visualizatoin types for Markers
 */
enum VisualizationType {
  POINT_CLOUD,
  VOXEL_MAP,
  VOXEL_GRID,
};

/**
 * Creates a "blank" visualization marker with some attributes
 * @param frame_id Base TF Origin for the map points
 * @param time Timestamp for relative message
 * @param id Unique id for marker identification
 * @param type Type of Marker.
 * @return
 */
visualization_msgs::Marker createVisualizationMarker(std::string frame_id,
                                                     ros::Time time, int id,
                                                     VisualizationType type) {

  /**
   * Creating Visualization Marker
   */
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = id;

  if (type == VisualizationType::POINT_CLOUD) {
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
  } else if (type == VisualizationType::VOXEL_MAP) {
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = mapParameters.map_resolution;
    marker.scale.y = mapParameters.map_resolution;
    marker.scale.z = mapParameters.map_resolution;
  } else if (type == VisualizationType::VOXEL_GRID) {
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = mapParameters.map_resolution;
    marker.scale.y = mapParameters.map_resolution;
    marker.scale.z = mapParameters.map_resolution;
  }
  return marker;
}

/**
 * Creates a Visualization Marker representing a Voxel Map of the environment
 * @param voxels_marker Marker to fill
 * @param voxels 3D Voxel list
 * @param min_weight_th Minimum weight for a voxel to be displayed
 */
void fillVisualizationMarkerWithVoxels(
    visualization_msgs::Marker &voxels_marker, std::vector<Voxel3D> &voxels,
    int min_weight_th) {

  cv::Mat colorSpace(1, voxels.size(), CV_32FC3);
  if (mapParameters.height_color) {
    for (int i = 0; i < voxels.size(); i++) {
      colorSpace.at<cv::Vec3f>(i)[0] = 180 - (voxels[i].z / 2) * 180;
      colorSpace.at<cv::Vec3f>(i)[1] = 1;
      colorSpace.at<cv::Vec3f>(i)[2] = 1;
    }
    cv::cvtColor(colorSpace, colorSpace, CV_HSV2BGR);
  }

  for (int i = 0; i < voxels.size(); i++) {

    // if (voxels[i].data->w < min_weight_th)
    //   continue;

    if((float)voxels[i].data->w / (voxels[i].data->w + voxels[i].data->miss) < 0.6)
      continue;
    /**
     * Create 3D Point from 3D Voxel
     */
    geometry_msgs::Point point;
    point.x = voxels[i].x;
    point.y = voxels[i].y;
    point.z = voxels[i].z;

    /**
     * Assign Cube Color from Voxel Color
     */
    std_msgs::ColorRGBA color;
    if (mapParameters.height_color) {
      color.r = colorSpace.at<cv::Vec3f>(i)[2];
      color.g = colorSpace.at<cv::Vec3f>(i)[1];
      color.b = colorSpace.at<cv::Vec3f>(i)[0];
    } else {
      color.r = float(voxels[i].data->r) / 255.0;
      color.g = float(voxels[i].data->g) / 255.0;
      color.b = float(voxels[i].data->b) / 255.0;
    }
    color.a = 1;

    voxels_marker.points.push_back(point);
    voxels_marker.colors.push_back(color);
  }
}

/**
 * Fills Visualization Marker with 2D Tiles coming from a 2D Query in SkiMap.
 * Represent in a black/white chessboard the occupied/free space respectively
 *
 * @param voxels_marker Marker to fill
 * @param tiles Tiles list
 */
void fillVisualizationMarkerWithTiles(visualization_msgs::Marker &voxels_marker,
                                      std::vector<Tiles2D> &tiles) {
  for (int i = 0; i < tiles.size(); i++) {

    /**
     * Create 3D Point from 3D Voxel
     */
    geometry_msgs::Point point;
    point.x = tiles[i].x;
    point.y = tiles[i].y;
    point.z = tiles[i].z;

    /**
     * Assign Cube Color from Voxel Color
     */
    std_msgs::ColorRGBA color;
    if (tiles[i].data != NULL) {
      color.r = color.g = color.b =
          tiles[i].data->w >= mapParameters.min_voxel_weight ? 0.0 : 1.0;
      color.a = 1;
    } else {
      color.r = color.g = color.b = 1.0;
      color.a = 1;
    }

    voxels_marker.points.push_back(point);
    voxels_marker.colors.push_back(color);
  }
}

/**
 * Fills a Visualization Marker with points coming from a SensorMeasuremetn
 * object. It's used
 * to show the Live Cloud
 *
 * @param voxels_marker
 * @param measurement
 * @param min_weight_th
 */
void fillVisualizationMarkerWithSensorMeasurement(
    visualization_msgs::Marker &voxels_marker, SensorMeasurement measurement) {
  for (int i = 0; i < measurement.points.size(); i++) {
    /**
     * Create 3D Point from 3D Voxel
     */
    geometry_msgs::Point point;
    point.x = measurement.points[i].point.x;
    point.y = measurement.points[i].point.y;
    point.z = measurement.points[i].point.z;

    /**
     * Assign Cube Color from Voxel Color
     */
    std_msgs::ColorRGBA color;
    color.r = measurement.points[i].color[2] / 255.0;
    color.g = measurement.points[i].color[1] / 255.0;
    color.b = measurement.points[i].color[0] / 255.0;
    color.a = 1;

    voxels_marker.points.push_back(point);
    voxels_marker.colors.push_back(color);
  }
}

/**
 */
struct IntegrationParameters {
  std::vector<VoxelDataColor> voxels_to_integrate;
  std::vector<tf::Vector3> poses_to_integrate;
  std::vector<bool> tiles_mask;
  int integration_counter;

  IntegrationParameters() { integration_counter = 0; }
} integrationParameters;

/**
 * Integrates measurements in global Map. Integration is made with OpenMP if
 * possibile so
 * real integration function is surrounded by "startBatchIntegration" and
 * "commitBatchIntegration". By removing
 * these two lines the integration will be launched in single-thread mode
 * @param measurement
 * @param map
 * @param base_to_camera
 */
void integrateMeasurement(const std::vector<tf::Vector3>& map_points, std::vector<VoxelDataColor>& voxels, SKIMAP *&map,
                          tf::Transform base_to_camera) {
  map->enableConcurrencyAccess(true);
  // #pragma omp parallel shared(points, map)
  int point_num = 0;
  for (int i = 0; i < map_points.size(); i++) {
    point_num ++;

    // 将每一个点插入到map中，如果对应的skipmap(x,y,z)为空，插入voxel，否则对应的data+1
    map->integrateVoxel(float(map_points[i].x()), float(map_points[i].y()),
                        float(map_points[i].z()), &voxels[i]);
  }
  for (int i = 0; i < map_points.size(); i++) {
    // 对于每一个点进行ray cast 更新 中间miss的voxel miss + 1
    map->updataMissVoxel(float(map_points[i].x()), float(map_points[i].y()), float(map_points[i].z()), 
                                                    float(base_to_camera.getOrigin().x()), float(base_to_camera.getOrigin().y()), float(base_to_camera.getOrigin().z()), &voxels[i]);
  }
  printf("point_num  = %d \n" , point_num);

  map->clearVoxelsUpdateFlag();

  integrationParameters.integration_counter++;
}

void integrateMeasurement(const std::vector<std::vector<int16_t>>& map_points_index, std::vector<VoxelDataColor>& voxels, SKIMAP *&map,
                          const std::vector<int16_t>& map_camera_index) {
  map->enableConcurrencyAccess(true);
  // #pragma omp parallel shared(points, map)
  int point_num = 0;
  for (int i = 0; i < map_points_index.size(); i++) {
    point_num ++;

    // 将每一个点插入到map中，如果对应的skipmap(x,y,z)为空，插入voxel，否则对应的data+1
    map->integrateVoxel(map_points_index[i], &voxels[i]);
  }
  for (int i = 0; i < map_points_index.size(); i++) {
    // 对于每一个点进行ray cast 更新 中间miss的voxel miss + 1
    map->updataMissVoxel(map_points_index[i], map_camera_index, &voxels[i]);
  }
  printf("point_num  = %d \n" , point_num);
  
  map->clearVoxelsUpdateFlag();

  integrationParameters.integration_counter++;
}

/**
 * RGB + DEPTH callback
 */
void callback(const sensor_msgs::ImageConstPtr &rgb_msg,
              const sensor_msgs::ImageConstPtr &depth_msg) {

  tf::StampedTransform base_to_camera;
  try {
    tf_listener->lookupTransform(base_frame_name, camera_frame_name,
                                ros::Time(0) , base_to_camera);//ros::Time(0)  rgb_msg->header.stamp
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // RGB
  cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
  cv::cvtColor(rgb, rgb, CV_BGR2BGRA);

  // DEPTH
  // cv_bridge::CvImagePtr cv_ptr;
  // try {
  //   cv_ptr = cv_bridge::toCvCopy(depth_msg);
  // } catch (cv_bridge::Exception &e) {
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // // Millimeters to meter conversion
  // cv::Mat depth = cv::Mat(rgb.rows, rgb.cols, CV_32FC1);
  // const uint8_t *depth_mm_p = cv_ptr->image.ptr<uint8_t>(0);
  // float *depth_p = depth.ptr<float>(0);
  // int size = rgb.rows * rgb.cols;
  // for (int j = 0; j < size; j++) {
  //   depth_p[j] = float(depth_mm_p[j]) / 5000.0f;
  //   printf("%f, ",  float(depth_mm_p[j]));
  // }
  // printf("\n");

  // TUM 数据集
  cv_bridge::CvImageConstPtr depth_ptr;
  depth_ptr =
      cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat  depth = depth_ptr->image;
  for (int m = 0; m < depth.rows; m++){
    for (int n = 0; n < depth.cols; n++){
      float d = depth.ptr<float>(m)[n];
    }
  }

  /**
   * Extracts sensor measurement
   */
  SensorMeasurement measurement;
  measurement.stamp = rgb_msg->header.stamp;
  extractPointCloud(rgb, depth, camera, camera.point_cloud_downscale,
                    measurement.points);

  printf("measurement.points.size = %d \n" , measurement.points.size());

  std::vector<tf::Vector3> map_points;
  std::vector<VoxelDataColor> voxels;
  std::vector<std::vector<int16_t>> map_points_index;
  
  for (int i = 0; i < measurement.points.size(); i++) {
    float tmpx = measurement.points[i].point.x ;
    float tmpy = measurement.points[i].point.y ;
    measurement.points[i].point.x = measurement.points[i].point.z;
    measurement.points[i].point.y = -tmpx;
    measurement.points[i].point.z = -tmpy;

    tf::Vector3 base_to_point(measurement.points[i].point.x, measurement.points[i].point.y, measurement.points[i].point.z);
    base_to_point = base_to_camera * base_to_point;
    // #ifdef point_z_filtter
    // if(base_to_point[2] > 1) continue;
    // #endif
    // map_points.push_back(base_to_point);
    // VoxelDataColor voxel(measurement.points[i].color[2], measurement.points[i].color[1],
    //                   measurement.points[i].color[0], 1.0);
    // voxels.push_back(voxel);

    int16_t ix, iy, iz;
    if(map->integrateVoxel(base_to_point.x(), base_to_point.y(), base_to_point.z(), ix, iy, iz)){
      map_points.push_back(base_to_point);
      VoxelDataColor voxel(measurement.points[i].color[2], measurement.points[i].color[1],
                        measurement.points[i].color[0], 1.0);
      voxels.push_back(voxel);
      std::vector<int16_t> data{ix , iy , iz};
      map_points_index.push_back(data);
    }
  }

  int16_t ix, iy, iz;
  map->integrateVoxel(base_to_camera.getOrigin().x(), base_to_camera.getOrigin().y(), base_to_camera.getOrigin().z(), ix, iy, iz);
  std::vector<int16_t> map_camera_index{ix , iy , iz};

  /**
   * Map Integration
   */
  timings.startTimer("Integration");
  // integrateMeasurement(map_points, voxels, map, base_to_camera);

  integrateMeasurement(map_points_index, voxels, map, map_camera_index);
  timings.printTime("Integration");

  /**
   * 3D Map Publisher
   */
  std::vector<Voxel3D> voxels1; 
  map->fetchVoxels(voxels1); // voxels存储所有map中的体素
  if(voxels1.size() != 0){
    visualization_msgs::Marker map_marker = createVisualizationMarker(
        base_frame_name, rgb_msg->header.stamp, 1, VisualizationType::VOXEL_MAP);
    fillVisualizationMarkerWithVoxels(map_marker, voxels1,
                                      mapParameters.min_voxel_weight);
    map_publisher.publish(map_marker);
  }
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
  
  std::string url = "udpm://239.255.76.67:7666";
    // string url = "udpm://239.255.76.67:7047";
  lcm::LCM lcm(url,true);
  ScanMsgHandle handler;
  lcm.subscribe("scan", &ScanMsgHandle::scanCallback,&handler);
    


  // Initialize ROS
  ros::init(argc, argv, "skimap_live");
  nh = new ros::NodeHandle("~");
  tf_listener = new tf::TransformListener();

  // Cloud Publisher
  std::string map_cloud_publisher_topic =
      nh->param<std::string>("map_cloud_publisher_topic", "live_cloud");
  std::string map_topic =
      nh->param<std::string>("map_publisher_topic", "live_map");
  std::string map_2d_topic =
      nh->param<std::string>("map_2d_publisher_topic", "live_map_2d");
  cloud_publisher =
      nh->advertise<visualization_msgs::Marker>(map_cloud_publisher_topic, 1);
  map_publisher = nh->advertise<visualization_msgs::Marker>(map_topic, 1);
  map_2d_publisher = nh->advertise<visualization_msgs::Marker>(map_2d_topic, 1);

  int hz;
  nh->param<int>("hz", hz, 30);

  bool viz;
  nh->param<bool>("viz", viz, true);

  // Camera params
  camera.fx = 542.461710;
  camera.fy = 543.536535;
  camera.cx = 311.081384;
  camera.cy = 236.535761;
  camera.cols = 640;
  camera.rows = 480;
  camera.min_distance = 0.4;
  camera.max_distance = 3.0;

  // SkiMap
  mapParameters.map_resolution = 0.05f;
  mapParameters.ground_level = 0.15f;
  mapParameters.min_voxel_weight = 10;
  mapParameters.enable_chisel = false;
  mapParameters.height_color = false;
  mapParameters.chisel_step = 10;
  mapParameters.agent_height = 1.0f;
  map = new SKIMAP(mapParameters.map_resolution, mapParameters.ground_level);

  // Topics
  std::string camera_rgb_topic, camera_depth_topic;
  nh->param<std::string>("camera_rgb_topic", camera_rgb_topic,
                         "/camera/rgb/image_raw");
  nh->param<std::string>("camera_depth_topic", camera_depth_topic,
                         "/camera/depth/image_raw");
  nh->param<std::string>("base_frame_name", base_frame_name, "slam_map");
  nh->param<std::string>("camera_frame_name", camera_frame_name, "camera");
  nh->param<int>("point_cloud_downscale", camera.point_cloud_downscale, 1);

  // Image/Depth synchronized callbacks
  int camera_queue_size = 1;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ApproxSync;
  message_filters::Subscriber<sensor_msgs::Image> m_rgb_sub(
      *nh, camera_rgb_topic, camera_queue_size);
  message_filters::Subscriber<sensor_msgs::Image> m_depth_sub(
      *nh, camera_depth_topic, camera_queue_size);
  message_filters::Synchronizer<ApproxSync> sync(ApproxSync(100), m_rgb_sub,
                                                 m_depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Spin & Time
  ros::Rate r(hz);

  // Spin
  while (nh->ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
