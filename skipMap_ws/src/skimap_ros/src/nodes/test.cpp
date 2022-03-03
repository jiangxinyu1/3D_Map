/*
 * @Author: your name
 * @Date: 2022-02-08 14:06:06
 * @LastEditTime: 2022-03-02 15:51:01
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /skipMap_ws/src/skimap_ros/src/nodes/test.cpp
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unordered_map>
//ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


ros::NodeHandle *nh;
ros::Publisher mapfull_publisher;
// visualization_msgs::Marker::ConstPtr fullmap =;
std::string base_frame_name = "map";

std::vector<std::vector<std::vector<float>>> voxels;

enum VisualizationType {
  POINT_CLOUD,
  VOXEL_MAP,
  VOXEL_GRID,
};


void debugPrint(const std::string &positionId)
{
  std::cout << "[debug]: +++++++  " << positionId << "\n";
}

visualization_msgs::Marker createVisualizationMarker(std::string frame_id,
                                                     ros::Time time, int id,
                                                     VisualizationType type) {
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
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
  } else if (type == VisualizationType::VOXEL_GRID) {
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
  }
  return marker;
}

struct Point3i{
  int xcm;
  int ycm;
  int zxm;
};



void mapCallback ( const visualization_msgs::Marker::ConstPtr& msg)
{
  std::cout << "Get new map , map size = " << msg->points.size() << "\n";
  // todo : make full map 
  // visualization_msgs::Marker::ConstPtr fullmap = new visualization_msgs::Marker (msg);
  // 
  debugPrint("1");

  if ( msg->points.size() == 0 )
  {
    return;
  }
  debugPrint("3");
  
  std::cout << " msg->points.size() = " <<  msg->points.size() << "\n";
  std::cout << " msg->colors.size() = " <<  msg->colors.size() << "\n";

  for ( int i = 0 ; i < msg->points.size(); i++ )
  {
    debugPrint("3.1");
    int x = std::round(msg->points[i].x*100);
    int y = std::round(msg->points[i].y*100);
    int z = std::round(msg->points[i].z*100);

    x = ( x < 0 ) ? -x : 2*x;
    y = ( y < 0 ) ? -y : 2*y;
    if (z < 0 || x < 0 || x >= 4000 || y < 0 || y >= 4000 || z >= 50)
    {
      continue;
    }
    debugPrint("3.2");
    voxels[x][y][z] = msg->colors[i].a;
  }
  debugPrint("4");
  visualization_msgs::Marker fullmap = createVisualizationMarker(base_frame_name, 
                                                                                                                             msg->header.stamp,
                                                                                                                             1,
                                                                                                                             VisualizationType::VOXEL_MAP);
debugPrint("5");
 for ( int x = 0 ; x < 4000 ; x++)
 {
   for (int y = 0 ; y < 4000 ; y++)
   {
     for (int z = 0 ; z < 50 ; z++ )
     {
       if (voxels[x][y][z] > 0.7)
       {
          geometry_msgs::Point point;
          point.x = (x < 2000) ? ((-x)/100.f) : (x/2.f/100.f);
          point.y = (y < 2000) ? ((-y)/100.f) : (y/2.f/100.f);
          point.z = z/100.f;
          std_msgs::ColorRGBA color;
          color.r = 255.0;
          color.b = 0;
          color.g = 0;
          color.a = 1;
          fullmap.points.emplace_back(point);
          fullmap.colors.emplace_back(color);
       } 
     }
   } 
 }
 debugPrint("6");
   mapfull_publisher.publish(fullmap);     
}

/** MAIN NODE **/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_test");
    nh = new ros::NodeHandle("~");
   voxels.resize(4000);
  for (int i = 0 ; i < 4000; i ++)
  {
    voxels[i].resize(4000);
    for (int j = 0; j < 50 ; j++)
    {
      voxels[i][j].resize(50);
    } 
  } 
    mapfull_publisher = nh->advertise<visualization_msgs::Marker>("full_map", 1);
   ros::Subscriber sub = nh->subscribe("/map_3d", 1000, mapCallback);
   ros::spin();

    // while (nh->ok())
    // {
    //     ros::spinOnce();
    // }
}
