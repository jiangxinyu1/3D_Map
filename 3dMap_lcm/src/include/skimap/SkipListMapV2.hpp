/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SkipListMapV2_HPP
#define SkipListMapV2_HPP

#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <omp.h>
#include <vector>
#include <skimap/SkipList.hpp>
#include <skimap/SkipListDense.hpp>
#include <skimap/voxels/GenericVoxel3D.hpp>
#include <skimap/probability_values.hpp>
#include <algorithm>

#define SkipListMapV2_MAX_DEPTH 16

struct voxel_index_node{
  int index; 
  std::vector<int16_t> map_index;
};


namespace skimap {

  #define Min_Index_Value -2000 //  map系栅格的最大值
  #define Max_Index_Value 2000 // map系栅格的最大值
  #define z_node_min -5 // z 方向栅格的最小值
  #define z_node_max 15 //z 方向栅格的最大值

  #define Map_Resolution 0.01

  // #define Min_Index_Value -400
  // #define Max_Index_Value 400

  // #define z_node_min -5
  // #define z_node_max 40

/**
     *
     * @param min_index
     * @param max_index
     */
template <class V, class K, class D, int X_DEPTH = 8, int Y_DEPTH = 8,int Z_DEPTH = 8>
class SkipListMapV2 
{
public:
  typedef GenericVoxel3D<V, D> Voxel3D;

  /**
  * Batch integration entry
  */
  struct IntegrationEntry {
    K x, y, z;
    V *data;
    int entry_depth;

    IntegrationEntry(K x, K y, K z, V *data, int max_depth = 3)
        : x(x), y(y), z(z), data(data), entry_depth(max_depth) {}
  };

  /**
       * Integration map for batch OMP integration
       */
  struct IntegrationMap 
  {
    std::map<K, std::vector<IntegrationEntry>> map;
    std::vector<K> map_keys;

    IntegrationMap() {}
    void addEntry(K x, K y, K z, V *data, int max_depth = 3) {
      IntegrationEntry entry(x, y, z, data, max_depth);
      if (map.find(x) == map.end()) {
        map[x] = std::vector<IntegrationEntry>();
        map_keys.emplace_back(x);
      }
      map[x].emplace_back(entry);
    }

    void clear() {
      map.clear();
      map_keys.clear();
    }
  };

  typedef K Index;
  typedef SkipListDense<Index, V *, Z_DEPTH> Z_NODE;
  typedef SkipListDense<Index, Z_NODE *, Y_DEPTH> Y_NODE;
  typedef SkipListDense<Index, Y_NODE *, X_DEPTH> X_NODE;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  SkipListMapV2(K min_index, K max_index, D resolution_x, D resolution_y,D resolution_z)
                           : _min_index_value(min_index), _max_index_value(max_index),
                             _resolution_x(resolution_x), _resolution_y(resolution_y),
                             _resolution_z(resolution_z), _voxel_counter(0), _xlist_counter(0),
                             _ylist_counter(0), _bytes_counter(0), _batch_integration(false),
                             _initialized(false), _self_concurrency_management(false),
                            hit_table_(ComputeLookupTableToApplyOdds(Odds(0.55))),
                            miss_table_(ComputeLookupTableToApplyOdds(Odds(0.46))) 
  {
    initialize(_min_index_value, _max_index_value);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  SkipListMapV2(D resolution)
      : _min_index_value(Min_Index_Value),
        _max_index_value(Max_Index_Value),
        _resolution_x(resolution), _resolution_y(resolution),
        _resolution_z(resolution), _voxel_counter(0), _xlist_counter(0),
        _ylist_counter(0), _bytes_counter(0), _batch_integration(false),
        _initialized(false), _self_concurrency_management(false),
        hit_table_(ComputeLookupTableToApplyOdds(Odds(0.55))),
        miss_table_(ComputeLookupTableToApplyOdds(Odds(0.46))) ,
        coordinatesToIndexTable_(preComputeCoordinatesToIndexTable_(resolution,(Max_Index_Value*resolution*1000))),
        indexToCoordinatesTable_(preComputeIndexToCoordinatesTable_(resolution))
  {
    initialize(_min_index_value, _max_index_value);
    std::cout << "[coordinatesToIndexTable_] = \n";
    for (int i = 0; i<Max_Index_Value*resolution*1000; i++)
    {
      std::cout << "," << coordinatesToIndexTable_[i] ;
    }
    std::cout << "[indexToCoordinatesTable_] = \n";
    for (int i = 0; i<Max_Index_Value; i++)
    {
      std::cout << "," << indexToCoordinatesTable_[i] ;
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual ~SkipListMapV2() 
  {
    for (typename std::map<K, boost::mutex *>::iterator it = this->mutex_map.begin(); it != this->mutex_map.end(); ++it) 
    {
      delete it->second;
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  void initialize(K min_index, K max_index) 
  {
    if (_initialized) 
    {
      delete _root_list;
    }
    _root_list = new X_NODE(min_index, max_index);
    _bytes_counter += sizeof(X_NODE);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   bool isValidIndex(K ix, K iy, K iz) 
  {
    bool result = true;
    result &= ix <= Max_Index_Value && ix >= Min_Index_Value;
    result &= iy <= Max_Index_Value && iy >= Min_Index_Value;
    result &= iz <= Max_Index_Value && iz >= Min_Index_Value;
    return result;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool coordinatesToIndex(D x, D y, D z, K &ix, K &iy, K &iz) 
  {
    ix = K(floor(x / _resolution_x ));
    iy = K(floor(y / _resolution_y ));
    iz = K(floor(z / _resolution_z ));
    return isValidIndex(ix, iy, iz);
  }

  virtual bool coordinatesToIndexWithTable(int x, int y, int z, K &ix, K &iy, K &iz) 
  {
    ix = (K)getValFromCoordinatesToIndexTable(x);
    iy = (K)getValFromCoordinatesToIndexTable(y);
    iz = (K)getValFromCoordinatesToIndexTable(z);
    return isValidIndex(ix, iy, iz);
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool singleIndexToCoordinate(K index, D &coordinate, D resolution) 
  {
    coordinate = index * resolution + resolution * 0.5;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool indexToCoordinates(K ix, K iy, K iz, D &x, D &y, D &z) 
  {
    x = ix * _resolution_x + _resolution_x * 0.5;
    y = iy * _resolution_y + _resolution_y * 0.5;
    z = iz * _resolution_z + _resolution_z * 0.5;
    return true;
  }


  virtual bool indexToCoordinatesWithTable(int ix, int iy, int iz, D &x, D &y, D &z) 
  {
    x = (D)getValFromIndexToCoordinatesTable(ix);
    y = (D)getValFromIndexToCoordinatesTable(iy);
    z = (D)getValFromIndexToCoordinatesTable(iz); 
    return true;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual V *find(K ix, K iy, K iz) 
  {
    const typename X_NODE::NodeType *ylist = _root_list->find(ix);
    if (ylist != NULL && ylist->value != NULL) {
      const typename Y_NODE::NodeType *zlist = ylist->value->find(iy);
      if (zlist != NULL && zlist->value != NULL) {
        const typename Z_NODE::NodeType *voxel = zlist->value->find(iz);
        if (voxel != NULL) {
          return voxel->value;
        }
      }
    }
    return NULL;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual V *find_O(D x, D y, D z) 
  {
    K ix, iy, iz;
    if (coordinatesToIndex(x, y, z, ix, iy, iz)) 
    {
      return find(ix, iy, iz);
    }
    return NULL;
  }

  virtual V *find(D x, D y, D z) 
  {
    K ix, iy, iz;
    if (integrateVoxelWithTable(x*1000, y*1000, z*1000, ix, iy, iz)) 
    {
      return find(ix, iy, iz);
    }
    return NULL;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
  * Lock concurrency access to a X branch
  * @param key x index
  * @return
  */
  virtual void lockMap(K key) 
  {
    this->mutex_map_mutex.lock();
    if (this->mutex_map.count(key) == 0) 
    {
      mutex_map[key] = new boost::mutex();
    }
    this->mutex_map_mutex.unlock();
    mutex_map[key]->lock();
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
  * UnLock concurrency access to a X branch
  * @param key x index
  * @return
  */
  virtual void unlockMap(K key) {
    this->mutex_map_mutex.lock();
    if (this->mutex_map.count(key) > 0) {
      mutex_map[key]->unlock();
    }
    this->mutex_map_mutex.unlock();
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool integrateVoxel(D x, D y, D z, V *data) 
  {
    K ix, iy, iz;
    if (coordinatesToIndex(x, y, z, ix, iy, iz)) 
    {
      return integrateVoxel(ix, iy, iz, data);
    }
    return false;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool integrateVoxel(D x, D y, D z, K& ix , K& iy , K& iz) 
  {
    // std::cout << "[TEST] : TABLE = " << coordinatesToIndexTable_[222] << "\n";
    if (coordinatesToIndex(x, y, z, ix, iy, iz)) 
    {
      return true;
    }
    return false;
  }

  D getValFromIndexToCoordinatesTable(int index) 
  {
      if (index < 0)
      {
        return (D) -indexToCoordinatesTable_.at(-index);
      }
      return indexToCoordinatesTable_.at(index);
  }

  K getValFromCoordinatesToIndexTable(int k) 
  {
      if (k < 0)
      {
        return (K)-coordinatesToIndexTable_.at(-k);
      }
      return coordinatesToIndexTable_.at(k);
  }


  bool integrateVoxelWithTable(int x, int  y, int z, K& ix , K& iy , K& iz) 
  {
    ix = getValFromCoordinatesToIndexTable(x);
    iy = getValFromCoordinatesToIndexTable(y);
    iz = getValFromCoordinatesToIndexTable(z);
    return true;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  virtual int manhattan(cv::Point2i a,cv::Point2i b)
  {
    return std::abs(a.x-b.x)+std::abs(a.y-b.y);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual float pointDistance(cv::Point2i a,cv::Point2i b)
  {
    return sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual int bresenhamLine(cv::Point2i start, cv::Point2i end, std::vector<cv::Point2i> &pointlist)
  {
    if((int)pointlist.size() < (manhattan(start,end)+1))
      pointlist.resize(manhattan(start,end)+5);

    int w = end.x - start.x;
    int h = end.y - start.y;
    int dx = ((w>0)<<1) - 1;
    int dy = ((h>0)<<1) - 1;
    w = abs(w);
    h = abs(h);
    int f , y , x, delta1,delta2;
    int l=0;

    if( w > h )
    {
      f = 2*h - w;
      delta1 = 2*h;
      delta2 = (h-w)*2;
      for( x = start.x , y = start.y ; x!=end.x ; x += dx )
      {
        pointlist[l]=(cv::Point2i(x,y));
        l++;
        if( f < 0 )
        {
          f += delta1;
        }
        else
        {
          y += dy;
          f += delta2;
        }
      }
      pointlist[l]=(cv::Point2i(x,y));
      l++;
    }
    else
    {
      f = 2*w - h;
      delta1 = w*2;
      delta2 = (w-h)*2;
      for( x = start.x , y = start.y ; y!=end.y ; y += dy )
      {
        pointlist[l]=(cv::Point2i(x,y));
        l++;
        if( f < 0 )
        {
          f += delta1;
        }
        else
        {
          x += dx;
          f += delta2;
        }
      }
      pointlist[l]=(cv::Point2i(x,y));
      l++;
    }

    if((int)pointlist.size()>l){
      pointlist.resize(l);
      return l;
    }
    else{
      return pointlist.size();
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void updataMissVoxel(D x, D y, D z, D c_x , D c_y , D c_z , V *data) 
  {
    K ix, iy, iz;
    K c_ix, c_iy, c_iz;
    if (coordinatesToIndexWithTable(x*1000, y*1000, z*1000, ix, iy, iz)) {
      coordinatesToIndexWithTable(c_x*1000, c_y*1000, c_z*1000, c_ix, c_iy, c_iz);
      cv::Point2i start(c_ix , c_iy);
      cv::Point2i end(ix , iy);
      std::vector<cv::Point2i> pointset;
      bresenhamLine(start, end , pointset);
      float dis1 = pointDistance(start , end);
      for(int i = 1 ; i < pointset.size() - 1 ; i++){
        float dis2 = pointDistance(start , pointset[i]);
        float rate = dis2 / dis1;
      
        float z = rate * (iz - c_iz) + c_iz;
        K z_index = K(z + 0.5);

        if (isValidIndex(pointset[i].x, pointset[i].y, z_index)) 
        {
          const typename X_NODE::NodeType *ylist = _root_list->find(pointset[i].x);
          if (ylist == NULL) 
          {
            continue;
          }
          const typename Y_NODE::NodeType *zlist = ylist->value->find(pointset[i].y);
          if (zlist == NULL) 
          {
            continue;
          }
          const typename Z_NODE::NodeType *voxel = zlist->value->find(z_index);
          if (voxel == NULL) 
          {
            continue;
          } 
          else
          {
            if((voxel->value->w) > 0 && voxel->value->update == false)
            {
              voxel->value->tableValue = miss_table_[voxel->value->tableValue];
            }
            voxel->value->update = true;
            voxelsUpdate.emplace_back(voxel);
          }
        }
      }
    }
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void updataMissVoxel(const std::vector<int16_t>& map_point_index,const std::vector<int16_t>& map_camera_index, V *data) 
  {
    cv::Point2i start(map_camera_index[0] , map_camera_index[1]);
    cv::Point2i end(map_point_index[0] , map_point_index[1]);
    std::vector<cv::Point2i> pointset;
    bresenhamLine(start, end , pointset);
    float dis1 = pointDistance(start , end);
    for(int i = 1 ; i < pointset.size() - 1 ; i++){                        
      float dis2 = pointDistance(start , pointset[i]);
      float rate = dis2 / dis1;
    
      float z = rate * (map_point_index[2] - map_camera_index[2]) + map_camera_index[2];
      K z_index = K(z + 0.5);

      const typename X_NODE::NodeType *ylist = _root_list->find(pointset[i].x);
      if (ylist == NULL) {
        continue;
      }
      const typename Y_NODE::NodeType *zlist = ylist->value->find(pointset[i].y);
      if (zlist == NULL) {
        continue;
      }
      const typename Z_NODE::NodeType *voxel = zlist->value->find(z_index);
      if (voxel == NULL) {
        continue;
      } else {
        if((voxel->value->w) > 0 && voxel->value->update == false){
          voxel->value->tableValue = miss_table_[voxel->value->tableValue];
        }
        voxel->value->update = true;
        voxelsUpdate.emplace_back(voxel);
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void updataMissVoxel1(const std::vector<int16_t>& map_camera_index, 
                                                       const std::unordered_map<int,std::vector<std::vector<int16_t>>>& voxel_index)
  {
    // 以相机在map系下的中心作为起点
    cv::Point2i start(map_camera_index[0] , map_camera_index[1]);
    // 通过迭代器遍历 -> unordered map 的每一个value (x,y,z)
    for(auto it = voxel_index.begin() ; it != voxel_index.end() ; it++)
    {
      // 取出当前key值对应 xy-index 
      cv::Point2i end(it->second[0][0] , it->second[0][1]);
      std::vector<cv::Point2i> pointset;
      // 通过划线算法获取xy平面点之上的点集
      bresenhamLine(start, end , pointset);
      // 对当前(xy)对应的三维点的高度进行排序
      std::vector<int16_t> vec_z;
      for(auto cell : it->second)
      {
        vec_z.emplace_back(cell[2]);
      }
      std::sort(vec_z.begin() , vec_z.end());
      //  计算
      float dis1 = pointDistance(start , end);
      // 遍历每一个pointSet中的二维free点，计算其到相机中心的距离
      for(int i = 1 ; i < pointset.size() - 1 ; i++)
      {
        float dis2 = pointDistance(start , pointset[i]);
        float rate = dis2 / dis1;

        // 开始聚类：对高度进行简单的聚类，将聚类的结果存储在cluster中
        std::vector<std::pair<int , int>> cluster;
        std::pair<int , int> pp(vec_z[0] , vec_z[0]);
        if(vec_z.size() == 1) 
        {
          cluster.emplace_back(pp);
        }

        for(int index = 1 ; index < vec_z.size() ; index ++)
        {
          if(vec_z[index] - vec_z[index - 1] > 3)
          {
            pp.second = vec_z[index - 1];
            cluster.emplace_back(pp);
            pp.first = vec_z[index];
          }
          else if(index = vec_z.size() - 1)
          {
            pp.second = vec_z[index];
            cluster.emplace_back(pp);
          }
        }//for
        // 结束聚类
        
        const typename X_NODE::NodeType *ylist = _root_list->find(pointset[i].x);
        if (ylist == NULL)
        {
          continue;
        }
        const typename Y_NODE::NodeType *zlist = ylist->value->find(pointset[i].y);
        if (zlist == NULL)
        {
          continue;
        }

        for(auto c : cluster)
        {
          float z1 = rate * (c.first - map_camera_index[2]) + map_camera_index[2];
          K z_index1 = K(z1 + 0.5);

          float z2 = rate * (c.second - map_camera_index[2]) + map_camera_index[2];
          K z_index2 = K(z2 + 0.5);
          
          for(int z_index = z_index1 ; z_index < z_index2+1 ; z_index++)
          {
            const typename Z_NODE::NodeType *voxel = zlist->value->find(z_index);
            // 如果之前没有这个voxel，跳过
            if (voxel == NULL) 
            {
              continue;
            }
            // 如果当前的voxel当前的
            else 
            {
              if((voxel->value->w) > 0 && voxel->value->update == false)
              {
                voxel->value->tableValue = miss_table_[voxel->value->tableValue];
              }
              voxel->value->update = true;
              voxelsUpdate.emplace_back(voxel);
              cv::Point3i map_point_index(pointset[i].x,pointset[i].y,z_index);
              addVoxelToUpdatedVoxelVector(map_point_index,voxel);
            }

          }
        }
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void updataMissVoxel2(const std::vector<int16_t>& map_camera_index, 
                                                       std::vector<std::pair<int , std::vector<int16_t>>> &voxel_index)
  {
    // 以相机在map系下的中心作为起点
    cv::Point2i start(map_camera_index[0] , map_camera_index[1]);
    // 通过迭代器遍历 -> unordered map 的每一个value (x,y,z)
    for(auto it =0 ; it != voxel_index.size() ; it++)
    {
      cv::Point2i end(voxel_index[it].second[0] ,voxel_index[it].second[1]);
      std::vector<cv::Point2i> pointset;
      // 通过划线算法获取每个平面点之上的点集
      bresenhamLine(start, end , pointset);
      // 对当前(xy)对应的三维点的高度进行排序
      std::vector<int16_t> vec_z;
      for(auto cell : voxel_index)
      {
        vec_z.emplace_back(cell.second[2]);
      }
      std::sort(vec_z.begin() , vec_z.end());
      //  计算
      float dis1 = pointDistance(start , end);
      // 遍历每一个pointSet中的二维free点，计算其到相机中心的距离
      for(int i = 1 ; i < pointset.size() - 1 ; i++)
      {
        float dis2 = pointDistance(start , pointset[i]);
        float rate = dis2 / dis1;

        // 开始聚类：对高度进行简单的聚类，将聚类的结果存储在cluster中
        std::vector<std::pair<int , int>> cluster;
        std::pair<int , int> pp(vec_z[0] , vec_z[0]);
        if(vec_z.size() == 1) 
        {
          cluster.emplace_back(pp);
        }
        for(int zz = 1 ; zz < vec_z.size() ; zz ++)
        {
          if(vec_z[zz] - vec_z[zz - 1] > 2)
          {
            pp.second = vec_z[zz - 1];
            cluster.emplace_back(pp);
            pp.first = vec_z[zz];
          }
          else if(zz = vec_z.size() - 1)
          {
            pp.second = vec_z[zz];
            cluster.emplace_back(pp);
          }
        }//for
        // 结束聚类

        const typename X_NODE::NodeType *ylist = _root_list->find(pointset[i].x);
        if (ylist == NULL)
        {
          continue;
        }
        const typename Y_NODE::NodeType *zlist = ylist->value->find(pointset[i].y);
        if (zlist == NULL)
        {
          continue;
        }

        for(auto c : cluster)
        {
          float z1 = rate * (c.first - map_camera_index[2]) + map_camera_index[2];
          K z_index1 = K(z1 + 0.5);

          float z2 = rate * (c.second - map_camera_index[2]) + map_camera_index[2];
          K z_index2 = K(z2 + 0.5);
          
          for(int z_index = z_index1 ; z_index < z_index2+1 ; z_index++)
          {
            const typename Z_NODE::NodeType *voxel = zlist->value->find(z_index);
            if (voxel == NULL) 
            {
              continue;
            }
            else
            {
              if((voxel->value->w) > 0 && voxel->value->update == false)
              {
                voxel->value->tableValue = miss_table_[voxel->value->tableValue];
              }
              voxel->value->update = true;
              voxelsUpdate.emplace_back(voxel);
            }
          }
        }
      }
    }
  }

  
  virtual void updataMissVoxel3(const std::vector<int16_t>& map_camera_index, 
                                                       std::vector<voxel_index_node> &voxel_index)
  {
    // 以相机在map系下的中心作为起点
    cv::Point2i start(map_camera_index[0] , map_camera_index[1]);
    // 通过迭代器遍历 -> unordered map 的每一个value (x,y,z)
    for(auto it =0 ; it != voxel_index.size() ; it++)
    {
      cv::Point2i end(voxel_index[it].map_index[0]  ,voxel_index[it].map_index[1]  );
      std::vector<cv::Point2i> pointset;
      // 通过划线算法获取每个平面点之上的点集
      bresenhamLine(start, end , pointset);
      // 对当前(xy)对应的三维点的高度进行排序
      std::vector<int16_t> vec_z;
      for(auto cell : voxel_index)
      {
        vec_z.emplace_back(cell.map_index[2]);
      }
      std::sort(vec_z.begin() , vec_z.end());
      //  计算
      float dis1 = pointDistance(start , end);
      // 遍历每一个pointSet中的二维free点，计算其到相机中心的距离
      for(int i = 1 ; i < pointset.size() - 1 ; i++)
      {
        float dis2 = pointDistance(start , pointset[i]);
        float rate = dis2 / dis1;

        // 开始聚类：对高度进行简单的聚类，将聚类的结果存储在cluster中
        std::vector<std::pair<int , int>> cluster;
        std::pair<int , int> pp(vec_z[0] , vec_z[0]);
        if(vec_z.size() == 1) 
        {
          cluster.emplace_back(pp);
        }
        for(int zz = 1 ; zz < vec_z.size() ; zz ++)
        {
          if(vec_z[zz] - vec_z[zz - 1] > 2)
          {
            pp.second = vec_z[zz - 1];
            cluster.emplace_back(pp);
            pp.first = vec_z[zz];
          }
          else if(zz = vec_z.size() - 1)
          {
            pp.second = vec_z[zz];
            cluster.emplace_back(pp);
          }
        }//for
        // 结束聚类

        const typename X_NODE::NodeType *ylist = _root_list->find(pointset[i].x);
        if (ylist == NULL)
        {
          continue;
        }
        const typename Y_NODE::NodeType *zlist = ylist->value->find(pointset[i].y);
        if (zlist == NULL)
        {
          continue;
        }

        for(auto c : cluster)
        {
          float z1 = rate * (c.first - map_camera_index[2]) + map_camera_index[2];
          K z_index1 = K(z1 + 0.5);

          float z2 = rate * (c.second - map_camera_index[2]) + map_camera_index[2];
          K z_index2 = K(z2 + 0.5);
          
          for(int z_index = z_index1 ; z_index < z_index2+1 ; z_index++)
          {
            const typename Z_NODE::NodeType *voxel = zlist->value->find(z_index);
            if (voxel == NULL) 
            {
              continue;
            }
            else
            {
              if((voxel->value->w) > 0 && voxel->value->update == false)
              {
                voxel->value->tableValue = miss_table_[voxel->value->tableValue];
              }
              voxel->value->update = true;
              voxelsUpdate.emplace_back(voxel);
            }
          }
        }
      }
    }
  }


  virtual bool integrateVoxel(K ix, K iy, K iz, V *data) 
  {
    if (isValidIndex(ix, iy, iz)) {

      if (this->hasConcurrencyAccess())
        this->_root_list->lock(ix);

      const typename X_NODE::NodeType *ylist = _root_list->find(ix);
      ////////////////////////////////////////////////
      if (ylist == NULL) 
      {
        ylist = _root_list->insert(ix, new Y_NODE(_min_index_value, _max_index_value));
        //_bytes_counter += sizeof(typename X_NODE::NodeType) + sizeof(Y_NODE);
      }
      const typename Y_NODE::NodeType *zlist = ylist->value->find(iy);
      if (zlist == NULL) 
      {
        zlist = ylist->value->insert(iy, new Z_NODE(z_node_min, z_node_max));
      }

      if(iz > z_node_min && iz <z_node_max)
      {
        const typename Z_NODE::NodeType *voxel = zlist->value->find(iz);
        if (voxel == NULL) {
          voxel = zlist->value->insert(iz, new V(data));
        } else {
          if(voxel->value->update == false)
          {
            *(voxel->value) = *(voxel->value) + *data;
            voxel->value->tableValue = hit_table_[voxel->value->tableValue];
          }
        }
        voxel->value->update = true;
        voxelsUpdate.emplace_back(voxel);
      }
      
      if (this->hasConcurrencyAccess())
        this->_root_list->unlock(ix);
      return true;
    }
    return false;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief 
   * 
   * @param map_point_index  点在地图栅格下的坐标 ( index_x , index_y , index_z )
   * @param data 
   * @param newP 
   * @return true 
   * @return false 
   */  
  virtual bool integrateVoxel(const std::vector<int16_t>& map_point_index, V *data , bool& newP) 
  {
    if (this->hasConcurrencyAccess())
      this->_root_list->lock(map_point_index[0]);

    const typename X_NODE::NodeType *ylist = _root_list->find(map_point_index[0]);
    if (ylist == NULL) 
    {
      ylist = _root_list->insert(map_point_index[0] , new Y_NODE(_min_index_value, _max_index_value));
    }
    const typename Y_NODE::NodeType *zlist = ylist->value->find(map_point_index[1]);
    if (zlist == NULL) 
    {
      zlist = ylist->value->insert(map_point_index[1],new Z_NODE(z_node_min, z_node_max));
    }
    // 如果 z_index 在指定范围内
    if(map_point_index[2] > z_node_min && map_point_index[2] < z_node_max)
    {
      const typename Z_NODE::NodeType *voxel = zlist->value->find(map_point_index[2]);
      if (voxel == NULL) 
      {
        voxel = zlist->value->insert(map_point_index[2], new V(data));
        voxel->value->tableValue = hit_table_[voxel->value->tableValue];
        voxel->value->update = true;
        voxelsUpdate.emplace_back(voxel);
        addVoxelToUpdatedVoxelVector(map_point_index, voxel);
        newP = true;
      }// if
      else
      {
        if(voxel->value->update == false)
        {
          // tableValue相加 
          // *(voxel->value) = *(voxel->value) + *data;
          voxel->value->tableValue = hit_table_[voxel->value->tableValue];
          voxel->value->update = true;
          voxelsUpdate.emplace_back(voxel);
          addVoxelToUpdatedVoxelVector(map_point_index,voxel);
          newP = true;
        }
      }//else
    }
    if (this->hasConcurrencyAccess())
    {
      this->_root_list->unlock(map_point_index[0]);
    }
    return true;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool startBatchIntegration() 
  {
    _batch_integration = true;
    _current_integration_map.clear();
    return true;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void fetchVoxels(std::vector<Voxel3D> &voxels) 
  {
    voxels.clear();
    std::vector<typename X_NODE::NodeType *> xnodes;
    _root_list->retrieveNodes(xnodes);

#pragma omp parallel
    {
      std::vector<Voxel3D> voxels_private;

#pragma omp for nowait
      for (int i = 0; i < xnodes.size(); i++) 
      {
        K ix, iy, iz;
        D x, y, z;
        std::vector<typename Y_NODE::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodes(ynodes);
        for (int j = 0; j < ynodes.size(); j++) 
        {
          std::vector<typename Z_NODE::NodeType *> znodes;
          ynodes[j]->value->retrieveNodes(znodes);

          for (int k = 0; k < znodes.size(); k++) 
          {
            ix = xnodes[i]->key;
            iy = ynodes[j]->key;
            iz = znodes[k]->key;
            if (znodes[k]->value->tableValue < 0.9)
            {
              continue;
            }
            indexToCoordinatesWithTable((int)ix, (int)iy, (int)iz, x, y, z);
            voxels_private.emplace_back(Voxel3D(x, y, z, znodes[k]->value));
          }
        }
      }

#pragma omp critical
      voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
    }
  }

  virtual void fetchUpdateVoxelsOnly(std::vector<Voxel3D> &voxelsOut) 
  {
    voxelsOut.clear();
    int updateNum = updatedVoxelVec.size();
    voxelsOut.resize(updateNum);
    memcpy(voxelsOut.data(), updatedVoxelVec.data(),updateNum*sizeof(Voxel3D));
    updatedVoxelVec.clear();
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 virtual void clearVoxelsUpdateFlag()
 {
  //  printf("voxelsUpdate.size = %d \n" , voxelsUpdate.size());
  for(auto voxel : voxelsUpdate)
  {
    voxel->value->update = false;
  }
  voxelsUpdate.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void radiusSearch(K cx, K cy, K cz, K radiusx, K radiusy, K radiusz,
                            std::vector<Voxel3D> &voxels, bool boxed = false) {
    voxels.clear();
    std::vector<typename X_NODE::NodeType *> xnodes;

    K ix_min = cx - radiusx;
    K ix_max = cx + radiusx;
    K iy_min = cy - radiusy;
    K iy_max = cy + radiusy;
    K iz_min = cz - radiusz;
    K iz_max = cz + radiusz;

    _root_list->retrieveNodesByRange(ix_min, ix_max, xnodes);

    D rx, ry, rz, radius;
    D centerx, centery, centerz;

    indexToCoordinates(radiusx, radiusy, radiusz, rx, ry, rz);
    indexToCoordinates(cx, cy, cz, centerx, centery, centerz);
    radius = (rx + ry + rz) / 3.0;

#pragma omp parallel
    {
      std::vector<Voxel3D> voxels_private;

#pragma omp for nowait
      for (int i = 0; i < xnodes.size(); i++) {
        K ix, iy, iz;
        D x, y, z;
        D distance;
        std::vector<typename Y_NODE::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodesByRange(iy_min, iy_max, ynodes);
        for (int j = 0; j < ynodes.size(); j++) {
          std::vector<typename Z_NODE::NodeType *> znodes;
          ynodes[j]->value->retrieveNodesByRange(iz_min, iz_max, znodes);
          ix = xnodes[i]->key;
          iy = ynodes[j]->key;

          for (int k = 0; k < znodes.size(); k++) {
            iz = znodes[k]->key;
            indexToCoordinates(ix, iy, iz, x, y, z);
            if (!boxed) {
              distance = sqrt(pow(centerx - x, 2) + pow(centery - y, 2) +
                              pow(centerz - z, 2));
              if (distance > radius)
                continue;
            }
            voxels_private.emplace_back(Voxel3D(x, y, z, znodes[k]->value));
          }
        }
      }

#pragma omp critical
      voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual void radiusSearch(D cx, D cy, D cz, D radiusx, D radiusy, D radiusz,
                                               std::vector<Voxel3D> &voxels, bool boxed = false) {
    K ix, iy, iz;
    if (coordinatesToIndex(cx, cy, cz, ix, iy, iz)) {
      K iradiusx, iradiusy, iradiusz;
      iradiusx = K(floor(radiusx / _resolution_x));
      iradiusy = K(floor(radiusy / _resolution_y));
      iradiusz = K(floor(radiusz / _resolution_z));
      radiusSearch(ix, iy, iz, iradiusx, iradiusy, iradiusz, voxels, boxed);
    }
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /**
       *
       * @return
       */
  virtual long sizeInBytes() 
  { 
    return _bytes_counter; 
  }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual void saveToFile(std::string filename) 
  {
    std::vector<Voxel3D> voxels;
    fetchVoxels(voxels);

    std::ofstream f;
    f.open(filename);
    V a1;
    K a2;
    D a3;
    f << "# SkipListMapV2<" << typeid(a1).name() << "," << typeid(a2).name()
      << "," << typeid(a3).name() << ">" << std::endl;
    f << _min_index_value << " " << _max_index_value << " ";
    f << _resolution_x << " ";
    f << _resolution_y << " ";
    f << _resolution_z << std::endl;
    for (int i = 0; i < voxels.size(); i++) {
      f << voxels[i] << std::endl;
    }
    f.close();
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual void loadFromFile(std::string filename) 
  {
    std::ifstream input_file(filename.c_str());
    if (input_file.is_open()) {
    }
    // Load Files
    std::string line;

    bool header_found = false;

    while (std::getline(input_file, line)) {
      std::istringstream iss(line);
      if (line.length() > 0 && line.at(0) == '#')
        continue;

      if (!header_found) {
        K min, max;
        iss >> min;
        iss >> max;
        iss >> _resolution_x;
        iss >> _resolution_y;
        iss >> _resolution_z;
        initialize(min, max);
        header_found = true;
        continue;
      }

      if (header_found) {
        Voxel3D voxel;
        iss >> voxel;
        this->integrateVoxel(voxel.x, voxel.y, voxel.z, voxel.data);
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  virtual void enableConcurrencyAccess(bool status = true) 
  {
    this->_self_concurrency_management = status;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual bool hasConcurrencyAccess() 
  {
    return this->_self_concurrency_management;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //  单位mm  ：vector的index代表mm，里面存储所对应的index 
  std::vector<int16_t> preComputeCoordinatesToIndexTable_(D resolution ,const int Len)
  {
    std::vector<int16_t> table(Len,0);
    for (int mm = 0 ;  mm < Len; mm++)
    {
      auto m = mm / 1000.0;
      int16_t index = floor(m/resolution);
      if ( index < Max_Index_Value)
      {
        table[mm] = index;
      }
    }
    return table;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::vector<float> preComputeIndexToCoordinatesTable_(D resolution)
  {
    std::vector<float> table(Max_Index_Value,0.f);
    for (int index = 0 ;  index < Max_Index_Value; index++)
    {
      table[index] = index * resolution + resolution * 0.5;
    }
    return table;
  }

  virtual void addVoxelToUpdatedVoxelVector(const std::vector<int16_t> &map_point_index,const typename Z_NODE::NodeType *voxel)
  {
    D x,y,z;
    int ix  = map_point_index[0];
    int iy = map_point_index[1];
    int iz = map_point_index[2];
    indexToCoordinatesWithTable(ix,iy,iz,x,y,z);
    updatedVoxelVec.emplace_back(Voxel3D(x,y,z,voxel->value));
  }


  virtual void addVoxelToUpdatedVoxelVector(const cv::Point3i &map_point_index, const typename Z_NODE::NodeType *voxel)
  {
    D x,y,z;
    int ix  = map_point_index.x;
    int iy = map_point_index.y;
    int iz = map_point_index.z;
    indexToCoordinatesWithTable(ix,iy,iz,x,y,z);
    updatedVoxelVec.emplace_back(Voxel3D(x,y,z,voxel->value));
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

protected:

  bool _integrateXNode(const typename X_NODE::NodeType *ylist, K &iy, K &iz, V *data) 
  {
    const typename Y_NODE::NodeType *zlist = ylist->value->find(iy);
    if (zlist == NULL) {
      zlist = ylist->value->insert(
          iy, new Z_NODE(_min_index_value, _max_index_value));
    }
    const typename Z_NODE::NodeType *voxel = zlist->value->find(iz);
    if (voxel == NULL) {
      voxel = zlist->value->insert(iz, new V(data));
    } else 
    {
      *(voxel->value) = *(voxel->value) + *data;
    }
    return true;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Index _max_index_value;
  Index _min_index_value;
  X_NODE *_root_list;
  D _resolution_x;
  D _resolution_y;
  D _resolution_z;
  int _voxel_counter;
  int _xlist_counter;
  int _ylist_counter;
  long _bytes_counter;
  bool _batch_integration;
  bool _initialized;
  bool _self_concurrency_management;
  IntegrationMap _current_integration_map;

  // concurrency
  boost::mutex mutex_map_mutex;
  std::map<K, boost::mutex *> mutex_map;

  // 用于记录当前帧被更新的node
  std::vector<const typename  Z_NODE::NodeType *> voxelsUpdate; 
  std::vector<Voxel3D> updatedVoxelVec;

  // std::vector<const typename  Z_NODE::NodeType *> voxelsUpdate; 

  const std::vector<u_int16_t> hit_table_;
  const std::vector<u_int16_t> miss_table_;
  const std::vector<D> indexToCoordinatesTable_;
  const std::vector<K> coordinatesToIndexTable_;
  
}; // class SkipListMapV2
} // namespace skimap

#endif /* SkipListMapV2_HPP */
