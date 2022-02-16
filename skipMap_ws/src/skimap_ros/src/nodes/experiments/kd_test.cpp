/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <utility> // std::pair
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include <fstream>
#include <iostream>

#include "sys/types.h"
#include "sys/sysinfo.h"

#include <ANN/ANN.h> // ANN declarations
#include <flann/flann.hpp>

//Eigen
#include <Eigen/Core>

// OPENCV
#include <opencv2/opencv.hpp>

// Skimap
#include <skimap/KDSkipList.hpp>
#include <skimap/voxels/VoxelDataMatrix.hpp>

#define MAX_RANDOM_COLOR 1.0
#define MIN_RANDOM_COLOR 0.0
double MAX_RANDOM_COORD = 500.0;
double MIN_RANDOM_COORD = 0.0;
int DIM = 2;
int N_POINTS = 100;
int _debug;

typedef float CoordinatesType;
typedef std::vector<std::vector<CoordinatesType>> Points;
typedef skimap::VoxelDataMatrix<CoordinatesType> VoxelData;
typedef int IndexType;

auto _current_time = std::chrono::high_resolution_clock::now();
auto getTime()
{
  _current_time = std::chrono::high_resolution_clock::now();
  return _current_time;
}

auto deltaTime()
{
  auto t1 = _current_time;
  auto t2 = getTime();
  auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  return dt / 1000000.0;
}

float fRand(float fMin, float fMax)
{
  float f = (float)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

float distance(std::vector<CoordinatesType> cds1, std::vector<CoordinatesType> cds2)
{
  float d = 0.0;
  for (int i = 0; i < cds1.size(); i++)
  {
    d += pow(cds1[i] - cds2[i], 2);
  }
  return sqrt(d);
}

bool checkPresence(std::vector<std::vector<CoordinatesType>> &search_data, std::vector<CoordinatesType> cds)
{

  // std::cout << "Searchin : " << cds[0] << "," << cds[1] << std::endl;
  for (int i = 0; i < search_data.size(); i++)
  {
    // std::cout << "  IDATA: " << search_data[i][0] << "," << search_data[i][1] << std::endl;
    if (distance(search_data[i], cds) < 0.1)
    {
      return true;
    }
  }
  return false;
}
void process_mem_usage(double &vm_usage, double &resident_set)
{
  using std::ios_base;
  using std::ifstream;
  using std::string;

  vm_usage = 0.0;
  resident_set = 0.0;

  // 'file' stat seems to give the most reliable results
  //
  ifstream stat_stream("/proc/self/stat", ios_base::in);

  // dummy vars for leading entries in stat that we don't care about
  //
  string pid, comm, state, ppid, pgrp, session, tty_nr;
  string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  string utime, stime, cutime, cstime, priority, nice;
  string O, itrealvalue, starttime;

  // the two fields we want
  //
  unsigned long vsize;
  long rss;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

  stat_stream.close();

  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;
}

void printResults(std::string name, double t_c, double t_s, double memory)
{
  printf(" %f %f %f", t_c, t_s, memory);
}

std::vector<int> computeFlann(int ndim, Points &points, CoordinatesType radius, std::string substring, bool print_output)
{

  cv::Mat_<CoordinatesType> features(points.size(), ndim);
  for (int i = 0; i < points.size(); i++)
  {
    for (int n = 0; n < ndim; n++)
    {
      features.at<CoordinatesType>(i, n) = points[i][n];
    }
  }

  cv::flann::IndexParams *ip;
  if (substring.compare("flann_brute") == 0)
  {
    ip = new cv::flann::LinearIndexParams();
  }
  else if (substring.compare("flann_kdtree") == 0)
  {
    ip = new cv::flann::KDTreeIndexParams(2);
  }
  else if (substring.compare("flann_lsh") == 0)
  {
    ip = new cv::flann::LshIndexParams(20, 15, 2);
  }
  else if (substring.compare("flann_auto") == 0)
  {
    ip = new cv::flann::AutotunedIndexParams();
  }
  else if (substring.compare("flann_composite") == 0)
  {
    ip = new cv::flann::CompositeIndexParams();
  }
  else if (substring.compare("flann_kmeans") == 0)
  {
    ip = new cv::flann::KMeansIndexParams(32, 50);
  }

  else
  {
    printf("ERORRO UNRECOGNIZED ALGO: %s\n", substring.c_str());
    exit(0);
  }

  getTime();
  cv::flann::Index flann_index(features, *ip);
  double time_creation = deltaTime();

  unsigned int max_neighbours = points.size() * 10;
  cv::Mat query = cv::Mat_<CoordinatesType>(1, ndim);
  for (int n = 0; n < ndim; n++)
  {
    query.at<CoordinatesType>(0, n) = MAX_RANDOM_COORD / 2.0;
  }

  std::vector<int> indices; //neither assume type nor size here !
  std::vector<CoordinatesType> dists;

  getTime();
  flann_index.radiusSearch(query, indices, dists, radius * radius, max_neighbours,
                           cv::flann::SearchParams(points.size()));
  double time_search = deltaTime();

  double vm, rss;
  process_mem_usage(vm, rss);
  double memory = rss;

  if (print_output)
  {
    printResults(substring, time_creation, time_search, memory);
  }
  return indices;
}

int main(int argc, char **argv)
{

  DIM = atoi(argv[1]);
  N_POINTS = atoi(argv[2]);
  MAX_RANDOM_COORD = atof(argv[3]);
  MIN_RANDOM_COORD = 0.0;
  float resolution = atof(argv[4]);
  CoordinatesType radius = atof(argv[5]);
  std::string algo(argv[6]);
  _debug = atoi(argv[7]);

  srand(time(NULL));

  cv::Mat image(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image2(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));

  std::vector<std::vector<CoordinatesType>> integration_data;

  for (int i = 0; i < N_POINTS; i++)
  {
    std::vector<CoordinatesType> cds;
    for (int d = 0; d < DIM; d++)
    {
      cds.push_back(fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD));
    }

    if (_debug == 1 && DIM == 2)
    {
      image.at<cv::Vec3b>(cds[1], cds[0]) = cv::Vec3b(255, 22, 255);
    }
    integration_data.push_back(cds);
  }

  printf("%s %d %d %f", algo.c_str(), DIM, N_POINTS, MAX_RANDOM_COORD);

  if (algo.find("flann") == 0)
  {
    std::vector<int> indices = computeFlann(DIM, integration_data, radius, algo, true);
    if (_debug == 1)
    {
      for (int i = 0; i < indices.size(); i++)
      {
        std::vector<CoordinatesType> cds = integration_data[indices[i]];

        image.at<cv::Vec3b>(cds[1], cds[0]) = cv::Vec3b(255, 255, 255);
      }
    }
    printf("S %d\n", int(indices.size()));
  }
  else if (algo.compare("kdskip") == 0)
  {

    skimap::KDSkipList<VoxelData, IndexType, CoordinatesType> kd_skip_list(DIM, resolution);
    typedef skimap::KDSkipList<VoxelData, IndexType, CoordinatesType>::VoxelKD Voxel;

    kd_skip_list.enableConcurrencyAccess(true);

    getTime();

#pragma omp parallel for
    for (int i = 0; i < integration_data.size(); i++)
    {
      VoxelData voxel;
      voxel.matrix.push_back(integration_data[i]);
      kd_skip_list.integrateVoxel(integration_data[i], &voxel);
    }
    double time_creation = deltaTime();

    std::vector<CoordinatesType> radius_center(DIM);
    for (int i = 0; i < DIM; i++)
    {
      radius_center[i] = MAX_RANDOM_COORD / 2.0;
    }

    getTime();
    std::vector<Voxel> voxels;
    kd_skip_list.radiusSearch(radius_center, radius, voxels);
    double time_search = deltaTime();

    double vm, rss;
    process_mem_usage(vm, rss);
    double memory = rss;

    printResults(algo, time_creation, time_search, memory);

    if (_debug == 1)
    {
      bool consistency = true;
      for (int i = 0; i < voxels.size(); i++)
      {
        Voxel &v = voxels[i];
        for (int vi = 0; vi < v.data->matrix.size(); vi++)
        {
          // consistency &= checkPresence(integration_data, v.data->matrix[vi]);
          std::vector<CoordinatesType> cds = v.data->matrix[vi];
          image.at<cv::Vec3b>(cds[1], cds[0]) = cv::Vec3b(255, 255, 255);

          // if (!checkPresence(integration_data, v.data->matrix[vi]))
          // {
          //   printf("Fail\n");
          // }
        }
      }
      if (consistency)
      {
        printf("Ski is good!\n");
      }
    }
  }

  if (_debug)
  {
    cv::imshow("img", image);
    cv::waitKey(0);
  }
  //cv::imshow("img", image);
  //cv::imshow("img2", image2);
  //cv::waitKey(0);
  printf("\n");
}
