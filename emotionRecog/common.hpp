#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <stdio.h>
#include <pcl/common/common.h>

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <typeinfo>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <fstream>
#include <string>
#include <sstream>
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/cvstd.hpp"

#define SSTR( x ) static_cast< std::ostringstream & >( ( std::ostringstream() << std::dec << x ) ).str()


void find_point_z(double value, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ * out_point);

void filter_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

void compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

void compute_VFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
								 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
								 pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
								 
void split(const std::string &s, char delim, std::vector<std::string> &elems);

std::vector<std::string> split(const std::string &s, char delim);

std::vector<int> get_number_images();

std::string get_emotion_from_index(int index);
