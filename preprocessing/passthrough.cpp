#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <stdio.h>
#include <string.h>
#include <pcl/common/common.h>

void find_point_z(double value, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ * out_point)
{
  for (int i = 0 ; i < cloud->points.size() ; i++)
  {
    if (cloud->points[i].z == value)
    {
      out_point->x = cloud->points[i].x;
      out_point->y = cloud->points[i].y;
      out_point->z = cloud->points[i].z;
    }
  }
}


int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//-------------READ FILE-------------------
  std::string type(argv[1]);
  std::string file(argv[2]);
  std::string file_name = ("../../data_cara/" + type + "/" + file + ".pcd").c_str();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

//----------FILTER FILE-------------------
  //filter that keeps only me  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*cloud_filtered);

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
  std::cout << maxPt.x << std::endl;
  std::cout << maxPt.y << std::endl;
  std::cout << maxPt.z << std::endl;
  
  pcl::PointXYZ nose;
  find_point_z(minPt.z, cloud_filtered, &nose);
  
  std::cout << "nose : " << nose.x << " , " << nose.y << " , " << nose.z << std::endl;
  
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (nose.z-0.1, nose.z+0.1);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (nose.x-0.07, nose.x+0.07);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (nose.y-0.1, nose.y+0.045);
  pass.filter (*cloud_filtered);


//-------------------WRITE NEW----------------------
  std::string out_file_name = ("../../data_cara/" + type + "/" + file + "_filtered.pcd").c_str();

  if (pcl::io::savePCDFileASCII (out_file_name, *cloud_filtered) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't write file test_pcd.pcd \n");
    return (-1);
  }
  return (0);
}
