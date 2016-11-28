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

void find_point_y(double value, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXYZRGBA * out_point)
{
  for (int i = 0 ; i < cloud->points.size() ; i++)
  {
    if (cloud->points[i].y == value)
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
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

//-------------READ FILE-------------------
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

//----------FILTER FILE-------------------



  //filter that keeps only me  
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*cloud_filtered);

	// remove shoulder...
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.7, 0.16);
  pass.filter (*cloud_filtered);

  pcl::PointXYZRGBA minPt, maxPt;
  pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);
  std::cout << maxPt.x << std::endl;
  std::cout << maxPt.y << std::endl;
  std::cout << maxPt.z << std::endl;
  
  pcl::PointXYZRGBA nose;
  find_point_y(minPt.y, cloud_filtered, &nose);
  
  std::cout << "nose : " << nose.x << " , " << nose.y << " , " << nose.z << std::endl;
  
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (nose.y+0.14, nose.y+0.25);
  pass.filter (*cloud_filtered);
/*
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (nose.x-0.07, nose.x+0.07);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (nose.y-0.1, nose.y+0.045);
  pass.filter (*cloud_filtered);
*/

//-------------------WRITE NEW----------------------
  if (pcl::io::savePCDFileASCII (argv[2], *cloud_filtered) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't write file test_pcd.pcd \n");
    return (-1);
  }
  return (0);
}
