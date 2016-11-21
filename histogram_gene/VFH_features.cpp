#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <typeinfo>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <iostream>
#include <fstream>

void compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}

void compute_VFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
								 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
								 pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);

  // Compute the features
  vfh.compute (*vfhs);
}

void create_histogram(int argc, char** argv)
{
//------------READ CLOUDS---------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);

  std::string type(argv[1]);
  std::string file(argv[2]);
  std::string file_name = ("../../data_cara/" + type + "/" + file + ".pcd").c_str();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloudA) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return;
  }

//--------------CALCULATE THE NORMALS------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normalsA (new pcl::PointCloud<pcl::Normal>);
  compute_normals(cloudA, cloud_normalsA);

//----------------CALCULATE PFH or VFH------------------------
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsA (new pcl::PointCloud<pcl::VFHSignature308> ());
	compute_VFH(cloudA, cloud_normalsA, vfhsA);

	std::cout << typeid(vfhsA->points).name() << std::endl;

  pcl::VFHSignature308 descriptorA = vfhsA->points[0];

  std::ofstream out_file;
  std::string out_file_name = ("../../data_cara/" + type + "/" + file + "_histogram.txt").c_str();
  out_file.open (("../../data_cara/" + type + "/" + file + "_histogram.txt").c_str());
  out_file << descriptorA;
  out_file.close();

//------------------VISUALIZER---------------------------
	// Plotter object.
  //pcl::visualization::PCLHistogramVisualizer viewer;
	// We need to set the size of the descriptor beforehand.
	//viewer.addFeatureHistogram(*vfhsA, 308);
	//viewer.spin();
}

int
main (int argc, char** argv)
{
	create_histogram(argc, argv);
	return 0;
}
