#include "common.hpp"

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


void filter_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  //filter that keeps only me  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*cloud_filtered);

	//get the nose (considered as the closest point to the camera)
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud_filtered, minPt, maxPt);

  pcl::PointXYZ nose;
  find_point_z(minPt.z, cloud_filtered, &nose);
  
  //filter the region below the nose
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
}


void compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
}

void compute_VFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
								 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
								 pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);

  vfh.compute (*vfhs);
}

void split(const std::string &s, char delim, std::vector<std::string> &elems)
{
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
      elems.push_back(item);
  }
}

std::vector<std::string> split(const std::string &s, char delim)
{
	std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

std::vector<int> get_number_images()
{
	std::string line;
	std::vector<int> nb_images;
	int i = 0;
	std::ifstream myfile (SSTR("/home/theo/Documents/3D/Projet/emotionRecog/data/images/nb_images.txt").c_str());
	if (myfile.is_open())
	{
	  while ( std::getline (myfile,line) )
	  {
			std::istringstream buffer(line);
			int temp;
			buffer >> temp;
	  	//int temp = std::stoi(SSTR(line).c_str());
			nb_images.push_back(temp);
			i++;
	  }
	  myfile.close();
	}
  else 
  {
  	std::cout << "Unable to open file";
		nb_images.push_back(0);
		nb_images.push_back(0);
		nb_images.push_back(0);
		nb_images.push_back(0);
		nb_images.push_back(0);
  }
  return nb_images;
}

std::string get_emotion_from_index(int index)
{
	switch (index)
	{
		case 0: 
			return "allegria";
		case 1:
			return "colera";
		case 2:
			return "miedo";
		case 3:
			return "sorpresa";
		case 4:
			return "tristeza";
		default:
			return "error";
	}
}
