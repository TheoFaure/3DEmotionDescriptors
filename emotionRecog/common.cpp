#include "common.hpp"

void find_point_y(double value, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ * out_point)
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


void filter_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
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
		case 5:
			return "2_sad";
		case 6:
			return "2_joyful";
		default:
			return "error";
	}
}
