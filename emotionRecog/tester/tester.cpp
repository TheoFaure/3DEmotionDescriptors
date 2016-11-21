#include "../common.hpp"

float test (std::string file_name, int classifier_version)
{
//CREATE HISTOGRAM

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
    return 0.0;
  }

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	filter_file(cloud, cloud_filtered);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	compute_normals(cloud_filtered, cloud_normals);

	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
	compute_VFH(cloud_filtered, cloud_normals, vfhs);
	pcl::VFHSignature308 descriptor = vfhs->points[0];

	std::ostringstream stream;
	stream << descriptor;
	std::string histogram_str = stream.str();

//Convert histogram to string
	histogram_str = histogram_str.substr(1, histogram_str.size() - 2);
	std::vector<std::string> histogram_vector = split(histogram_str, ',');
	cv::Mat histogram = cv::Mat::zeros(1, 308, CV_32FC1);
	for (int j = 0 ; j < 308 ; j++)
	{
		std::istringstream buffer(histogram_vector[j]);
		int value;
		buffer >> value;
		histogram.at<float>(0, j) = value;
		//histogram.at<float>(0, j) = std::stoi(histogram_vector[j]);
	}

//TEST WITH MODEL

	if (classifier_version == 0)
	{
		cv::Ptr<cv::ml::RTrees> rtree = cv::ml::StatModel::load<cv::ml::RTrees>("/home/theo/Documents/3D/Projet/emotionRecog/trainer/model_RTree");
		cv::Mat res;   // output
		rtree->predict(histogram, res);
	
		return res.at<float>(0, 0);
	} else
	{
		cv::Ptr<cv::ml::SVM> svm = cv::ml::StatModel::load<cv::ml::SVM>("/home/theo/Documents/3D/Projet/emotionRecog/trainer/model_SVM");
		cv::Mat res;   // output
		svm->predict(histogram, res);
	
		return res.at<float>(0, 0);
	}
}


int main (int argc, char** argv)
{
	int classifier_version = 1;
	if (argc >= 3)
	{
		std::istringstream buffer(argv[2]);
		buffer >> classifier_version;
	}
	if (argc >= 2)
	{
		std::string file_name = argv[1];
		float res = test(file_name, classifier_version);
		std::cout << res << std::endl;
	} else
	{
		std::vector<int> nb_images = get_number_images();
		for (int i = 0 ; i < nb_images.size() ; i++)
		{
			std::vector<float> res;
			for (int j = 1 ; j <= nb_images[i] ; j++)
			{
				std::string type(get_emotion_from_index(i));
				std::string file(SSTR(j));
				std::string file_name = ("/home/theo/Documents/3D/Projet/emotionRecog/data/images/" + type + "/" + file + ".pcd").c_str();
				res.push_back(test(file_name, classifier_version));
			}
			std::cout << i << " (" << get_emotion_from_index(i) << "): ";
			for (int j = 0 ; j < res.size() ; j++)
			{
				std::cout << res[j] << " ";
			}
			std::cout << endl;
		}
	}	
	return 0;
}
