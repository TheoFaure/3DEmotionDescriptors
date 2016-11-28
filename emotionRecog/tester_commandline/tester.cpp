#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include "../common.hpp"


cv::Mat read_histograms(const std::string file_name)
{
	std::string line;
	std::ifstream myfile (SSTR(file_name).c_str());
  cv::Mat histo_line_int = cv::Mat::zeros(1, 308, CV_32F);
	if (myfile.is_open())
	{
	  if ( std::getline (myfile,line) )
	  {
	    line = line.substr(1, line.size() - 2);
	    std::vector<std::string> histo_line = split(line, ',');

	    for (int j = 0 ; j < 308 ; j++)
	    {
				std::istringstream buffer(histo_line[j]);
				int temp;
				buffer >> temp;
	    	histo_line_int.at<float>(0, j) = temp;
	    }
	  }
	  myfile.close();
	}
	return histo_line_int;
}


int main (int argc, char** argv)
{
	std::string file_name = argv[1];
	cv::Mat histo = read_histograms(file_name);

	/*if (classifier_version == 0)
	{
		cv::Ptr<cv::ml::RTrees> rtree = cv::ml::StatModel::load<cv::ml::RTrees>("/home/theo/Documents/3D/Projet/emotionRecog/trainer/model_RTree");
		cv::Mat res;   // output
		float res2 = rtree->predict(histo);
		return res2;
		//return res.at<float>(0, 0);
	} else
	{*/
		cv::Ptr<cv::ml::SVM> svm = cv::ml::StatModel::load<cv::ml::SVM>("/home/theo/Documents/3D/Projet/emotionRecog/trainer/model_SVM");
		float res2 = svm->predict(histo);
		std::cout<<  << std::endl;	
		
		std::cout<< res2 << std::endl;	
		//return res.at<float>(0, 0);
	//}

	return 0;
}

