#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/cvstd.hpp"
#include <vector>

#define SSTR( x ) static_cast< std::ostringstream & >( ( std::ostringstream() << std::dec << x ) ).str()

using namespace std;


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

cv::Mat read_histograms()
{
	cv::Mat histograms = cv::Mat::zeros(0, 308, CV_32F);
	for (int i = 0 ; i < 54 ; i++)
	{
		std::string line;
//		std::string index = std::to_string(i);
//		std::string file_name = ;
		std::ifstream myfile (SSTR("../../data_cara/histograms/inputCloud" << i << "_filtered_histogram.txt").c_str());
		if (myfile.is_open())
		{
		  if ( std::getline (myfile,line) )
		  {
		    line = line.substr(1, line.size() - 2);
		    std::vector<std::string> histo_line = split(line, ',');
		    cv::Mat histo_line_int = cv::Mat::zeros(1, 308, CV_32F);
		    for (int j = 0 ; j < 308 ; j++)
		    {
		    	histo_line_int.at<float>(0, j) = std::stoi(histo_line[j]);
		    }
				histograms.push_back(histo_line_int);
		  }
		  myfile.close();
		}
	  else std::cout << "Unable to open file";
	}
	
	return histograms;
}

cv::Mat create_labels()
{
	cv::Mat labels = cv::Mat::zeros(1, 54, CV_32F);
	//Tristeza = 0, Allegria = 1, Miedo = 2, Sorpresa = 3, Colera = 4
  for (int i = 0 ; i < 54 ; i++)
  {
		if (i < 12)
			labels.at<float>(0, i) = 0;
		else if (i < 25)
			labels.at<float>(0, i) = 1;
		else if (i < 35)
			labels.at<float>(0, i) = 2;
		else if (i < 40)
			labels.at<float>(0, i) = 3;
		else
			labels.at<float>(0, i) = 4;
  }			
  
  return labels;
}

void SVM_classifier()
{
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	svm->setType(cv::ml::SVM::C_SVC);
	svm->setKernel(cv::ml::SVM::POLY);
	svm->setGamma(3); 

	cv::Mat trainData = read_histograms();
	std::cout << trainData.size[0] << " / " << trainData.size[1] << std::endl;
	
	cv::Mat labels = create_labels();
	std::cout << labels.size[0] << " / " << labels.size[1] << std::endl;
	svm->train( trainData , cv::ml::ROW_SAMPLE , labels );
	
	cv::Mat query = trainData.row(0); // input, 1channel, 1 row (apply reshape(1,1) if nessecary)
	cv::Mat res;   // output
	svm->predict(query, res);	
	
//	std::cout << res << std::endl;
}

void RTree_classifier(int row)
{	
	cv::Ptr<cv::ml::RTrees> rtree = cv::ml::RTrees::create();
	cv::Mat trainData = read_histograms();
	std::cout << trainData.size[0] << " / " << trainData.size[1] << std::endl;
	
	cv::Mat labels = create_labels();
	std::cout << labels.size[0] << " / " << labels.size[1] << std::endl;
	rtree->train( trainData , cv::ml::ROW_SAMPLE , labels );
	
	cv::Mat query = trainData.row(row); // input, 1channel, 1 row (apply reshape(1,1) if nessecary)
	cv::Mat res;   // output
	rtree->predict(query, res);	
	
	std::cout << res << std::endl;
}


int
main (int argc, char** argv)
{
	RTree_classifier(std::stoi(argv[1]));
	
	return 0;
}
