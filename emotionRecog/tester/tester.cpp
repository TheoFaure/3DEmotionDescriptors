#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include "../common.hpp"

float test (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int classifier_version)
{
//CREATE HISTOGRAM

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

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"), use_method(0) {}

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }
		
		void test_cloud_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
    	if (use_method == 1)
    	{
    		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    		*cloud2 += *cloud;
    	  //pcl::PointCloud<pcl::PointXYZ> cloud2(*cloud);
    		//pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(&cloud2);
				std::cout << "Guessing your emotion using SVM..." << std::endl;
		  	float res = test(cloud2, 1);
				std::cout << res << std::endl;
    	} else if (use_method == 2)
    	{
    		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    		*cloud2 += *cloud;
    	  //pcl::PointCloud<pcl::PointXYZ> cloud2(*cloud);
    		//pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(&cloud2);
				std::cout << "Guessing your emotion using RTree..." << std::endl;
		  	float res = test(cloud2, 0);
				std::cout << res << std::endl;
    	}
			use_method = 0;
    }
    
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
		                        void* cookie)
		{
			if (event.getKeySym () == "s" && event.keyDown ())
			{
				use_method = 1;
			} else if (event.getKeySym () == "r" && event.keyDown ())
			{
				use_method = 2;
			}
		}
    
		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();

			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
				boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
			boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f1 = 
				boost::bind (&SimpleOpenNIViewer::test_cloud_, this, _1);
    		
			interface->registerCallback (f);
			interface->registerCallback (f1);
			
			interface->start ();
		  viewer.registerKeyboardCallback (&SimpleOpenNIViewer::keyboardEventOccurred, *this, NULL);
			while (!viewer.wasStopped())
			{
				boost::this_thread::sleep (boost::posix_time::seconds (1));
			}	
			interface->stop ();
		}

		pcl::visualization::CloudViewer viewer;
		int use_method;
};

int main ()
{
	std::cout << "Usage:\n"
						<< "Press s to classify your face using SVM.\n"
						<< "Press a to classify your face using RTree.\n";

	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}

