#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include "../common.hpp"


class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"), take_photo(0)
    {
    	nb_images = get_number_images();
    }

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }
		
		void take_photo_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
  		std::string type(get_emotion_from_index(take_photo-1));
			//std::istringstream buffer(+1);
			//int index;
			//buffer >> temp;
  		//int index = ;
			std::string file(SSTR(nb_images[take_photo-1]+1));
			std::string file_name = SSTR("/home/theo/Documents/3D/Projet/emotionRecog/data/images/" + type + "/" + file + ".pcd");
      if (take_photo == 1)
      {
				std::cout << "Photo taken for joy" << std::endl;
				pcl::io::savePCDFile( file_name, *cloud, true ); // Binary format
      } else if (take_photo == 2)
      {
   			std::cout << "Photo taken for anger" << std::endl;
				pcl::io::savePCDFile( file_name, *cloud, true ); // Binary format
      } else if (take_photo == 3)
      {
   			std::cout << "Photo taken for fear" << std::endl;
				pcl::io::savePCDFile( file_name, *cloud, true ); // Binary format
      } else if (take_photo == 4)
      {
   			std::cout << "Photo taken for surprise" << std::endl;
				pcl::io::savePCDFile( file_name, *cloud, true ); // Binary format
      } else if (take_photo == 5)
      {
   			std::cout << "Photo taken for sadness" << std::endl;
				pcl::io::savePCDFile( file_name, *cloud, true ); // Binary format
      }
			nb_images[take_photo-1]++;
      take_photo = 0;
    }
    
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
		                        void* cloud_void)
		{
			if (event.getKeySym () == "j" && event.keyDown ())
			{
				take_photo = 1;
			} else if (event.getKeySym () == "a" && event.keyDown ())
			{
				take_photo = 2;
			} else if (event.getKeySym () == "f" && event.keyDown ())
			{
				take_photo = 3;
			} else if (event.getKeySym () == "s" && event.keyDown ())
			{
				take_photo = 4;
			} else if (event.getKeySym () == "t" && event.keyDown ())
			{
				take_photo = 5;
			}
		}
    
		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();

			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
				boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f1 = 
				boost::bind (&SimpleOpenNIViewer::take_photo_, this, _1);
    		
			interface->registerCallback (f);
			interface->registerCallback (f1);
			
			interface->start ();
		  viewer.registerKeyboardCallback (&SimpleOpenNIViewer::keyboardEventOccurred, *this, NULL);
			while (!viewer.wasStopped())
			{
				boost::this_thread::sleep (boost::posix_time::seconds (1));
			}	
			interface->stop ();
			
			std::ofstream out_file("/home/theo/Documents/3D/Projet/emotionRecog/data/images/nb_images.txt");
			if (out_file.is_open())
			{
				out_file << nb_images[0] << "\n" << nb_images[1] << "\n" << nb_images[2] << "\n" << nb_images[3] << "\n" << nb_images[4];
				out_file.close();
			}
		}

		pcl::visualization::CloudViewer viewer;
		int take_photo;
		std::vector<int> nb_images;
};

int main ()
{
	std::cout << "Usage:\n"
						<< "Press j to take a photo of joy.\n"
						<< "Press a to take a photo of anger.\n"
						<< "Press f to take a photo of fear.\n"
						<< "Press s to take a photo of surprise.\n"
						<< "Press t to take a photo of sadness.\n";

	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}

