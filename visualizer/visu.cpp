#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>



class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer"), take_photo(0) {}

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }
		
		void take_photo_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (take_photo == 1)
      {
				std::cout << "Photo taken for joy" << std::endl;
				pcl::io::savePCDFile( "joy.pcd", *cloud, true ); // Binary format
      } else if (take_photo == 2)
      {
   			std::cout << "Photo taken for anger" << std::endl;
				pcl::io::savePCDFile( "anger.pcd", *cloud, true ); // Binary format
      }
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
		}

		pcl::visualization::CloudViewer viewer;
		int take_photo;
};

int main ()
{
 std::cout << "Usage:\n"
 					 << "Press j to take a photo of joy.\n"
 					 << "Press a to take a photo of anger.\n";
 
 SimpleOpenNIViewer v;
 v.run ();
 return 0;
}

