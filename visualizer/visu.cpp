#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      if (!viewer.wasStopped())
        viewer.showCloud (cloud);
    }
   
    void take_photo_ ( const pcl::visualization::KeyboardEvent &event)
  	{
      if( event.getKeyCode() == VK_SPACE && event.keyDown() )
      {
      	cout << "Photo taken for emotion " << endl;
	    	//pcl::io::savePCDFile( "cloud.pcd", *this, true ); // Binary format
  		}
		}
		
		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();

			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
				boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
    	
    	boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard_function = 
    		boost::bind (&SimpleOpenNIViewer::take_photo_, this, _1);
    		
			interface->registerCallback (f);
			interface->registerCallback (keyboard_function);
			interface->start ();

			while (!viewer.wasStopped())
			{
				boost::this_thread::sleep (boost::posix_time::seconds (1));
			}
			interface->stop ();
		}

		pcl::visualization::CloudViewer viewer;
	};

int main ()
{
 SimpleOpenNIViewer v;
 v.run ();
 return 0;
}

