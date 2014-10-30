#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include "pcl/io/openni2/openni.h"
#include <pcl/common/time.h> //fps calculations


#include <pcl/console/parse.h>
#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
  do \
{ \
}while (false)
#endif
enum {
	REDDIFF_MODE, ONLYDIFF_MODE, MODE_COUNT
};

class OpenNIChangeViewer {
public:
	typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
	typedef Cloud::ConstPtr CloudConstPtr;

	OpenNIChangeViewer (double resolution, int mode, int noise_filter)
	: cloud_viewer (new pcl::visualization::PCLVisualizer ("3D Camera")),
	image_viewer (new pcl::visualization::ImageViewer ("2D Camera")),
	rgb_data_ (0), rgb_data_size_ (0)
	{
		octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA>(resolution);
		mode_ = mode;
		noise_filter_ = noise_filter;
	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		std::cerr << cloud->points.size() << " -- ";

		// assign point cloud to octree
		octree->setInputCloud(cloud);

		// add points from cloud to octree
		octree->addPointsFromInputCloud();

		std::cerr << octree->getLeafCount() << " -- ";
		boost::shared_ptr<std::vector<int> > newPointIdxVector(
				new std::vector<int>);

		// get a vector of new points, which did not exist in previous buffer
		octree->getPointIndicesFromNewVoxels(*newPointIdxVector, noise_filter_);

		std::cerr << newPointIdxVector->size() << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud;

		switch (mode_) {
		case REDDIFF_MODE:
			//modify those new point to red
			filtered_cloud.reset(
					new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
			filtered_cloud->points.reserve(newPointIdxVector->size());

			for (std::vector<int>::iterator it = newPointIdxVector->begin();
					it != newPointIdxVector->end(); it++)
				filtered_cloud->points[*it].rgba = 255 << 16;
			break;
		case ONLYDIFF_MODE:
			filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
			filtered_cloud->points.reserve(newPointIdxVector->size());

			for (std::vector<int>::iterator it = newPointIdxVector->begin();
					it != newPointIdxVector->end(); it++)
				filtered_cloud->points.push_back(cloud->points[*it]);

			break;
		}
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = filtered_cloud;

		// switch buffers - reset tree
		octree->switchBuffers();
	}

	void image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image) {
		FPS_CALC("image callback");
		{
			boost::mutex::scoped_lock lock(image_mutex_);
			image_ = image;
		}

		if (image->getEncoding() != pcl::io::openni2::Image::RGB) {
			if (rgb_data_size_ < image->getWidth() * image->getHeight()) {
				if (rgb_data_)
					delete[] rgb_data_;
				rgb_data_size_ = image->getWidth() * image->getHeight();
				rgb_data_ = new unsigned char[rgb_data_size_ * 3];
			}
			image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
		}
	}

	void run() {
		boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager =
				pcl::io::openni2::OpenNI2DeviceManager::getInstance();
		if (deviceManager->getNumOfConnectedDevices() > 0) {
			boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device =
					deviceManager->getAnyDevice();
			cout << "Device ID not set, using default device: "
					<< device->getStringID() << endl;
		}
		std::string device_id("");
		pcl::io::OpenNI2Grabber::Mode depth_mode =
				pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
		pcl::io::OpenNI2Grabber::Mode image_mode =
				pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
		pcl::io::OpenNI2Grabber grabber_(device_id, depth_mode, image_mode);

		boost::function<
				void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> cloud_cb =
				boost::bind(&OpenNIChangeViewer::cloud_cb_, this, _1);
		boost::signals2::connection cloud_connection =
				grabber_.registerCallback(cloud_cb);

		boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&)> image_cb =
				boost::bind(&OpenNIChangeViewer::image_callback, this, _1);
		boost::signals2::connection image_connection =
				grabber_.registerCallback(image_cb);

		cloud_viewer->setCameraFieldOfView(1.02259994f);

		bool cloud_init = false;
		bool image_init = false;

		grabber_.start();
		//interface->start ();
		while (!cloud_viewer->wasStopped()) {
			cloud_viewer->spinOnce();

			CloudConstPtr cloud;
			if (cloud_mutex_.try_lock()) {
				cloud_.swap(cloud);
				cloud_mutex_.unlock();
			}
			if (cloud) {
				//viewer.showCloud (cloud);
				if (!cloud_init) {
					cloud_viewer->setPosition(0, 0);
					cloud_viewer->setSize(cloud->width, cloud->height);
					cloud_init = !cloud_init;
				}

				if (!cloud_viewer->updatePointCloud(cloud, "OpenNICloud")) {
					cloud_viewer->addPointCloud(cloud, "OpenNICloud");
					cloud_viewer->resetCameraViewpoint("OpenNICloud");
					cloud_viewer->setCameraPosition(0, 0, 0,		// Position
							0, 0, 1,		// Viewpoint
							0, -1, 0);	// Up
				}
			}

		    boost::shared_ptr<pcl::io::openni2::Image> image;
			if(image_mutex_.try_lock())
			{
				image_.swap(image);
		        image_mutex_.unlock ();
			}

			if(image)
		      {
		        if (!image_init && cloud && cloud->width != 0)
		        {
		          image_viewer->setPosition (cloud->width, 0);
		          image_viewer->setSize (cloud->width, cloud->height);
		          image_init = !image_init;
		        }

		        if (image->getEncoding () == pcl::io::openni2::Image::RGB)
		          image_viewer->addRGBImage ( (const unsigned char*)image->getData (), image->getWidth (), image->getHeight ());
		        else
		          image_viewer->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
		        image_viewer->spinOnce ();
		      }


		}
		grabber_.stop();
		cloud_connection.disconnect();
		image_connection.disconnect();

	}

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> *octree;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;
	boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer;

	int mode_;
	int noise_filter_;

	boost::mutex cloud_mutex_;
	boost::mutex image_mutex_;

	CloudConstPtr cloud_;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;

};

int main(int argc, char* argv[]) {

	std::cout << "Syntax is " << argv[0]
			<< " [-r octree resolution] [-d] [-n noise_filter intensity] \n";

	int mode = REDDIFF_MODE;
	int noise_filter = 7;
	double resolution = 0.01;

	pcl::console::parse_argument(argc, argv, "-r", resolution);

	pcl::console::parse_argument(argc, argv, "-n", noise_filter);

	if (pcl::console::find_argument(argc, argv, "-d") > 0) {
		mode = ONLYDIFF_MODE;
	}

	OpenNIChangeViewer v(resolution, mode, noise_filter);
	v.run();
	return 0;
}
