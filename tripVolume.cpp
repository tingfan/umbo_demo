#include <opencv2/opencv.hpp>

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

#include <opencv2/opencv.hpp>

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

class MyGrabber {

	boost::signals2::connection cloud_connection;
	boost::signals2::connection image_connection;
	boost::shared_ptr<pcl::io::OpenNI2Grabber> grabber_;

public:
	MyGrabber(boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> cloud_cb,
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&)> image_cb) {
		boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager =
				pcl::io::openni2::OpenNI2DeviceManager::getInstance();
		if (deviceManager->getNumOfConnectedDevices() > 0) {
			boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getAnyDevice();
			cout << "Device ID not set, using default device: " << device->getStringID() << endl;
		}
		std::string device_id("");
		pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
		pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
		grabber_.reset(new pcl::io::OpenNI2Grabber(device_id, depth_mode, image_mode));
		cout << "Framerate:" << grabber_->getFramesPerSecond() << std::endl;

		cloud_connection = grabber_->registerCallback(cloud_cb);

		image_connection = grabber_->registerCallback(image_cb);
	}

	void start() {
		grabber_->start();
	}

	void stop() {
		grabber_->stop();
		cloud_connection.disconnect();
		image_connection.disconnect();
	}
};

class DummyGrabber {
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> cloud_cb;
	boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&)> image_cb;
	boost::thread generator;
	bool runThead;
public:
	DummyGrabber(boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> cloud_cb_,
			boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image>&)> image_cb_) :
			cloud_cb(cloud_cb_), image_cb(image_cb_) {
	}

	void run() {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		while (runThead) {
			cloud->clear();
			float z_offset = 0.1*((float)rand()/RAND_MAX-0.5);
			for (int i = 0; i < 640*480; i++) {
				pcl::PointXYZRGBA point;
				point.x = (float)rand()/RAND_MAX-0.5;
				point.y = (float)rand()/RAND_MAX-0.5;

				float z_baseline = 1.25+0.1*point.x +0.2*point.y;
				if(fabs(point.x)<0.3 && fabs(point.y)<0.2)
				{
					point.z =  z_offset + z_baseline;
				}
				else
				{
					point.z = z_baseline;
				}
				point.r = 128;
				point.g = 128;
				point.b = 128;
//				point.a = 255;
				cloud->points.push_back(point);
			}

			//z-coloring
			float zmax=std::numeric_limits<float>::min();
			float zmin=std::numeric_limits<float>::max();
			for(int i=0;i<cloud->points.size();i++)
			{
				zmax = MAX(cloud->points[i].z,zmax);
				zmin= MIN(cloud->points[i].z,zmin);
			}
			for(int i=0;i<cloud->points.size();i++)
			{
				cloud->points[i].b = 255.0*(cloud->points[i].z-zmin)/(zmax-zmin);
			}

			//
			cloud->width = 1024; //cloud->points.size();
			cloud->height =768; // 1;

			cloud_cb(cloud);

			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
	}
	void start() {
		runThead = true;
		generator = boost::thread(boost::bind(&DummyGrabber::run, this));
	}

	void stop() {
		runThead = false;
		generator.join();
	}
}
;

class OpenNIChangeViewer {
public:
	typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
	typedef Cloud::ConstPtr CloudConstPtr;

	OpenNIChangeViewer(double resolution, int mode, int noise_filter) :
			cloud_viewer(new pcl::visualization::PCLVisualizer("3D Camera")), rgb_data_(0), rgb_data_size_(0) {
		octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA>(resolution);
		mode_ = mode;
		noise_filter_ = noise_filter;
	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
		FPS_CALC("cloud callback");
		std::cerr << cloud->points.size() << " -- ";

		// assign point cloud to octree
		octree->setInputCloud(cloud);

		// add points from cloud to octree
		octree->addPointsFromInputCloud();

		std::cerr << octree->getLeafCount() << " -- ";
		boost::shared_ptr<std::vector<int> > newPointIdxVector(new std::vector<int>);

		// get a vector of new points, which did not exist in previous buffer
		octree->getPointIndicesFromNewVoxels(*newPointIdxVector, noise_filter_);

		std::cerr << newPointIdxVector->size() << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud;

		switch (mode_) {
		case REDDIFF_MODE:
			//modify those new point to red
			filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
			filtered_cloud->points.reserve(newPointIdxVector->size());

			//set interested ones to red.
			for (std::vector<int>::iterator it = newPointIdxVector->begin(); it != newPointIdxVector->end(); it++) {
				//filtered_cloud->points[*it].rgba = 255 << 16;
				pcl::PointXYZRGBA& pt = filtered_cloud->points[*it];
				if (pt.z > 1.0 && pt.z < 1.5)
					pt.rgba = 255 << 16;
				//cout << pt.x << " " << pt.y << " " << pt.z << endl;
			}
			break;
		case ONLYDIFF_MODE:
			filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
			filtered_cloud->points.reserve(newPointIdxVector->size());

			for (std::vector<int>::iterator it = newPointIdxVector->begin(); it != newPointIdxVector->end(); it++)
				filtered_cloud->points.push_back(cloud->points[*it]);

			break;
		}

		{
			boost::mutex::scoped_lock lock(cloud_mutex_);
			cloud_ = filtered_cloud;
		}
		// switch buffers - reset tree
		octree->switchBuffers();
	}

	void image_callback(const boost::shared_ptr<pcl::io::openni2::Image>& image) {
		FPS_CALC("image callback");
		{
			boost::mutex::scoped_lock lock(image_mutex_);
			image_ = image;
		}

//		if (image->getEncoding() != pcl::io::openni2::Image::RGB) {
		if (rgb_data_size_ < image->getWidth() * image->getHeight()) {
			if (rgb_data_)
				delete[] rgb_data_;
			rgb_data_size_ = image->getWidth() * image->getHeight();
			rgb_data_ = new unsigned char[rgb_data_size_ * 3];
		}

		//convert to OpenCV
		image_->fillRGB(image_->getWidth(), image_->getHeight(), rgb_data_);
//		}
	}

	void run() {

		cloud_viewer->setCameraFieldOfView(1.02259994f);

		bool cloud_init = false;
		bool image_init = false;

//		MyGrabber grabber_(boost::bind(&OpenNIChangeViewer::cloud_cb_, this, _1), boost::bind(&OpenNIChangeViewer::image_callback, this, _1));
		DummyGrabber grabber_(boost::bind(&OpenNIChangeViewer::cloud_cb_, this, _1),
				boost::bind(&OpenNIChangeViewer::image_callback, this, _1));
		grabber_.start();
		while (!cloud_viewer->wasStopped()) {
			CloudConstPtr cloud;
			if (cloud_mutex_.try_lock()) {
				cloud_.swap(cloud);
				cloud_mutex_.unlock();
			}
			if (cloud) {
				if (!cloud_init) {
					cloud_viewer->setPosition(0, 0);
					cloud_viewer->setSize(cloud->width, cloud->height);
					cloud_viewer->setBackgroundColor(0,0,0.1);
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
//			cloud_viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 1), "line", 0);
			cloud_viewer->addCoordinateSystem(0,2);
			cloud_viewer->spinOnce();

			boost::shared_ptr<pcl::io::openni2::Image> image;
			if (image_mutex_.try_lock()) {
				image_.swap(image);
				image_mutex_.unlock();
			}

			if (image) {

				cv::Mat mat(image->getHeight(), image->getWidth(), CV_8UC3, rgb_data_);
				cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

				if (lastImage.size == mat.size) {
					cv::absdiff(mat, lastImage, diffImage);
				}
				mat.copyTo(lastImage);
				if (diffImage.cols > 0) {
					for (int i = 0; i < diffImage.rows; i++)
						for (int j = 0; j < diffImage.cols; j++) {
							cv::Vec3b& v = diffImage.at<cv::Vec3b>(i, j);
							int sum = (int) v[0] + (int) v[1] + (int) v[2];
							if (sum > 100) {
								mat.at<cv::Vec3b>(i, j)[2] = 255; //paint it red
							}
						}
					cv::imshow("test", mat);
				}

			}

			if (!image && !cloud) {
				cv::waitKey(1);
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			}
		}
		grabber_.stop();

		if (rgb_data_)
			delete[] rgb_data_;

	}

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> *octree;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer;

	int mode_;
	int noise_filter_;

	boost::mutex cloud_mutex_;
	boost::mutex image_mutex_;

	CloudConstPtr cloud_;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;
	cv::Mat diffImage;
	cv::Mat diffImageMono;
	cv::Mat lastImage;

};

int main(int argc, char* argv[]) {

	std::cout << "Syntax is " << argv[0] << " [-r octree resolution] [-d] [-n noise_filter intensity] \n";

	int mode3dDiff = REDDIFF_MODE;
	int noise_filter = 7;
	double resolution = 0.01;

	pcl::console::parse_argument(argc, argv, "-r", resolution);

	pcl::console::parse_argument(argc, argv, "-n", noise_filter);

	if (pcl::console::find_argument(argc, argv, "-d") > 0) {
		mode3dDiff = ONLYDIFF_MODE;
	}

	OpenNIChangeViewer v(resolution, mode3dDiff, noise_filter);
	v.run();
	return 0;
}

