#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>


#include <mico/base/StereoCamera.h>
#include <mico/base/cjson/json.h>



#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

int main(int _argc, char** _argv){

	ros::init(_argc, _argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	image_transport::Publisher pubDepth = it.advertise("camera/image_depth", 1);
	ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("camera/PointCloud", 1);
	sensor_msgs::PointCloud2 outputCloud;

	cjson::Json mConfigFile;
	mico::StereoCamera *camera;
	if (_argc != 2) {
            std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
            return -1;
        }

        std::ifstream file(_argv[1]);
        if (!file.is_open()) {
            std::cout << "Error opening config file" << std::endl;
            return -1;
        }

        if (!mConfigFile.parse(file)) {
            std::cout << "Error parsing config file" << std::endl;
            return -1;
        }

	// Instantiate camera
	camera = mico::StereoCamera::create((std::string) mConfigFile["cameraType"]);

	// Init camera
	if (camera == nullptr || !camera->init(mConfigFile["deviceConfig"])) {
	    std::cout << "Failed initialization of the camera" << std::endl;
	    return false;
	}
	
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	// viewer->setBackgroundColor (0, 0, 0);
	//int i =0;
	ros::Rate loop_rate(10);
	while (nh.ok()) {
		cv::Mat left, right;
		camera->grab();
		camera->rgb(left, right);
		if(left.rows != 0){
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
			pub.publish(msg);
		}
		cv::Mat depth;
		camera->depth(depth);
		if(depth.rows != 0){
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();
			pubDepth.publish(msg);
		}

		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		camera->cloud(cloud);
		// viewer->removeAllPointClouds();
  		// viewer->addPointCloud<pcl::PointXYZRGB> (cloud.makeShared(), "cloud");
		// viewer->spinOnce(15);
		// i++;

		pcl::PCLPointCloud2 pcl_pc2;
		pcl::toPCLPointCloud2( cloud,pcl_pc2);
		pcl_pc2.header.frame_id = "camera";
		pubCloud.publish(pcl_pc2);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
