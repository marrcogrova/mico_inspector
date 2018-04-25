#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rgbd_tools/StereoCamera.h>
#include <rgbd_tools/cjson/json.h>

int main(int _argc, char** _argv){

	ros::init(_argc, _argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);


	cjson::Json mConfigFile;
	rgbd::StereoCamera *camera;
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
	camera = rgbd::StereoCamera::create((std::string) mConfigFile["cameraType"]);

	// Init camera
	if (camera == nullptr || !camera->init(mConfigFile["deviceConfig"])) {
	    std::cout << "Failed initialization of the camera" << std::endl;
	    return false;
	}
	
	ros::Rate loop_rate(10);
	while (nh.ok()) {
		cv::Mat left, right;
		camera->rgb(left, right);
		if(left.rows != 0){
			cv::imshow("left", left);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left).toImageMsg();
			pub.publish(msg);
		}
		cv::waitKey(30);

		ros::spinOnce();
		loop_rate.sleep();
std::cout << "as" <<std::endl;
	}
}
