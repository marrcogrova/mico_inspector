////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#include "StereoCameraVirtual.h"

#include <fstream>

#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace pcl;
using namespace std;

namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::init(const cjson::Json &_json) {
		if (_json.isObject()) {
			mLeftImageFilePathTemplate = std::string(_json["left"]);
			mRightImageFilePathTemplate = std::string(_json["right"]);
			mDepthImageFilePathTemplate = std::string(_json["depth"]);
			mPointCloudFilePathTemplate = std::string(_json["pointCloud"]);

			return true;
		}
		else {
			std::cout << "[STEREO CAMERA][VIRTUAL] Virtual stereo camera couldn't be initialized" << std::endl;
			return false;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::rgb(Mat & _left, Mat & _right, bool _undistort) {
		if (mLeftImageFilePathTemplate == "" && mRightImageFilePathTemplate == "") {
			return false;
		}

		if (mLeftImageFilePathTemplate != "") {
			int indexEntryPoint = mLeftImageFilePathTemplate.find("%d");
			string imagePath = mLeftImageFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mDepthImageFilePathTemplate.substr(indexEntryPoint + 2);

#ifdef _WIN32
			_left = imread(imagePath);
#elif __linux__
			_left = imread(imagePath.substr(0, imagePath.size() - 1));
#endif
		}

		if (mRightImageFilePathTemplate != "") {
			int indexEntryPoint = mRightImageFilePathTemplate.find("%d");
			string imagePath = mRightImageFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mDepthImageFilePathTemplate.substr(indexEntryPoint + 2);

#ifdef _WIN32
			_right = imread(imagePath);
#elif __linux__
			_right = imread(imagePath.substr(0, imagePath.size() - 1));
#endif
		}

		if (_right.rows == 0 && _left.rows == 0) {
			return false;
		}
		else {
			if (_right.rows == 0 || _left.rows == 0) {
				std::cout << "[STEREO CAMERA][VIRTUAL] Warning, this camera only provide one color image\n";
			}

			return true;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::depth(Mat & _depth) {
		if (mDepthImageFilePathTemplate == "") {
			return false;
		}

		int indexEntryPoint = mDepthImageFilePathTemplate.find("%d");
		string imagePath = mDepthImageFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mDepthImageFilePathTemplate.substr(indexEntryPoint + 2);
#ifdef _WIN32
		_depth = imread(imagePath, CV_LOAD_IMAGE_UNCHANGED);
#elif __linux__
		_depth = imread(imagePath.substr(0, imagePath.size() - 1), CV_LOAD_IMAGE_UNCHANGED);
#endif

		if (_depth.rows == 0)
			return false;
		else
			return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(PointCloud<PointXYZ>& _cloud) {

		if (mPointCloudFilePathTemplate == "") {
			if (mDepthImageFilePathTemplate == "") {
				return false;
			}
			else {
				Mat depthFrame;
				depth(depthFrame);
				depthToPointcloud(depthFrame, _cloud);
				return true;
			}
		}
		else {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			_cloud.sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
			_cloud.sensor_origin_ = { 0,0,0,0 };
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			_cloud.sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
			_cloud.sensor_origin_ = { 0,0,0,0 };
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			_cloud.sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
			_cloud.sensor_origin_ = { 0,0,0,0 };
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			_cloud.sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
			_cloud.sensor_origin_ = { 0,0,0,0 };
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}


	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::grab() {
		mFrameCounter++;
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraVirtual::depthToPointcloud(Mat & _depth, PointCloud<PointXYZ>& _cloud) {
		// Fake parameters
		int cx = 318.6;
		int cy = 255.3;
		double fx = 517.3;
		double fy = 516.5;

		for (int i = 0; i < _depth.rows; i++) {
			for (int j = 0; j < _depth.cols; j++) {
				double z = double(_depth.at<unsigned short>(i*_depth.cols + j)) / 5000;
				if (!z)
					continue;
				double x = double(j - cx)*z / fx;
				double y = double(i - cy)*z / fy;
				_cloud.push_back(PointXYZ(x, y, z));
			}
		}
	}
}	//	namespace rgbd