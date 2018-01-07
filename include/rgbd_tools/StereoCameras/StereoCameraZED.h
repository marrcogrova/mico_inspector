////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAZED_H_
#define RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAZED_H_


#include <rgbd_tools/StereoCamera.h>

#ifdef HAS_ZED_SDK
    #include <zed/Camera.hpp>
#endif

namespace rgbd {
	/// \brief Wrapper of ZED stereo camera.
	class StereoCameraZed :public StereoCamera {
	public:
		/// \brief Initialize the ZED stereo camera using a config file.
		/// Config file must have following structure.
		///
		/// \code
		///		{
		///			"mode":"default"	// Now is the only option.
		///		}
		/// \endcode
		///
		/// \param _json: Configuration file.
		bool init(const cjson::Json &_json = "");

		/// \brief Get pair of rgb images from the camera.
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		bool rgb(cv::Mat &_left, cv::Mat &_right);

		/// \brief Get depth image from the camera.
		/// \param _depth: referente to a container for the depth image
		bool depth(cv::Mat &_depth);

		/// \brief Retrieve an uncolorized point cloud from the camera
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

		/// \brief Retrieve a colorized point cloud from the camera
		bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) {
			std::cout << "[STEREO CAMERA][ZED] Cloud method is not currently implemented in ZED cameras" << std::endl;
			return false;
		}

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		virtual bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) {
			std::cout << "[STEREO CAMERA][ZED] Cloud method is not currently implemented in ZED cameras" << std::endl;
			return false;
		}

		/// \brief The function grabs a new image, rectifies it and computes the disparity map and optionally the depth map.
		bool grab();

        bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){return false;}
	private:
#ifdef HAS_ZED_SDK
		sl::zed::Camera *mZedCamera = nullptr;
#endif	// HAS_ZED_SDK

		bool mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;
		cv::Mat mLeftFrame, mRightFrame, mDepth;
		pcl::PointCloud<pcl::PointXYZ> mCloudXYZ;
		pcl::PointCloud<pcl::PointXYZRGB> mCloudXYZRGB;
		pcl::PointCloud<pcl::PointXYZRGBNormal> mCloudXYZRGBNormal;
		pcl::PointCloud<pcl::PointNormal> mCloudXYZNormal;

	};
}	//	namespace rgbd

#endif		// RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAZED_H_
