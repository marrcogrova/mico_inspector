////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Authors: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAEVA_H_
#define RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAEVA_H_

#include "../StereoCamera.h"

#if HAS_ARTEC_SDK
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/base/IO/ObjIO.h>
#include <artec/sdk/base/TArrayRef.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TRef.h>

namespace asdk {
	using namespace artec::sdk::base;
	using namespace artec::sdk::capturing;
};

#endif  //  HAS_ARTEC_SDK

#include <chrono>
#include <thread>


namespace rgbd {
	class StereoCameraEva :public StereoCamera {
	public:
		/// \brief Initialize the camera using a config file.
		/// Currently any configuration file is needed for this cameras.
		///
		/// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief Get a new point cloud from the camera with only spatial information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial, surface normals and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);

		/// \brief Get a new point cloud from the camera with spatial information and surface normals.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud);

		/// \brief Get the last image from the color camera. Take care, this method is only for debugging purposes
		/// ARTEC SDK manage the texture internally using OpenGL, so it is only possible to grab the texture and the cloud
		/// at the same time, and for performance purposes is done in the cloud methods that provide color information. Due to 
		/// that, the RGB image will be available only after calling some of those methods.
		bool rgb(cv::Mat &_left, cv::Mat &_right);

		/// \brief [DUMMY] Override of rbg method of StereoCamera. 
		bool depth(cv::Mat &_depth) {
			std::cout << "[STEREO CAMERA][ARTEC EVA] Artec cameras do not provide depth map" << std::endl;
			return false;
		}

		/// \brief Grab a new frame and prepare it to be processed. 
		bool grab();

		#if HAS_ARTEC_SDK
			/// \brief Defaul destructor of the class
			~StereoCameraEva() {
				mScanner = NULL;
				std::cout << "[STEREO CAMERA][ARTEC EVA] Disconnecting Artec's EVA camera ..." << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(5));
				std::cout << "[STEREO CAMERA][ARTEC EVA] Artec's EVA camera disconected." << std::endl;
			}

		#endif  //  HAS_ARTEC_SDK
	private:
		bool mHasRgb = true;
		cv::Mat mRgb;

		#if HAS_ARTEC_SDK
			asdk::TRef<asdk::IScanner> mScanner;
			asdk::TRef<asdk::IFrame> mFrame;
			asdk::TRef<asdk::IFrameMesh> mMesh;
			asdk::TRef<asdk::IFrameProcessor> mProcessor;


			asdk::ErrorCode mLastErrorCode;
		#endif  //  HAS_ARTEC_SDK
	};
}	//	namespace rgbd

#endif		// RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAEVA_H_
