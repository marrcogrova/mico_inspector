////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "StereoCameraRealSense.h"

#ifdef ENABLE_LIBREALSENSE
	#include <librealsense/rs.hpp>
#endif

//#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

#include <cstdio>

namespace rgbd {
	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::init(const cjson::Json & _json){
		#ifdef ENABLE_LIBREALSENSE
			mConfig = _json;
			mRsContext = new rs::context();
			if (mRsContext->get_device_count() == 0) {
				std::cout << "[STEREOCAMERA][REALSENSE] There's no any compatible device connected." << std::endl;
				return false;
			}

			// Get device
			mRsDevice = mRsContext->get_device(0);
			std::cout << "[STEREOCAMERA][REALSENSE] Using device 0, an "<< mRsDevice->get_name() << std::endl;
			std::cout << "[STEREOCAMERA][REALSENSE]     Serial number: " << mRsDevice->get_serial() << std::endl;
			std::cout << "[STEREOCAMERA][REALSENSE]     Firmware version: " << mRsDevice->get_firmware_version() << std::endl;

			// Initialize streams of data.
			mRsDevice->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
			mRsDevice->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
			mRsDevice->start();

			mRsDepthIntrinsic = new rs::intrinsics();
			auto tempDepthIntrinsic = mRsDevice->get_stream_intrinsics(rs::stream::depth);
			memcpy(mRsDepthIntrinsic, &tempDepthIntrinsic, sizeof(rs::intrinsics));
		
			mRsDepthToColor = new rs::extrinsics(); 
			auto tempDepth2Color = mRsDevice->get_extrinsics(rs::stream::depth, rs::stream::color);
			memcpy(mRsDepthToColor, &tempDepth2Color, sizeof(rs::extrinsics));
		
			mRsColorIntrinsic = new rs::intrinsics(); 
			auto tempColorIntrinsic = mRsDevice->get_stream_intrinsics(rs::stream::color);
			memcpy(mRsColorIntrinsic, &tempColorIntrinsic, sizeof(rs::intrinsics));
		
			mRsDepthScale		= mRsDevice->get_depth_scale();

			mUseUncolorizedPoints = (bool) mConfig["useUncolorizedPoints"];

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::rgb(cv::Mat & _left, cv::Mat & _right, bool _undistort){
		#ifdef ENABLE_LIBREALSENSE
			_left = mLastRGB;
			return mHasRGB;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::depth(cv::Mat & _depth){
		#ifdef ENABLE_LIBREALSENSE
			_depth = mLastDepth;
			return mComputedDepth;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::grab(){
		#ifdef ENABLE_LIBREALSENSE
			mRsDevice->wait_for_frames();

			cv::cvtColor(cv::Mat(mRsColorIntrinsic->height, mRsColorIntrinsic->width, CV_8UC3, (uchar*)mRsDevice->get_frame_data(rs::stream::color)), mLastRGB, CV_RGB2BGR);
			mHasRGB = true;
		
			mLastDepth = cv::Mat(mRsDepthIntrinsic->height, mRsDepthIntrinsic->width, CV_16U, (uchar*) mRsDevice->get_frame_data(rs::stream::depth));
			mComputedDepth = true;

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
		#ifdef ENABLE_LIBREALSENSE
			for (int dy = 0; dy < mLastDepth.rows; ++dy) {
				for (int dx = 0; dx < mLastDepth.cols; ++dx) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
					uint16_t depth_value = mLastDepth.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;

				// Skip over pixels with a depth value of zero, which is used to indicate no data
				if (depth_value == 0) {
					if (mUseUncolorizedPoints) {
						_cloud.push_back(pcl::PointXYZ(NAN, NAN, NAN));
					}
					else
						continue;
				}
				else {
					// Map from pixel coordinates in the depth image to pixel coordinates in the color image
					rs::float2 depth_pixel = { (float)dx, (float)dy };
					rs::float3 depth_point = mRsDepthIntrinsic->deproject(depth_pixel, depth_in_meters);

						_cloud.push_back(pcl::PointXYZ(depth_point.x, depth_point.y, depth_point.z));
					}
				}
			}
			if (mUseUncolorizedPoints)
				setOrganizedAndDense(_cloud);

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
		#ifdef ENABLE_LIBREALSENSE
			for (int dy = 0; dy < mLastDepth.rows; ++dy) {
				for (int dx = 0; dx < mLastDepth.cols; ++dx) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
					uint16_t depth_value = mLastDepth.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;

				// Set invalid pixels with a depth value of zero, which is used to indicate no data
				pcl::PointXYZRGB point;
				if (depth_value == 0) {
					if (mUseUncolorizedPoints) {
						point.x = NAN;
						point.y = NAN;
						point.z = NAN;
					}
					else
						continue;
				}
				else {
					// Map from pixel coordinates in the depth image to pixel coordinates in the color image
					rs::float2 depth_pixel = { (float)dx, (float)dy };
					rs::float3 depth_point = mRsDepthIntrinsic->deproject(depth_pixel, depth_in_meters);
					rs::float3 color_point = mRsDepthToColor->transform(depth_point);
					rs::float2 color_pixel = mRsColorIntrinsic->project(color_point);

						// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
						const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
						point.x = depth_point.x;
						point.y = depth_point.y;
						point.z = depth_point.z;

						if (cx < 0 || cy < 0 || cx >= mRsColorIntrinsic->width || cy >= mRsColorIntrinsic->height) {
							if (mUseUncolorizedPoints) {
								point.r = 255;
								point.g = 255;
								point.b = 255;
							} else {
								continue;
							}
						} else {
							auto rgb = mLastRGB.at<cv::Vec3b>(cy, cx);
							point.r = rgb[2];
							point.g = rgb[1];
							point.b = rgb[0];
						}
					}

					_cloud.push_back(point);
				}
			}

			if (_cloud.size() == 0) {
				return false;
			}

			if(mUseUncolorizedPoints)
				setOrganizedAndDense(_cloud);

			return true;

		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
		if (!mUseUncolorizedPoints) {
			std::cout << "[STEREOCAMERA][REALSENSE] Cannot compute the normals if points out of the colorized region are ignored. Please set the \"UseUncolorizedPoints\" to true in the configuration of the camera" << std::endl;
			return false;
		}
		else {
			pcl::PointCloud<pcl::PointXYZRGB> cloudWoNormals;
			if (!cloud(cloudWoNormals)) {
				return false;
			}

			//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			ne.setInputCloud(cloudWoNormals.makeShared());
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.02f);
			ne.setNormalSmoothingSize(10.0f);
			ne.compute(_cloud);

			// Fill XYZ and RGB of cloud
			for (unsigned i = 0; i < _cloud.size(); i++) {
				_cloud[i].x = cloudWoNormals[i].x;
				_cloud[i].y = cloudWoNormals[i].y;
				_cloud[i].z = cloudWoNormals[i].z;
				_cloud[i].r = cloudWoNormals[i].r;
				_cloud[i].g = cloudWoNormals[i].g;
				_cloud[i].b = cloudWoNormals[i].b;
			}

			return true;
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
		//pcl::PointCloud<pcl::PointXYZ> cloudWoNormals;
		//if (!cloud(cloudWoNormals)) {
		//	return false;
		//}

		//pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
		//ne.setInputCloud(cloudWoNormals.makeShared());

		//// Create an empty kdtree representation, and pass it to the normal estimation object.
		//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		//ne.setSearchMethod(tree);

		//// Use all neighbors in a sphere of radius 3cm
		//ne.setRadiusSearch(0.03);

		//// Compute the features
		//ne.compute(_cloud);

		//return true;
		return false;
	}
}	//	namespace rgbd