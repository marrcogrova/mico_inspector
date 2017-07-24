////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDSLAM_VISION_STEREOCAMERA_H_
#define RGBDSLAM_VISION_STEREOCAMERA_H_

#include <cjson/json.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <iostream>

namespace rgbd {
	/// \brief Abstract class of a stereo camera device. It defines the typical generic interfaces for devices that acquire 3d information
	/// as 3d scanners, RGB-D sensors, stereo cameras, etc.
	class StereoCamera {
	public:	// Static interface
		enum class eModel { Zed, ArtecEva, Virtual, Custom, Http, RealSense };

		static StereoCamera *create(eModel _model);

	public:	// Public interface
		/// \brief Abstract method to define the interface for the initialization of the cameras.
		/// \param _filePath: path to the file.
		virtual bool init(const cjson::Json &_json = "") = 0;

		/// \brief Abstract method to define the interface for retrieving RGB information (Or pair of RGB images).
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		virtual bool rgb(cv::Mat &_left, cv::Mat &_right) = 0;

		/// \brief Abstract method to define the interface for retrieving depth image.
		virtual bool depth(cv::Mat &_depth) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with
		/// only spatial information.
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with 
		/// spatial and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with
		/// spatial information and surface normals
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) = 0;

		/// \brief Abstract method to define the interface for retrieving a new point cloud from the camera with
		/// spatial, surface normals and RGB information.
		/// \param _cloud: reference to a container for the point cloud.
		virtual bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) = 0;

		/// \brief Abstract method to define the interface for grabing the current data from camera to make sure that is synchronized
		virtual bool grab() = 0;

        /// \brief get the calibration matrices of the left camera in opencv format.  Matrices are CV_32F.
        virtual bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the calibration matrices of the right camera in opencv format.  Matrices are CV_32F.
        virtual bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera. Matrices are CV_32F.
        virtual bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera.
        virtual bool extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation);

        /// \brief get disparty-to-depth parameter typical from RGB-D devices.
        virtual bool disparityToDepthParam(double &_dispToDepth);

        /// \brief default destructor.
        virtual ~StereoCamera() {};

	};	//	class StereoCamera
}	//	namespace rgbd

#endif	//	RGBDSLAM_VISION_STEREOCAMERA_H_
