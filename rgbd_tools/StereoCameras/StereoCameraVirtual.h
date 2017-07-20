////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_
#define RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_

#include "../StereoCamera.h"

namespace rgbd {
	/// Util class to simulate an stereo camera using a dataset.
	class StereoCameraVirtual :public StereoCamera {
	public:		// Public interface
		/// \brief Initialize the camera using a config file.
		/// Config file must have following structure.
		///
		/// \code
		///     {
		///         "left":"/dir/to/file/template %d.jpg",          // Path template to a set of left images files.
		///         "right":"/dir/to/file/template %d.jpg",         // Path template to a set of right images files.
		///         "depth":"/dir/to/file/template %d.jpg",         // Path template to a set of depth images files.
		///         "pointCloud":"/dir/to/file/template %d.jpg"     // Path template to a set of point cloud files.
		///     }
		/// \endcode
		///
		/// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief Get RGB pair of images
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		bool rgb(cv::Mat &_left, cv::Mat &_right, bool _undistort = true);

		/// \brief Obtaing depth image.
		/// \param _depth: referente to a container for the depth image.
		bool depth(cv::Mat &_depth);

		/// \brief Grab current data from camera to make sure that is synchronized
		bool grab();

		/// \brief Get point cloud.
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud);

		/// \brief Get colorized point cloud
		/// \param _cloud: reference to a container for the point cloud.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		bool cloud(pcl::PointCloud<pcl::PointNormal> &_cloud);

		/// \brief [DUMMY] Override of cloud method of StereoCamera.
		bool cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud);


	private:	// Private methods
		void depthToPointcloud(cv::Mat &_depth, pcl::PointCloud<pcl::PointXYZ> &_cloud);
	private:	// Private members
        unsigned mFrameCounter = 1;

		std::string mLeftImageFilePathTemplate;
		std::string mRightImageFilePathTemplate;
		std::string mDepthImageFilePathTemplate;
		std::string mPointCloudFilePathTemplate;
	};
}	//	namespace rgbd

#endif		// RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_
