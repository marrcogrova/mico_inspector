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
        ///         "input":
        ///             {
        ///                 "left":"/dir/to/file/template %d.jpg",          // Path template to a set of left images files.
        ///                 "right":"/dir/to/file/template %d.jpg",         // Path template to a set of right images files.
        ///                 "depth":"/dir/to/file/template %d.jpg",         // Path template to a set of depth images files.
        ///                 "pointCloud":"/dir/to/file/template %d.jpg"     // Path template to a set of point cloud files.
        ///             },
        ///         "calibFile":"/dir/to/calib/file.xml"            // Path to the calibration file (optional).
		///     }
		/// \endcode
		///
		/// \param _filePath: path to the file.
		bool init(const cjson::Json &_json = "");

		/// \brief Get RGB pair of images
		/// \param _left: referente to a container for the left image.
		/// \param _right: referente to a container for the right image.
		bool rgb(cv::Mat &_left, cv::Mat &_right);

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

        /// \brief templatized method to define the interface for retrieving a new point cloud from the camera with
        /// spatial, surface normals and RGB information (custom types out of PCL).
        /// \param _cloud: reference to a container for the point cloud.
        template<typename PointType_>
        bool cloud(pcl::PointCloud<PointType_> &_cloud);

        /// \brief get the calibration matrices of the left camera in opencv format. Matrices are CV_32F.
        virtual bool leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the calibration matrices of the right camera in opencv format. Matrices are CV_32F.
        virtual bool rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera. Matrices are CV_32F.
        virtual bool extrinsic(cv::Mat &_rotation, cv::Mat &_translation);

        /// \brief get the extrinsic matrices, i.e., transformation from left to right camera.
        virtual bool extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation);

        /// \brief get disparty-to-depth parameter typical from RGB-D devices. Not all devices have this variable.
        virtual bool disparityToDepthParam(double &_dispToDepth);

        bool colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point);

	private:	// Private methods
		void depthToPointcloud(cv::Mat &_depth, pcl::PointCloud<pcl::PointXYZ> &_cloud);
	private:	// Private members
		unsigned mFrameCounter = 0;

        cv::Mat mLeft, mRight, mDepth;
        pcl::PointCloud<pcl::PointXYZ> mCloudXYZ;
        pcl::PointCloud<pcl::PointXYZRGB> mCloudXYZRGB;
        pcl::PointCloud<pcl::PointXYZRGBA> mCloudXYZRGBA;
        pcl::PointCloud<pcl::PointXYZRGBNormal> mCloudXYZRGBNormal;

		std::string mLeftImageFilePathTemplate;
		std::string mRightImageFilePathTemplate;
		std::string mDepthImageFilePathTemplate;
		std::string mPointCloudFilePathTemplate;

        bool mHasCalibration = false;
        cv::Mat mMatrixLeft, mDistCoefLeft, mMatrixRight, mDistCoefRight, mRot, mTrans;

        double mDispToDepth;

	};
}	//	namespace rgbd

#include <StereoCameras/StereoCameraVirtual.inl>

#endif		// RGBDSLAM_VISION_STEREOCAMERAS_STEREOCAMERAVIRTUAL_H_
