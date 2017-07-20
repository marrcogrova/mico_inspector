////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Authors: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDSLAM_STEREOCAMERAS_STEREOCAMERACUSTOM_H_
#define RGBDSLAM_STEREOCAMERAS_STEREOCAMERACUSTOM_H_

#include "../StereoCamera.h"
#include <opencv2/opencv.hpp>

#ifdef HAS_ZED_SDK
	#include <zed/Camera.hpp>
#endif

namespace rgbd {
	/// This class is used to integrate with the system custom stereo cameras.
	class StereoCameraCustom : public StereoCamera {
	public:		// Public interface
		/// Default constructor
		StereoCameraCustom();

		/// Default destructor of class. Ensure that cameras are properly detached.
		~StereoCameraCustom();

		/// \brief Initialize the camera using a config file in json.
		/// Config file must have following structure.
		///
		/// \code
		///     {
		///         "device":
		///             {
		///                 "type":"opencv|zed",                        // One of these. Currently only opencv and zed cameras are implemented as sources
		///                 "left":"/path/to/file/template %d.jpg",     // In case of OpenCV compatible cameras, provide the source of left and right.
		///                 "left":0,									// This can be an index for the camera, a path to a video file, or a template
		///                 "right":"/path/to/video.avi",               // path to a set of images.
		///                 "right":1,
		///					"indexZed":1,								// Index of zed camera, only if using zed
		///                 "calibFile":"/path/to/calib.yml"            // This element is a path to calibration files in OpenCV format stored in yml files.
		///                                                             // This file should contain following entries: {MatrixLeft, DistCoeffsLeft, MatrixRight,
		///                                                             // DistCoeffsRight, Rotation, Translation, Essential, Fundamental, RectificationLeft,
		///																// RectificationRight, ProjectionLeft, ProjectionRight, DisparityToDepth }.
		///					"resolution":
		///						{
		///							"width":4416|3840|2560|1344,
		///							"height":1242|1080|720|376
		///						}
		///             },
		///         "cloud":
		///             {
		///                 "type":"null|sparse|dense",       	// In case of OpenCV cameras, it is necessary to choose between sparse cloud generation
		///                                             		// dense cloud generation.
		///                 "sparse":   						// In case of type=sparse, this need to be present.
		///                     {
		///							"detector":
		///							{
		///								"type":"SIFT|SURF|FAST|ORB|ShiTomasi",
		///									"params" :
		///								{
		///									"nFeatures":5000,
		///										"qualityLevel" : 0.01,
		///										"minDist" : 0.5
		///								}
		///							},
		///								"descriptor":
		///							{
		///								"type":"SIFT|SURF|BRISK",
		///									"params" :
		///								{
		///								}
		///							},
		///								"matcher":
		///							{
		///								"type":"brute|flann|template",
		///									"params" :
		///								{
		///								}
		///							}
		///                     },
		///					"dense":
		///						{
		///							"disparity":
		///								{
		///									"algorithm":"BM|SGBM|elas",
		///									"params:		// BM and SGM need this parameter list
		///										{
		///											"numDisp":64,			// BM and SGBM.
		///											"sadWindow":21,			// BM and SGBM.
		///											"minDisp":16,			// only SGBM.
		///											"P1":0,					// only SGBM.
		///											"P2":0,					// only SGBM.
		///											"disp12MaxDiff":0,		// only SGBM.
		///											"preFilterCap":0,		// only SGBM.
		///											"uniquenessRatio":0,	// only SGBM.
		///											"speckleWindowSize":0,	// only SGBM.
		///											"speckleRange":0,		// only SGBM.
		///											"fullDP":false			// only SGBM.
		///										}
		///								}
		///						}
		///             }
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

	private:	// Private methods
		// Configuration methods
		bool configureDevice(const cjson::Json &_json);
		bool loadCalibrationFile(const std::string &_filePath);
		bool decodeCloudType(const cjson::Json &_json);
		void decodeDetector(const cjson::Json &_json);
		void decodeDescriptor(const cjson::Json &_json);
		void decodeMatcher(const cjson::Json &_json);
		void decodeDisparityAlgorithm(const cjson::Json &_json);

		// Depth algorithms
		bool disparityLibelas(cv::Mat &_depth);
		bool disparityCvBm(cv::Mat &_depth);
		bool disparityCvSgbm(cv::Mat &_depth);

		// Cloud type switcher
		bool computeCloudDense(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
		bool computeCloudDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
		bool computeCloudDense(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);

		bool computeCloudSparse(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
		bool computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
		bool computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);

		// Auxiliar methods
		bool computePairFeatures(const cv::Mat &_left, const cv::Mat &_right, std::vector<cv::Point2i> &_features1, std::vector<cv::Point2i> &_features2);
		bool computeFeatures(const cv::Mat &_frame, std::vector<cv::Point2i> &_features);
		bool computeFeatures(const cv::Mat &_frame, std::vector<cv::KeyPoint> &_features, cv::Mat &_descriptors);
		void computeEpipoarLines(const std::vector<cv::Point2i> &_points, std::vector<cv::Vec3f> &_epilines);
		bool computeMatchesTemplate(const cv::Mat &_left, const cv::Mat &_right, std::vector<cv::Point2i> &_points1, std::vector<cv::Point2i> &_points2);
		bool computeMatchesFlann(const cv::Mat &_left, const cv::Mat &_right, std::vector<cv::Point2i> &_points1, std::vector<cv::Point2i> &_points2);
		std::vector<cv::Point3f> triangulate(const std::vector<cv::Point2i> &_points1, const std::vector<cv::Point2i> &_points2);

	private:	// Members
		// Internal type to switch between functions.
		enum eDeviceType { opencv, zed };
		eDeviceType mType;

		// Calibration matrixes.
		cv::Mat mMatrixLeft, mMatrixRight, mCoefLeft, mCoefRight, mR, mT, mE, mF;
		cv::Mat mRectificationLeft, mRectificationRight, mProjectionLeft, mProjectionRight, mDisparityToDepth;
		cv::Mat mRectificationMap[2][2];
		bool mIsCalibrated = false;

		// For opencv custom stereo cameras.
		cv::VideoCapture *mCameraLeft = nullptr, *mCameraRight = nullptr;

		// For zed custom stereo camera.
#ifdef HAS_ZED_SDK
		sl::zed::Camera *mZedCamera = nullptr;
#endif	// HAS_ZED_SDK

		// Configuration for data type
		enum class eCloudType { Null, Sparse, Dense };
		enum class eFeatureDetector { Null, SIFT, SURF, ORB, FAST, ShiTomasi };
		enum class eFeatureDescriptor { Null, SIFT, SURF, ORB };
		enum class eMatchingAlgorithm { Null, Brute, Flann, TemplateMatching };
		enum class eDepthAlgorithm { Null, BM, SGBM, elas };

		eCloudType			mCloudType = eCloudType::Null;
		eFeatureDetector	mFeatureDetector = eFeatureDetector::Null;
		eFeatureDescriptor	mFeatureDescriptor = eFeatureDescriptor::Null;
		eMatchingAlgorithm	mMatchingAlgorithm = eMatchingAlgorithm::Null;
		eDepthAlgorithm		mDepthAlgorithm = eDepthAlgorithm::Null;

		cv::Rect mRoiLeft, mRoiRight;

		cjson::Json mDetectorParams;
		cjson::Json mDisparityParams;

		// Resusable variables and variables to decide if reuse data or not.
		bool mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;
		cv::Mat mLeftFrame, mRightFrame, mDepth;
		pcl::PointCloud<pcl::PointXYZ>::Ptr				mCloudXYZ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr			mCloudXYZRGB;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr	mCloudXYZRGBNormal;
		pcl::PointCloud<pcl::PointNormal>::Ptr			mCloudXYZNormal;
	};
}	//	namespace rgbd

#endif /* RGBDSLAM_STEREOCAMERAS_STEREOCAMERACUSTOM_H_ */
