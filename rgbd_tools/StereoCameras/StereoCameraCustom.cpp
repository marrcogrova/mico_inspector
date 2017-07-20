////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Authors: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <StereoCameras/StereoCameraCustom.h>
#ifdef USE_LIBELAS
	#include <libelas/elas.h>
#endif
#include "ParallelFeatureMatcher.h"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>

using namespace std;
using namespace cv;
using namespace pcl;

namespace rgbd{
	StereoCameraCustom::StereoCameraCustom() {
		mCloudXYZ			= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		mCloudXYZRGB		= pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		mCloudXYZRGBNormal	= pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		mCloudXYZNormal		= pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	}
	//---------------------------------------------------------------------------------------------------------------------
	StereoCameraCustom::~StereoCameraCustom(){
		if (mCameraLeft) {
			mCameraLeft->release();
			delete mCameraLeft;
		}

		if(mCameraRight) {
			mCameraRight->release();
			delete mCameraRight;
		}
		#ifdef HAS_ZED_SDK
			if(mZedCamera)
				delete mZedCamera;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::init(const cjson::Json &_json){
		if(!_json.isObject()){
			std::cout << "[STEREO CAMERA][CUSTOM] This kind of camera needs a configuration file" << std::endl;
			return false;
		}

		if(!configureDevice(_json["device"])){
			return false;
		}

		if(!decodeCloudType(_json["cloud"])){
			return false;
		}

		std::cout << "[STEREO CAMERA][CUSTOM] Configured and ready to be used" << std::endl;
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::rgb(cv::Mat &_left, cv::Mat &_right, bool _undistort){
		if(mHasRGB){
			_left = mLeftFrame;
			_right = mRightFrame;
			return true;
		}
		else{
			switch(mType){
			case eDeviceType::opencv:{
				//	 Retrieve data from OpenCV drivers
				bool res = mCameraLeft->retrieve(mLeftFrame);
				res &= mCameraRight->retrieve(mRightFrame);

				// Copy to out variables.
				_left = mLeftFrame;
				_right = mRightFrame;

				// Set flag to reuse data until next grab.
				mHasRGB = true;
				return res;
			}
			case eDeviceType::zed:{
				#ifdef HAS_ZED_SDK
					// Retrieve data from ZED SDK
					sl::zed::Mat left = mZedCamera->retrieveImage(sl::zed::SIDE::LEFT);
					sl::zed::Mat right = mZedCamera->retrieveImage(sl::zed::SIDE::RIGHT);

					if(left.width == 0 || right.width == 0)
						return false;

					mLeftFrame = cv::Mat(left.height, left.width, CV_8UC4);
					memcpy(mLeftFrame.data,left.data,left.width*left.height*4*sizeof(uchar));

					mRightFrame = cv::Mat(right.height, right.width, CV_8UC4);
					memcpy(mRightFrame.data,right.data,right.width*right.height*4*sizeof(uchar));

					// Copty to out variables
					_left = mLeftFrame;
					_right = mRightFrame;

					// Set flag to reuse data until next grab.
					mHasRGB = true;
					return true;
				#else
					cv::Mat doubleImage;
					bool res = mCameraLeft->retrieve(doubleImage);
					if (doubleImage.rows == 0)
						return false;

					mLeftFrame = doubleImage(Rect(0, 0, doubleImage.cols / 2, doubleImage.rows)).clone();
					mRightFrame = doubleImage(Rect(doubleImage.cols / 2, 0, doubleImage.cols / 2, doubleImage.rows)).clone();
					// Copy to out variables.
					_left = mLeftFrame.clone();
					_right = mRightFrame.clone();

					// Set flag to reuse data until next grab.
					mHasRGB = true;
					return res;
				#endif	// HAS_ZED_SDK
			}
			default:
				return false;
			}
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::depth(cv::Mat &_depth){
		if(mComputedDepth){	// Reuse data if it is previously computed.
			_depth = mDepth.clone();
			return true;
		}else{
			// Switch between methods
			switch (mDepthAlgorithm) {
			case eDepthAlgorithm::elas:
				#ifdef USE_LIBELAS
					if(!disparityLibelas(mDepth)){
						return false;
					}
				#else
					return false;
				#endif
				break;
			case eDepthAlgorithm::BM:
				if(!disparityCvBm(mDepth)){
					return false;
				}
				break;
			case eDepthAlgorithm::SGBM:
				if(!disparityCvSgbm(mDepth)){
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Not configured any depth algorithm, check configuration file" << std::endl;
				return false;
				break;
			}

			// Copy to out variables
			_depth = mDepth.clone();
			mComputedDepth = true;
			return true;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::grab(){
		// Reset variables in order not to reuse old data
		mHasRGB = false, mComputedDepth = false, mComputedCloudXYZ = false, mComputedCloudXYZRGB = false, mComputedCloudXYZRGBNormal = false, mComputedCloudNormal = false;

		// Switch between camera implementations.
		switch(mType){
		case eDeviceType::opencv:{
			bool res = mCameraLeft->grab();
			res &= mCameraRight->grab();
			return res;
		}
		case eDeviceType::zed:
			#ifdef HAS_ZED_SDK
				return mZedCamera->grab(sl::zed::SENSING_MODE::FULL, 0, 0);
			#else
				return mCameraLeft->grab();
			#endif	// HAS_ZED_SDK
		default:
				return false;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointXYZ> &_cloud){	//666 Clean this
		if(mComputedCloudXYZ){
			_cloud = *mCloudXYZ;
			return true;
		}else{
			mCloudXYZ->clear();
			switch (mCloudType) {
			case eCloudType::Sparse:
				if(!computeCloudSparse(mCloudXYZ)){
					return false;
				}
				break;
			case eCloudType::Dense:
				if(!computeCloudDense(mCloudXYZ)){
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Cloud type not defined" << std::endl;
				return false;
				break;
			}

			// Copy to out variable.
			_cloud = *mCloudXYZ;
			mComputedCloudXYZ = true;
			return true;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud){	//666 Clean this
		if(mComputedCloudXYZRGB){
			_cloud = *mCloudXYZRGB;
			return true;
		}else{
			mCloudXYZRGB->clear();
			switch (mCloudType) {
			case eCloudType::Sparse:
				if(!computeCloudSparse(mCloudXYZRGB)){
					return false;
				}
				break;
			case eCloudType::Dense:
				if(!computeCloudDense(mCloudXYZRGB)){
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Cloud type not defined" << std::endl;
				return false;
				break;
			}

			// Copy to out variable.
			_cloud = *mCloudXYZRGB;
			mComputedCloudXYZRGB = true;
			return true;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud){	//666 Clean this
		if(mComputedCloudXYZRGBNormal){
			_cloud = *mCloudXYZRGBNormal;
			return true;
		}else{
			mCloudXYZRGBNormal->clear();
			switch (mCloudType) {
			case eCloudType::Sparse:
				if(!computeCloudSparse(mCloudXYZRGBNormal)){
					return false;
				}
				break;
			case eCloudType::Dense:
				if(!computeCloudDense(mCloudXYZRGBNormal)){
					return false;
				}
				break;
			default:
				std::cout << "[STEREO CAMERAS][CUSTOM] Cloud type not defined" << std::endl;
				return false;
				break;
			}

			// Copy to out variable.
			_cloud = *mCloudXYZRGBNormal;
			mComputedCloudXYZRGBNormal = true;
			return true;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::cloud(pcl::PointCloud<pcl::PointNormal> &_cloud){
		std::cout << "[STEREO CAMERAS][CUSTOM] Cloud with normals is not currently implemented" << std::endl;
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	// Private methods
	bool StereoCameraCustom::configureDevice(const cjson::Json &_json){
		if(_json["type"] == "opencv"){
			// Getting source for left image
			if(_json["left"].isString()){
				mCameraLeft = new cv::VideoCapture(std::string(_json["left"]));
			}else if(_json["left"].isNumber()){
				mCameraLeft = new cv::VideoCapture((int) _json["left"]);
			}else{
				std::cout << "[STEREO CAMERA][CUSTOM] Can't recognize source for left images." << std::endl;
				return false;
			}
			if(!mCameraLeft){
				std::cout << "Error opening left camera." << std::endl;
				return false;
			}

			// Getting source for right image
			if(_json["right"].isString()){
				mCameraRight = new cv::VideoCapture(std::string(_json["right"]));
			}else if(_json["right"].isNumber()){
				mCameraRight = new cv::VideoCapture((int) _json["right"]);
			}else{
				std::cout << "[STEREO CAMERA][CUSTOM] Can't recognize source for right images." << std::endl;
				return false;
			}
			if(!mCameraRight){
				std::cout << "Error opening right camera." << std::endl;
				return false;
			}


			mType = eDeviceType::opencv;

			if(_json.contains("calibFile")){
				mIsCalibrated = loadCalibrationFile(_json["calibFile"]);

				cv::Size imageSize((int)_json["resolution"]["width"], (int)_json["resolution"]["height"]);

				initUndistortRectifyMap(mMatrixLeft, mCoefLeft, mRectificationLeft, mProjectionLeft, imageSize, CV_16SC2, mRectificationMap[0][0], mRectificationMap[0][1]);
				initUndistortRectifyMap(mMatrixRight, mCoefRight, mRectificationRight, mProjectionRight, imageSize, CV_16SC2, mRectificationMap[1][0], mRectificationMap[1][1]);

			}else{
				mIsCalibrated = false;
				return false;
			}
		}else if(_json["type"] == "zed"){
			#ifdef HAS_ZED_SDK
				// Opening zed camera, but only for getting pair of images.
				mZedCamera = new sl::zed::Camera(sl::zed::ZEDResolution_mode::VGA);
				auto errCode = mZedCamera->init(sl::zed::MODE::PERFORMANCE);
				mType = eDeviceType::zed;
				if(errCode == sl::zed::ERRCODE::SUCCESS){
					if(_json.contains("calibFile")){
						mIsCalibrated = loadCalibrationFile(_json["calibFile"]);
					}else{
						mIsCalibrated = false;
					}
				}else{
					std::cout << "[STEREO CAMERA][CUSTOM] Internal error configuring ZED camera" << std::endl;
					return false;
				}
			#else
				mCameraLeft = new VideoCapture(int(_json["indexZed"]));
				
				mCameraLeft->set(CV_CAP_PROP_FRAME_WIDTH, int(_json["resolution"]["width"]));
				mCameraLeft->set(CV_CAP_PROP_FRAME_HEIGHT, int(_json["resolution"]["height"]));
				

				if (mCameraLeft->get(CV_CAP_PROP_FRAME_WIDTH) != int(_json["resolution"]["width"])) {
					std::cout << "[STEREO CAMERA][CUSTOM] couldn't set camera resolution. Width set to " << mCameraLeft->get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
				}
				if (mCameraLeft->get(CV_CAP_PROP_FRAME_HEIGHT) != int(_json["resolution"]["height"])) {
					std::cout << "[STEREO CAMERA][CUSTOM] couldn't set camera resolution. height set to " << mCameraLeft->get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
				}
				
				mType = eDeviceType::zed;
				if (!mCameraLeft->isOpened())
					return false;

				if (_json.contains("calibFile")) {
					mIsCalibrated = loadCalibrationFile(_json["calibFile"]);
					cv::Size imageSize(((int)_json["resolution"]["width"])/2, (int)_json["resolution"]["height"]);
					
					initUndistortRectifyMap(mMatrixLeft, mCoefLeft, mRectificationLeft, mProjectionLeft, imageSize, CV_16SC2, mRectificationMap[0][0], mRectificationMap[0][1]);
					initUndistortRectifyMap(mMatrixRight, mCoefRight, mRectificationRight, mProjectionRight, imageSize, CV_16SC2, mRectificationMap[1][0], mRectificationMap[1][1]);
				}
				else {
					mIsCalibrated = false;
					return false;
				}
			#endif	// HAS_ZED_SDK
		}else{
			std::cout << "[STEREO CAMERA][CUSTOM] That device is not currently supported." << std::endl;
			return false;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::loadCalibrationFile(const std::string &_filePath){
		cv::FileStorage fs(_filePath, cv::FileStorage::READ);
		if(!fs.isOpened()){
			return false;
		}

		fs["MatrixLeft"] >> mMatrixLeft;					if (mMatrixLeft.rows == 0) return false;
		fs["DistCoeffsLeft"] >> mCoefLeft;					if (mCoefLeft.rows == 0) return false;
		fs["MatrixRight"] >> mMatrixRight;					if (mMatrixRight.rows == 0) return false;
		fs["DistCoeffsRight"] >> mCoefRight;				if (mCoefRight.rows == 0) return false;
		fs["Rotation"] >> mR;								if (mR.rows == 0) return false;
		fs["Translation"] >> mT;							if (mT.rows == 0) return false;
		fs["Essential"] >> mE;								if (mE.rows == 0) return false;
		fs["Fundamental"] >> mF;							if (mF.rows == 0) return false;
		fs["RectificationLeft"] >> mRectificationLeft;		if (mRectificationLeft.rows == 0) return false;
		fs["RectificationRight"] >> mRectificationRight;	if (mRectificationRight.rows == 0) return false;
		fs["ProjectionLeft"] >> mProjectionLeft;			if (mProjectionLeft.rows == 0) return false;
		fs["ProjectionLeft"] >> mProjectionRight;			if (mProjectionRight.rows == 0) return false;
		fs["DisparityToDepth"] >> mDisparityToDepth;		if (mDisparityToDepth.rows == 0) return false;


		// Make sure that all the matrixes has the proper format to avoid crashes.
		mMatrixLeft.convertTo(			mMatrixLeft, 		CV_64F);
		mMatrixRight.convertTo(			mMatrixRight, 		CV_64F);
		mCoefLeft.convertTo(			mCoefLeft, 			CV_64F);
		mCoefRight.convertTo(			mCoefRight, 		CV_64F);
		mRectificationLeft.convertTo(	mRectificationLeft, CV_64F);
		mRectificationRight.convertTo(	mRectificationRight,CV_64F);
		mProjectionLeft.convertTo(		mProjectionLeft, 	CV_64F);
		mProjectionRight.convertTo(		mProjectionRight, 	CV_64F);
		mDisparityToDepth.convertTo(	mDisparityToDepth, 	CV_64F);
		mR.convertTo(					mR,					CV_64F);
		mT.convertTo(					mT,					CV_64F);
		mE.convertTo(					mE,					CV_64F);
		mF.convertTo(					mF,					CV_64F);

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::decodeCloudType(const cjson::Json & _json) {
		if (!_json.contains("type"))
			return false;

		if (_json["type"] == "sparse") {
			mCloudType = eCloudType::Sparse;
			if (_json.contains("sparse")) {
				decodeDetector(_json["sparse"]["detector"]);
				decodeDescriptor(_json["sparse"]["descriptor"]);
				decodeMatcher(_json["sparse"]["matcher"]);
			}
			else {
				return false;
			}
		}
		else if(_json["type"] == "dense"){
			mCloudType = eCloudType::Dense;
			if (_json.contains("dense")) {
				decodeDisparityAlgorithm(_json["dense"]["disparity"]);
			}
			else {
				return false;
			}

		}
		else if(_json["type"] == "null"){
			std::cout << "[STEREO CAMERA][CUSTOM] Camera configured without any algorithm for cloud generation" << std::endl;
			return true;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Not recognize type of cloud in configuration file" << std::endl;
			return false;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeDetector(const cjson::Json &_json) {
		if (_json["type"] == "ShiTomasi") {
			mFeatureDetector = eFeatureDetector::ShiTomasi;
			mDetectorParams = _json["params"];
		}
		else if (_json["type"] == "SIFT") {
			mFeatureDetector = eFeatureDetector::SIFT;
		}
		else if (_json["type"] == "SURF") {
			mFeatureDetector = eFeatureDetector::SURF;
		}
		else if (_json["type"] == "FAST") {
			std::cout << "[STEREO CAMERAS][CUSTOM] FAST Feature detector is not currently implemented." << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
		else if (_json["type"] == "ORB") {
			mFeatureDetector = eFeatureDetector::ORB;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined feature detector" << std::endl;
			mFeatureDetector = eFeatureDetector::Null;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeDescriptor(const cjson::Json &_json) {
		if (_json["type"] == "SIFT") {
			mFeatureDescriptor = eFeatureDescriptor::SIFT;
		}
		else if (_json["type"] == "SURF") {
			mFeatureDescriptor = eFeatureDescriptor::SURF;
		}
		else if (_json["type"] == "ORB") {
			mFeatureDescriptor = eFeatureDescriptor::ORB;
		}
		else if (_json["type"] == "BRISK") {
			std::cout << "[STEREO CAMERAS][CUSTOM] BRISK feature descriptor is not implemented yet." << std::endl;
			mFeatureDescriptor = eFeatureDescriptor::Null;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined feature detector" << std::endl;
			mFeatureDescriptor = eFeatureDescriptor::Null;
		}
	
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeMatcher(const cjson::Json &_json) {
		if (_json["type"] == "template") {
			mMatchingAlgorithm = eMatchingAlgorithm::TemplateMatching;
			mDisparityParams = _json["params"];
			cjson::Json roiLeft = _json["ROI"]["left"];
			cjson::Json roiRight = _json["ROI"]["right"];
			mRoiLeft = Rect((int) roiLeft["x"], (int)roiLeft["y"], (int)roiLeft["width"], (int)roiLeft["height"]);
			mRoiRight = Rect((int)roiRight["x"], (int)roiRight["y"], (int)roiRight["width"], (int)roiRight["height"]);
		}
		else if (_json["type"] == "brute") {
			std::cout << "[STEREO CAMERAS][CUSTOM] Force brute matcher not implemented" << std::endl;
			mMatchingAlgorithm = eMatchingAlgorithm::Null;
		}
		else if (_json["type"] == "flann") {
			mMatchingAlgorithm = eMatchingAlgorithm::Flann; 
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined feature matcher" << std::endl;
			mMatchingAlgorithm = eMatchingAlgorithm::Null;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::decodeDisparityAlgorithm(const cjson::Json &_json) {
		if (_json["algorithm"] == "BM") {
			mDepthAlgorithm = eDepthAlgorithm::BM;
			mDisparityParams = _json["params"];
		}
		else if (_json["algorithm"] == "SGBM") {
			mDepthAlgorithm = eDepthAlgorithm::SGBM;
			mDisparityParams = _json["params"];
		}
		else if (_json["algorithm"] == "elas") {
			mDepthAlgorithm = eDepthAlgorithm::elas;
		}
		else {
			std::cout << "[STEREO CAMERAS][CUSTOM] Undefined depth algorithm" << std::endl;
			mDepthAlgorithm = eDepthAlgorithm::Null;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::disparityLibelas(cv::Mat & _depth){
		#ifdef USE_LIBELAS
			cv::Mat left, right;
			if (!((StereoCameraCustom*)this)->rgb(left, right))
				return false;

			cv::cvtColor(left, left, CV_BGR2GRAY);
			cv::cvtColor(right, right, CV_BGR2GRAY);
			remap(left, left, mRectificationMap[0][0], mRectificationMap[0][1], cv::INTER_LINEAR);
			remap(right, right, mRectificationMap[1][0], mRectificationMap[1][1], cv::INTER_LINEAR);

			int width = left.cols;
			int height = left.rows;

			const int32_t dims[3] = { width,height,width }; // bytes per line = width
			float* D1_data = (float*)malloc(width*height*sizeof(float));
			float* D2_data = (float*)malloc(width*height*sizeof(float));


			// 666 TODO, choose between long and short range parameters.
			elas::Elas::parameters params;
			params = elas::Elas::parameters(elas::Elas::ROBOTICS);
			/*params.disp_min              = 100;
			params.disp_max              = 450;
			params.support_threshold     = 0.95;
			params.support_texture       = 10;
			params.candidate_stepsize    = 5;
			params.incon_window_size     = 5;
			params.incon_threshold       = 5;
			params.incon_min_support     = 5;
			params.add_corners           = 0;
			params.grid_size             = 20;
			params.beta                  = 0.02;
			params.gamma                 = 3;
			params.sigma                 = 1;
			params.sradius               = 2;
			params.match_texture         = 1;
			params.lr_threshold          = 2;
			params.speckle_sim_threshold = 1;
			params.speckle_size          = 200;
			params.ipol_gap_width        = 300;
			params.filter_median         = 0;
			params.filter_adaptive_mean  = 1;
			params.postprocess_only_left = 1;
			params.subsampling           = 0;*/

			elas::Elas elas(params);

			elas.process(left.data, right.data, D1_data, D2_data, dims);

			_depth = cv::Mat(height, width, CV_32FC1, D1_data);
			_depth.convertTo(_depth, CV_64F);
			return true;
		#else
			return false;
		#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::disparityCvBm(cv::Mat &_depth){
		/*cv::StereoBM matcher(StereoBM::BASIC_PRESET, mDisparityParams["numDisp"], mDisparityParams["sadWindow"]);

		cv::Mat left, right;
		if (!((StereoCameraCustom*)this)->rgb(left, right))
			return false;

		cv::cvtColor(left, left, CV_BGR2GRAY);
		cv::cvtColor(right, right, CV_BGR2GRAY);
		remap(left, left, mRectificationMap[0][0], mRectificationMap[0][1], cv::INTER_LINEAR);
		remap(right, right, mRectificationMap[1][0], mRectificationMap[1][1], cv::INTER_LINEAR);
		matcher(left, right, _depth, CV_32F);

		return true;*/
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::disparityCvSgbm(cv::Mat &_depth){
		/*cv::StereoSGBM matcher;

		matcher.minDisparity 		= mDisparityParams["minDisp"];
		matcher.numberOfDisparities = mDisparityParams["numDisp"];
		matcher.SADWindowSize 		= mDisparityParams["sadWindow"];
		matcher.P1 					= mDisparityParams["P1"];
		matcher.P2					= mDisparityParams["P2"];
		matcher.disp12MaxDiff	 	= mDisparityParams["disp12MaxDiff"];
		matcher.preFilterCap		= mDisparityParams["preFilterCap"];
		matcher.uniquenessRatio 	= mDisparityParams["uniquenessRatio"];
		matcher.speckleWindowSize 	= mDisparityParams["speckleWindowSize"];
		matcher.speckleRange 		= mDisparityParams["speckleRange"];
		matcher.fullDP				= (bool) mDisparityParams["fullDP"];

		cv::Mat left, right;
		if (!((StereoCameraCustom*)this)->rgb(left, right))
			return false;

		cv::cvtColor(left, left, CV_BGR2GRAY);
		cv::cvtColor(right, right, CV_BGR2GRAY);
		remap(left, left, mRectificationMap[0][0], mRectificationMap[0][1], cv::INTER_LINEAR);
		remap(right, right, mRectificationMap[1][0], mRectificationMap[1][1], cv::INTER_LINEAR);
		matcher(left, right, _depth);
		_depth.convertTo(_depth, CV_64F, 1.0/16.0);
		return true;*/
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudDense(pcl::PointCloud<pcl::PointXYZ>::Ptr & _cloud) {
		cv::Mat depth;
		((StereoCameraCustom*)this)->depth(depth);

		cv::Mat_<double> vecTmp(4,1);
		for (int i = 0; i < depth.rows; i++) {
			for (int j = 0; j < depth.cols; j++) {
				vecTmp(2)=depth.at<double>(i,j);
				if(vecTmp(2) == 0.0)
					continue;

				vecTmp(0)=j; vecTmp(1)=i; vecTmp(3)=1;

				vecTmp = mDisparityToDepth*vecTmp;
				vecTmp /= vecTmp(3);
				_cloud->push_back(PointXYZ( vecTmp(0),  vecTmp(1),  vecTmp(2)));

			}
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _cloud) {
		cv::Mat depth, left, right;
		if (!((StereoCameraCustom*)this)->rgb(left, right)) {
			return false;
		}
		if (!((StereoCameraCustom*)this)->depth(depth)) {
			return false;
		}
		
		cv::Mat_<double> vecTmp(4,1);
		for (int i = 0; i < depth.rows; i++) {
			for (int j = 0; j < depth.cols; j++) {
				vecTmp(2)=depth.at<double>(i,j);
				if(vecTmp(2) < 0.0)
					continue;

				vecTmp(0)=j; vecTmp(1)=i; vecTmp(3)=1;
				//std::cout << vecTmp(0) << ", " << vecTmp(1) << ", " << vecTmp(2) << std::endl;
				vecTmp = mDisparityToDepth*vecTmp;
				vecTmp /= vecTmp(3);
				pcl::PointXYZRGB point;
				point.x = vecTmp(0);
				point.y = vecTmp(1);
				point.z = vecTmp(2);

				 if(left.channels() == 3){
					point.r = left.at<cv::Vec3b>(i, j)[2];
					point.g = left.at<cv::Vec3b>(i, j)[1];
					point.b = left.at<cv::Vec3b>(i, j)[0];
				}else if(left.channels() == 4){
					point.r = left.at<cv::Vec4b>(i, j)[2];
					point.g = left.at<cv::Vec4b>(i, j)[1];
					point.b = left.at<cv::Vec4b>(i, j)[0];
					point.a = 255;
				}else{
					std::cout << "Camera do not provide colors!" << std::endl;
				}

				_cloud->push_back(point);
			}
		}
		
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudDense(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _cloud) {
		std::cout << "[STEREO CAMERA][CUSTOM] Dense XYZRGBNormal cloud not currently implemented" << std::endl;
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudSparse(pcl::PointCloud<pcl::PointXYZ>::Ptr& _cloud) {
		Mat left, right;
		((StereoCameraCustom*)this)->rgb(left, right);
		if (left.rows == 0 || right.rows == 0)
			return false;

		std::vector<cv::Point2i> points1, points2;

		if (!computePairFeatures(left, right, points1, points2)) {
			return false;
		}
		

		//std::cout << "--> STEREO: Features matched: " << points1.size() << std::endl;
		// Triangulate points using features in both images.
		vector<Point3f> points3d = triangulate(points1, points2);
		// Filter points using reprojection.

		for (auto point : points3d) {
			if (point.z > double(mDisparityParams["cloudRange"]["z"](0)) && point.z < double(mDisparityParams["cloudRange"]["z"](1))) {
				PointXYZ pclPoint(point.x, point.y, point.z);
				_cloud->push_back(pclPoint);
			}
		}
		//std::cout << "--> STEREO: Points in the selected range: " << _cloud->size() << std::endl;

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _cloud) {
		Mat left, right;
		((StereoCameraCustom*)this)->rgb(left, right);
		if (left.rows == 0 || right.rows == 0)
			return false;

		std::vector<cv::Point2i> points1, points2;

		if (!computePairFeatures(left, right, points1, points2)) {
			return false;
		}

		//std::cout << "--> STEREO: Features matched: " << points1.size() << std::endl;
		// Triangulate points using features in both images.
		vector<Point3f> points3d = triangulate(points1, points2);
		// Filter points using reprojection.

		for (unsigned i = 0; i < points3d.size(); i++) {
			auto point = points3d[i];
			PointXYZRGB pclPoint;
			pclPoint.x = point.x;
			pclPoint.y = point.y;
			pclPoint.z = point.z;
			//if (point.z > double(mDisparityParams["cloudRange"]["z"](0)) && point.z < double(mDisparityParams["cloudRange"]["z"](1))) {
				if(left.channels() == 3){
					pclPoint.r = left.at<cv::Vec3b>(points1[i].y, points1[i].x)[2];
					pclPoint.g = left.at<cv::Vec3b>(points1[i].y, points1[i].x)[1];
					pclPoint.b = left.at<cv::Vec3b>(points1[i].y, points1[i].x)[0];
				}else if(left.channels() == 4){
					pclPoint.r = left.at<cv::Vec4b>(points1[i].y, points1[i].x)[2];
					pclPoint.g = left.at<cv::Vec4b>(points1[i].y, points1[i].x)[1];
					pclPoint.b = left.at<cv::Vec4b>(points1[i].y, points1[i].x)[0];
				}else{
					std::cout << "Camera do not provide colors!" << std::endl;
				}

				_cloud->push_back(pclPoint);
			//}
		}
		//std::cout << "--> STEREO: Points in the selected range: " << _cloud->size() << std::endl;

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeCloudSparse(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& _cloud) {
		std::cout << "[STEREO CAMERA][CUSTOM] Sparse XYZRGBNormal cloud not currently implemented" << std::endl;
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computePairFeatures(const cv::Mat & _left, const cv::Mat & _right, std::vector<cv::Point2i>& _features1, std::vector<cv::Point2i>& _features2) {
		if (mMatchingAlgorithm == eMatchingAlgorithm::TemplateMatching) {
			if (!computeMatchesTemplate(_left, _right, _features1, _features2))
				return false;
		}
		else if (mMatchingAlgorithm == eMatchingAlgorithm::Flann) {
			if(!computeMatchesFlann(_left, _right, _features1, _features2))
				return false;
		}
		else {
			return false;
		}
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeFeatures(const Mat &_frame, vector<Point2i> &_features){
		switch (mFeatureDetector) {
		case eFeatureDetector::FAST:
		{
			break;
		}
		case eFeatureDetector::ORB:
		{
			auto detector = cv::ORB::create();
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(_frame, keypoints);
			if (keypoints.size() == 0)
				return false;
			for (auto kp : keypoints) {
				_features.push_back(kp.pt);
			}
			break;
		}
		case eFeatureDetector::SIFT:
		{
			auto detector = cv::xfeatures2d::SIFT::create();
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(_frame, keypoints);
			if (keypoints.size() == 0)
				return false;
			for (auto kp : keypoints) {
				_features.push_back(kp.pt);
			}
			break;
		}
		case eFeatureDetector::SURF:
		{
			auto  detector = cv::xfeatures2d::SURF::create();
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(_frame, keypoints);
			if (keypoints.size() == 0)
				return false;
			for (auto kp : keypoints) {
				_features.push_back(kp.pt);
			}
			break;
		}
		case eFeatureDetector::ShiTomasi:
		{
			goodFeaturesToTrack(_frame, _features, mDetectorParams["nFeatures"], mDetectorParams["qualityLevel"], mDetectorParams["minDist"]);
			break;
		}
		default:
		{
			std::cout << "[STEREO CAMERA][CUSTOM] Not feature detector configured" << std::endl;
			return false;
			break;
		}
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeFeatures(const cv::Mat & _frame, std::vector<cv::KeyPoint>& _features, cv::Mat& _descriptors) {
		switch (mFeatureDescriptor) {

		case eFeatureDescriptor::ORB:
		{
			auto  detector = cv::ORB::create();
			detector->detectAndCompute(_frame, Mat(), _features, _descriptors);
			break;
			break;
		}
		case eFeatureDescriptor::SIFT:
		{
			auto  detector = cv::xfeatures2d::SIFT::create();
			detector->detectAndCompute(_frame, Mat(), _features, _descriptors);
			break;
		}
		case eFeatureDescriptor::SURF:
		{
			auto  detector = cv::xfeatures2d::SURF::create();
			detector->detectAndCompute(_frame, Mat(), _features, _descriptors);
			break;
		}
		default:
		{
			std::cout << "[STEREO CAMERA][CUSTOM] Not feature detector configured" << std::endl;
			return false;
			break;
		}
		}

		if (_features.size() == 0) {
			return false;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void StereoCameraCustom::computeEpipoarLines(const vector<Point2i> &_points, vector<Vec3f> &_epilines){
		vector<Point2f> points;
		for(auto point: _points){
			points.push_back(Point2f(point.x, point.y));
		}

		computeCorrespondEpilines(points, 1, mF, _epilines);
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraCustom::computeMatchesTemplate(const cv::Mat &_left, const cv::Mat &_right, std::vector<cv::Point2i>& _points1, std::vector<cv::Point2i>& _points2) {
		cv::Mat leftGray, rightGray, leftGrayUndistort, rightGrayUndistort;
		cv::cvtColor(_left, leftGray, CV_BGR2GRAY);
		cv::cvtColor(_right, rightGray, CV_BGR2GRAY);

		cv::undistort(leftGray, leftGrayUndistort, mMatrixLeft, mCoefLeft);
		cv::undistort(rightGray, rightGrayUndistort, mMatrixRight, mCoefRight);

		// Compute keypoint only in first image
		vector<Point2i> keypoints;
		if (!computeFeatures(leftGrayUndistort, keypoints))
			return false;

		// Compute projection of epipolar lines into second image.
		std::vector<cv::Vec3f> epilines;
		computeEpipoarLines(keypoints, epilines);

		// For each epipolar line calculate equivalent feature by template matching.
		const int squareSize = mDisparityParams["size"];
		Rect secureRegion(squareSize / 2 + 1,
			squareSize / 2 + 1,
			rightGrayUndistort.cols - 2 * (squareSize / 2 + 1),
			rightGrayUndistort.rows - 2 * (squareSize / 2 + 1));
		Rect validLeft(0, 0, mLeftFrame.cols, mLeftFrame.rows);
		validLeft &= secureRegion & mRoiLeft;
		Rect validRight(0, 0, mLeftFrame.cols, mLeftFrame.rows);
		validRight &= secureRegion & mRoiRight;	// 666 TODO: input arg?


		const unsigned cNumProcs = 8;
		vector<vector<Point2i>> vpoints1(cNumProcs), vpoints2(cNumProcs);

		std::pair<int, int> disparityRange(mDisparityParams["disparityRange"](0), mDisparityParams["disparityRange"](1));
		double maxTemplateScore = mDisparityParams["maxScore"];
		// Match features using ParallelFeatureMatcher Class
		parallel_for_(Range(0, cNumProcs), ParallelFeatureMatcher(leftGrayUndistort, rightGrayUndistort, keypoints, epilines, disparityRange, squareSize, maxTemplateScore, vpoints1, vpoints2, validLeft, validRight));

		for (vector<Point2i> v : vpoints1) {
			_points1.insert(_points1.end(), v.begin(), v.end());
		}

		for (vector<Point2i> v : vpoints2) {
			_points2.insert(_points2.end(), v.begin(), v.end());
		}

		if (_points1.size() == 0 || _points2.size() == 0)
			return false;

		return true;
	}

	bool StereoCameraCustom::computeMatchesFlann(const cv::Mat & _left, const cv::Mat & _right, std::vector<cv::Point2i>& _points1, std::vector<cv::Point2i>& _points2) {
		cv::Mat leftGray, rightGray, leftGrayUndistort, rightGrayUndistort;
		cv::cvtColor(_left, leftGray, CV_BGR2GRAY);
		cv::cvtColor(_right, rightGray, CV_BGR2GRAY);

		cv::undistort(leftGray, leftGrayUndistort, mMatrixLeft, mCoefLeft);
		cv::undistort(rightGray, rightGrayUndistort, mMatrixRight, mCoefRight);

		// Compute keypoint only in first image
		cv::Mat descriptorsLeft, descriptorsRight;
		std::vector<KeyPoint> kpLeft, kpRight;
		if (!computeFeatures(leftGrayUndistort, kpLeft, descriptorsLeft))
			return false;

		if (!computeFeatures(rightGrayUndistort, kpRight, descriptorsRight))
			return false;

		//-- Step 3: Matching descriptor vectors using FLANN matcher
		FlannBasedMatcher matcher;
		std::vector< DMatch > matches;
		descriptorsLeft.convertTo(descriptorsLeft, CV_32F);
		descriptorsRight.convertTo(descriptorsRight, CV_32F);
		matcher.match(descriptorsLeft, descriptorsRight, matches);

		std::vector< DMatch > goodMatches;
		std::vector<cv::Point2i> pLeft, pRight;
		for (int i = 0; i < descriptorsLeft.rows; i++){
			pLeft.push_back(kpLeft[matches[i].queryIdx].pt);
			pRight.push_back(kpRight[matches[i].trainIdx].pt);
		}

		std::vector<int> mask;
		Mat H = findHomography(pLeft, pRight, mask, CV_RANSAC);
		Mat outImage;
		hconcat(leftGrayUndistort, rightGrayUndistort, outImage);
		
		
		int nInliers = std::accumulate(mask.begin(), mask.end(), 0);

		_points1.resize(nInliers);
		_points2.resize(nInliers);
		unsigned inlier = 0;
		for (unsigned i = 0; i < pLeft.size(); i++) {
			if (mask[i] == 1) {
				_points1[inlier] = pLeft[i];
				_points2[inlier] = pRight[i];
				inlier++;
			}
		}

		if (_points1.size() == 0 || _points2.size() == 0)
			return false;

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	vector<Point3f> StereoCameraCustom::triangulate(const vector<Point2i> &_points1, const vector<Point2i> &_points2) {
		Mat pnts3D	(4,_points1.size(),CV_64F);
		Mat cam1pnts(2,_points1.size(),CV_64F);
		Mat cam2pnts(2,_points1.size(),CV_64F);

		for (unsigned i = 0; i < _points1.size(); i++) {
			cam1pnts.at<double>(0,i) = _points1[i].x;
			cam1pnts.at<double>(1,i) = _points1[i].y;
			cam2pnts.at<double>(0,i) = _points2[i].x;
			cam2pnts.at<double>(1,i) = _points2[i].y;
		}

		Mat R1,R2,P1,P2, Q;
		stereoRectify(mMatrixLeft, mCoefLeft, mMatrixRight, mCoefRight, mLeftFrame.size(), mR, mT, R1, R2, P1, P2, Q);

		triangulatePoints(P1,P2, cam1pnts, cam2pnts, pnts3D);

		vector<Point3f> points3d;
		for (int i = 0 ; i < pnts3D.cols ; i++) {
			float w = (float) pnts3D.at<double>(3,i);
			float x = (float) pnts3D.at<double>(0,i)/w;
			float y = (float) pnts3D.at<double>(1,i)/w;
			float z = (float) pnts3D.at<double>(2,i)/w;
			points3d.push_back(Point3f(x,y,z));
		}

		return points3d;
	}
}	//	namespace rgbd
