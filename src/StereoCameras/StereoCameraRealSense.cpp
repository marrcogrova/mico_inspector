////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <rgbd_tools/StereoCameras/StereoCameraRealSense.h>

#include <pcl/features/integral_image_normal.h>

#include <cstdio>

namespace rgbd {
	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::init(const cjson::Json & _json){
		#ifdef ENABLE_LIBREALSENSE
			mConfig = _json;

			// Initialize context and get device det device
			#if (RS2_API_MAJOR_VERSION  == 1)
				mRsContext = new rs::context();
				if (mRsContext->get_device_count() == 0) {
					std::cout << "[STEREOCAMERA][REALSENSE] There's no any compatible device connected." << std::endl;
					return false;
				}
				mRsDevice = mRsContext->get_device(mDeviceId);
				std::cout << "[STEREOCAMERA][REALSENSE] Using device 0, an "<< mRsDevice->get_name() << std::endl;
				std::cout << "[STEREOCAMERA][REALSENSE]     Serial number: " << mRsDevice->get_serial() << std::endl;
				std::cout << "[STEREOCAMERA][REALSENSE]     Firmware version: " << mRsDevice->get_firmware_version() << std::endl;

			#elif (RS2_API_MAJOR_VERSION == 2)
				auto list = mRsContext.query_devices();
				if (list.size() == 0) {
					std::cout << "[STEREOCAMERA][REALSENSE] There's no any compatible device connected." << std::endl;
					return false;
				}
				mRsDevice = list[mDeviceId];
			
			#endif			

			if (mConfig.contains("cloudDownsampleStep")) {
				mDownsampleStep = mConfig["cloudDownsampleStep"];
			}

			// Initialize streams of data.
			#if (RS2_API_MAJOR_VERSION  == 1)
				mRsDevice->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
				mRsDevice->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
				mRsDevice->start();

			#elif (RS2_API_MAJOR_VERSION == 2)
				mRsPipe.start();

			#endif
			
			// Get intrinsics and extrinsics
			#if (RS2_API_MAJOR_VERSION  == 1)
				mRsDepthIntrinsic 	= *mRsDevice->get_stream_intrinsics(rs::stream::depth);
				mRsDepthToColor 	= *mRsDevice->get_extrinsics(rs::stream::depth, rs::stream::color);
				mRsColorToDepth 	= *mRsDevice->get_extrinsics(rs::stream::color, rs::stream::depth);
				mRsColorIntrinsic 	= *mRsDevice->get_stream_intrinsics(rs::stream::color);
				mRsDepthScale		=  mRsDevice->get_depth_scale();

			#elif (RS2_API_MAJOR_VERSION == 2)


			#endif

            // Projection matrix Depth
            mCvDepthIntrinsic = cv::Mat::eye(3,3,CV_32F);
            mCvDepthIntrinsic.at<float>(0,0) = mRsDepthIntrinsic.fx;
            mCvDepthIntrinsic.at<float>(1,1) = mRsDepthIntrinsic.fy;
            mCvDepthIntrinsic.at<float>(0,2) = mRsDepthIntrinsic.ppx;
            mCvDepthIntrinsic.at<float>(1,2) = mRsDepthIntrinsic.ppy;

            // Projection matrix Color
            mCvColorIntrinsic= cv::Mat::eye(3,3,CV_32F);
            mCvColorIntrinsic.at<float>(0,0) = mRsColorIntrinsic.fx;
            mCvColorIntrinsic.at<float>(1,1) = mRsColorIntrinsic.fy;
            mCvColorIntrinsic.at<float>(0,2) = mRsColorIntrinsic.ppx;
            mCvColorIntrinsic.at<float>(1,2) = mRsColorIntrinsic.ppy;

            mExtrinsicColorToDepth = cv::Mat::eye(4,4,CV_32F);
            cv::Mat(3,3,CV_32F, &mRsColorToDepth->rotation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(0,0,3,3)));
            mExtrinsicColorToDepth(cv::Rect(0,0,3,3)) = mExtrinsicColorToDepth(cv::Rect(0,0,3,3)).t(); // RS use color major instead of row mayor.
            cv::Mat(3,1,CV_32F, &mRsColorToDepth->translation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(3,0,1,3)));

			mUseUncolorizedPoints = (bool) mConfig["useUncolorizedPoints"];

			// Other params
			#if (RS2_API_MAJOR_VERSION  == 1)
				if(mConfig.contains("others")){
					if(mConfig["others"].contains("r200_lr_autoexposure")){
						mRsDevice->set_option(rs::option::r200_lr_auto_exposure_enabled, mConfig["others"]["r200_lr_autoexposure"]?1.0f:0.0f);
					}
					if(mConfig["others"].contains("r200_emitter_enabled")){
						mRsDevice->set_option(rs::option::r200_emitter_enabled, mConfig["others"]["r200_emitter_enabled"]?1.0f:0.0f);
					}
					if(mConfig["others"].contains("r200_lr_exposure")){
						mRsDevice->set_option(rs::option::r200_lr_exposure, (int) mConfig["others"]["r200_lr_exposure"]);
					}
					if(mConfig["others"].contains("r200_lr_gain")){
						mRsDevice->set_option(rs::option::r200_lr_gain, (int) mConfig["others"]["r200_lr_gain"]);
					} 
				}

			#elif (RS2_API_MAJOR_VERSION == 2)
				if(mConfig.contains("others")){
					std::cout << "[STEREOCAMERAS][REALSENSE]Custom parameters are not yet coded" << std::endl;
				}
			#endif

            

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::rgb(cv::Mat & _left, cv::Mat & _right){
		#ifdef ENABLE_LIBREALSENSE
            mLastRGB.copyTo(_left);
			return mHasRGB;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::depth(cv::Mat & _depth){
		#ifdef ENABLE_LIBREALSENSE
            mLastDepthInColor.copyTo(_depth);
			return mComputedDepth;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::grab(){
		#ifdef ENABLE_LIBREALSENSE
			#if (RS2_API_MAJOR_VERSION  == 1)
				mRsDevice->wait_for_frames();

				cv::cvtColor(cv::Mat(mRsColorIntrinsic->height, mRsColorIntrinsic->width, CV_8UC3, (uchar*)mRsDevice->get_frame_data(rs::stream::color)), mLastRGB, CV_RGB2BGR);
				mHasRGB = true;

				mLastDepthInColor = cv::Mat(mRsDepthIntrinsic->height, mRsDepthIntrinsic->width, CV_16U, (uchar*) mRsDevice->get_frame_data(rs::stream::depth_aligned_to_color));
				mComputedDepth = true;

			#elif (RS2_API_MAJOR_VERSION == 2)
				rs2::frameset frames = pipe.wait_for_frames();
				rs2::frame frameDepth = frames.first(RS2_STREAM_DEPTH);
				rs2::frame frameRGB = frames.first(RS2_STREAM_RGB);

				cv::cvtColor(cv::Mat(mRsColorIntrinsic->height, mRsColorIntrinsic->width, CV_8UC3, (uchar*)frameRGB.data, mLastRGB, CV_RGB2BGR);
				mHasRGB = true;

				mLastDepthInColor = cv::Mat(mRsDepthIntrinsic->height, mRsDepthIntrinsic->width, CV_16U, (uchar*) frameDepth.data));
				mComputedDepth = true;

			#endif

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
		#ifdef ENABLE_LIBREALSENSE
		for (int dy = 0; dy < mLastRGB.rows; dy = dy + mDownsampleStep) {
			for (int dx = 0; dx < mLastRGB.cols; dx = dx + mDownsampleStep) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;

                    // Skip over pixels with a depth value of zero, which is used to indicate no data
                    if (depth_value == 0) {
                        if (mUseUncolorizedPoints) {
                            _cloud.push_back(pcl::PointXYZ(NAN, NAN, NAN));
                        }
                        //else
                            continue;
                    }
                    else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        rs::float2 depth_pixel = { (float)dx, (float)dy };
                        rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);

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
            for (int dy = 0; dy < mLastRGB.rows; dy = dy + mDownsampleStep) {
                for (int dx = 0; dx < mLastRGB.cols; dx = dx + mDownsampleStep) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
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
                        rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);
                        point.x = depth_point.x;
                        point.y = depth_point.y;
                        point.z = depth_point.z;
                        auto rgb = mLastRGB.at<cv::Vec3b>(dy, dx);
                        point.r = rgb[2];
                        point.g = rgb[1];
                        point.b = rgb[0];

                    }

					_cloud.push_back(point);
                }
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

            if(cloudWoNormals.size() == 0){
                std::cout << "[STEREOCAMERA][REALSENSE] Empty cloud, can't compute normals" << std::endl;
                _cloud.resize(0);
                return true;
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
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
		#ifdef ENABLE_LIBREALSENSE
			mCvColorIntrinsic.copyTo(_intrinsic);
			_coefficients = cv::Mat(1,5, CV_32F, mRsColorIntrinsic->coeffs);
			return true;
		#else
			return false;
		#endif
	}

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
		#ifdef ENABLE_LIBREALSENSE
			mCvDepthIntrinsic.copyTo(_intrinsic);
			_coefficients = cv::Mat(1,5, CV_32F, mRsColorIntrinsic->coeffs);
			return true;
		#else
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
		#ifdef ENABLE_LIBREALSENSE
			cv::Mat(3,3,CV_32F, &mRsDepthToColor->rotation[0]).copyTo(_rotation);
			cv::Mat(3,1,CV_32F, &mRsDepthToColor->translation[0]).copyTo(_translation);
			return true;
		#else
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
		#ifdef ENABLE_LIBREALSENSE
			_rotation = Eigen::Matrix3f(&mRsDepthToColor->rotation[0]);
			_translation = Eigen::Vector3f(&mRsDepthToColor->translation[0]);
			return true;
		#else
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mRsDepthScale;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){
		#ifdef ENABLE_LIBREALSENSE
			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = mLastDepthInColor.at<uint16_t>(_pixel.y, _pixel.x);
			float depth_in_meters = depth_value * mRsDepthScale;
			// Set invalid pixels with a depth value of zero, which is used to indicate no data
			pcl::PointXYZRGB point;
			if (depth_value == 0) {
				return false;
			}
			else {
				// Map from pixel coordinates in the depth image to pixel coordinates in the color image
				rs::float2 depth_pixel = { _pixel.x, _pixel.y };
				rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);

				_point.x = depth_point.x;
				_point.y = depth_point.y;
				_point.z = depth_point.z;
				return true;
			}
		#else
			return false;
		#endif 
    }

	//----------------------------------------------------------------------------------------------------------------- 
    bool StereoCameraRealSense::laserPower(double power_level){ 
        #ifdef ENABLE_LIBREALSENSE 
            mRsDevice->set_option(rs::option::f200_laser_power, power_level); 
            return true; 
        #else 
            return false; 
        #endif 
    }

    //---------------------------------------------------------------------------------------------------------------------
    cv::Point StereoCameraRealSense::distortPixel(const cv::Point &_point, const rs::intrinsics * const _intrinsics) const {
		#ifdef ENABLE_LIBREALSENSE
			float x = (_point.x - _intrinsics->ppx) / _intrinsics->fx;
			float y = (_point.y - _intrinsics->ppy) / _intrinsics->fy;

			float r2  = x*x + y*y;
			float f = 1 + _intrinsics->coeffs[0]*r2 + _intrinsics->coeffs[1]*r2*r2 + _intrinsics->coeffs[4]*r2*r2*r2;
			x *= f;
			y *= f;
			float dx = x + 2*_intrinsics->coeffs[2]*x*y + _intrinsics->coeffs[3]*(r2 + 2*x*x);
			float dy = y + 2*_intrinsics->coeffs[3]*x*y + _intrinsics->coeffs[2]*(r2 + 2*y*y);
			x = dx;
			y = dy;

			cv::Point distortedPixel;
			distortedPixel.x = x * _intrinsics->fx + _intrinsics->ppx;
			distortedPixel.y = y * _intrinsics->fy + _intrinsics->ppy;

			return distortedPixel;
		#else
			return cv::Point();
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    cv::Point StereoCameraRealSense::undistortPixel(const cv::Point &_point,  const rs::intrinsics * const _intrinsics) const {
		#ifdef ENABLE_LIBREALSENSE
			float x = (_point.x - _intrinsics->ppx) / _intrinsics->fx;
			float y = (_point.y - _intrinsics->ppy) / _intrinsics->fy;

			float r2  = x*x + y*y;
			float f = 1 + _intrinsics->coeffs[0]*r2 + _intrinsics->coeffs[1]*r2*r2 + _intrinsics->coeffs[4]*r2*r2*r2;
			float ux = x*f + 2*_intrinsics->coeffs[2]*x*y + _intrinsics->coeffs[3]*(r2 + 2*x*x);
			float uy = y*f + 2*_intrinsics->coeffs[3]*x*y + _intrinsics->coeffs[2]*(r2 + 2*y*y);

			cv::Point undistortedPixel;
			undistortedPixel.x = ux* _intrinsics->fx + _intrinsics->ppx;;
			undistortedPixel.y = uy* _intrinsics->fy + _intrinsics->ppy;;

			return undistortedPixel;
		#else
			return cv::Point();
		#endif
    }

}	//	namespace rgbd
