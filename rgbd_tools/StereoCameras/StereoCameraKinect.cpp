////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "StereoCameraKinect.h"

#include <pcl/features/integral_image_normal.h>

#include <cstdio>

namespace rgbd {
//-----------------------------------------------------------------------------------------------------------------
StereoCameraKinect::~StereoCameraKinect() {
    freenect_stop_depth(mFreenectDevice);
    freenect_stop_video(mFreenectDevice);
    freenect_close_device(mFreenectDevice);
    freenect_shutdown(mFreenectContext);
}

//-----------------------------------------------------------------------------------------------------------------
bool StereoCameraKinect::init(const cjson::Json & _json){
        #ifdef ENABLE_LIBFREENECT
			mConfig = _json;

            // Init streams
            if (freenect_init(&mFreenectContext, NULL) < 0) {
                std::cout << "[STEREO CAMERA][KINECT] freenect_init() failed" << std::endl;
                return false;
            }

            freenect_select_subdevices(mFreenectContext, FREENECT_DEVICE_CAMERA);
            int num_devices = freenect_num_devices(mFreenectContext);
            if (num_devices <= 0) {
                std::cout << "[STEREOCAMERA][KINECT] No device found!" << std::endl;
                freenect_shutdown(mFreenectContext);
                return false;
            }
            // Open the first device.
            freenect_device* fn_dev;
            int ret = freenect_open_device(mFreenectContext, &mFreenectDevice, 0);
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return ret;
            }

            ret = freenect_set_depth_mode(mFreenectDevice, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return ret;
            }
            ret = freenect_set_video_mode(mFreenectDevice, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return ret;
            }

            mLeft = cv::Mat(480,640, CV_8UC3);
            //--->freenect_set_depth_callback(mFreenectDevice, depthCallback);
            //--->freenect_set_video_callback(mFreenectDevice, rgbCallback);

            ret = freenect_start_depth(mFreenectDevice);
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return false;
            }
            ret = freenect_start_video(mFreenectDevice);
            if (ret < 0) {
                freenect_shutdown(mFreenectContext);
                return false;
            }

            mFreenectEventProcessor = std::thread([&](){    /// 666 MANAGE TO CLOSE THIS THREAD!
                while(mRunning &&  freenect_process_events(mFreenectContext) >= 0){}
            });


            //mRsDepthIntrinsic = new rs::intrinsics();
            //auto tempDepthIntrinsic = mRsDevice->get_stream_intrinsics(rs::stream::depth);
            //memcpy(mRsDepthIntrinsic, &tempDepthIntrinsic, sizeof(rs::intrinsics));
            //
            //mRsDepthToColor = new rs::extrinsics();
            //auto tempDepth2Color = mRsDevice->get_extrinsics(rs::stream::depth, rs::stream::color);
            //memcpy(mRsDepthToColor, &tempDepth2Color, sizeof(rs::extrinsics));
            //
            //
            //mRsColorToDepth = new rs::extrinsics();
            //auto tempColor2Depth = mRsDevice->get_extrinsics(rs::stream::color, rs::stream::depth);
            //memcpy(mRsColorToDepth, &tempColor2Depth, sizeof(rs::extrinsics));
            //
            //mRsColorIntrinsic = new rs::intrinsics();
            //auto tempColorIntrinsic = mRsDevice->get_stream_intrinsics(rs::stream::color);
            //memcpy(mRsColorIntrinsic, &tempColorIntrinsic, sizeof(rs::intrinsics));
            //
            //mRsDepthScale		= mRsDevice->get_depth_scale();
            //
            //
            //// Projection matrix Depth
            //mCvDepthIntrinsic = cv::Mat::eye(3,3,CV_32F);
            //mCvDepthIntrinsic.at<float>(0,0) = mRsDepthIntrinsic->fx;
            //mCvDepthIntrinsic.at<float>(1,1) = mRsDepthIntrinsic->fy;
            //mCvDepthIntrinsic.at<float>(0,2) = mRsDepthIntrinsic->ppx;
            //mCvDepthIntrinsic.at<float>(1,2) = mRsDepthIntrinsic->ppy;
            //
            //// Projection matrix Color
            //mCvColorIntrinsic= cv::Mat::eye(3,3,CV_32F);
            //mCvColorIntrinsic.at<float>(0,0) = mRsColorIntrinsic->fx;
            //mCvColorIntrinsic.at<float>(1,1) = mRsColorIntrinsic->fy;
            //mCvColorIntrinsic.at<float>(0,2) = mRsColorIntrinsic->ppx;
            //mCvColorIntrinsic.at<float>(1,2) = mRsColorIntrinsic->ppy;
            //
            //mExtrinsicColorToDepth = cv::Mat::eye(4,4,CV_32F);
            //cv::Mat(3,3,CV_32F, &mRsColorToDepth->rotation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(0,0,3,3)));
            //mExtrinsicColorToDepth(cv::Rect(0,0,3,3)) = mExtrinsicColorToDepth(cv::Rect(0,0,3,3)).t(); // RS use color major instead of row mayor.
            //cv::Mat(3,1,CV_32F, &mRsColorToDepth->translation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(3,0,1,3)));

			mUseUncolorizedPoints = (bool) mConfig["useUncolorizedPoints"];

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::rgb(cv::Mat & _left, cv::Mat & _right){
        #ifdef ENABLE_LIBFREENECT
            mLastRGB.copyTo(_left);
			return mHasRGB;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::depth(cv::Mat & _depth){
        #ifdef ENABLE_LIBFREENECT
            return false;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::grab(){
        #ifdef ENABLE_LIBFREENECT

            return false;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
        #ifdef ENABLE_LIBFREENECT
        for (int dy = 0; dy < mLastRGB.rows; dy = dy) {
            for (int dx = 0; dx < mLastRGB.cols; dx = dx) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
                    float depth_in_meters = depth_value/1000; //<-- 666 Scale into mm

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
                        //rs::float2 depth_pixel = { (float)dx, (float)dy };    // 666 --- NEED CALIBRATION
                        //rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);
                        //
                        //_cloud.push_back(pcl::PointXYZ(depth_point.x, depth_point.y, depth_point.z));
					}
				}
			}
			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
        #ifdef ENABLE_LIBFREENECT
            for (int dy = 0; dy < mLastRGB.rows; dy = dy) {
                for (int dx = 0; dx < mLastRGB.cols; dx = dx ) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
                    float depth_in_meters = depth_value/1000; //<-- 666 Scale into mm
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
                        //rs::float2 depth_pixel = { (float)dx, (float)dy };
                        //rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);
                        //point.x = depth_point.x;
                        //point.y = depth_point.y;
                        //point.z = depth_point.z;
                        //auto rgb = mLastRGB.at<cv::Vec3b>(dy, dx);
                        //point.r = rgb[2];
                        //point.g = rgb[1];
                        //point.b = rgb[0]; 666 NEED CALIBRATION

                    }

					_cloud.push_back(point);
                }
            }
            return true;

		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
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
    bool StereoCameraKinect::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraKinect::disparityToDepthParam(double &_dispToDepth){
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void StereoCameraKinect::rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp) {
        mRgbMutex.lock();
        memcpy(mLeft.data, rgb , 640*480*3);
        mRgbMutex.unlock();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void StereoCameraKinect::depthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
        cv::Mat cvdepth = cv::Mat( 640, 480, CV_16UC1, depth);
        mDepthMutex.lock();
        cvdepth.copyTo(mDepth);
        mDepthMutex.unlock();
    }

}	//	namespace rgbd
