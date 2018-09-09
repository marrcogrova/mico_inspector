//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#include <rgbd_tools/StereoCameras/StereoCameraRosBag.h>
#include <pcl/features/integral_image_normal.h>

#include <cv_bridge/cv_bridge.h>

namespace rgbd {

    //-----------------------------------------------------------------------------------------------------------------
    StereoCameraRosBag::~StereoCameraRosBag() {
		        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::init(const cjson::Json & _json){
        #ifdef RGBDTOOLS_USE_ROS
            mBag.open(_json["bag_file"], rosbag::bagmode::Read);

            if(_json.contains("left") && ((std::string)_json["left"] != "")){
                leftView = new rosbag::View(mBag, rosbag::TopicQuery(_json["left"]));
                leftIt = leftView->begin();
            }
            if(_json.contains("right") && ((std::string) _json["right"] != "")){
                rightView = new rosbag::View(mBag, rosbag::TopicQuery(_json["right"]));
                rightIt = rightView->begin();
            }
            if(_json.contains("depth") && ((std::string) _json["depth"] != "")){
                depthView = new rosbag::View(mBag, rosbag::TopicQuery(_json["depth"]));
                depthIt = depthView->begin();
            }
            //if(_json.contains("cloud")){
            //    mSubscriberCloud = it.subscribe(_json["cloud"], &StereoCameraRosBag::cloudCallback, this);
            //}

            // Load Calibration files if path exist
            if(_json.contains("calibFile") && std::string(_json["calibFile"]) != ""){
                mHasCalibration = true;

                cv::FileStorage fs((std::string)_json["calibFile"], cv::FileStorage::READ);

                fs["MatrixLeft"]            >> mMatrixLeft;
                fs["DistCoeffsLeft"]        >> mDistCoefLeft;
                fs["MatrixRight"]           >> mMatrixRight;
                fs["DistCoeffsRight"]       >> mDistCoefRight;
                fs["Rotation"]              >> mRot;
                fs["Translation"]           >> mTrans;
                fs["DisparityToDepthScale"] >> mDispToDepth;

            }else{
                mHasCalibration = false;
            }

            return true;
        #else
            return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::rgb(cv::Mat & _left, cv::Mat & _right){
        #ifdef RGBDTOOLS_USE_ROS
            _left = mLastRGB;
            _right = mRight;
            return true;
        #else
            return false;
        #endif
    }

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::depth(cv::Mat & _depth){
        #ifdef RGBDTOOLS_USE_ROS
            _depth = mLastDepthInColor;
            return true;
        #else
            return false;
        #endif
    }

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::grab(){
        #ifdef RGBDTOOLS_USE_ROS

            if(leftIt != leftView->end()){
                auto msg = leftIt->instantiate<sensor_msgs::Image>();
                mLastRGB = cv_bridge::toCvCopy(msg, "bgr8")->image;
                leftIt++;
            }else{
                return false;
            }

            if(rightIt != rightView->end()){
                auto msg = leftIt->instantiate<sensor_msgs::Image>();
                mRight = cv_bridge::toCvCopy(msg, "bgr8")->image;
                rightIt++;
            }else{
                return false;
            }

            if(depthIt != depthView->end()){
               auto msg = leftIt->instantiate<sensor_msgs::Image>();
                mLastDepthInColor = cv_bridge::toCvCopy(msg, msg->encoding)->image;
                depthIt++;
            }else{
                return false;
            }

            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
        #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
       #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
       #ifdef RGBDTOOLS_USE_ROS
            return true;
        #else
            return false;
        #endif
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixLeft.copyTo(_intrinsic);
        mDistCoefLeft.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mMatrixRight.copyTo(_intrinsic);
        mDistCoefRight.copyTo(_coefficients);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mDispToDepth;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRosBag::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point) {
        return false;
    }
}	//	namespace rgbd