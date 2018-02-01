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


#include <rgbd_tools/StereoCamera.h>

#include <rgbd_tools/StereoCameras/StereoCameraVirtual.h>
#include <rgbd_tools/StereoCameras/StereoCameraZED.h>
#include <rgbd_tools/StereoCameras/StereoCameraCustom.h>
#include <rgbd_tools/StereoCameras/StereoCameraRealSense.h>
#include <rgbd_tools/StereoCameras/StereoCameraKinect.h>


namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	StereoCamera * StereoCamera::create(eModel _model) {
		if (_model == eModel::Virtual) {
			return new StereoCameraVirtual();
		}
		else if (_model == eModel::Zed) {
			return new StereoCameraZed();
		}
		else if (_model == eModel::ArtecEva) {
            std::cerr << "[STERECAMERA] Deprecated model: Artec EVA" << std::endl;
            return nullptr;
		}
		else if (_model == eModel::Custom) {
			return new StereoCameraCustom();
		} 
        else if (_model == eModel::RealSense) {
            return new StereoCameraRealSense();
        }
        else if (_model == eModel::Kinect) {
            return new StereoCameraKinect();
        }
		else {
            std::cerr << "[STEREOCAMERA]  unknown model type" << std::endl;
			return nullptr;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        std::cerr << "[STEREOCAMERA] leftCalibration method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        std::cerr << "[STEREOCAMERA] rightCalibration method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        std::cerr << "[STEREOCAMERA] extrinsic method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        std::cerr << "[STEREOCAMERA] extrinsic method not implemented for given point type." << std::endl;
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCamera::disparityToDepthParam(double &_dispToDepth) {
        std::cerr << "[STEREOCAMERA] cloud method not implemented for given point type." << std::endl;
        return false;
    }

	//---------------------------------------------------------------------------------------------------------------------
}	//	namespace rgbd
