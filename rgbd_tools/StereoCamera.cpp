////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#include "StereoCamera.h"

#include "StereoCameras/StereoCameraVirtual.h"
#include "StereoCameras/StereoCameraZED.h"
#include "StereoCameras/StereoCameraCustom.h"
#include "StereoCameras/StereoCameraRealSense.h"


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
