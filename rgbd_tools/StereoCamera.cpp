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
#include "StereoCameras/StereoCameraEVA.h"
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
			return new StereoCameraEva();
		}
		else if (_model == eModel::Custom) {
			return new StereoCameraCustom();
				}
		else if (_model == eModel::RealSense) {
			return new StereoCameraRealSense();
		}
		else {
			return nullptr;
		}
	}


	//---------------------------------------------------------------------------------------------------------------------
}	//	namespace rgbd
