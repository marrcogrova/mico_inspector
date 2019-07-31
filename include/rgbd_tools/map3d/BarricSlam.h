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

#ifndef BARRICSLAM_H_
#define BARRICSLAM_H_

#include <rgbd_tools/StereoCameras/StereoCameraRealSense.h>
#include <rgbd_tools/StereoCameras/StereoCameraRosBag.h>
#include <rgbd_tools/StereoCameras/StereoCameraVirtual.h>

#include <rgbd_tools/rgbd_tools.h>
#include <rgbd_tools/cjson/json.h>
#include <rgbd_tools/StereoCamera.h>
#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/LoopClosureDetector.h>
#include <rgbd_tools/map3d/Odometry.h>
#include <rgbd_tools/map3d/OdometryRgbd.h>
#include <rgbd_tools/utils/LogManager.h>
#include <rgbd_tools/map3d/BundleAdjuster.h>
#include <rgbd_tools/map3d/Database.h>
#include <rgbd_tools/map3d/Visualizer.h>
#include <rgbd_tools/map3d/PointProb.h>
#include <rgbd_tools/object_detection/dnn/WrapperDarknet_cl.h>

#include <vector>
#include <fstream>
#include <pcl/common/pca.h>

namespace rgbd{
class BarricSlam : public rgbd::LoggableInterface<rgbd::DebugLevels::Debug, rgbd::OutInterfaces::Cout> {
  public:
    typedef PointProb<3> PointType_;

    /// Initializes camera and visualizer
    bool init(int _arc, char **_argv);
    
    bool initCamera(cjson::Json _cameraConfig);
    
    /// Main loop
    bool step();

  private:
    //rgbd::WrapperDarknet_cl mObjectDetector;

    cjson::Json mConfigFile;
    rgbd::StereoCamera *mCamera;
    rgbd::Odometry<PointType_, rgbd::DebugLevels::Debug> *mOdometry;
    rgbd::Visualizer<PointType_> *mVisualization = nullptr;
    rgbd::BundleAdjuster<PointType_, rgbd::DebugLevels::Debug> *mBA;
    //rgbd::LoopClosureDetector<PointType_, rgbd::DebugLevels::Debug> mLoopClosureDetector;
    rgbd::Database<PointType_> mDatabase;
    cv::Ptr<cv::ORB> mFeatureDetector ;
    bool mDataframeVisualization=false;
    
    rgbd::LoopClosureDetector<> *mLoopDetector = nullptr;

    int mDfCounter = 0;

    bool mEnableOptimization = false;
    int mStepCounter = 0;
    std::ofstream mPosesFile;
    std::ofstream mAllEstimatesFile;
    std::ifstream mGroundTruthFile, mImageTimeStampFile;
    bool mHasTimeStamp = false;
    bool mHasGroundTruth  = false;
    Eigen::Matrix4f mFirstPose;
    std::string mPoseFileName;
  
    int mDraw1FrameEachX = 1;

    bool mKeyOptimize = false;

    std::vector<std::vector<float>> mCandidates;
    std::vector<int> mCandidatesColor;

    int mLostFramesCounter = 0;
    bool mLostState = false;

};
}

#endif // BARRICSLAM_H_