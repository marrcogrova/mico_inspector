#ifndef BARRICSLAM_H_
#define BARRICSLAM_H_

#include "PointProb.h"

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
#include <rgbd_tools/object_detection/dnn/WrapperDarknet_cl.h>
#include <vector>

//#include "Visualizer.h"
#include <fstream>

#include <pcl/common/pca.h>

class BarricSlam : public rgbd::LoggableInterface<rgbd::DebugLevels::Debug, rgbd::OutInterfaces::Cout> {
  public:
    typedef PointProb<3> PointType_;

    /// Initializes camera and visualizer
    bool init(int _arc, char **_argv){
        return true;
    }
    
    /// Main loop
    bool step(){
        return true;
    }

  private:
    //rgbd::WrapperDarknet_cl mObjectDetector;

    cjson::Json mConfigFile;
    rgbd::StereoCamera *mCamera;
    rgbd::Odometry<PointType_, rgbd::DebugLevels::Debug> *mOdometry;
    //rgbd::Visualizer<PointType_> *mVisualization = nullptr;
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


#endif // BARRICSLAM_H_