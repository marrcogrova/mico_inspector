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

#include <rgbd_tools/map3d/BarricSlam.h>
#include <rgbd_tools/map3d/PointProb.h>
#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/Word.h>
#include <rgbd_tools/map3d/BundleAdjusterCvsba.h>
#include <rgbd_tools/map3d/BundleAdjuster_g2o.h>
#include <rgbd_tools/map3d/BundleAdjusterRos.h>
#include <rgbd_tools/map3d/LoopClosureDetectorDorian.h>
#include <rgbd_tools/utils/Gui.h>
#include <rgbd_tools/utils/Graph2d.h>
#include <rgbd_tools/cjson/json.h>
#include <DVision/DVision.h> // Brief

#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>

#include <iostream>
#include <fstream>

#include <sys/stat.h>

#ifdef RGBDTOOLS_USE_ROS
    #include <ros/ros.h>
#endif

using namespace pcl;
using namespace std;
using namespace rgbd;


namespace rgbd {

    bool BarricSlam::init(int _arc, char **_argv){

    #ifdef RGBDTOOLS_USE_ROS
        if(!ros::isInitialized()){
            this->error("MAIN_APPLICATION", "Failed ros initialization");
            return false;
        }
    #endif

    // Open Json
    std::ifstream file(_argv[1]);
    if (!file.is_open())
    {
        std::cout << "Cannot open file." << std::endl;
        return false;
    }
    if (!mConfigFile.parse(file))
    {
        std::cout << "Cannot parse config file." << std::endl;
        return false;
    }

    std::string runLabel = std::to_string(time(NULL));
    std::string folderName = "run_"+runLabel;
    const int dir_err = mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    LogManager::init(folderName+"/MARK_I_" + runLabel);
    mPoseFileName = folderName+"/poses_" + runLabel;
    if(mConfigFile.contains("ground_truth")){
        mHasGroundTruth = true;
        mGroundTruthFile.open(mConfigFile["ground_truth"]);
    }
    if(mConfigFile.contains("image_time_stamp")){
        mHasTimeStamp = true;
        mImageTimeStampFile.open(mConfigFile["image_time_stamp"]);
    }
    mAllEstimatesFile.open(folderName+"/all_poses_"+runLabel);
    
    // Initializing Camera
    if (!initCamera(mConfigFile["camera"])) {
        std::cout << "Error initializing camera" << std::endl;
        return false;
    }
    // Dataframe-Cluster visualization option in json
    if (mConfigFile["visualizer"]["DataframeVisualization"] == "true") {
        std::cout << "Dataframe visualization ON" << std::endl;
        mDataframeVisualization = true;
    }
    if(mConfigFile["visualizer"].contains("draw_1_frame_each")){
        mDraw1FrameEachX = (int) mConfigFile["visualizer"]["draw_1_frame_each"];
    }

    // Initializing odometry module
    mOdometry = new rgbd::OdometryRgbd<PointType_, rgbd::DebugLevels::Debug>();
    if (!mOdometry->init(mConfigFile["registrator_params"])) {
        std::cout << "Error initializing odometry parameters" << std::endl;
        return false;
    }
    

    if (mConfigFile.contains("database")) {
        if (!mDatabase.init(mConfigFile["database"])) {
            std::cout << "FAILED INIT OF VOCABULARY in database" << std::endl;
        }
    }

    // Initializing vocabulary in mapping and loop closure modules
    if (mConfigFile.contains("LoopClosure")) {
        std::cout << "Initializating vocabulary" << std::endl;
        mLoopDetector = new rgbd::LoopClosureDetectorDorian<>;
        mLoopDetector->init(mConfigFile["LoopClosure"]);
    
    }
    // Initializing visualization module
    if (!Visualizer<PointType_>::init(mConfigFile["visualizer"], &mDatabase)) {
        std::cout << "Error initializing viewer" << std::endl;
        return false;
    }
    mVisualization = Visualizer<PointType_>::get();

    if(mVisualization != nullptr){
        mVisualization->addCustomKeyCallback([&](const pcl::visualization::KeyboardEvent &_event, void *_data){
            if (_event.keyDown() && _event.getKeySym() == "l") {
                mKeyOptimize = true;
            }
        });
    }

    // Selecting optimization algorithm
    if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "cvsba") {
        mBA = new rgbd::BundleAdjusterCvsba<PointType_, rgbd::DebugLevels::Debug>;
        mEnableOptimization = true;
        std::cout << "Optimization algorithm: cvsba" << std::endl;
    }
    else if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "g2o") {
        mBA = new rgbd::BundleAdjuster_g2o<PointType_, rgbd::DebugLevels::Debug>;
        mEnableOptimization = true;
        std::cout << "Optimization algorithm: g2o" << std::endl;
    }
    #ifdef RGBDTOOLS_USE_ROS_DEPRECATED
        else if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "ros_sba") {
        mBA = new rgbd::BundleAdjusterRos<PointType_, rgbd::DebugLevels::Debug>;
        mEnableOptimization = true;
        std::cout << "Optimization algorithm: ros_sba" << std::endl;
        }
    #endif
    else if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "null") {
        mBA = nullptr; // WARNING
        mEnableOptimization = false;
        std::cout << "Optimization disabled" << std::endl;
    }
    if(mBA != nullptr){
        // Initializing optimization module
        mBA->minError((double)mConfigFile["bundle_adjuster"]["minError"]);
        mBA->iterations((int)mConfigFile["bundle_adjuster"]["iterations"]);
        mBA->minAparitions((int)mConfigFile["bundle_adjuster"]["minAparitions"]);
        mBA->minWords((int)mConfigFile["bundle_adjuster"]["minWords"]);
    }
    

    if(mHasGroundTruth){
        const std::string delimiter = " ";
        std::string line;
        int counterLine = 0;
        std::ifstream file(mConfigFile["ground_truth"]);
        pcl::PointXYZ prevPoint;
        while(std::getline(file, line)){
            size_t pos = 0;
            if(counterLine%50 == 0){
                std::vector<std::string> tokens;
                while ((pos = line.find(delimiter)) != std::string::npos) {
                    auto token = line.substr(0, pos);
                    tokens.push_back(token);
                    line.erase(0, pos + delimiter.length());
                }
                auto token = line.substr(0, pos);
                tokens.push_back(token);

                pcl::PointXYZ p (
                        atof(tokens[1].c_str()),
                        atof(tokens[2].c_str()),
                        atof(tokens[3].c_str())
                    );
                if(counterLine==0){
                    prevPoint = p;  

                    Eigen::Vector3f position = {atof(tokens[1].c_str()),
                                                atof(tokens[2].c_str()),
                                                atof(tokens[3].c_str())};
                    Eigen::Quaternionf q;
                    q.x() =  atof(tokens[4].c_str());
                    q.y() =  atof(tokens[5].c_str());
                    q.z() =  atof(tokens[6].c_str());
                    q.w() =  atof(tokens[7].c_str());
                    
                    mFirstPose.block<3,1>(0,3) = position;
                    mFirstPose.block<3,3>(0,0) = q.matrix();
                }else{
                    if(fabs(p.x) < 10 || fabs(p.y) < 10 || fabs(p.z) < 10){
                        if(mVisualization->mViewer != nullptr){
                            mVisualization->mViewer->addLine(prevPoint, p, 1,0,0, "gt_"+std::to_string(counterLine));
                            mVisualization->mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3,"gt_"+std::to_string(counterLine));
                        }
                        prevPoint = p;
                    }
                }
            }
            counterLine++;
        }
    }

    //if(mConfigFile.contains("dnn")){
    //    mObjectDetector.init(mConfigFile["dnn"]["cfg"],mConfigFile["dnn"]["weights"]);
    //}


    mFeatureDetector = cv::ORB::create(1000);//(2000, 4,31,0,3, cv::ORB::HARRIS_SCORE,31,30);
    mVisualization->pause();
    return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BarricSlam::initCamera(cjson::Json _cameraConfig) {
        mCamera = rgbd::StereoCamera::create(_cameraConfig["type"]);

        if (mCamera != nullptr && mCamera->init(_cameraConfig["config"]))
            return true;
        else
            return false;

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BarricSlam::step(){
    }
}