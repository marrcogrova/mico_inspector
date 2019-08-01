//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92) ricloplop@gmail.com
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

namespace rgbd
{

bool BarricSlam::init(int _arc, char **_argv)
{

#ifdef RGBDTOOLS_USE_ROS
    if (!ros::isInitialized())
    {
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
    std::string folderName = "run_" + runLabel;
    const int dir_err = mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    LogManager::init(folderName + "/MARK_I_" + runLabel);
    mPoseFileName = folderName + "/poses_" + runLabel;
    if (mConfigFile.contains("ground_truth"))
    {
        mHasGroundTruth = true;
        mGroundTruthFile.open(mConfigFile["ground_truth"]);
    }
    if (mConfigFile.contains("image_time_stamp"))
    {
        mHasTimeStamp = true;
        mImageTimeStampFile.open(mConfigFile["image_time_stamp"]);
    }
    mAllEstimatesFile.open(folderName + "/all_poses_" + runLabel);

    // Initializing Camera
    if (!initCamera(mConfigFile["camera"]))
    {
        std::cout << "Error initializing camera" << std::endl;
        return false;
    }
    // Dataframe-Cluster visualization option in json
    if (mConfigFile["visualizer"]["DataframeVisualization"] == "true")
    {
        std::cout << "Dataframe visualization ON" << std::endl;
        mDataframeVisualization = true;
    }
    if (mConfigFile["visualizer"].contains("draw_1_frame_each"))
    {
        mDraw1FrameEachX = (int)mConfigFile["visualizer"]["draw_1_frame_each"];
    }

    // Initializing odometry module
    mOdometry = new rgbd::OdometryRgbd<PointType_, rgbd::DebugLevels::Debug>();
    if (!mOdometry->init(mConfigFile["registrator_params"]))
    {
        std::cout << "Error initializing odometry parameters" << std::endl;
        return false;
    }

    if (mConfigFile.contains("database"))
    {
        if (!mDatabase.init(mConfigFile["database"]))
        {
            std::cout << "FAILED INIT OF VOCABULARY in database" << std::endl;
        }
    }

    // Initializing vocabulary in mapping and loop closure modules
    if (mConfigFile.contains("LoopClosure"))
    {
        std::cout << "Initializating vocabulary" << std::endl;
        mLoopDetector = new rgbd::LoopClosureDetectorDorian<>;
        mLoopDetector->init(mConfigFile["LoopClosure"]);
    }
    // Initializing visualization module
    if (!Visualizer<PointType_>::init(mConfigFile["visualizer"], &mDatabase))
    {
        std::cout << "Error initializing viewer" << std::endl;
        return false;
    }
    mVisualization = Visualizer<PointType_>::get();

    if (mVisualization != nullptr)
    {
        mVisualization->addCustomKeyCallback([&](const pcl::visualization::KeyboardEvent &_event, void *_data) {
            if (_event.keyDown() && _event.getKeySym() == "l")
            {
                mKeyOptimize = true;
            }
        });
    }

    // Selecting optimization algorithm
    if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "cvsba")
    {
        mBA = new rgbd::BundleAdjusterCvsba<PointType_, rgbd::DebugLevels::Debug>;
        mEnableOptimization = true;
        std::cout << "Optimization algorithm: cvsba" << std::endl;
    }
    else if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "g2o")
    {
        mBA = new rgbd::BundleAdjuster_g2o<PointType_, rgbd::DebugLevels::Debug>;
        mEnableOptimization = true;
        std::cout << "Optimization algorithm: g2o" << std::endl;
    }
#ifdef RGBDTOOLS_USE_ROS_DEPRECATED
    else if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "ros_sba")
    {
        mBA = new rgbd::BundleAdjusterRos<PointType_, rgbd::DebugLevels::Debug>;
        mEnableOptimization = true;
        std::cout << "Optimization algorithm: ros_sba" << std::endl;
    }
#endif
    else if (mConfigFile["bundle_adjuster"]["bundleAdjusterType"] == "null")
    {
        mBA = nullptr; // WARNING
        mEnableOptimization = false;
        std::cout << "Optimization disabled" << std::endl;
    }
    if (mBA != nullptr)
    {
        // Initializing optimization module
        mBA->minError((double)mConfigFile["bundle_adjuster"]["minError"]);
        mBA->iterations((int)mConfigFile["bundle_adjuster"]["iterations"]);
        mBA->minAparitions((int)mConfigFile["bundle_adjuster"]["minAparitions"]);
        mBA->minWords((int)mConfigFile["bundle_adjuster"]["minWords"]);
    }

    if (mHasGroundTruth)
    {
        const std::string delimiter = " ";
        std::string line;
        int counterLine = 0;
        std::ifstream file(mConfigFile["ground_truth"]);
        pcl::PointXYZ prevPoint;
        while (std::getline(file, line))
        {
            size_t pos = 0;
            if (counterLine % 50 == 0)
            {
                std::vector<std::string> tokens;
                while ((pos = line.find(delimiter)) != std::string::npos)
                {
                    auto token = line.substr(0, pos);
                    tokens.push_back(token);
                    line.erase(0, pos + delimiter.length());
                }
                auto token = line.substr(0, pos);
                tokens.push_back(token);

                pcl::PointXYZ p(
                    atof(tokens[1].c_str()),
                    atof(tokens[2].c_str()),
                    atof(tokens[3].c_str()));
                if (counterLine == 0)
                {
                    prevPoint = p;

                    Eigen::Vector3f position = {atof(tokens[1].c_str()),
                                                atof(tokens[2].c_str()),
                                                atof(tokens[3].c_str())};
                    Eigen::Quaternionf q;
                    q.x() = atof(tokens[4].c_str());
                    q.y() = atof(tokens[5].c_str());
                    q.z() = atof(tokens[6].c_str());
                    q.w() = atof(tokens[7].c_str());

                    mFirstPose.block<3, 1>(0, 3) = position;
                    mFirstPose.block<3, 3>(0, 0) = q.matrix();
                }
                else
                {
                    if (fabs(p.x) < 10 || fabs(p.y) < 10 || fabs(p.z) < 10)
                    {
                        if (mVisualization->mViewer != nullptr)
                        {
                            mVisualization->mViewer->addLine(prevPoint, p, 1, 0, 0, "gt_" + std::to_string(counterLine));
                            mVisualization->mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "gt_" + std::to_string(counterLine));
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

    mFeatureDetector = cv::ORB::create(1000); //(2000, 4,31,0,3, cv::ORB::HARRIS_SCORE,31,30);
    mVisualization->pause();
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool BarricSlam::initCamera(cjson::Json _cameraConfig)
{
    mCamera = rgbd::StereoCamera::create(_cameraConfig["type"]);

    if (mCamera != nullptr && mCamera->init(_cameraConfig["config"]))
        return true;
    else
        return false;

    return true;
}

//-----------------------------------------------------------------------------------------------------------------
bool BarricSlam::step()
{
    // Creating new Dataframe
    std::shared_ptr<rgbd::DataFrame<PointType_>> df(new rgbd::DataFrame<PointType_>);

    // data acquisition & feature detector
    if (mConfigFile.contains("data_fake"))
    {
        if (!dataAcquisition_fake(df))
            return false;
    }
    else
    {
        if (!dataAcquisition(df))
        {
            // End of dataset. Global optimization and pause
            if (mEnableOptimization)
            {
                mBA->iterations(20);
                mBA->clusterframes(mDatabase.mClusterframes);
                mBA->optimizeClusterframes();
                for (auto &cluster : mDatabase.mClusterframes)
                {
                    mVisualization->drawClusterframe(cluster.second);
                }
            }

            for (auto &cf : mDatabase.mClusterframes)
            {
                //if(mHasTimeStamp){
                Eigen::Quaternionf q(cf.second->pose.block<3, 3>(0, 0));
                mAllEstimatesFile << cf.second->timeStamp << " " << cf.second->pose(0, 3) << " " << cf.second->pose(1, 3) << " " << cf.second->pose(2, 3) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
                //}
            }
            mAllEstimatesFile.flush();
            mAllEstimatesFile.close();
            std::cout << "Stored all poses. Pausing before finishing." << std::endl;

            mVisualization->pause();

            return false;
        }
    }

    if (mHasGroundTruth && df->id == 0)
    {
        df->position = mFirstPose.block<3, 1>(0, 3);
        Eigen::Quaternionf q(mFirstPose.block<3, 3>(0, 0));
        df->orientation = q;
        df->pose = mFirstPose;
    }

    std::string timeStampGt = "0.0000";
    if (mHasTimeStamp)
    {
        const std::string delimiter = " ";
        std::string line;
        std::getline(mImageTimeStampFile, line);
        size_t pos = 0;
        std::vector<std::string> tokens;
        while ((pos = line.find(delimiter)) != std::string::npos)
        {
            auto token = line.substr(0, pos);
            tokens.push_back(token);
            std::cout << token << std::endl;
            line.erase(0, pos + delimiter.length());
        }
        auto token = line.substr(0, pos);
        tokens.push_back(token);

        timeStampGt = tokens[0];
    }
    df->timeStamp = timeStampGt;

    bool is_newCluster = false;
    // Odometry
    LogManager::get()->saveTimeMark("odomInit");
    if (!mLostState)
    {
        if (mOdometry->computeOdometry(mDatabase.mLastClusterframe, df))
        {
            LogManager::get()->saveTimeMark("odomEnd");
            LogManager::get()->message("time_odom", std::to_string(LogManager::get()->measureTimeBetweenMarks("odomEnd", "odomInit")));

            // Mapping
            LogManager::get()->saveTimeMark("addDFInit");
            is_newCluster = mDatabase.addDataframe(df);
            LogManager::get()->saveTimeMark("addDFEnd");
            LogManager::get()->message("time_adddf", std::to_string(LogManager::get()->measureTimeBetweenMarks("addDFEnd", "addDFInit")));

            LogManager::get()->message("time_draw", std::to_string(LogManager::get()->measureTimeBetweenMarks("drawEnd", "drawInit")));

            if (is_newCluster && mLoopDetector != nullptr)
            {
                auto result = mLoopDetector->appendCluster(mDatabase.mLastClusterframe->left, mDatabase.mLastClusterframe->id);
                if (result.found)
                {
                    std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> loopClosureSubset;
                    loopClosureSubset[mDatabase.mLastClusterframe->id] = mDatabase.mLastClusterframe;
                    loopClosureSubset[result.matchId] = mDatabase.mClusterframes[result.matchId];
                    mDatabase.clusterComparison(loopClosureSubset, false);

                    mVisualization->drawClusterframe(mDatabase.mLastClusterframe);
                    mVisualization->drawClusterframe(mDatabase.mClusterframes[result.matchId]);
                }
            }
            mLostFramesCounter = 0;
        }
        else
        {
            mLostFramesCounter++;
            if (mLostFramesCounter > 15)
            {
                mLostFramesCounter = 0;
                mLostState = true;
                this->error("MAIN_APPLICATION", "Lost! starting new line of data");
                return true;
            }
        }
    }
    else
    {
        df->updatePose(mDatabase.mLastClusterframe->pose);
        mDatabase.mLastClusterframe = nullptr;
        mDatabase.mLastDataFrame = nullptr;
        is_newCluster = mDatabase.addDataframe(df);
        mLostState = false;
    }

    if (mEnableOptimization && mKeyOptimize)
    {
        mBA->clusterframes(mDatabase.mClusterframes);
        mBA->optimizeClusterframes();
        for (auto &cluster : mDatabase.mClusterframes)
        {
            mVisualization->drawClusterframe(cluster.second);
        }
        mKeyOptimize = false;
    }

    // Semantic info
    //semanticLabeling(df);

    // Visualization
    LogManager::get()->saveTimeMark("drawInit");

    if (is_newCluster)
    {
        if (mDatabase.mLastClusterframe->id % ((int)mDraw1FrameEachX) != 0)
        {
            mDatabase.mLastClusterframe->cloud = nullptr;
        }
        mVisualization->drawClusterframe(mDatabase.mLastClusterframe);
        mDatabase.mLastClusterframe->timeStamp = timeStampGt;

        if (mEnableOptimization && mDatabase.mLastClusterframe->id % 15 == 0)
        {
            // Optimizacion of clusters without loop closure
            if (mDatabase.mClusterframes.size() > (int)mConfigFile["bundle_adjuster"]["minAparitions"] + 2)
            {
                std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> subset;

                std::vector<int> clusterCovisibility = mDatabase.mLastClusterframe->covisibility;
                // Optimize a maximun of 15 clusters
                std::sort(clusterCovisibility.rbegin(), clusterCovisibility.rend());
                int nMaxCluster = 0;
                for (auto cov = clusterCovisibility.begin(); cov != clusterCovisibility.end() && nMaxCluster < 15; ++cov)
                {
                    subset[*cov] = mDatabase.mClusterframes[*cov];
                    ++nMaxCluster;
                }
                mBA->clusterframes(subset);
                if (mBA->optimizeClusterframes())
                {
                    for (auto &cluster : subset)
                    {
                        mVisualization->drawClusterframe(cluster.second);
                    }
                }
            }
        }
    }

    //mVisualization->drawWords(mDatabase.mWordDictionary);

    if (mDatabase.mLastClusterframe->id % 3 == 0)
    {
        mPosesFile.open(mPoseFileName + "cf_" + std::to_string(mDatabase.mLastClusterframe->id));
        for (auto &cluster : mDatabase.mClusterframes)
        {
            Eigen::Matrix4f bestPose = cluster.second->pose;
            Eigen::Quaternionf q(bestPose.block<3, 3>(0, 0));
            mPosesFile << cluster.second->timeStamp << " " << bestPose(0, 3) << " " << bestPose(1, 3) << " " << bestPose(2, 3) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            mPosesFile.flush();
        }
        mPosesFile.close();
    }

    mVisualization->updateCurrentPose(df->pose);
    mVisualization->spinOnce();

    LogManager::get()->saveTimeMark("drawEnd");

    return true;
}
//-----------------------------------------------------------------------------------------------------------------
bool BarricSlam::dataAcquisition(std::shared_ptr<rgbd::DataFrame<PointType_>> _df)
{
    // Getting data from camera
    LogManager::get()->saveTimeMark("dataInit");
    mCamera->leftCalibration(_df->intrinsic, _df->coefficients);
    mCamera->grab();
    cv::Mat left, right;
    mCamera->rgb(left, right);

    if (left.rows == 0)
    {
        this->error("MAIN_APPLICATION", "Color image empty");
        return false;
    }

    undistort(left, _df->left, _df->intrinsic, _df->coefficients);

    cv::Mat depth;
    mCamera->depth(depth);
    if (depth.rows == 0)
    {
        this->error("MAIN_APPLICATION", "Depth image empty");
        return false;
    }

    pcl::PointCloud<PointType_> cloud;
    (static_cast<rgbd::StereoCameraRosBag *>(mCamera))->cloud(cloud);
    // Downsample clouds because it is used only for visualization purposes, so to reduce mem consumption and enhance visualization
    // pcl::VoxelGrid<PointType_> sor;
    // sor.setInputCloud (cloud.makeShared());
    // sor.setLeafSize (0.05f, 0.05f, 0.05f);
    // sor.filter (cloud);
    _df->cloud = cloud.makeShared();
    LogManager::get()->saveTimeMark("dataEnd");
    LogManager::get()->message("time_data", std::to_string(LogManager::get()->measureTimeBetweenMarks("dataEnd", "dataInit")));

    // Detect and compute descriptors
    LogManager::get()->saveTimeMark("featureInit");
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kpts;
    cv::Mat leftGrayUndistort;

    cv::cvtColor(_df->left, leftGrayUndistort, CV_BGR2GRAY);
    mFeatureDetector->detectAndCompute(leftGrayUndistort, cv::Mat(), kpts, descriptors);
    if (kpts.size() < 8)
    {
        this->error("MAIN_APPLICATION", "Error, less than 8 descriptors in the current image. Skipping image");
        return false;
    }

    // Create feature cloud.
    _df->featureCloud = pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
    for (unsigned k = 0; k < kpts.size(); k++)
    {
        cv::Point3f point;
        if (((rgbd::StereoCameraVirtual *)mCamera)->colorPixelToPoint(kpts[k].pt, point))
        { // Using coordinates of distorted points to match depth
            float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            // if (!std::isnan(point.x) && dist > 0.25 && dist < 6.0) { // 666 min and max dist?
            PointType_ pointpcl;
            pointpcl.x = point.x;
            pointpcl.y = point.y;
            pointpcl.z = point.z;
            pointpcl.r = 255;
            pointpcl.g = 0;
            pointpcl.b = 0;
            _df->featureCloud->push_back(pointpcl);
            _df->featureDescriptors.push_back(descriptors.row(k)); // 666 TODO: filter a bit?
            _df->featureProjections.push_back(kpts[k].pt);         //  Store undistorted points
            //}
        }
    }
    LogManager::get()->saveTimeMark("featureEnd");
    LogManager::get()->message("time_feature", std::to_string(LogManager::get()->measureTimeBetweenMarks("featureEnd", "featureInit")));

    // Filling new dataframe
    _df->orientation = Eigen::Matrix3f::Identity();
    _df->position = Eigen::Vector3f::Zero();
    _df->id = mDfCounter;
    mDfCounter++; // TODO: Database info?
    return true;
}
//-----------------------------------------------------------------------------------------------------------------
bool BarricSlam::dataAcquisition_fake(std::shared_ptr<rgbd::DataFrame<PointType_>> _df)
{
    // Getting data from camera
    LogManager::get()->saveTimeMark("dataInit");
    mCamera->grab();
    cv::Mat left, right;
    // mCamera->rgb(left, right);
    // _df->left = left.clone();   /// 666 REMOVE
    // if (left.rows == 0)
    //     return false;

    _df->left = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
    left = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::Mat depth;
    mCamera->depth(depth);

    pcl::PointCloud<PointType_> cloud;
    mCamera->cloud(cloud);
    _df->cloud = cloud.makeShared();
    mCamera->leftCalibration(_df->intrinsic, _df->coefficients);
    LogManager::get()->saveTimeMark("dataEnd");
    LogManager::get()->message("time_data", std::to_string(LogManager::get()->measureTimeBetweenMarks("dataEnd", "dataInit")));

    // Detect and compute descriptors
    LogManager::get()->saveTimeMark("featureInit");
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kpts;
    cv::Mat leftGray, leftGrayUndistort;

    _df->id = mStepCounter;
    cv::Mat r = (cv::Mat_<float>(3, 3) << 1.0f, 0.0f, 0.0f,
                 0.0f, 1.0f, 0.0f,
                 0.0f, 0.0f, 1.0f);
    cv::Mat t = (cv::Mat_<float>(3, 1) << -0.3 + (mStepCounter + (mStepCounter % 2 == 0 ? -1 : 0)) * 0.01f,
                 mStepCounter % 2 == 0 ? -0.1f : 0.1f,
                 0.0f);
    // std::cout<<"True pose of " << mStepCounter << ": " << t.t() << std::endl;
    _df->position[0] = mStepCounter * 0.01f;
    _df->pose(0, 3) = mStepCounter * 0.01f;
    mStepCounter++;
    std::vector<cv::Point3f> points;
    float zDist = 3.0;
    float incX = 0.12;
    float incY = 0.08;
    for (int i = -5; i < 9; i++)
    {
        for (int j = -5; j < 9; j++)
        {
            cv::Point3f p1(i * incX, j * incY, zDist);
            points.push_back(p1);
        }
    }

    std::cout << "Number of real points is " << points.size() << std::endl;

    std::vector<cv::Point2f> points2d;
    for (unsigned i = 0; i < points.size(); i++)
    {
        cv::Mat upoint = (cv::Mat_<float>(3, 1) << points[i].x, points[i].y, points[i].z);
        upoint = r.inv() * (upoint - t);

        cv::Point2f proj(
            _df->intrinsic.at<float>(0, 2) + _df->intrinsic.at<float>(0, 0) * upoint.at<float>(0) / upoint.at<float>(2),
            _df->intrinsic.at<float>(1, 2) + _df->intrinsic.at<float>(1, 1) * upoint.at<float>(1) / upoint.at<float>(2));
        if (proj.x > 0 && proj.x < 640 && proj.y > 0 && proj.y < 480)
            points2d.push_back(proj);
        //std::cout << proj << std::endl;
    }
    // cv::projectPoints(points, r, t, _df->intrinsic, _df->coefficients, points2d);

    _df->cloud = pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
    _df->featureCloud = pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());

    int outlierCounter = 0;
    for (unsigned i = 0; i < points2d.size(); i++)
    {
        if (((double)rand()) / RAND_MAX < (float)mConfigFile["data_fake"]["remove_ratio"]) // Removing  20% of points
            continue;

        if (points2d[i].x > 0 && points2d[i].y > 0 &&
            points2d[i].x < 640 && points2d[i].y < 480)
        {
            PointType_ pointPcl;
            pointPcl.x = points[i].x + ((double)rand()) / RAND_MAX * 0.01 - t.at<float>(0);
            pointPcl.y = points[i].y + ((double)rand()) / RAND_MAX * 0.01 - t.at<float>(1);
            pointPcl.z = points[i].z + ((double)rand()) / RAND_MAX * 0.01 - t.at<float>(2);
            _df->featureCloud->push_back(pointPcl);
            _df->featureProjections.push_back(points2d[i]);

            cv::Mat descriptors;
            if (((double)rand()) / RAND_MAX < (float)mConfigFile["data_fake"]["outlier_ratio"])
            { // Outlier
                outlierCounter++;
                descriptors = cv::Mat::ones(1, 32, CV_8U) * ((int)((double)rand()) / RAND_MAX * points.size());
            }
            else
            {
                descriptors = cv::Mat::ones(1, 32, CV_8U) * i;
            }
            // std::cout << descriptors << std::endl;
            _df->featureDescriptors.push_back(descriptors); // 666 TODO: filter a bit?
        }
    }

    this->warning("outliers", "Added " + std::to_string(outlierCounter) + " outliers");

    return true;
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
bool BarricSlam::optimization()
{
}
//-----------------------------------------------------------------------------------------------------------------

// void BarricSlam::semanticLabeling(std::shared_ptr<rgbd::DataFrame<PointType_>> _df)
// {
//     if (mConfigFile.contains("dnn"))
//     {
//         cv::Mat display = _df->left.clone();
//         auto detections = mObjectDetector.detect(display);
//         std::cout << "Preseg" << _df->cloud->width << ", " << _df->cloud->height << "," << _df->cloud->is_dense << std::endl;

//         for (auto &detection : detections)
//         {
//             cv::Rect rec(detection[2], detection[3], detection[4] - detection[2], detection[5] - detection[3]);
//             cv::rectangle(display, rec, cv::Scalar(detection[0] == 0 ? 255 : 0, detection[0] == 1 ? 255 : 0, detection[0] == 2 ? 255 : 0), 3);
//             // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
//             const float score = detection[1];
//             if (score >= (double)mConfigFile["dnn"]["min_score"])
//             {
//                 std::cout << "Detection score " + std::to_string(score) << std::endl;
//                 int x0 = detection[2];
//                 int y0 = detection[3];
//                 int x1 = detection[4];
//                 int y1 = detection[5];

//                 x0 = x0 < 0 ? 0 : x0;
//                 y0 = y0 < 0 ? 0 : y0;
//                 x1 = x1 > _df->left.cols ? _df->left.cols : x1;
//                 y1 = y1 > _df->left.rows ? _df->left.rows : y1;

//                 pcl::PointCloud<PointType_> cloud, bbCloud;
//                 pcl::transformPointCloudWithNormals(*_df->cloud, cloud, _df->pose);
//                 float minX = 9999, minY = 9999, minZ = 9999, maxX = -9999, maxY = -9999, maxZ = -9999;
//                 for (unsigned x = x0; x < x1; x++)
//                 {
//                     for (unsigned y = y0; y < y1; y++)
//                     {
//                         int label = detection[0];
//                         auto p = cloud.at(x, y);
//                         if (std::isnan(p.x))
//                             continue;

//                         bbCloud.push_back(p);

//                         cloud.at(x, y).probs[label] = score;
//                         cloud.at(x, y).r = (detection[0] == 0 ? 255 : 0);
//                         cloud.at(x, y).g = (detection[0] == 1 ? 255 : 0);
//                         cloud.at(x, y).b = (detection[0] == 2 ? 255 : 0);

//                         minX = std::min(minX, p.x);
//                         maxX = std::max(maxX, p.x);
//                         minY = std::min(minY, p.y);
//                         maxY = std::max(maxY, p.y);
//                         minZ = std::min(minZ, p.z);
//                         maxZ = std::max(maxZ, p.z);
//                     }
//                 }

//                 bool addCandidate = true;
//                 for (auto candidate : mCandidates)
//                 {
//                     double minXOverlap = std::max(minX, candidate[0]);
//                     double minYOverlap = std::max(minY, candidate[1]);
//                     double minZOverlap = std::max(minZ, candidate[2]);
//                     double maxXOverlap = std::min(maxX, candidate[3]);
//                     double maxYOverlap = std::min(maxY, candidate[4]);
//                     double maxZOverlap = std::min(maxZ, candidate[5]);

//                     double vol1 = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
//                     double vol2 = (candidate[3] - candidate[0]) * (candidate[4] - candidate[1]) * (candidate[5] - candidate[2]);
//                     double volInter = (maxXOverlap - minXOverlap) * (maxYOverlap - minYOverlap) * (maxZOverlap - minZOverlap);

//                     double scoreOverlap = volInter / (vol1 + vol2 - volInter);

//                     if (scoreOverlap > 0.1)
//                     {
//                         addCandidate = false;
//                         break;
//                     }
//                 }

//                 // pcl::PCA<PointType_> pca;
//                 // pca.setInputCloud(bbCloud.makeShared());

//                 // Eigen::Vector4f mean = pca.getMean();
//                 // Eigen::Matrix3f rot = pca.getEigenVector();
//                 // Eigen::Quaternionf qRot(rot);
//                 // Eigen::Vector3f shape = pca.getEigenValues()*3;

//                 if (addCandidate)
//                 {
//                     std::stringstream text;
//                     text << "Added new candidate, type:" << detection[0] << ". ";
//                     this->error("MAIN_APPLICATION", text.str());
//                     mCandidates.push_back({minX, minY, minZ, maxX, maxY, maxZ, score});
//                     mCandidatesColor.push_back(detection[0]);
//                     // Draw cube
//                     std::string candidateName = "cube_" + std::to_string(mCandidates.size());
//                     if (mVisualization->mViewer != nullptr)
//                     {
//                         this->error("MAIN_APPLICATION", "DRAWED CANDIDATE");
//                         mVisualization->mViewer->addCube(minX,
//                                                          maxX,
//                                                          minY,
//                                                          maxY,
//                                                          minZ,
//                                                          maxZ,
//                                                          detection[0] == 2 ? 1.0 : 0.0,
//                                                          detection[0] == 1 ? 1.0 : 0.0,
//                                                          detection[0] == 0 ? 1.0 : 0.0,
//                                                          candidateName);

//                         // mVisualization->mViewer->addCube(mean.block<3,1>(0,0), qRot, shape[0], shape[1], shape[2], candidateName);
//                         // mVisualization->mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0,candidateName);
//                         //mVisualization->mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3,candidateName);
//                         //mVisualization->mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,candidateName);

//                         // mVisualization->mViewer->addPointCloud<PointType_>(cloud.makeShared(), "candidate" + std::to_string(mCandidates.size()));
//                         // mVisualization->mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "candidate" + std::to_string(mCandidates.size()));
//                     }
//                     ///mVisualization->pause();
//                 }
//             }
//         }
//         cv::imshow("detections", display);
//         //---
//     }
}