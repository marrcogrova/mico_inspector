///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#include <mico/base/cjson/json.h>
#include <mico/base/StereoCamera.h>
#include <opencv2/imgproc.hpp>
#include <mico/base/utils/LogManager.h>

#include <pcl/io/pcd_io.h>
#ifdef USE_DARKNET
    #include <darknet_cpp/WrapperDarknet.h>
#endif

#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif
#include <mico/base/map3d/ClusterFrames.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/xfeatures2d.hpp>
#include <thread>


#include <mico/base/utils/Gui.h>
#include <mico/base/StereoCameras/StereoCameraRealSense.h>
#include <pcl/io/pcd_io.h>
#include <mico/base/utils/Graph2d.h>
// csv
#include <iostream>
#include <fstream>

#ifdef _WIN32
#include <Windows.h>
        inline void do_mkdir(std::string _filename) {
            CreateDirectory(_filename.c_str(), NULL);
        }
#elif __linux__ || __APPLE__
#include <sys/stat.h>
#include <sys/types.h>
        inline void do_mkdir(std::string _filename) {
            mkdir(_filename.c_str(), 0700);
        }
#endif


class MainApplication {
public:
    typedef pcl::PointXYZRGBNormal PointType_;
	bool init(int _argc, char **_argv);
	void deinit();
	bool step();

private:
	bool initCamera(const cjson::Json &_cameraConfig);
 
    bool updateMap(cv::Mat &_rgb, cv::Mat &_depth, pcl::PointCloud<PointType_> &_cloud);


private:
	cjson::Json mConfigFile;
	mico::StereoCamera *mCamera;
    std::vector<std::vector<float>> mCandidates;
    std::vector<int> mCandidatesColor;

	unsigned mStepCounter = 0;

	int mdfCounter = 0;
bool mSave = false;
	std::thread inputThread;

};

int main(int _argc, char** _argv) {
	MainApplication app;
	
	if (!app.init(_argc, _argv)) {
		return -1;
	}

	// Loop.
	bool condition = true;
	while(condition){
		condition = app.step();
	}

	app.deinit();

	return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


using namespace pcl;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::init(int _argc, char ** _argv) {
	// Init logging system.
    mico::LogManager::init("semanticGrasping");

	// Load arguments
	if (_argc != 2) {
        mico::LogManager::get()->error("Bad input arguments. Only a ath to a JSON file is needed.", true);
		return false;
	}

	std::ifstream file(_argv[1]);
	if (!file.is_open()) {
        mico::LogManager::get()->error("Cannot open given file.", true);
		return false;
	}

	if (!mConfigFile.parse(file)) {
        mico::LogManager::get()->error("Cannot parse config file.", true);
		return false;
	}

	// Init camera.
	if (initCamera(mConfigFile["camera"])) {
        mico::LogManager::get()->status("Initialized camera.", true);
	}else{
        mico::LogManager::get()->error("Error configuring camera.", true);
		return false;
	}


    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void MainApplication::deinit() {
    mico::LogManager::get()->status("Ending application politely. Press \"space\" key to end.");
    //mico::Gui::end();
	mico::LogManager::close();
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::step() {
    mico::LogManager::get()->status("Step "+to_string(mStepCounter++), true);

    mico::LogManager::get()->saveTimeMark("initDataCapture");
    mCamera->grab();

    cv::Mat left, right;
    mCamera->rgb(left, right);


    cv::Mat depth;
    mCamera->depth(depth);
    mico::LogManager::get()->saveTimeMark("endDataCapture");

    // Get cloud
    PointCloud<PointType_> cloud;
    ((mico::StereoCameraVirtual *)mCamera)->cloud(cloud);

    if(cloud.size() == 0){
        mico::LogManager::get()->status("Input cloud is empty. Closing application.", true);
        return true;
    }
    mico::LogManager::get()->status("Captured point cloud with " + to_string(cloud.size()) + " points.", true);

    if(!updateMap(left, depth, cloud)){
        mico::LogManager::get()->error("Failed map update", true);
    }else{
        mico::LogManager::get()->status("Map Updated.", true);
    }


    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initCamera(const cjson::Json & _cameraConfig) {
	if (_cameraConfig["type"] == "virtual") {
		mCamera = mico::StereoCamera::create(mico::StereoCamera::eModel::Virtual);
	}
	else if (_cameraConfig["type"] == "zed") {
		mCamera = mico::StereoCamera::create(mico::StereoCamera::eModel::Zed);
	}
    else if (_cameraConfig["type"] == "intel") {
        mCamera = mico::StereoCamera::create(mico::StereoCamera::eModel::RealSense);
    }
    else if (_cameraConfig["type"] == "kinect") {
        mCamera = mico::StereoCamera::create(mico::StereoCamera::eModel::Kinect);
    }
	else {
        mico::LogManager::get()->error("Bad camera type.", true);
		return false;
	}
	mCamera->init(_cameraConfig["config"]);
	

inputThread = std::thread([&](){
int a;
for(;;){
	std::cin >> a;
	mSave = true;

}	
});
    return true;
}


//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::updateMap(cv::Mat &_rgb, cv::Mat &_depth, pcl::PointCloud<PointType_> &_cloud) {
    mico::LogManager::get()->status("Preparing data.", true);
    std::shared_ptr<mico::Dataframe<PointType_>> df(new mico::Dataframe<PointType_>);
    df->cloud = _cloud.makeShared();
    df->left = _rgb;
    df->depth = _depth;

    mCamera->leftCalibration(df->intrinsic, df->coefficients);
    df->orientation = Eigen::Matrix3f::Identity();
    df->position = Eigen::Vector3f::Zero();

    mico::LogManager::get()->saveTimeMark("initFeatureComp");
    // Compute features.
    // auto featureDetector = cv::xfeatures2d::SIFT::create();
    auto featureDetector = cv::ORB::create();
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kpts;
    cv::Mat leftGray;
    cv::cvtColor(df->left, leftGray, CV_BGR2GRAY);
    mico::LogManager::get()->status("Detecting features.", true);
    featureDetector->detectAndCompute(leftGray, cv::Mat(),kpts, descriptors);
    mico::LogManager::get()->status("Detected features.", true);
    cv::Mat display;
    cv::drawKeypoints(df->left, kpts, display);
    if (kpts.size() < 8) {
       std::cout << "Error, less than 8 descriptors in the current image. Skipping image" << std::endl;
       return false;
    }
    mico::LogManager::get()->saveTimeMark("endFeatureComp");

    mico::LogManager::get()->saveTimeMark("initFeatureCloud");
    // Create feature cloud.
    df->featureCloud = pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
    for(unsigned k = 0; k < kpts.size(); k++) {
       auto correspondingPoint = df->cloud->at(kpts[k].pt.x, kpts[k].pt.y);
       cv::Point3f point(correspondingPoint.x, correspondingPoint.y, correspondingPoint.z);
       //if(((mico::StereoCameraVirtual*)mCamera)->colorPixelToPoint(kpts[k].pt, point)){
           if(!std::isnan(point.x)){
               PointType_ pointpcl;
               pointpcl.x = point.x;
               pointpcl.y = point.y;
               pointpcl.z = point.z;
               df->featureCloud->push_back(pointpcl);
               df->featureProjections.push_back(kpts[k].pt);
               df->featureDescriptors.push_back(descriptors.row(k));    // 666 TODO: filter a bit?
           }
       //}
    }
    mico::LogManager::get()->status("Created feature cloud.", true);
    mico::LogManager::get()->saveTimeMark("endFeatureCloud");

    // Set id
    df->id = mdfCounter;

    mico::LogManager::get()->saveTimeMark("initAdddf");
    // Append keyframe1

    // Add probs to map
    mico::LogManager::get()->saveTimeMark("initObjectDetection");

    mico::LogManager::get()->saveTimeMark("endObjectDetection");
    cv::imshow("left", _rgb);
    cv::waitKey(3);
    if (mSave){
        std::vector<cv::Point3f> points;
        cv::Mat descriptors;
        for (unsigned i = 0; i < df->featureCloud->size(); i++){
            cv::Point3f p3d(df->featureCloud->at(i).x,
                            df->featureCloud->at(i).y,
                            df->featureCloud->at(i).z);
            points.push_back(p3d);
            descriptors.push_back(df->featureDescriptors.row(i));

            //for(auto &observation:mp->mObservations){
            //	cv::Mat desc =
            //	break;
            //}
        }

        cv::FileStorage fs("crawler_model" + std::to_string(mdfCounter) + ".xml", cv::FileStorage::WRITE);
        if (!fs.isOpened())
            return false;

        fs << "points" << points;
        fs << "descriptors" << descriptors;
        fs.release();
        mdfCounter++;
        mSave = false;
}

    return true;
}







