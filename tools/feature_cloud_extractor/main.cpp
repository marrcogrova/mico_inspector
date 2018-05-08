///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#include <rgbd_tools/cjson/json.h>
#include <rgbd_tools/StereoCamera.h>
#include <opencv2/imgproc.hpp>
#include <rgbd_tools/utils/LogManager.h>

#include <pcl/io/pcd_io.h>
#ifdef USE_DARKNET
    #include <darknet_cpp/WrapperDarknet.h>
#endif

#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif
#include <rgbd_tools/map3d/ClusterFrames.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <thread>


#include <rgbd_tools/utils/Gui.h>
#include <rgbd_tools/StereoCameras/StereoCameraRealSense.h>
#include <pcl/io/pcd_io.h>
#include <rgbd_tools/utils/Graph2d.h>
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
	rgbd::StereoCamera *mCamera;
    std::vector<std::vector<float>> mCandidates;
    std::vector<int> mCandidatesColor;

	unsigned mStepCounter = 0;

	int mKfCounter = 0;
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
    LogManager::init("semanticGrasping");

	// Load arguments
	if (_argc != 2) {
        LogManager::get()->error("Bad input arguments. Only a ath to a JSON file is needed.", true);
		return false;
	}

	std::ifstream file(_argv[1]);
	if (!file.is_open()) {
        LogManager::get()->error("Cannot open given file.", true);
		return false;
	}

	if (!mConfigFile.parse(file)) {
        LogManager::get()->error("Cannot parse config file.", true);
		return false;
	}

	// Init camera.
	if (initCamera(mConfigFile["camera"])) {
        LogManager::get()->status("Initialized camera.", true);
	}else{
        LogManager::get()->error("Error configuring camera.", true);
		return false;
	}


    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void MainApplication::deinit() {
    LogManager::get()->status("Ending application politely. Press \"space\" key to end.");
    //rgbd::Gui::end();
	LogManager::close();
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::step() {
    LogManager::get()->status("Step "+to_string(mStepCounter++), true);

    LogManager::get()->saveTimeMark("initDataCapture");
    mCamera->grab();

    cv::Mat left, right;
    mCamera->rgb(left, right);


    cv::Mat depth;
    mCamera->depth(depth);
    LogManager::get()->saveTimeMark("endDataCapture");

    // Get cloud
    PointCloud<PointType_> cloud;
    ((rgbd::StereoCameraVirtual *)mCamera)->cloud(cloud);

    if(cloud.size() == 0){
        LogManager::get()->status("Input cloud is empty. Closing application.", true);
        return true;
    }
    LogManager::get()->status("Captured point cloud with " + to_string(cloud.size()) + " points.", true);

    if(!updateMap(left, depth, cloud)){
        LogManager::get()->error("Failed map update", true);
    }else{
        LogManager::get()->status("Map Updated.", true);
    }


    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initCamera(const cjson::Json & _cameraConfig) {
	if (_cameraConfig["type"] == "virtual") {
		mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Virtual);
	}
	else if (_cameraConfig["type"] == "zed") {
		mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Zed);
	}
    else if (_cameraConfig["type"] == "intel") {
        mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::RealSense);
    }
    else if (_cameraConfig["type"] == "kinect") {
        mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Kinect);
    }
	else {
        LogManager::get()->error("Bad camera type.", true);
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
    LogManager::get()->status("Preparing data.", true);
    std::shared_ptr<rgbd::DataFrame<PointType_>> kf(new rgbd::DataFrame<PointType_>);
    kf->cloud = _cloud.makeShared();
    kf->left = _rgb;
    kf->depth = _depth;

    mCamera->leftCalibration(kf->intrinsic, kf->coefficients);
    kf->orientation = Eigen::Matrix3f::Identity();
    kf->position = Eigen::Vector3f::Zero();

    LogManager::get()->saveTimeMark("initFeatureComp");
    // Compute features.
    auto featureDetector = cv::xfeatures2d::SIFT::create();
    //auto featureDetector = cv::ORB::create();
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kpts;
    cv::Mat leftGray;
    cv::cvtColor(kf->left, leftGray, CV_BGR2GRAY);
    LogManager::get()->status("Detecting features.", true);
    featureDetector->detectAndCompute(leftGray, cv::Mat(),kpts, descriptors);
    LogManager::get()->status("Detected features.", true);
    cv::Mat display;
    cv::drawKeypoints(kf->left, kpts, display);
    if (kpts.size() < 8) {
       std::cout << "Error, less than 8 descriptors in the current image. Skipping image" << std::endl;
       return false;
    }
    LogManager::get()->saveTimeMark("endFeatureComp");

    LogManager::get()->saveTimeMark("initFeatureCloud");
    // Create feature cloud.
    kf->featureCloud = pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
    for(unsigned k = 0; k < kpts.size(); k++) {
       auto correspondingPoint = kf->cloud->at(kpts[k].pt.x, kpts[k].pt.y);
       cv::Point3f point(correspondingPoint.x, correspondingPoint.y, correspondingPoint.z);
       //if(((rgbd::StereoCameraVirtual*)mCamera)->colorPixelToPoint(kpts[k].pt, point)){
           if(!std::isnan(point.x)){
               PointType_ pointpcl;
               pointpcl.x = point.x;
               pointpcl.y = point.y;
               pointpcl.z = point.z;
               kf->featureCloud->push_back(pointpcl);
               kf->featureProjections.push_back(kpts[k].pt);
               kf->featureDescriptors.push_back(descriptors.row(k));    // 666 TODO: filter a bit?
           }
       //}
    }
    LogManager::get()->status("Created feature cloud.", true);
    LogManager::get()->saveTimeMark("endFeatureCloud");

    // Set id
    kf->id = mKfCounter;

    LogManager::get()->saveTimeMark("initAddKf");
    // Append keyframe1

    // Add probs to map
    LogManager::get()->saveTimeMark("initObjectDetection");

    LogManager::get()->saveTimeMark("endObjectDetection");
cv::imshow("left", _rgb);
cv::waitKey(3);
if(mSave){

	std::vector<cv::Point3f> points;
	cv::Mat descriptors;
	for(unsigned i = 0;i  < kf->featureCloud->size(); i++){
		
		cv::Point3f p3d(kf->featureCloud->at(i).x,
				kf->featureCloud->at(i).y,
				kf->featureCloud->at(i).z);
			points.push_back(p3d);
			descriptors.push_back(kf->featureDescriptors.row(i));
		
		//for(auto &observation:mp->mObservations){
		//	cv::Mat desc =					
		//	break;
		//}
	}

	cv::FileStorage fs("crawler_model"+std::to_string(mKfCounter)+".xml", cv::FileStorage::WRITE);
	if(!fs.isOpened())
		return false;

	fs << "points" << points;
	fs << "descriptors" << descriptors;
	fs.release();
    mKfCounter++;
      mSave = false;
}

    return true;
}







