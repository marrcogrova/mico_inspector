//---------------------------------------------------------------------------------------------------------------------
//  MONO2RGBD
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018  Ricardo Lopez Lopez (a.k.a. ricloplop) & Pablo Ramon Soria (a.k.a. Bardo91) & Marco Montes Grova (a.k.a marrcogrova)
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

#include <ros/ros.h>
#include "mono2rgbd.h"


/* ------------------------------------------------------------------------------------------------------------------------- */
bool Mono2RGBD::init(int _argc, char **_argv) {

  mico::LogManager::init("inspector_slam" + std::to_string(time(NULL)));
 
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);  
  featurePub_     = it.advertise("/inspector/debug_image", 1);
  posePub_        = nh.advertise<geometry_msgs::PoseStamped>("/inspector/pose",1);
  cloudPub_       = nh.advertise<pcl::PointCloud<PointType_>>("/inspector/panelsCloud",1);
  mapPub_         = nh.advertise<pcl::PointCloud<PointType_>>("/inspector/map", 1);

  // Configure data to visualize in RVIZ
  markersVO_ = nh.advertise<visualization_msgs::Marker>("/inspector/lines_odometry", 1);
  lineStrip_.ns = "vo_trajectory";
  lineStrip_.action = visualization_msgs::Marker::ADD;
  lineStrip_.pose.orientation.w = 1.0;
  lineStrip_.id = 1;
  lineStrip_.type = visualization_msgs::Marker::POINTS;
  lineStrip_.scale.x = 0.1;
  lineStrip_.color.r = 1.0;
  lineStrip_.color.a = 1.0; //alpha

  markersEKF_ = nh.advertise<visualization_msgs::Marker>("/inspector/lines_EKF", 1);
  EKFlineStrip_.ns = "EKF_trajectory";
  EKFlineStrip_.action = visualization_msgs::Marker::ADD;
  EKFlineStrip_.pose.orientation.w = 1.0;
  EKFlineStrip_.id = 1;
  EKFlineStrip_.type = visualization_msgs::Marker::LINE_STRIP;
  EKFlineStrip_.scale.x = 0.1;
  EKFlineStrip_.color.b = 1.0;
  EKFlineStrip_.color.a = 1.0; //alpha

  // Load arguments
  std::string imageTopic,GPSTopic,imuTopic,infoTopic,fileConfig;
  nh.getParam("image_topic",imageTopic);
  nh.getParam("info_topic",infoTopic);
  nh.getParam("img_is_rect",imgIsRaw_);
  nh.getParam("GPS_topic",GPSTopic);
  nh.getParam("imu_topic",imuTopic);
  nh.getParam("file_config",fileConfig);
  nh.getParam("save_logs",saveLogs_);
  nh.getParam("publish_pointCloud",publishPointCloud_);
  
  // Load JSON config parameters
  std::ifstream file(fileConfig);
  if (!file.is_open()) {
      std::cout << "Cannot open file." << std::endl;
      return false;
  }
  cjson::Json configFile;
  if (!configFile.parse(file)) {
      std::cout << "Cannot parse config file." << std::endl;
      return false;
  }
  odometry_ = new mico::OdometryPhotogrammetry<PointType_, mico::DebugLevels::Debug>();
  if (!odometry_->init(configFile["registrator_params"])) {
      std::cout << "Error initializing odometry parameters" << std::endl;
      return false;
  }
  // database_ = new mico::DatabaseCF<PointType_, mico::DebugLevels::Debug>();
  database_ = new mico::DatabaseMarkI<PointType_, mico::DebugLevels::Debug>();
  if (configFile.contains("database")) {
      if (!database_->init(configFile["database"])) {
          std::cout << "FAILED INIT OF VOCABULARY in database" << std::endl;
      }
  }
  if (configFile.contains("LoopClosure")) {
      std::cout << "Initializating vocabulary..." << std::endl;
      loopDetector_ = new mico::LoopClosureDetectorDorian<>;
      loopDetector_->init(configFile["LoopClosure"]);
  }
  if (configFile.contains("Extended_Kalman_Filter")) {
      if (!ekf.init(configFile["Extended_Kalman_Filter"])) {
        std::cout << "Error inicializating EKF parameters" << std::endl;
        return false;
      }
  }

  // Logs files TUM
  if (saveLogs_){
  	logVO_.open("/home/marrcogrova/Documents/INSPECTOR/data/log_data/VO_" + std::to_string(time(NULL)) + ".tum");
  	logEKF_.open("/home/marrcogrova/Documents/INSPECTOR/data/log_data/EKF_" + std::to_string(time(NULL)) + ".tum");
  	std::cout << "Opening logs files ..." <<std::endl;
  }

  ORBdetector_ = cv::ORB::create(2000);//,1.2f,8,31,0,2, cv::ORB::FAST_SCORE,31,30);

  imageSub_ = nh.subscribe<sensor_msgs::Image>(imageTopic, 1, [&](const sensor_msgs::Image::ConstPtr& _msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(_msg);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (intrinsics_.empty() || coefficients_.empty()){
      this->error("MONO2RGBD", "Error obtaining camera parameters. Skipping frame");
      return;
    }

    imageCallback(cv_ptr->image , abs(altitude_ - firstAltitude_));
  });
  
  GPSSub_   = nh.subscribe<sensor_msgs::NavSatFix>(GPSTopic,   1, [&](const sensor_msgs::NavSatFix::ConstPtr& _msg){
    altitude_ = _msg->altitude;
    if(!savedFirstAltitude_){
      firstAltitude_ = _msg->altitude;
      savedFirstAltitude_ = true;
    }
  });
  
  infoSub_  = nh.subscribe<sensor_msgs::CameraInfo>(infoTopic,  1, [&](const sensor_msgs::CameraInfo::ConstPtr& _msg){
    // intrinsics_  = cv::Mat(3, 3, CV_64FC1, (void *) _msg->K.data()).clone();
    // coefficients_= cv::Mat(5, 1, CV_64FC1, (void *) _msg->D.data()).clone();
    intrinsics_ = (cv::Mat_<float>(3,3) << 617.8070678710938, 0.0, 326.96380615234375, 0.0, 617.9169311523438, 241.34239196777344, 0.0, 0.0, 1.0);
    coefficients_ = (cv::Mat_<float>(5,1) << 0.000, 0.000, 0.000, 0.000, 0.000);

  });
  
  imuSub_   = nh.subscribe<sensor_msgs::Imu>(imuTopic , 1 , [&](const sensor_msgs::Imu::ConstPtr &_msg){
    lastOrientation_ = Eigen::Quaternionf(_msg->orientation.w, _msg->orientation.x, _msg->orientation.y, _msg->orientation.z);
    ImuAcceleration_ = Eigen::Vector3d(_msg->linear_acceleration.x,_msg->linear_acceleration.y,_msg->linear_acceleration.z);
  });

  return true;
}


/* --------------------------------------------------x----------------------------------------------------------------------- */
bool Mono2RGBD::step() {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return true;
}

/* ------------------------------------------------------------------------------------------------------------------------- */
Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q)
{
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::fabs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
    angles[0] = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

/* ------------------------------------------------------------------------------------------------------------------------- */
void Mono2RGBD::imageCallback(cv::Mat _image, float _altitude){
  cv::Mat img_rect;
  if(!imgIsRaw_){
    cv::cvtColor(_image,img_rect, cv::COLOR_BGR2GRAY);  
  }else{
    cv::undistort(_image,img_rect,intrinsics_,coefficients_);
    cv::cvtColor(img_rect,img_rect, cv::COLOR_BGR2GRAY);
  }

  // Obtain keypoints and descriptors ORB
  cv::Mat img_features;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  ORBdetector_->detectAndCompute(img_rect,cv::Mat(),keypoints,descriptors);

  drawKeypoints(img_rect, keypoints, img_features);
  if(keypoints.size() < 10 ){
    this->error("MONO2RGBD", "Less than 10 features in current frame. Skipping frame");
    return;
  }

  // Publish ROS image topic
  sensor_msgs::ImagePtr img_msg;
  img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, img_features).toImageMsg();
  img_msg->header.frame_id = "camera_link";
  img_msg->header.stamp=ros::Time::now();
  featurePub_.publish(img_msg);
  
  // bad slam inicialization
  if (_altitude < initSLAMAltitude_ ){
    std::cout << "Actual altitude " << _altitude <<" m , SLAM inicializate when " << initSLAMAltitude_ << "\n";
    return;
  }

  // Create pointCloud
  pcl::PointCloud<PointType_>::Ptr monoCloud(new pcl::PointCloud<PointType_>);
  monoCloud->header.frame_id = "map";
  monoCloud->width    = keypoints.size();
  monoCloud->height   = 1;
  monoCloud->points.resize(monoCloud->width * monoCloud->width);
  monoCloud->points.clear(); // To avoid add camera point to pointcloud

  // std::cout << " Altitude used to obtain pointcloud: " << _altitude << std::endl;
  if(!ObtainPointCloud(_altitude  , keypoints , monoCloud)){
    return;
  }
  // std::cout << "Pointcloud obtained size: " << monoCloud->size() << std::endl;

  // pcl::io::savePCDFileASCII("test_pcd.pcd", *monoCloud);
  // std::cerr << "Saved " << monoCloud->points.size () << " data points to test_pcd.pcd." << std::endl;

  // for (std::size_t i = 0; i < monoCloud->points.size (); ++i)
  //   std::cerr << "    " << monoCloud->points[i].x << " " << monoCloud->points[i].y << " " << monoCloud->points[i].z << std::endl;

  // pcl::visualization::PCLVisualizer viewer ("viewer simple");
  // pcl::visualization::PointCloudColorHandlerCustom<PointType_>monoCloud_model_color_handler (monoCloud, 255, 0, 0);
  // viewer.addPointCloud (monoCloud, monoCloud_model_color_handler, "monoCloud");
  // while (!viewer.wasStopped()){
  //     viewer.spinOnce ();
  // }
  
  cloudPub_.publish(monoCloud);

  // Update dataFrame
  std::shared_ptr<mico::DataFrame<PointType_>> df(new mico::DataFrame<PointType_>);
  df->left = img_rect;
  df->intrinsic = intrinsics_;
  df->coefficients = coefficients_;
  df->featureCloud = monoCloud;
  df->featureDescriptors = descriptors;
  df->id = dfCounter_;
  // if (savefirstPosition_ && savefirstOrient_ ){
  //   df->pose=firstPose_;
  // }
  df->featureProjections.resize(keypoints.size());
  for(unsigned i = 0; i < keypoints.size(); i++){
    df->featureProjections[i] = keypoints[i].pt;
  }
  dfCounter_++;   // TODO: Database info?
  
  // Compute phothogrametry odometry
  // if (odometry_->computeOdometry(database_->lastCluster(), df)) {
  if (odometry_->computeOdometry(database_->mLastClusterframe, df)) {
    
    bool is_newCluster = database_->addDataframe(df);
  
    if(is_newCluster){
      Eigen::Matrix4f AxisRot=Eigen::Matrix4f::Identity();
      Eigen::Matrix3f rot;
      rot << 0,-1,0, 1,0,0, 0,0,1;  // 90º positive in Z
      AxisRot.block<3,3>(0,0) = rot;

      // OdomPose_ = firstPose_ * AxisRot *  database_->lastCluster()->getPose();
      OdomPose_ = firstPose_ * AxisRot *  database_->mLastClusterframe->getPose();

      geometry_msgs::Point p;
      p.x = OdomPose_(0,3);
      p.y = OdomPose_(1,3);
      p.z = OdomPose_(2,3);
      lineStrip_.points.push_back(p);
      lineStrip_.header.frame_id = "map"; 
      lineStrip_.header.stamp=ros::Time::now();
      markersVO_.publish(lineStrip_);

      // Save Visual Odometry log
	  if (saveLogs_){
      // timestamp tx ty tz qx qy qz qw (all in float)
      double timeStamp=ros::Time::now().toSec();
		  Eigen::Quaternionf q(OdomPose_.block<3,3>(0,0));
		  logVO_ << std::to_string( timeStamp ) + " " + std::to_string( float(OdomPose_(0,3)) ) + " " + std::to_string( float(OdomPose_(1,3)) ) + " " + std::to_string( float(OdomPose_(2,3)) ) 
			  	+ " " + std::to_string( float(q.x()) ) + " " + std::to_string( float(q.y()) ) + " " + std::to_string( float(q.z()) ) + " " + std::to_string( float(q.w()) )
				  + "\n";
		  logVO_.flush();
	  }

    // New observation EKF
    Eigen::Matrix<double,6,1> Zk = Eigen::Matrix<double,6,1>::Identity();
    Zk << double(OdomPose_(0,3)),double(OdomPose_(1,3)),double(OdomPose_(2,3)), // VO position
          ImuAcceleration_(0), ImuAcceleration_(1), ImuAcceleration_(2); // IMU data                                    
    
    ekf.stepEKF(Zk,double(0.3));

    // Obtain and print EKF estimate state
    Eigen::Matrix<double,12,1> Xk = ekf.state();

    // Publish ekf in RVIZ
    geometry_msgs::Point p_ekf;
    p_ekf.x = Xk(0,0);
    p_ekf.y = Xk(1,0);
    p_ekf.z = Xk(2,0);
    EKFlineStrip_.points.push_back(p_ekf);
    EKFlineStrip_.header.frame_id = "map"; 
    EKFlineStrip_.header.stamp=ros::Time::now();
    markersEKF_.publish(EKFlineStrip_);

    // Save EKF log
	  if (saveLogs_){
		logEKF_ << std::to_string( (double)ros::Time::now().toSec() ) + " " + std::to_string( float(Xk(0,0)) ) + " " + std::to_string( float(Xk(1,0)) ) + " " + std::to_string( float(Xk(2,0)) ) + "\n";
		logEKF_.flush();
	  }
 
    // Mapping module
	  if (publishPointCloud_){
      //auto copyDic = database_->Dictionary();
      auto copyDic = database_->mWordDictionary;
      pcl::PointCloud<PointType_> map;
      map.points.resize(copyDic.size());
      for(auto &word: copyDic){
        PointType_ p;
        p.x = word.second->point[0];
        p.y = word.second->point[1];
        p.z = word.second->point[2];
        map[word.second->id] = p;
      }
      Eigen::Matrix4f MapT=Eigen::Matrix4f::Identity();
      MapT.block<3,3>(0,0) = rot;
      MapT = firstPose_ * MapT;
      pcl::transformPointCloud(map,map, MapT);

      map.header.frame_id = "map";
      mapPub_.publish(map.makeShared());
	  }

    // Create new vocabulary
    // if (database_->lastCluster()->id > 1000){ // number of Cf used to create vocabulary
    if (database_->mLastClusterframe->id > 1000){
      // std::cout << "ID last Cf used to create vocabulary :  " << database_->lastCluster()->id << std::endl; 
      std::cout << "ID last Cf used to create vocabulary :  " << database_->mLastClusterframe->id << std::endl; 
      if (!createVocabulary()){
        return;
      }
    }

    }

    // if(is_newCluster && loopDetector_ != nullptr){
    //   auto result = loopDetector_->appendCluster(database_->mLastClusterframe->left, database_->mLastClusterframe->id);
    //   if(result.found ){
    //     std::map<int,std::shared_ptr<mico::ClusterFrames<PointType_>>> loopClosureSubset;
    //     loopClosureSubset[database_->mLastClusterframe->id] = database_->mLastClusterframe;
    //     loopClosureSubset[result.matchId] = database_->mClusterframes[result.matchId];
    //     database_->clusterComparison(loopClosureSubset, false);
    //   }
    // }
  }

}

/* ------------------------------------------------------------------------------------------------------------------------- */
bool Mono2RGBD::ObtainPointCloud(float _altitude ,std::vector<cv::KeyPoint> _keypoints, pcl::PointCloud<PointType_>::Ptr _OutputPointCloud){
  float fx = intrinsics_.at<float>(0,0);
  float fy = intrinsics_.at<float>(1,1);
  float cx = intrinsics_.at<float>(0,2);
  float cy = intrinsics_.at<float>(1,2);
  
  // std::cout << "Camera parameters -> \n fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << " \n ";
 
  for(unsigned ii = 0; ii < _keypoints.size(); ii++){
    
    PointType_ p;
    p.z = - (_altitude) ;
    p.x = -( ( _keypoints[ii].pt.x - cx )/fx ) * (-p.z);
    p.y =  ( ( _keypoints[ii].pt.y - cy )/fy ) * (-p.z);

    p.r = 0; p.g = 255; p.b = 0;
    
    // std::cout << "px " << p.x << " py " << p.y << " pz " << p.z << "\n";

    _OutputPointCloud->points.push_back(p);
  }
  
  return true;
  // if(!OutputPointCloud->is_dense){
  //   this->error("MONO2RGBD", "PointCloud contain some Nan or Inf. Skipping frame");
  //   return false;
  // }
}

/* ------------------------------------------------------------------------------------------------------------------------- */
bool Mono2RGBD::createVocabulary(){
  const int K = 6; // Values from article Bags of Binary Words for Fast ...
  const int L = 4;

  OrbVocabulary voc(K, L);

  auto clusters = database_->mClusterframes;
  std::vector<std::vector<cv::Mat>> allFeatures;

  for(unsigned i=0; i<clusters.size(); i++){
    allFeatures.push_back(std::vector<cv::Mat>());
  
    for(int r=0; r < (clusters[i]->featureDescriptors.rows); r++){
      allFeatures[i].push_back(clusters[i]->featureDescriptors.row(r));
    }
  }

  voc.create(allFeatures);
  voc.save("/chome/marrcogrova/programming/slam/inspector_ws/src/mono2rgbd/cfg/vocabulary_dbow2_solarpanels_orb_K" + std::to_string(K) + "L" + std::to_string(L) + ".xml");

  std::cout << "Vocabulary saved in 'vocabulary_dbow2_solarpanels_orb_K" + std::to_string(K) + "L" + std::to_string(L) + ".xml'" << std::endl;

  return true;
}

// template <typename dataType_>
// dataType_ altitudeFilter(std::vector<dataType_> _VecAltitude){
//   dataType sumData;
//   for (auto dataAlt : _VecAltitude ){
//     sumData += dataAlt;
//   }
//   return sumData/_VecAltitude.size();
// }