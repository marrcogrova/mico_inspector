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
  pathDataPub_    = nh.advertise<nav_msgs::Path>("/inspector/path_UAVdata", 1);
 
  std::string imageTopic,poseTopic,imuTopic,infoTopic,fileConfig;
  nh.getParam("image_topic",imageTopic);
  nh.getParam("info_topic",infoTopic);
  nh.getParam("img_is_rect",img_is_raw);
  nh.getParam("pose_topic",poseTopic);
  nh.getParam("imu_topic",imuTopic);
  nh.getParam("file_config",fileConfig);
  nh.getParam("save_logs",_save_logs);
  nh.getParam("publish_pointCloud",_publish_pointCloud);

  imageSub_ = nh.subscribe(imageTopic, 1, &Mono2RGBD::imageCb, this);
  infoSub_  = nh.subscribe(infoTopic,  1, &Mono2RGBD::infoCb , this);
  poseSub_  = nh.subscribe(poseTopic,  1, &Mono2RGBD::poseCb , this);
  imuSub_   = nh.subscribe(imuTopic,   1, &Mono2RGBD::imuCb  , this);

  markersCf = nh.advertise<visualization_msgs::Marker>("/inspector/lines_cf", 1);
  lineStrip_.ns = "cf_trajectory";
  lineStrip_.action = visualization_msgs::Marker::ADD;
  lineStrip_.pose.orientation.w = 1.0;
  lineStrip_.id = 1;
  lineStrip_.type = visualization_msgs::Marker::POINTS;
  lineStrip_.scale.x = 0.1;
  lineStrip_.color.r = 1.0;
  lineStrip_.color.a = 1.0; //alpha

  markersEKF = nh.advertise<visualization_msgs::Marker>("/inspector/lines_EKF", 1);
  EKFlineStrip_.ns = "EKF_trajectory";
  EKFlineStrip_.action = visualization_msgs::Marker::ADD;
  EKFlineStrip_.pose.orientation.w = 1.0;
  EKFlineStrip_.id = 1;
  EKFlineStrip_.type = visualization_msgs::Marker::LINE_STRIP;
  EKFlineStrip_.scale.x = 0.1;
  EKFlineStrip_.color.b = 1.0;
  EKFlineStrip_.color.a = 1.0; //alpha

  
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
  mOdometry = new mico::OdometryPhotogrametry<PointType_, mico::DebugLevels::Debug>();
  if (!mOdometry->init(configFile["registrator_params"])) {
      std::cout << "Error initializing odometry parameters" << std::endl;
      return false;
  }
  mDatabase = new mico::Database<PointType_, mico::DebugLevels::Debug>();
  if (configFile.contains("database")) {
      if (!mDatabase->init(configFile["database"])) {
          std::cout << "FAILED INIT OF VOCABULARY in database" << std::endl;
      }
  }
//   if (configFile.contains("LoopClosure")) {
//       std::cout << "Initializating vocabulary..." << std::endl;
//       mLoopDetector = new mico::LoopClosureDetectorDorian<>;
//       mLoopDetector->init(configFile["LoopClosure"]);
//   }
  if (configFile.contains("Extended_Kalman_Filter")) {
      if (!ekf.init(configFile["Extended_Kalman_Filter"])) {
        std::cout << "Error inicializating EKF parameters" << std::endl;
        return false;
      }
  } 
  
  ORBdetector_ = cv::ORB::create(3000);//,1.2f,8,31,0,2, cv::ORB::FAST_SCORE,31,30);

  // Logs files TUM
  if (_save_logs){
  	logGT_.open("/home/marrcogrova/Documents/grvc/Inspector/data/log_data/groundtruth_" + std::to_string(time(NULL)) + ".tum");
  	logVO_.open("/home/marrcogrova/Documents/grvc/Inspector/data/log_data/VO_" + std::to_string(time(NULL)) + ".tum");
  	logEKF_.open("/home/marrcogrova/Documents/grvc/Inspector/data/log_data/EKF_" + std::to_string(time(NULL)) + ".tum");
  	std::cout << "Opening logs files ..." <<std::endl;
  }
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
void Mono2RGBD::imageCb(const sensor_msgs::Image::ConstPtr& _msg){
  cv_bridge::CvImageConstPtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (intrinsics_.empty() || coefficients_.empty()){
    this->error("MONO2RGBD", "Error obtaining camera parameters. Skipping frame");
    return;
  }

  cv::Mat img_rect;
  if(!img_is_raw){
    cv::cvtColor(cv_ptr->image,img_rect, CV_BGR2GRAY);  
  }else{
    cv::undistort(cv_ptr->image,img_rect,intrinsics_,coefficients_);
    cv::cvtColor(img_rect,img_rect, CV_BGR2GRAY);
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

  // Publish ROS topics with UAV data
  if (!PublishUAV_Path(lastPosition_,lastOrientation_)){
	return;
  }
  auto ea=ToEulerAngles(Eigen::Quaterniond(lastOrientation_));

  // Save GroundTruth log
  // timestamp tx ty tz qx qy qz qw (all in float)
  double timeStamp=ros::Time::now().toSec();
  if (_save_logs){
	logGT_ << std::to_string( timeStamp ) + " " + std::to_string( float(lastPosition_[0]) ) + " " + std::to_string( float(lastPosition_[1]) ) + " " + std::to_string( float(lastPosition_[2]) ) 
			+ " " + std::to_string( float(lastOrientation_.x()) ) + " " + std::to_string( float(lastOrientation_.y()) ) + " " + std::to_string( float(lastOrientation_.z()) ) + " " + std::to_string( float(lastOrientation_.w()) )
			+ "\n";
	logGT_.flush();
  }

  // Create pointCloud
  std::vector<double> px_center={ intrinsics_.at<double>(0,2) , intrinsics_.at<double>(1,2) };
  pcl::PointCloud<PointType_>::Ptr monoCloud(new pcl::PointCloud<PointType_>);
  monoCloud->header.frame_id = "uav_pose";
  monoCloud->points.resize(keypoints.size());
  monoCloud->points.clear(); // To avoid add camera point to pointcloud

  if(!ObtainPointCloud(px_center, intrinsics_.at<double>(0,0),lastPosition_[2], ea  , keypoints , monoCloud)){
    return;
  }
  //cloudPub_.publish(monoCloud);

  // Update dataFrame
  std::shared_ptr<mico::DataFrame<PointType_>> df(new mico::DataFrame<PointType_>);
  df->left = img_rect;
  df->intrinsic = intrinsics_;
  df->coefficients = coefficients_;
  df->featureCloud = monoCloud;
  df->featureDescriptors = descriptors;
  df->id = dfCounter_;
  if (savefirstPosition_ && savefirstOrient_ ){
    df->pose=firstPose_;
  }
  df->featureProjections.resize(keypoints.size());
  for(unsigned i = 0; i < keypoints.size(); i++){
    df->featureProjections[i] = keypoints[i].pt;
  }
  dfCounter_++;   // TODO: Database info?
  
  // Compute phothogrametry odometry
  if (mOdometry->computeOdometry(mDatabase->mLastClusterframe, df)) {
    
    bool is_newCluster = mDatabase->addDataframe(df);
  
    if(is_newCluster){
      Eigen::Matrix4f AxisRot=Eigen::Matrix4f::Identity();
      Eigen::Matrix3f rot;
      rot << 0,-1,0, 1,0,0, 0,0,1;  // 90º positive in Z
      AxisRot.block<3,3>(0,0) = rot;

      OdomPose_ = firstPose_ * AxisRot *  mDatabase->mLastClusterframe->getPose();

      geometry_msgs::Point p;
      p.x = OdomPose_(0,3);
      p.y = OdomPose_(1,3);
      p.z = OdomPose_(2,3);
      lineStrip_.points.push_back(p);
      lineStrip_.header.frame_id = "map"; 
      lineStrip_.header.stamp=ros::Time::now();
      markersCf.publish(lineStrip_);

      // Save Visual Odometry log
	  if (_save_logs){
      // timestamp tx ty tz qx qy qz qw (all in float)
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

      geometry_msgs::Point p_ekf;
      p_ekf.x = Xk(0,0);
      p_ekf.y = Xk(1,0);
      p_ekf.z = Xk(2,0);
      EKFlineStrip_.points.push_back(p_ekf);
      EKFlineStrip_.header.frame_id = "map"; 
      EKFlineStrip_.header.stamp=ros::Time::now();
      markersEKF.publish(EKFlineStrip_);

      // Save EKF log
	  if (_save_logs){
		logEKF_ << std::to_string( timeStamp ) + " " + std::to_string( float(Xk(0,0)) ) + " " + std::to_string( float(Xk(1,0)) ) + " " + std::to_string( float(Xk(2,0)) ) + "\n";
		logEKF_.flush();
	  }

      // Mapping module
	  if (_publish_pointCloud){
		auto copyDic = mDatabase->mWordDictionary;
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
      if (mDatabase->mLastClusterframe->id > 1000){ // number of Cf used to create vocabulary
        std::cout << "ID last Cf used to create vocabulary :  " << mDatabase->mLastClusterframe->id << std::endl; 
        if (!createVocabulary()){
          return;
        }
      }

    }
  }

}

/* ------------------------------------------------------------------------------------------------------------------------- */
void Mono2RGBD::infoCb(const sensor_msgs::CameraInfo::ConstPtr& _msg){
    intrinsics_=cv::Mat(3, 3, CV_64FC1, (void *) _msg->K.data()).clone();
    coefficients_=cv::Mat(5, 1, CV_64FC1, (void *) _msg->D.data()).clone();
}


/* ------------------------------------------------------------------------------------------------------------------------- */
void Mono2RGBD::imuCb(const sensor_msgs::Imu::ConstPtr& _msg){
  lastOrientation_=Eigen::Quaternionf(_msg->orientation.w, _msg->orientation.x, _msg->orientation.y, _msg->orientation.z);
  ImuAcceleration_=Eigen::Vector3d(_msg->linear_acceleration.x,_msg->linear_acceleration.y,_msg->linear_acceleration.z);

  if (savefirstOrient_){
    firstPose_.block<3,3>(0,0) =lastOrientation_.matrix();
    savefirstOrient_=false;
  }
}


/* ------------------------------------------------------------------------------------------------------------------------- */
void Mono2RGBD::poseCb(const geometry_msgs::PoseStamped::ConstPtr& _msg){
  lastPosition_ = Eigen::Vector3f(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
  
  if (savefirstPosition_){
    firstPose_.block<3,1>(0,3)=lastPosition_;
    savefirstPosition_=false;
  }

}

/* ------------------------------------------------------------------------------------------------------------------------- */
bool Mono2RGBD::ObtainPointCloud(std::vector<double> camera_center, double focalL , double cam_height , Eigen::Vector3d ea ,std::vector<cv::KeyPoint> keypoints, pcl::PointCloud<PointType_>::Ptr OutputPointCloud){
  
  for(unsigned ii = 0; ii < keypoints.size(); ii++){

    std::vector<double> px={ keypoints[ii].pt.x , keypoints[ii].pt.y };
    
    PointType_ p;
    p.z = - (cam_height) ;
    p.x = -( ( keypoints[ii].pt.x - camera_center[0] )/focalL ) * double(-p.z);
    p.y =  ( ( keypoints[ii].pt.y - camera_center[1] )/focalL ) * double(-p.z);

    p.r = 255; p.g = 255; p.b = 255;
    
    OutputPointCloud->points.push_back(p);
  }
  
  if(!OutputPointCloud->is_dense){
    this->error("MONO2RGBD", "PointCloud contain some Nan or Inf. Skipping frame");
    return false;
  }
  return true;

}


/* ------------------------------------------------------------------------------------------------------------------------- */
bool Mono2RGBD::PublishUAV_Path(Eigen::Vector3f Position , Eigen::Quaternionf Orientation){
  geometry_msgs::PoseStamped pose_msg;
  tf::Transform transform;
  pose_msg.header.frame_id="map";
  pose_msg.header.stamp=ros::Time::now();
  
 // In a real flight, obtained local pose by fixing origin with the first GPS value
  pose_msg.pose.position.x=Position[0]; //pose_msg.pose.position.x=lastPosition_[0]-firstPose_(0,3);
  pose_msg.pose.position.y=Position[1]; //pose_msg.pose.position.y=lastPosition_[1]-firstPose_(1,3);
  pose_msg.pose.position.z=Position[2];

  pose_msg.pose.orientation.x=Orientation.x();
  pose_msg.pose.orientation.y=Orientation.y();
  pose_msg.pose.orientation.z=Orientation.z();
  pose_msg.pose.orientation.w=Orientation.w();
  
  transform.setOrigin( tf::Vector3(Position[0],Position[1],Position[2]) ); //transform.setOrigin( tf::Vector3(lastPosition_[0]-firstPose_(0,3),lastPosition_[1]-firstPose_(1,3),lastPosition_[2]) );
  transform.setRotation( tf::Quaternion(Orientation.x(),Orientation.y(),Orientation.z(),Orientation.w()) );
  br_.sendTransform(tf::StampedTransform(transform, pose_msg.header.stamp, "map", "uav_pose"));


  pathUAV_msg_.header=pose_msg.header;
  pathUAV_msg_.poses.push_back(pose_msg);
  posePub_.publish(pose_msg);
  pathDataPub_.publish(pathUAV_msg_);


  return true;
}

/* ------------------------------------------------------------------------------------------------------------------------- */
bool Mono2RGBD::createVocabulary(){
  const int K = 6; // Values from article Bags of Binary Words for Fast ...
  const int L = 4;

  OrbVocabulary voc(K, L);

  auto clusters = mDatabase->mClusterframes;
  std::vector<std::vector<cv::Mat>> allFeatures;

  for(unsigned i=0; i<clusters.size(); i++){
    allFeatures.push_back(std::vector<cv::Mat>());
  
    for(int r=0; r < (clusters[i]->featureDescriptors.rows); r++){
      allFeatures[i].push_back(clusters[i]->featureDescriptors.row(r));
    }
  }

  voc.create(allFeatures);
  voc.save("/home/marrcogrova/programming/catkin_ws/src/mono2rgbd/cfg/vocabulary_dbow2_solarpanels_orb_K" + std::to_string(K) + "L" + std::to_string(L) + ".xml");

  std::cout << "Vocabulary saved in 'vocabulary_dbow2_solarpanels_orb_K" + std::to_string(K) + "L" + std::to_string(L) + ".xml'" << std::endl;

  return true;
}
