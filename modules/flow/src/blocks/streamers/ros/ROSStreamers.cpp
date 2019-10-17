//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#include <mico/flow/blocks/streamers/ros/ROSStreamers.h>

namespace mico{

    // Declaration of Trait structs
	#ifdef MICO_USE_ROS
	
		//-------------------------------------------------------------------------------------------------------------
		std::string TraitPoseStamped::blockName_ = "Ros Pose Subscriber";
		std::vector<std::string> TraitPoseStamped::output_ = {"pose"};

		std::any TraitPoseStamped::conversion_(std::string _tag, const geometry_msgs::PoseStamped::ConstPtr &_msg){
			Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			pose.block<3,1>(0,3) = Eigen::Vector3f(_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);

			Eigen::Quaternionf q = Eigen::Quaternionf(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
			pose.block<3,3>(0,0) = q.matrix();

			return pose;
		}

		//-------------------------------------------------------------------------------------------------------------
		std::string TraitImu::blockName_ = "Ros Imu Subscriber";
		std::vector<std::string> TraitImu::output_ = {"orientation" , "acceleration"};

		std::any TraitImu::conversion_(std::string _tag, const sensor_msgs::Imu::ConstPtr &_msg){
			if (_tag == "orientation"){
				Eigen::Quaternionf q = Eigen::Quaternionf(_msg->orientation.w, _msg->orientation.x, _msg->orientation.y, _msg->orientation.z);
				return q;
			}else if (_tag == "acceleration"){
				Eigen::Vector3f acc = Eigen::Vector3f(_msg->linear_acceleration.x, _msg->linear_acceleration.y, _msg->linear_acceleration.z);	
				return acc;
			}
		}
		
		//-------------------------------------------------------------------------------------------------------------
		std::string TraitImage::blockName_ = "Ros Image Subscriber";
		std::vector<std::string> TraitImage::output_ = {"color"};

		std::any TraitImage::conversion_(std::string _tag, const sensor_msgs::Image::ConstPtr &_msg){
			return cv_bridge::toCvCopy(_msg, "bgr8")->image;;
		}

		//-------------------------------------------------------------------------------------------------------------
		std::string TraitCloud::blockName_ = "Ros PointCloud Subscriber";
		std::vector<std::string> TraitCloud::output_ = {"cloud"};

		std::any TraitCloud::conversion_(std::string _tag, const sensor_msgs::PointCloud2::ConstPtr &_msg){
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    		pcl::fromROSMsg(*_msg, *cloud);

			return cloud;
		}
	#endif


}