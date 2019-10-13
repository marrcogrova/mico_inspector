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

#ifndef MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSTREAMERS_H_
#define MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSTREAMERS_H_

#include <mico/flow/blocks/streamers/ros/BlockROSSuscriber.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef MICO_USE_ROS
	#include <geometry_msgs/PoseStamped.h>
	#include <geometry_msgs/Pose.h>
	#include <sensor_msgs/Image.h>
	#include <sensor_msgs/PointCloud2.h>
#endif

namespace mico{
	#ifdef MICO_USE_ROS
    	// Declaration of conversion callbacks
    	Eigen::Matrix4f PoseToMatrix4f(const geometry_msgs::Pose::ConstPtr &_msg);
    	Eigen::Matrix4f PoseStampedToMatrix4f(const geometry_msgs::PoseStamped::ConstPtr &_msg);
		
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ROSPointCloudToPCL(const sensor_msgs::PointCloud2::ConstPtr &_msg);
		
    	cv::Mat RosImageToCvImage(const sensor_msgs::Image::ConstPtr &_msg);
	
    	// Declaration of blocks
		char BlockRosPoseName[] = "Ros Pose Subscriber";
		char BlockRosPoseTag [] = "pose";
		typedef BlockROSSuscriber<  BlockRosPoseName, 
									BlockRosPoseTag, 
									geometry_msgs::Pose, 
									Eigen::Matrix4f,
									&PoseToMatrix4f> BlockRosPose;

		char BlockRosPoseStampedName[] = "Ros PoseStamped Subscriber";
		char BlockRosPoseStampedTag [] = "pose";
		typedef BlockROSSuscriber<  BlockRosPoseStampedName,
									BlockRosPoseStampedTag ,
									geometry_msgs::PoseStamped, 
									Eigen::Matrix4f,
									&PoseStampedToMatrix4f> BlockRosPoseStamped;

		char BlockRosImageName[] = "Ros Image Subscriber";
		char BlockRosImageTag [] = "color";
		typedef BlockROSSuscriber<  BlockRosImageName,
									BlockRosImageTag ,
									sensor_msgs::Image, 
									cv::Mat,
									&RosImageToCvImage> BlockRosImage;

		char BlockRosCloudName[] = "Ros PointCloud Subscriber";
		char BlockRosCloudTag [] = "cloud";
		typedef BlockROSSuscriber<  BlockRosCloudName,
									BlockRosCloudTag ,
									sensor_msgs::PointCloud2, 
									pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
									&ROSPointCloudToPCL> BlockRosCloud;
	#endif
}

#endif