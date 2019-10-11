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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

namespace mico{

    // Declaration of conversion callbacks
    Eigen::Matrix4f PoseToMatrix4f(const geometry_msgs::Pose::ConstPtr &_msg);
    Eigen::Matrix4f PoseStampedToMatrix4f(const geometry_msgs::PoseStamped::ConstPtr &_msg);


	// 666 . not is the better solution
	char NameBlockPose_[] = "ROS Pose";
	char TagBlockPose_[] = "pose";
	char NameBlockPoseStamped_[] = "ROS PoseStamped";
	char TagBlockPoseStamped_[] = "pose stamped";

    // Declaration of blocks
    typedef BlockROSSuscriber<NameBlockPose_, TagBlockPose_ , geometry_msgs::Pose, PoseToMatrix4f> BlockRosPose;
    typedef BlockROSSuscriber<NameBlockPoseStamped_, TagBlockPoseStamped_, geometry_msgs::PoseStamped, PoseStampedToMatrix4f> BlockRosPoseStamped;

}

#endif