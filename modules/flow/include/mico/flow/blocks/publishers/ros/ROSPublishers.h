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

#ifndef MICO_FLOW_BLOCKS_PUBLISHERS_ROS_ROSPUBLISHERS_H_
#define MICO_FLOW_BLOCKS_PUBLISHERS_ROS_ROSPUBLISHERS_H_

#include <mico/flow/blocks/publishers/ros/BlockROSPublisher.h>
#include <Eigen/Eigen>

#ifdef MICO_USE_ROS
	#include <tf2_eigen/tf2_eigen.h>
	#include <geometry_msgs/PoseStamped.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
#endif

namespace mico{
	#ifdef MICO_USE_ROS
	    struct TraitPoseStampedPublisher{
	    	static std::string blockName_;
	    	static std::string input_;
	    	static geometry_msgs::PoseStamped conversion_(std::unordered_map<std::string,std::any> _data);
	    	typedef geometry_msgs::PoseStamped RosType_;
	    };

        struct TraitPointCloudPublisher{
	    	static std::string blockName_;
	    	static std::string input_;
	    	// typedef geometry_msgs::PoseStamped RosType_;
	    	// static RosType_ conversion_(std::unordered_map<std::string,std::any> _data);
	    };
    
    typedef BlockROSPublisher< TraitPoseStampedPublisher > BlockROSPublisherPoseStamped;
    #endif
}
#endif