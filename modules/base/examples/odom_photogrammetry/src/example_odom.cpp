//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018  Ricardo Lopez Lopez (a.k.a. ricloplop) & Pablo Ramon Soria (a.k.a. Bardo91)
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

#include "OdometryPhotogrametry.h"
#include <ros/ros.h>
#include <csignal>
#include <mico/base/mico.h>
#include "mono2rgbd.h"

//#include <gperftools/profiler.h>
//#include <gperftools/heap-profiler.h>
//#include <gperftools/heap-checker.h>


void finishHandler(int sig){
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int _argc, char** _argv) {


	ros::init(_argc, _argv, "mono2rgbd_node");
	ros::AsyncSpinner spinner(4); // Use 4 threads
	std::signal(SIGINT, finishHandler);

	spinner.start();
	// mico::Odometry<pcl::PointXYZ, mico::DebugLevels::Debug> *mOdometry = new (mico::OdometryPhotogrametry<pcl::PointXYZ, mico::DebugLevels::Debug>);

	Mono2RGBD converter;
	
	if (!converter.init(_argc, _argv)){
	 	return -1;
	}


	//std::string profileName = "nameOfProfile"+std::to_string(time(NULL))+".prof";
	//ProfilerStart(profileName.c_str());
	// HeapProfilerStart(profileName.c_str());
	// HeapLeakChecker heap_checker(profileName.c_str());

	bool condition = true;
	while(ros::ok() && condition){
		condition = converter.step();
	}

	// if (!heap_checker.NoLeaks()) assert(NULL == "heap memory leak");
	// HeapProfilerStop();
	//ProfilerStop();


	return 0;
}
