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

#include <iostream>

#include <rgbd_tools/map3d/BarricSlam.h>
#include <rgbd_tools/map3d/PointProb.h>

#ifdef RGBDTOOLS_USE_ROS
	#include <ros/ros.h>
#endif

//#include <QApplication>

int main(int _argc, char** _argv)
{
   std::cout << "SLAM example\n";

   #ifdef RGBDTOOLS_USE_ROS
		ros::init(_argc, _argv,"test_sba");
		ros::AsyncSpinner spinner(4); // Use 4 threads
		spinner.start();
	#endif
	// QApplication qapp(_argc, _argv);
	// std::thread qThread([&](){
	// 	qapp.exec();
	// });
	
	rgbd::BarricSlam app;
	
	if (!app.init(_argc, _argv)) {
		return -1;
	}
	// Loop.
	std::string profileName = "nameOfProfile"+std::to_string(time(NULL))+".log";
	//ProfilerStart(profileName.c_str());
	// HeapProfilerStart(profileName.c_str());
	// HeapLeakChecker heap_checker(profileName.c_str());
	bool condition = true;
	while(condition){
		condition = app.step();
	}
	// if (!heap_checker.NoLeaks()) assert(NULL == "heap memory leak");
	// HeapProfilerStop();
	// ProfilerStop();
	return 0;

}