
#ifdef RGBDTOOLS_USE_ROS
	#include <ros/ros.h>
#endif

#include <iostream>

//#include <QApplication>

#include <rgbd_tools/map3d/BarricSlam.h>
#include <rgbd_tools/map3d/PointProb.h>

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