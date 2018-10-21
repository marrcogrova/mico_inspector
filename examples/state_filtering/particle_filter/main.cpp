///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		GLHL Tests - Particle Filter
//			Author:	Pablo R.S.
//			Date:	2014-Dec-21
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main

#include <iostream>
#include <string>

#include <rgbd_tools/state_filtering/ParticleFilterCPU.h>

#include "Robot.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace rgbd;

class ParticleRobot : public Particle, public Robot{
public:
	ParticleRobot() { 
		setNoise(0.1, 0.1, 10.0); 
		set(	((double)rand()/RAND_MAX)*WORLD_SIZE,
				((double)rand()/RAND_MAX)*WORLD_SIZE,
				((double)rand()/RAND_MAX)*2*M_PI);
				std::cout << "inited" << std::endl;
		};
	void simulate() { 
			move(0.1, 0.5); 
		};
	void calcWeigh(Particle &_realParticle) { 
		mWeigh = measurementProb(static_cast<ParticleRobot &>(_realParticle).sense()); 
	};

	operator std::array<double, 3>(){ return position(); }

};

//---------------------------------------------------------------------------------------------------------------------
std::array<double, 3> mediumState(std::vector<ParticleRobot> _particles);
void particleFilterCPU();

//---------------------------------------------------------------------------------------------------------------------
int main(void){
	
	particleFilterCPU();

	std::cout << "Finished" << std::endl;
	
	system("PAUSE");

}

//---------------------------------------------------------------------------------------------------------------------

std::array<double, 3> mediumState(std::vector<ParticleRobot> _particles){
	std::array<double, 3> position = {0.0, 0.0, 0.0};
	for (unsigned i = 0; i < _particles.size(); i++){
		std::array<double, 3> pos = _particles[i];
		position[0] += pos[0];
		position[1] += pos[1];
		position[2] += pos[2];
	}
	position[0] /= _particles.size();
	position[1] /= _particles.size();
	position[2] /= _particles.size();

	return position;
}

void particleFilterCPU() {

	ParticleFilterCPU<ParticleRobot> filter(1000);
	filter.init();

	ParticleRobot robot;
	robot.set(30, 80, 1.5);
	double time = 0.0;

	float cScaleFactor=10;

	cv::namedWindow("display", CV_WINDOW_FREERATIO);
	for (;;) {
		std::array<double, 3> medState = mediumState(filter.particles());
		std::array<double, 3> realState = robot.position();
		std::cout << "-------------------------------------------------------------------" << std::endl;
		std::cout << "Real state. X:" << realState[0] << " ; Y: " << realState[1] << " ; Ori: " << realState[2] << std::endl;
		std::cout << "Promediate state. X:" << medState[0] << " ; Y: " << medState[1] << " ; Ori: " << medState[2] << std::endl;

		cv::Mat display(WORLD_SIZE*cScaleFactor,WORLD_SIZE*cScaleFactor, CV_8UC3, cv::Scalar(0,0,0));
		// Draw landmarks
		cv::circle(display, cv::Point2i(LAND_MARKS[0][0]*cScaleFactor, LAND_MARKS[0][1]*cScaleFactor), 3, cv::Scalar(0,0,255), 3);
		cv::circle(display, cv::Point2i(LAND_MARKS[1][0]*cScaleFactor, LAND_MARKS[1][1]*cScaleFactor), 3, cv::Scalar(0,0,255), 3);
		cv::circle(display, cv::Point2i(LAND_MARKS[2][0]*cScaleFactor, LAND_MARKS[2][1]*cScaleFactor), 3, cv::Scalar(0,0,255), 3);
		cv::circle(display, cv::Point2i(LAND_MARKS[3][0]*cScaleFactor, LAND_MARKS[3][1]*cScaleFactor), 3, cv::Scalar(0,0,255), 3);

		// Draw particles
		for(auto &particle: filter.particles()){
			std::array<double, 3> pos = particle;
			cv::arrowedLine(display, 	cv::Point2i(pos[0]*cScaleFactor, pos[1]*cScaleFactor), 
										cv::Point2i(pos[0]*cScaleFactor, pos[1]*cScaleFactor) + cv::Point2i(cos(realState[2])*5*cScaleFactor, sin(realState[2])*5*cScaleFactor), 
										cv::Scalar(255,0,0), 1);
		}

		// Draw robot
		cv::arrowedLine(display, 	cv::Point2i(realState[0]*cScaleFactor, realState[1]*cScaleFactor), 
									cv::Point2i(realState[0]*cScaleFactor, realState[1]*cScaleFactor) + cv::Point2i(cos(realState[2])*5*cScaleFactor, sin(realState[2])*5*cScaleFactor), 
									cv::Scalar(0,0,255), 2);

		// Draw robot
		cv::arrowedLine(display, 	cv::Point2i(medState[0]*cScaleFactor, medState[1]*cScaleFactor), 
									cv::Point2i(medState[0]*cScaleFactor, medState[1]*cScaleFactor) + cv::Point2i(cos(medState[2])*5*cScaleFactor, sin(medState[2])*5*cScaleFactor), 
									cv::Scalar(0,255,0), 2);


		cv::imshow("display", display);
		cv::waitKey(3);

		robot.simulate();
		filter.step(robot);

	}
	std::cout << "Step Time: " << time / 50 << std::endl;

}