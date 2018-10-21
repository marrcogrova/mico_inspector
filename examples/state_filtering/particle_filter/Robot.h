///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		GLHL Tests - Particle Filter
//			Author:	Pablo R.S.
//			Date:	2014-Dec-21
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Robot class



#ifndef _GLHL_TEST_ROBOT_H_
#define _GLHL_TEST_ROBOT_H_

#include <array>
#include <random>
#include <ctime>

const double LAND_MARKS[4][2] = { {20.0,20.0}, {80.0,80.0}, {20.0,80.0}, {80.0,20.0} };
const double WORLD_SIZE = 100.0;

inline double gauss(const double & _nu, const double & _sigma) {
	std::random_device rd;
	std::mt19937 gen(rd());

	std::normal_distribution<> d(_nu, _sigma);

	return d(gen);
}

class Robot {
public:
	void set(const double &_x, const double & _y, const double & _ori);
	void setNoise(const double & _forward, const double & _turn, const double & _sense) { 
		mNoises.forward = _forward; 
		mNoises.turn = _turn; 
		mNoises.sense = _sense; 
		};
		
	const std::array<double, 4> sense();
	void move(const double & _turn, const double & _forward);

	std::array<double, 3> position() const;

public:
	double gaussian(const double & _mu, const double & _sigma, const double & _x);
	double measurementProb(std::array<double, 4> _measurement);
	
private:
	struct Position { double x = double(rand()) / RAND_MAX * WORLD_SIZE, y = double(rand()) / RAND_MAX * WORLD_SIZE, ori = double(rand()) / RAND_MAX * 2.0 * 3.1416; } mPosition;
	struct Noises { double forward = 0.0, turn = 0.0, sense = 0.0; } mNoises;
	

};	//	class Robot


#endif	//	_GLHL_TEST_ROBOT_H_