///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		GLHL Tests - Particle Filter
//			Author:	Pablo R.S.
//			Date:	2014-Dec-21
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Robot class

#include "Robot.h"

//---------------------------------------------------------------------------------------------------------------------
void Robot::set(const double & _x, const double & _y, const double & _ori){
	mPosition.x = _x - fmod(_x,WORLD_SIZE)*WORLD_SIZE;
	mPosition.y = _y - fmod(_y,WORLD_SIZE)*WORLD_SIZE;
	mPosition.ori = _ori - fmod(_ori,2.0*3.1416)*3.1416;

}
//---------------------------------------------------------------------------------------------------------------------
const std::array<double, 4> Robot::sense(){
	std::array<double, 4> z;
	for (int i = 0; i < 4; i++) {
		double dist = sqrt(pow(mPosition.x - LAND_MARKS[i][0], 2) + pow(mPosition.y - LAND_MARKS[i][1], 2));
		dist += gauss(0.0, mNoises.sense);
		z[i] = dist;
	}

	return z;
}

//---------------------------------------------------------------------------------------------------------------------
void Robot::move(const double & _turn, const double & _forward) {
	mPosition.ori = fmod(mPosition.ori + _turn + gauss(0.0, mNoises.turn), 2*3.1416);

	double dist = _forward + gauss(0.0, mNoises.forward);
	mPosition.ori += gauss(0.0, mNoises.turn);
	mPosition.x += cos(mPosition.ori)*dist;
	mPosition.y += sin(mPosition.ori)*dist;
	mPosition.x = fmod(mPosition.x, WORLD_SIZE);
	mPosition.y = fmod(mPosition.y, WORLD_SIZE);
	if (mPosition.x < 0)
		mPosition.x += WORLD_SIZE;
	if (mPosition.y < 0)
		mPosition.y += WORLD_SIZE;
	if (mPosition.ori < 0)
		mPosition.ori += 2 * 3.1416;

}



//---------------------------------------------------------------------------------------------------------------------
std::array<double, 3> Robot::position() const{
	std::array<double, 3> pos = { mPosition.x, mPosition.y, mPosition.ori };
	return pos;

}

//---------------------------------------------------------------------------------------------------------------------
double Robot::gaussian(const double & _nu, const double & _sigma, const double & _x) {
	return exp(-(pow(_nu - _x, 2)) / (pow(_sigma,2)) / 2.0) / sqrt(2.0 * 3.1416 * (pow(_sigma, 2)));
}

//---------------------------------------------------------------------------------------------------------------------
double Robot::measurementProb(std::array<double, 4> _measurement) {
	double prob = 1.0;
	for (int i = 0; i < 4; i++) {
		double dist = sqrt(pow(mPosition.x - LAND_MARKS[i][0], 2) + pow(mPosition.y - LAND_MARKS[i][1], 2));
		prob *= gaussian(dist, mNoises.sense, _measurement[i]);
	}

	return prob;
}