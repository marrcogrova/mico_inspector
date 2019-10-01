//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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



#ifndef MICO_BASE_OBJECTDETECTION_FEATUREBASED_EKFIMU_H_
#define MICO_BASE_OBJECTDETECTION_FEATUREBASED_EKFIMU_H_


#include <mico/base/state_filtering/ExtendedKalmanFilter.h>


namespace mico{
	// Estimation of th drone position using Acc and gyro.
	//
	// State vector Xk = {x, y, z, vx, vy, vz, ax, ay, az, bax, bay, baz}
	//
	//
	//
	class EkfImu : public ExtendedKalmanFilter<float, 12,3> {
	public:
		void parameters(Eigen::Vector3f _scaleFactor, Eigen::Vector3f _c1, Eigen::Vector3f _c2, Eigen::Vector3f _t);

	protected:
		void updateJf(const double _incT);
		void updateHZk();
		void updateJh();

	private:
		Eigen::Vector3f mScaleFactor, mC1, mC2, mT;
	};

}

#endif	//	MICO_BASE_OBJECTDETECTION_FEATUREBASED_EKFIMU_H_