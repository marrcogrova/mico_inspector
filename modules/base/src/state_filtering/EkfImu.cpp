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


#include <mico/base/state_filtering/EkfImu.h>

namespace mico{

	void EkfImu::parameters(Eigen::Vector3f _scaleFactor, Eigen::Vector3f _c1, Eigen::Vector3f _c2, Eigen::Vector3f _t) {
		mScaleFactor = _scaleFactor;
		mC1 = _c1;
		mC2 = _c2;
		mT = _t;

	}

	//---------------------------------------------------------------------------------------------------------------------
	double sign2(double _var) {
		return _var < 0? -1:1;
	}

	void EkfImu::updateJf(const double _incT) {
		mJf = Eigen::MatrixXf::Identity(mJf.rows(), mJf.cols());
		Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
		mJf.block<3, 3>(0, 3) =I*_incT;
		mJf.block<3, 3>(0, 6) =I*_incT*_incT/2;
		mJf.block<3, 3>(3, 6) =I*_incT;
		
		Eigen::Vector3f auxAcc({sign2(mXfk(6,0)*mScaleFactor[0]), sign2(mXfk(6,0)*mScaleFactor[1]), sign2(mXfk(6,0)*mScaleFactor[2])});
		Eigen::Matrix3f auxMatAcc = (auxAcc*Eigen::Matrix<float,1,3>({1,1,1})).cwiseProduct(I);;
		mJf.block<3, 3>(6, 6) += auxMatAcc;

		mJf.block<3, 3>(6, 9) =-I;
		
		Eigen::Matrix3f diagT = (mT*Eigen::Matrix<float,1,3>({1,1,1})).cwiseProduct(I);;
		Eigen::Matrix3f diagTpluxTime = (diagT + I*_incT);
		mJf(9, 9) = diagT(0,0)/diagTpluxTime(0,0);
		mJf(10, 10) = diagT(1,1)/diagTpluxTime(1,1);
		mJf(11, 11) = diagT(2,2)/diagTpluxTime(2,2);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EkfImu::updateHZk() {
		mHZk = mXfk.block<3,1>(6,0);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EkfImu::updateJh() {
		mJh(0,6) = 1;
		mJh(1,7) = 1;
		mJh(2,8) = 1;
	}

}