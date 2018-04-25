//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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


#include <rgbd_tools/object_detection/feature_based/ExtendedKalmanFilter.h>

#include <iostream>

using namespace Eigen;

namespace rgbd{
	
		//-----------------------------------------------------------------------------
		ExtendedKalmanFilter::ExtendedKalmanFilter(){

		}
		
		//-----------------------------------------------------------------------------
		void ExtendedKalmanFilter::setUpEKF(const MatrixXd _Q, const MatrixXd _R, const MatrixXd _x0){
			mQ = _Q;
			mR = _R;
			mXak = _x0;
			mXfk = _x0;
			mK =	MatrixXd::Zero(_Q.rows(), _R.rows());
			mJf =	MatrixXd::Identity(_Q.rows(), _Q.rows());
			mP =	MatrixXd::Identity(_Q.rows(), _Q.rows());
			mHZk =	MatrixXd::Zero(_R.rows(), 1);
			mJh =	MatrixXd::Zero(_R.rows(), _Q.rows());
		}
		
		//-----------------------------------------------------------------------------
		MatrixXd ExtendedKalmanFilter::state() const{
			return mXak;
		}

		//-----------------------------------------------------------------------------
		void ExtendedKalmanFilter::stepEKF(const MatrixXd& _Zk, const double _incT){
			forecastStep(_incT);

			filterStep(_Zk);
		}

		//-----------------------------------------------------------------------------
		void ExtendedKalmanFilter::forecastStep(const double _incT){
			updateJf(_incT);
			
			mXfk = mJf * mXak;

			mP = mJf * mP * mJf.transpose() + mQ;
		}

		//-----------------------------------------------------------------------------
		void ExtendedKalmanFilter::filterStep(const MatrixXd& _Zk){
			updateHZk();
			updateJh();

			mK = mP * mJh.transpose() * ((mJh * mP * mJh.transpose() + mR).inverse());

			mXak = mXfk + mK * (_Zk - mHZk);

			MatrixXd I = MatrixXd::Identity(mK.rows(), mK.rows());

			mP = (I - mK * mJh) * mP;
		}

		//-----------------------------------------------------------------------------


		//-----------------------------------------------------------------------------
}
