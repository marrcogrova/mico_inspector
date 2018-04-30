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


#ifndef RGBDTOOLS_OBJECTDETECTION_FEATUREBASED_EXTENDEDKALMANFILTER_H_
#define RGBDTOOLS_OBJECTDETECTION_FEATUREBASED_EXTENDEDKALMANFILTER_H_

#include <Eigen/Eigen>	// Eigen linear algebra library

namespace rgbd{
	/** Abstrac class that implements Extended Kalman Filter (EKF) pipeline.
	*/
	class ExtendedKalmanFilter{
	public:
		/** \brief EKF class construction and initialization.
		*/
		ExtendedKalmanFilter();		// 666 TODO: initialize matrixes.

		/** \brief set EKF initial matrixes.
		*	@param _Q: 
		*	@param _R: 
		*	@param _x0: 
		*/
		void setUpEKF(const Eigen::MatrixXd _Q, const  Eigen::MatrixXd _R, const  Eigen::MatrixXd _x0);

		/** \brief get last filtered estimation. 777 rename to state().
		*/
		Eigen::MatrixXd state() const;

	public:
		/** \brief compute single step of EKF.
		*	@param _zK: observable state.
		*	@param _incT: elapsed time between previous and current state.
		*/
		void stepEKF(const Eigen::MatrixXd& _Zk, const double _incT);

	protected:
		// Non specific funtcions of the EKF.
		virtual void updateJf(const double _incT) = 0;
		virtual void updateHZk() = 0;
		virtual void updateJh() = 0;

		// EKF steps.
		void forecastStep(const double _incT);
		void filterStep(const Eigen::MatrixXd&_Zk);

	protected:
		Eigen::MatrixXd mXfk, mXak, mK, mJf, mJh, mP, mQ, mR, mHZk;

	};	//	class ExtendedKalmanFilter
}	//	namespace 

#endif	// _BOVIL_ALGORITHMS_STATE_ESTIMATORS_EXTENDEDKALMANFILTER_H_
