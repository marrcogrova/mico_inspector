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

#include <iostream>

namespace rgbd{
	
		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		ExtendedKalmanFilter<Type_, D1_, D2_>::ExtendedKalmanFilter(){

		}
		
		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		void ExtendedKalmanFilter<Type_, D1_, D2_>::setUpEKF(	const Eigen::Matrix<Type_, D1_, D1_ > 	_Q, 
																const Eigen::Matrix<Type_, D2_, D2_ >	_R, 
																const Eigen::Matrix<Type_, D1_, 1 > 	_x0){
			mQ = _Q;
			mR = _R;
			mXak = _x0;
			mXfk = _x0;
			mK.setZero();
			mJf.setIdentity();
			mP.setIdentity();
			mHZk.setZero();
			mJh.setZero();
			//Añadida Forekast
			mPfk.setIdentity();
			//Añadida Prueba
			//W.setIdentity();
			//S.setIdentity();
		}
		
		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		Eigen::Matrix<Type_, D1_, 1> ExtendedKalmanFilter<Type_, D1_, D2_>::state() const{
			return mXak;
		}

		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		void ExtendedKalmanFilter<Type_, D1_, D2_>::stepEKF(const Eigen::Matrix<Type_, D2_, 1 > & _Zk, const double _incT){
			
			forecastStep(_incT);

			filterStep(_Zk);

		}

		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		void ExtendedKalmanFilter<Type_, D1_, D2_>::forecastStep(const double _incT){
			updateJf(_incT);
			//std::cout << "Función mXak actualizada"  << mXak << std::endl;
			// mxfk mas o menos estable
			mXfk = mJf * mXak;
			const double PI  =3.141592653589793238463;
			if (mXfk(1,0) > PI){
			    mXfk(1,0)=-PI;
			//std::cout << "He entrado en ROLL"  << mXfk(1,0) << std::endl;
			}
			if(mXfk(1,0)<-PI){
				mXfk(1,0)=PI;
			//std::cout << "He entrado en ROLL"  << mXfk(1,0) << std::endl;
			}
			if (mXfk(2,0) > PI){
			    mXfk(2,0)=-PI;
			//std::cout << "He entrado en PITCH"  << mXfk(2,0) << std::endl;
			}
			if(mXfk(2,0)<-PI){
				mXfk(2,0)=PI;
			//std::cout << "He entrado en PITCH"  << mXfk(2,0) << std::endl;
			}
			if (mXfk(3,0) > PI){
			    mXfk(3,0)=-PI;
			}
			if(mXfk(3,0)<-PI){
				mXfk(3,0)=PI;
			}


			mPfk = mJf * mP * mJf.transpose() + mQ;//actualización de la matriz de covarianza


		}

		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		void ExtendedKalmanFilter<Type_, D1_, D2_>::filterStep(const Eigen::Matrix<Type_, D2_, 1 >&_Zk){
	

			updateHZk();
			updateJh();

			mK = mPfk * mJh.transpose() * ((mJh * mPfk * mJh.transpose() + mR).inverse());  
			mXak = mXfk + mK * (_Zk - mHZk);
			const double PI  =3.141592653589793238463;
			if (mXak(1,0) > PI){
				mXak(1,0)=-PI;
			//std::cout << "He entrado en ROLL"  << mXak(1,0) << std::endl;
			}
			if(mXak(1,0)<-PI){
				mXak(1,0)=PI;
			//std::cout << "He entrado en ROLL"  << mXak(1,0) << std::endl;
			}
			if (mXak(2,0) > PI){
			    mXak(2,0)=-PI;
			//std::cout << "He entrado en PITCH"  << mXak(2,0) << std::endl;
			}
			if(mXak(2,0)<-PI){
				mXak(2,0)=PI;
			//std::cout << "He entrado en PITCH"  << mXak(2,0) << std::endl;
			}
			if (mXak(3,0) > PI){
			    mXak(3,0)=-PI;
			}
			if(mXak(3,0)<-PI){
				mXak(3,0)=PI;
			}

			Eigen::Matrix<Type_, D1_, D1_> I; I.setIdentity();
			// cambios para hallar la matriz mPfk y mP de manera independiente. 

			mP = (I - mK * mJh) * mPfk;
		}

		//-----------------------------------------------------------------------------


		//-----------------------------------------------------------------------------
}
