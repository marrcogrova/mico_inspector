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
			
			std::cout << "]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]" << std::endl;
			std::cout << "----------PRE-----\n" << std::endl;
			std::cout << "xfk \n"  << mXfk << std::endl;
			std::cout << "jf"  << mJf << std::endl;
			std::cout << "mP"  << mP << std::endl;
			std::cout << "mQ"  << mQ << std::endl;
			std::cout << "mR"  << mR << std::endl;
			std::cout << "----------POST---------\n" << std::endl;
			forecastStep(_incT);
			std::cout << "xfk \n"  << mXfk << std::endl;
			std::cout << "jf"  << mJf << std::endl;
			std::cout << "mP"  << mP << std::endl;
			std::cout << "----------------------" << std::endl;
			std::cout << "----------PRE-FILTER STEP-----\n" << std::endl;
			//std::cout << "mK"  << mK << std::endl;
			std::cout << "mJh"  << mJh << std::endl;
			std::cout << "mXak \n"  << mXak << std::endl;
			std::cout << "mHZk"  << mHZk << std::endl;
			std::cout << "mP"  << mP << std::endl;

			filterStep(_Zk);
			std::cout << "----------POST-FILTER STEP---------\n" << std::endl;
			std::cout << "mK"  << mK << std::endl;
			std::cout << "mJh"  << mJh << std::endl;
			std::cout << "mXak \n"  << mXak << std::endl;
			std::cout << "mHZk"  << mHZk << std::endl;
			std::cout << "mP"  << mP << std::endl;
			std::cout << "----------------------\n" << std::endl;
		}

		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		void ExtendedKalmanFilter<Type_, D1_, D2_>::forecastStep(const double _incT){
			updateJf(_incT);
			//std::cout << "Función mXak actualizada"  << mXak << std::endl;
			// mxfk mas o menos estable
			mXfk = mJf * mXak;
			
			///////////////////////////////////////////////////////////////////////////////////////////////NORMA DEL VECTOR
			//auto q = mXfk.head(4);
			//float norma= q.norm();
			//if (norma!=1){
			//mXfk.head(4) /= q.norm();
			//}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// mJh inicial
			//mJh.setIdentity();
			//mJh*=100;

			mPfk = mJf * mP * mJf.transpose() + mQ;//actualización de la matriz de covarianza

			//std::cout << "mP forecast \n"  << mP << std::endl;
		}

		//-----------------------------------------------------------------------------
		template<typename Type_, int D1_, int D2_>
		void ExtendedKalmanFilter<Type_, D1_, D2_>::filterStep(const Eigen::Matrix<Type_, D2_, 1 >&_Zk){
		/// Problemas en el filter step

			updateHZk();
			updateJh();
			//std::cout << "----------JACOBIANOS ANTES DEL FILTER STEP-----\n" << std::endl;
			//std::cout << "Función H actualizada \n"  << mHZk << std::endl;
			//std::cout << "Jacobiano de H actualizado \n"  << mJh << std::endl;
			/////////////////////////////////////////////////////////////////////////////////////// Intento paper
			//S=mJh*mPfk*mJh.transpose()+mR;
			//W=mPfk*mJh*S.inverse();
			//mXak=mXfk+W*(_Zk-mHZk);
			//mP=mPfk+W*S*W.transpose();
			//////////////////////////////////////////////////////////////////////////////////////////////////

			mK = mPfk * mJh.transpose() * ((mJh * mPfk * mJh.transpose() + mR).inverse());
			//// Problemas a partir de esta linea  
			mXak = mXfk + mK * (_Zk - mHZk);
			//std::cout << "Jacobiano mK filter step\n"  << mK << std::endl;
			
			////////////////////////////////////////////////////////////////////////////////// norma del vector
			auto q = mXak.head(4);
			float norma=q.norm();
			if (norma!=1){
			
			mXak.head(4) /= q.norm();		
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
			Eigen::Matrix<Type_, D1_, D1_> I; I.setIdentity();
			// cambios para hallar la matriz mPfk y mP de manera independiente. 

			mP = (I - mK * mJh) * mPfk;
		}

		//-----------------------------------------------------------------------------


		//-----------------------------------------------------------------------------
}
