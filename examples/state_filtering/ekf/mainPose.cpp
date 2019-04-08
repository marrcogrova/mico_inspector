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

#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <thread>
#include <mutex>

#include <rgbd_tools/utils/Graph2d.h>
#include <pcl/visualization/pcl_visualizer.h>

//			            0  1 2  3  4  5  6   7   8   9
// State variable x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]lambda=correlation time bias
//
// Observation variable z = [xa ya za Yaw] a=acelerometro m=magnetometro
//

//// Captación de los quaternions
float q0new = 99, q1new = 99, q2new = 99, q3new = 99;
float wi_new=99, wj_new=99, wk_new=99;
float anew = 99, bnew = 99, cnew = 99, dnew = 99,enew=99, fnew=99;
float a = 99, b = 99, c = 99, d = 99, e = 99, f = 99;
const double PI  =3.141592653589793238463;
float g=9.81;

//boost::array<double, 9> _linear_acceleration_covariance = {};
//boost::array<double, 9> _magnetic_field_covariance = {};

std::mutex mtx_com;
// reading accelerometer
void accel_Callback(const sensor_msgs::Imu &msgaccel)
{	/////////////////////////////////////////////////////////////////////////////// 
	mtx_com.lock();
	//q0new = msgaccel.orientation.x;
	//q1new = msgaccel.orientation.y;
	//q2new = msgaccel.orientation.z;
  	//q3new = msgaccel.orientation.w;
	////////////////////////////////////////////////////////////////////////////////////////////////// lo que sería lógico
	q1new = msgaccel.orientation.x;
	q2new = msgaccel.orientation.y;
	q3new = msgaccel.orientation.z;
  	q0new = msgaccel.orientation.w;
	////////////////////////////////////////////////////////////////////////////////////////////////////
	anew = msgaccel.linear_acceleration.x;
	bnew = msgaccel.linear_acceleration.y;
	cnew = msgaccel.linear_acceleration.z;
	wi_new=msgaccel.angular_velocity.x;
	wj_new=msgaccel.angular_velocity.y;
	wk_new=msgaccel.angular_velocity.z;
	mtx_com.unlock();
	// std::cout << "Updated acell" << std::endl;
	//_linear_acceleration_covariance = msgaccel.linear_acceleration_covariance;
}
// reading magnetometer
void mag_Callback(const sensor_msgs::MagneticField &msgmag)
{
	mtx_com.lock();
	dnew = msgmag.magnetic_field.x;
	enew = msgmag.magnetic_field.y;
	fnew = msgmag.magnetic_field.z;
	//_magnetic_field_covariance = msgmag.magnetic_field_covariance;
	mtx_com.unlock();
	// std::cout << "Updated mag" << std::endl;
}

class EkfPose : public rgbd::ExtendedKalmanFilter<float, 10, 3>
{
  private:
	const float lambda = 1;

  protected:
	void updateJf(const double _incT)
	{ ///bien al menos en concepto
		float q0J = mXak(0, 0);
		float q1J = mXak(1, 0);
		float q2J = mXak(2, 0);
		float q3J = mXak(3, 0);
		float WiJ = wi_new;
		float WjJ = wj_new;
		float WkJ = wk_new; 
		/// fila 1
		mJf.setIdentity();
		mJf(0, 0) = 1;
		mJf(0, 1) = (-1)*WiJ*(_incT/2);
		mJf(0, 2) = (-1)*WjJ*(_incT/2);
		mJf(0, 3) = (-1)*WkJ*(_incT/2);
		mJf(0, 4) = (-1)*q1J*(_incT/2);
		mJf(0, 5) = (-1)*q2J*(_incT/2);
		mJf(0, 6) = (-1)*q3J*(_incT/2);
		mJf(0, 7) = 0;
		mJf(0, 8) = 0;
		mJf(0, 9) = 0;
		// fila 2
		mJf(1, 0) = WiJ*(_incT / 2);
		mJf(1, 1) = 1;
		mJf(1, 2) = WkJ*(_incT/2);
		mJf(1, 3) = (-1)*WjJ*(_incT/2);    
		mJf(1, 4) = q0J*(_incT/2);
		mJf(1, 5) = (-1)*q3J*(_incT/2);
		mJf(1, 6) = q2J*(_incT/2);
		mJf(1, 7) = 0;
		mJf(1, 8) = 0;
		mJf(1, 9) = 0;
		// fila 3
		mJf(2, 0) = WjJ*(_incT/2);
		mJf(2, 1) = (-1)*WkJ*(_incT/2);
		mJf(2, 2) = 1;
		mJf(2, 3) = WiJ*(_incT/2);
		mJf(2, 4) = q3J*(_incT/2);
		mJf(2, 5) = q0J*(_incT/2);
		mJf(2, 6) = (-1)*q1J*(_incT/2);
		mJf(2, 7) = 0;
		mJf(2, 8) = 0;
		mJf(2, 9) = 0;
		// fila 4
		mJf(3, 0) = WkJ*(_incT/2);
		mJf(3, 1) = WjJ*(_incT/2);
		mJf(3, 2) = (-1)*WiJ*(_incT/2);
		mJf(3, 3) = 1;
		mJf(3, 4) = (-1)*q2J*(_incT/2);
		mJf(3, 5) = q1J*(_incT/2);
		mJf(3, 6) = q0J*(_incT/2);
		mJf(3, 7) = 0;
		mJf(3, 8) = 0;
		mJf(3, 9) = 0;
		// fila 5
		mJf(4, 0) = 0;
		mJf(4, 1) = 0;
		mJf(4, 2) = 0;
		mJf(4, 3) = 0;
		mJf(4, 4) = 0;
		mJf(4, 5) = 0;
		mJf(4, 6) = 0;
		mJf(4, 7) = (-1);
		mJf(4, 8) = 0;
		mJf(4, 9) = 0;
		// fila 6
		mJf(5, 1) = 0;
		mJf(5, 2) = 0;
		mJf(5, 3) = 0;
		mJf(5, 4) = 0;
		mJf(5, 5) = 0;
		mJf(5, 6) = 0;
		mJf(5, 7) = 0;
		mJf(5, 8) = (-1);
		mJf(5, 9) = 0;
		// fila 7
		mJf(6, 0) = 0;
		mJf(6, 1) = 0;
		mJf(6, 2) = 0;
		mJf(6, 3) = 0;
		mJf(6, 4) = 0;
		mJf(6, 5) = 0;
		mJf(6, 6) = 0;
		mJf(6, 7) = 0;
		mJf(6, 8) = 0;
		mJf(6, 9) = (-1);
		// fila 8
		mJf(7, 0) = 0;
		mJf(7, 1) = 0;
		mJf(7, 2) = 0;
		mJf(7, 3) = 0;
		mJf(7, 4) = 0;
		mJf(7, 5) = 0;
		mJf(7, 6) = 0;
		mJf(7, 7) = (1-lambda*_incT);
		mJf(7, 8) = 0;
		mJf(7, 9) = 0;
		// fila 9
		mJf(8, 0) = 0;
		mJf(8, 1) = 0;
		mJf(8, 2) = 0;
		mJf(8, 3) = 0;
		mJf(8, 4) = 0;
		mJf(8, 5) = 0;
		mJf(8, 6) = 0;
		mJf(8, 7) = 0;
		mJf(8, 8) = (1-lambda*_incT);
		mJf(8, 9) = 0;
		// fila 10
		mJf(9, 0) = 0;
		mJf(9, 1) = 0;
		mJf(9, 2) = 0;
		mJf(9, 3) = 0;
		mJf(9, 4) = 0;
		mJf(9, 5) = 0;
		mJf(9, 6) = 0;
		mJf(9, 7) = 0;
		mJf(9, 8) = 0;
		mJf(9, 9) = (1-lambda*_incT);
	}

	//---------------------------------------------------------------------------------------------------
	void updateHZk()
	{ /////////////////////////////////////////////////////////////////////////////////////////////////////////////////Actualización Propia
		float q0H = mXak(0, 0);
		float q1H = mXak(1, 0);
		float q2H = mXak(2, 0);
		float q3H = mXak(3, 0);
		float roll=atan2(2*(q0H*q1H+q2H*q3H),1-2*(q1H*q1H+q2H*q2H));
		float pitch=asin(2*(q0H*q2H-q1H*q3H));
    	float yaw=atan2(2*(q0H*q3H+q1H*q2H),1-2*(q2H*q2H+q3H*q3H));
		//if (roll<0){
		//	roll=2*PI+roll;
		//}
		//if (yaw<0){
		//	yaw=2*PI+yaw;
		//}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Normalización de angulos
		//if (roll<0){
		//	float incremento_roll=PI+roll;
		//	roll=PI+incremento_roll;
		//}
		//if (yaw<0){
		//	float incremento_yaw=PI+yaw;
		//	yaw=PI+incremento_yaw;
		//}
		///////////////////////////////////////////////////////////////////////////////Suponemos que el Pitch debe estar contenido en el primer cuadrante por lo que el seno será sipre positivo

		mHZk[0]=roll;
		mHZk[1]=pitch;
		mHZk[2]=yaw;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////Actualización normal
	///q0 es w
	//	float q0H = mXak(0, 0);
	//	float q1H = mXak(1, 0);
	//	float q2H = mXak(2, 0);
	//	float q3H = mXak(3, 0);
	//
	////	std::cout << "----------Actualización de H -----\n" << std::endl;
	////	std::cout << "Función mXak actualizada"  << mXak << std::endl;
//
	//	float ah = (-1) * 2 * ((q1H*q3H)-(q0H*q2H));
	//	float bh= (-1) * 2 * ((q2H*q3H)-(q0H*q1H));
	//	float ch= (-1) * ((q0H*q0H)-(q1H*q1H)-(q2H*q2H)-(q3H*q3H));
	//	//float normah=sqrt(ah*ah+bh*bh+ch*ch);
	//	//mHZk[0]=ah/normah;
	//	//mHZk[1]=bh/normah;
	//	//mHZk[2]=ch/normah;
	//	mHZk[0]=ah;
	//	mHZk[1]=bh;
	//	mHZk[2]=ch;
//
	///// esta medida que estamos introduciendo aquí es el YAW
	////	Eigen::Matrix<float, 3, 1>M_ym;
	////	Eigen::Matrix<float, 3, 3>M_mb;
	////	Eigen::Matrix<float, 3, 3> mn;
	////	Eigen::Matrix<float, 3, 3>Rnbt;
	////	mn << q0H*q0H+q1H*q1H-q2H*q2H-q3H*q3H, 2*(q1H*q2H-q0H*q3H), 2*(q2H*q3H+q0H*q1H),
	////		    2*(q1H*q2H-q0H*q3H), (q0H*q0H-q1H*q1H+q2H*q2H-q3H*q3H), 2*(q2H*q3H-q0H*q1H),
	////			  0, 0, 0;
	////	Rnbt << q0H*q0H+q1H*q1H-q2H*q2H-q3H*q3H, 2*(q1H*q2H-q0H*q3H), 2*(q0H*q2H+q1H*q3H),
	////			    2*(q1H*q2H+q0H*q3H), q0H*q0H-q1H*q1H+q2H*q2H-q3H*q3H, 2*(q2H*q3H-q0H*q1H),
	////				2*(q1H*q3H-q0H*q2H), 2*(q0H*q1H+q2H*q3H), q0H*q0H-q1H*q1H-q2H*q2H+q3H*q3H;
	////	M_ym << dnew,enew,fnew;
	////	M_mb = Rnbt*mn; 
	////float angulo=atan2(-1*(M_mb(1,0)+M_mb(1,1)+M_mb(1,2)),M_mb(0,0)+M_mb(0,1)+M_mb(0,2));
  //
	//float angulo=atan2(2*(q0H*q3H+q1H*q2H),1-2*(q2H*q2H+q3H*q3H));
	////float angulo=atan2(2*q0H*q2H-2*q1H*q3H,q0H*q0H+q1H*q1H-q2H*q2H-q3H*q3H);
	//if (angulo<0){
	//	mHZk[3]=2*PI+angulo;
	//}
	//if (angulo>0){
	//	mHZk[3]=angulo;
	//}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Fin Actualización normal
	}

	//---------------------------------------------------------------------------------------------------
	void updateJh()
	{	float q0Jh = mXfk(0, 0);
		float q1Jh = mXfk(1, 0);
		float q2Jh = mXfk(2, 0);
		float q3Jh = mXfk(3, 0);
		// fila 1
		mJh(0, 0) = (q1Jh*((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0)*-2.0)/(pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0)+pow(q0Jh*q1Jh*2.0+q2Jh*q3Jh*2.0,2.0));
		mJh(0, 1) = -(((q0Jh*2.0)/((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0)-q1Jh*(q0Jh*q1Jh*2.0+q2Jh*q3Jh*2.0)*1.0/pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0)*4.0)*pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0))/(pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0)+pow(q0Jh*q1Jh*2.0+q2Jh*q3Jh*2.0,2.0));
		mJh(0, 2) = -(((q3Jh*2.0)/((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0)-q2Jh*(q0Jh*q1Jh*2.0+q2Jh*q3Jh*2.0)*1.0/pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0)*4.0)*pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0))/(pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0)+pow(q0Jh*q1Jh*2.0+q2Jh*q3Jh*2.0,2.0));
		mJh(0, 3) =  (q2Jh*((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0)*-2.0)/(pow((q1Jh*q1Jh)*2.0+(q2Jh*q2Jh)*2.0-1.0,2.0)+pow(q0Jh*q1Jh*2.0+q2Jh*q3Jh*2.0,2.0));
		mJh(0, 4) = 0;
		mJh(0, 5) = 0;
		mJh(0, 6) = 0;
		mJh(0, 7) = 0;
		mJh(0, 8) = 0;
		mJh(0, 9) = 0;
		// fila 2
		mJh(1, 0) = q2Jh*1.0/sqrt(-pow(q0Jh*q2Jh*2.0-q1Jh*q3Jh*2.0,2.0)+1.0)*2.0;
		mJh(1, 1) = q3Jh*1.0/sqrt(-pow(q0Jh*q2Jh*2.0-q1Jh*q3Jh*2.0,2.0)+1.0)*-2.0;
		mJh(1, 2) = q0Jh*1.0/sqrt(-pow(q0Jh*q2Jh*2.0-q1Jh*q3Jh*2.0,2.0)+1.0)*2.0;
		mJh(1, 3) = q1Jh*1.0/sqrt(-pow(q0Jh*q2Jh*2.0-q1Jh*q3Jh*2.0,2.0)+1.0)*-2.0;
		mJh(1, 4) = 0;
		mJh(1, 5) = 0;
		mJh(1, 6) = 0;
		mJh(1, 7) = 0;
		mJh(1, 8) = 0;
		mJh(1, 9) = 0;
		
		
		// fila 3

		// Los vectores U no son actuaciones, son formas de simplifiacar la derivación de atan2 

		mJh(2, 0) = (q3Jh*((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0)*-2.0)/(pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0)+pow(q0Jh*q3Jh*2.0+q1Jh*q2Jh*2.0,2.0));
		mJh(2, 1) = (q2Jh*((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0)*-2.0)/(pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0)+pow(q0Jh*q3Jh*2.0+q1Jh*q2Jh*2.0,2.0));
		mJh(2, 2) = -(((q1Jh*2.0)/((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0)-q2Jh*(q0Jh*q3Jh*2.0+q1Jh*q2Jh*2.0)*1.0/pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0)*4.0)*pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0))/(pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0)+pow(q0Jh*q3Jh*2.0+q1Jh*q2Jh*2.0,2.0));
		mJh(2, 3) = -(((q0Jh*2.0)/((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0)-q3Jh*(q0Jh*q3Jh*2.0+q1Jh*q2Jh*2.0)*1.0/pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0)*4.0)*pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0))/(pow((q2Jh*q2Jh)*2.0+(q3Jh*q3Jh)*2.0-1.0,2.0)+pow(q0Jh*q3Jh*2.0+q1Jh*q2Jh*2.0,2.0));
		mJh(2, 4) = 0;
		mJh(2, 5) = 0;
		mJh(2, 6) = 0;
		mJh(2, 7) = 0;
		mJh(2, 8) = 0;
		mJh(2, 9) = 0;
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Actualización paper
		//float q0Jh = mXfk(0, 0);
		//float q1Jh = mXfk(1, 0);
		//float q2Jh = mXfk(2, 0);
		//float q3Jh = mXfk(3, 0);
		//// fila 1
		//mJh.setIdentity();
		//mJh(0, 0) = 2 * q2Jh;
		//mJh(0, 1) = (-1) * 2 * q3Jh;
		//mJh(0, 2) = 2 * q0Jh;
		//mJh(0, 3) = (-1) * 2 * q1Jh;
		//mJh(0, 4) = 0;
		//mJh(0, 5) = 0;
		//mJh(0, 6) = 0;
		//mJh(0, 7) = 0;
		//mJh(0, 8) = 0;
		//mJh(0, 9) = 0;
		//// fila 2
		//mJh(1, 0) = (-1) * 2 * q1Jh;
		//mJh(1, 1) = (-1) * 2 * q0Jh;
		//mJh(1, 2) = (-1) * 2 * q3Jh;
		//mJh(1, 3) = (-1) * 2 * q2Jh;
		//mJh(1, 4) = 0;
		//mJh(1, 5) = 0;
		//mJh(1, 6) = 0;
		//mJh(1, 7) = 0;
		//mJh(1, 8) = 0;
		//mJh(1, 9) = 0;
		//// fila 3
		//mJh(2, 0) = (-1) * 2 * q0Jh;
		//mJh(2, 1) = 2 * q1Jh;
		//mJh(2, 2) = 2 * q2Jh;
		//mJh(2, 3) = (-1)*2 * q3Jh;
		//mJh(2, 4) = 0;
		//mJh(2, 5) = 0;
		//mJh(2, 6) = 0;
		//mJh(2, 7) = 0;
		//mJh(2, 8) = 0;
		//mJh(2, 9) = 0;
		//// fila 4
//
		//// Los vectores U no son actuaciones, son formas de simplifiacar la derivación de atan2 
		//float Funcion = (2*(q0Jh*q3Jh+q1Jh*q2Jh))/(1-2*(q2Jh*q2Jh+q3Jh*q3Jh)) ;
		//float Numerador = (2*(q0Jh*q3Jh+q1Jh*q2Jh));
		//float Divisor =(1-2*(q2Jh*q2Jh+q3Jh*q3Jh));
//
		//mJh(3, 0) = (2*q0Jh*Divisor)/(1+Funcion*Funcion);
		//mJh(3, 1) = (2*q2Jh*Divisor)/(1+Funcion*Funcion);
		//mJh(3, 2) = (2*q1Jh*Divisor+4*q2Jh*Numerador)/(1+Funcion*Funcion);
		//mJh(3, 3) = (2(2*q0Jh*Divisor3+4*q3Jh*Numerador3)/(1+Funcion3*Funcion3)
		//mJh(3, 4) = 0;(2*q0Jh*Divisor3+4*q3Jh*Numerador3)/(1+Funcion3*Funcion3)
		//mJh(3, 5) = 0;(2*q0Jh*Divisor3+4*q3Jh*Numerador3)/(1+Funcion3*Funcion3)
		//mJh(3, 6) = 0;(2*q0Jh*Divisor3+4*q3Jh*Numerador3)/(1+Funcion3*Funcion3)
		//mJh(3, 7) = 0;
		//mJh(3, 8) = 0;
		//mJh(3, 9) = 0;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////// Fin Actualización paper
	}
};

int main(int _argc, char **_argv)
{
	const float NOISE_LEVEL = 0.01;
	// contadores para la actualización de valores

	// starting comunication
	//std::cout << "Starting filter \n";
	ros::init(_argc, _argv, "mainPose");
	ros::NodeHandle n;

	ros::Subscriber sub_accel = n.subscribe("/mavros/imu/data", 2, accel_Callback);
	ros::Subscriber sub_mag = n.subscribe("/mavros/imu/mag", 2, mag_Callback);
	
	// Opcion 1
	ros::AsyncSpinner spinner(4);
	spinner.start();

  pcl::visualization::PCLVisualizer viewer("3d viewer");


	Eigen::Matrix<float, 10, 10> mQ; // State covariance
	// x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]
	mQ.setIdentity();
	mQ.block<4, 4>(0, 0) *= 0;
	mQ.block<3, 3>(4, 4) *= 0.3*0.3;
	mQ.block<3, 3>(6, 6) *= 0.1*0.1;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Matriz MR Propio
	Eigen::Matrix<float, 3, 3> mR; // Observation covariance
	mR.setIdentity();
	mR *= 0.01;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Fin MR Propio
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// MAtriz MR paper
	//Eigen::Matrix<float, 4, 4> mR; // Observation covariance
	//							   // Errrores en la medida medida de  nuestros sensores z = [xa ya za zm]
	//mR.setIdentity();
	////mR.block<3, 3>(0, 0) *= 0.01266*0.01266;
	////mQ.block<1, 1>(3, 3) *= 0;
	//mR *= 1;
	//// Aceleración lineal
	//// Eje X
	//// mR(0, 0) = _linear_acceleration_covariance[0];
	//// mR(0, 1) = _linear_acceleration_covariance[1];
	//// mR(0, 2) = _linear_acceleration_covariance[2];
	//// // Eje Y
	//// mR(1, 0) = _linear_acceleration_covariance[3];
	//// mR(1, 1) = _linear_acceleration_covariance[4];
	//// mR(1, 2) = _linear_acceleration_covariance[5];
	//// // Eje Z
	//// mR(2, 0) = _linear_acceleration_covariance[6];
	//// mR(2, 1) = _linear_acceleration_covariance[7];
	//// mR(2, 2) = _linear_acceleration_covariance[8];
	//// Magnetometro
	//// Eje Z
	////mR(3, 3) = _magnetic_field_covariance[8];
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Fin matriz MR paper
	Eigen::Matrix<float, 10, 1> x0; // condiciones iniciales
									// x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]
	x0 <<-0.727352989246,	0.675486234532, 0.00361206921108, 0.121091478254,// (q00,q10,q20,q30)
		   0, 0, 0,															// (wi0, wj0, wk0)
		   0.11976887285 , 0.138149321079, 10.5257024765;					// (xgi0, xgj0, xgk0)

	EkfPose ekf;
	ekf.setUpEKF(mQ, mR, x0);
	float fakeTimer = 0;
	/// matrices para cambio de base magnetometro
	Eigen::Matrix<float, 3, 3> mn;
	Eigen::Matrix<float, 3, 3>Rnbt;
	Eigen::Matrix<float, 3, 3>Rnb;
	Eigen::Matrix<float, 3, 3>Rbn;


	// Matriz captación de medidas magnetometro
	Eigen::Matrix<float, 3, 1>M_ym;
	Eigen::Matrix<float, 3, 1>M_mb;
	Rbn.setIdentity();
	


	rgbd::Graph2d data_plot("Quaternion");
	std::vector<double> QXs, QYs, QZs, QWs;
	QWs.push_back(-0.727352989246);
	QXs.push_back(0.675486234532);
	QYs.push_back(0.00361206921108);
	QZs.push_back(0.121091478254);

	std::vector<double> QXRs, QYRs, QZRs, QWRs;
	QWRs.push_back(-0.727352989246);
	QXRs.push_back(0.675486234532);
	QYRs.push_back(0.00361206921108);
	QZRs.push_back(0.121091478254);


	ros::Rate framerate(20);

	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	while (ros::ok())
	{///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Falta actualizar a roll pitch y yaw
		//std::cout << "Pre mutex \n";
		mtx_com.lock();
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Cálculo directo del magnetometro
		//float yaw_act=atan2(fnew,dnew);
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Cálculo segun transparencias MAGNETOMETRO
		////mn << q0new*q0new+q1new*q1new-q2new*q2new-q3new*q3new, 2*(q1new*q2new-q0new*q3new), 2*(q2new*q3new+q0new*q1new),
		////		2*(q1new*q2new-q0new*q3new), (q0new*q0new-q1new*q1new+q2new*q2new-q3new*q3new), 2*(q2new*q3new-q0new*q1new),
		////		0, 0, 0;
		////Rnbt << q0new*q0new+q1new*q1new-q2new*q2new-q3new*q3new, 2*(q1new*q2new-q0new*q3new), 2*(q0new*q2new+q1new*q3new),
		////		2*(q1new*q2new+q0new*q3new), q0new*q0new-q1new*q1new+q2new*q2new-q3new*q3new, 2*(q2new*q3new-q0new*q1new),
		////		2*(q1new*q3new-q0new*q2new), 2*(q0new*q1new+q2new*q3new), q0new*q0new-q1new*q1new-q2new*q2new+q3new*q3new;
		////M_ym << dnew,enew,fnew;
	  //  //M_mb = Rnbt*mn;
		////float angulo = atan2(-1*(M_mb(1,0)+M_mb(1,1)+M_mb(1,2)),M_mb(0,0)+M_mb(0,1)+M_mb(0,2));
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Cálculo segun paper MAGNETOMETRO
		////	Rbn << q0new*q0new+q1new*q1new-q2new*q2new-q3new*q3new, 2*(q1new*q2new+q0new*q3new), 2*(q0new*q2new-q1new*q3new),
		////			2*(q1new*q2new-q0new*q3new), q0new*q0new-q1new*q1new+q2new*q2new-q3new*q3new, 2*(q2new*q3new-q0new*q1new),
		////			2*(q1new*q3new+q0new*q2new), 2*(q2new*q3new-q0new*q1new), q0new*q0new-q1new*q1new-q2new*q2new+q3new*q3new;
		////	M_ym << dnew,enew,fnew;
		////	M_mb = Rbn*M_ym;
		////	float angulo = atan2(-1*(M_mb(1,0)),M_mb(0,0));
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Medida normalizada de las aceleraciones 
		float norma=sqrt(anew*anew+bnew*bnew+cnew*cnew);
		a = anew/norma;
		b = bnew/norma;
		c = cnew/norma;
		////a = anew-g*sin(pitch);
		////b = bnew-g*cos(pitch)*sin(roll);
		////c = cnew+g*cos(pitch)*sin(roll);
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Actualización propuesta
		float pitch_act=asin(a/g);
		float roll_act=atan2((-1)*b,c);
		float yaw_act=atan2(f,d);
		
		//if (roll_act<0){
		//	float incremento_roll=PI+roll_act;
		//	roll_act=PI+incremento_roll;
		//}
		//if (yaw_act<0){
		//	float incremento_yaw=PI+yaw_act;
		//	yaw_act=PI+incremento_yaw;
		//}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Cálculo directo con el Cuaternion
		//float angulo = atan2(2*(q0new*q3new+q1new*q2new),1-2*(q2new*q2new+q3new*q3new));
	
		//if (yaw<0){
		//	d=2*PI+yaw;
		//}
		//if (yaw>0){
		//	d=yaw;
		//}
		//std::cout << "Valor Roll calculado PX4\n "  << roll << std::endl;
		//std::cout << "Valor Pitch calculado PX4\n "  << pitch << std::endl;
		//std::cout << "Valor Yaw calculado PX4\n "  << d << std::endl;

		/// no se envia simplemente lo actualizo para ver como varia
		e = enew;
		f = fnew;
		mtx_com.unlock();
		
		//std::cout << "Post mutex \n";
		// vector observación xa ya za Yaw
		//Eigen::Matrix<float, 4, 1> z; // New observation
		//z << a,
		//	b,
		//	c,
		//	d;
		Eigen::Matrix<float, 3, 1> z; // New observation
		z << roll_act,
		     pitch_act,
			 yaw_act;

		ekf.stepEKF(z, 0.05);

		Eigen::Matrix<float, 10, 1> filteredX = ekf.state();

		// Visualization
		if(std::isnan(filteredX[0]) || std::isnan(filteredX[1] )|| std::isnan(filteredX[2] )|| std::isnan(filteredX[3]) ) {
			std::cout << "State contains nan, ending" << std::endl;
			break;
		}
		QWs.push_back(filteredX[0]);
		QXs.push_back(filteredX[1]);
		QYs.push_back(filteredX[2]);
		QZs.push_back(filteredX[3]);
		float q0f=filteredX[0];
		float q1f=filteredX[1];
		float q2f=filteredX[2];
		float q3f=filteredX[3];
		float ROLL=atan2(2*(q0f*q1f+q2f*q3f) , (1-2*(q1f*q1f+q2f*q2f)));
		float PITCH=asin(2*(q0f*q2f-q1f*q3f));
    	float YAW=atan2(2*(q0f*q3f+q1f*q2f) , (1-2*(q2f*q2f+q3f*q3f)));
		float YAW_filtro=0;
		float ROLL_filtro=0;
	//float angulo=atan2(2*q0H*q2H-2*q1H*q3H,q0H*q0H+q1H*q1H-q2H*q2H-q3H*q3H);
	  if (YAW<0){
		YAW_filtro=2*PI+YAW;
  	}
	  if (YAW>0){
		YAW_filtro=YAW;
	  }
		if (ROLL<0){
		ROLL_filtro=2*PI+ROLL;
  	}
	  if (ROLL>0){
		ROLL_filtro=ROLL;
	  }
		std::cout << "Valor Roll calculado Filtro\n "  << ROLL_filtro << std::endl;
		std::cout << "Valor Pitch calculado Filtro\n "  << PITCH << std::endl;
		std::cout << "Valor Yaw calculado Filtro\n "  << YAW_filtro << std::endl;

		if(QXs.size()>100)
			QXs.erase(QXs.begin());

		if(QYs.size()>100)
			QYs.erase(QYs.begin());

		if(QZs.size()>100)
			QZs.erase(QZs.begin());

		if(QWs.size()>100)
			QWs.erase(QWs.begin());

		QWRs.push_back(q0new);
		QXRs.push_back(q1new);
		QYRs.push_back(q2new);
		QZRs.push_back(q3new);

		Eigen::Quaternionf quat_filtro(filteredX[0],filteredX[1],filteredX[2],filteredX[3]);
		Eigen::Quaternionf quat_observacion(q0new,q1new,q2new,q3new);

		Eigen::Matrix4f TFiltro = Eigen::Matrix4f::Identity();
		TFiltro.block<3,3>(0,0) = quat_filtro.matrix();
		Eigen::Matrix4f TObs = Eigen::Matrix4f::Identity();
		TObs.block<3,3>(0,0) = quat_observacion.matrix();

		viewer.removeAllCoordinateSystems();
		viewer.addCoordinateSystem(0.6,	 Eigen::Affine3f(TFiltro), "Filtro");
		viewer.addCoordinateSystem(1, Eigen::Affine3f(TObs), "obs");
		viewer.spinOnce(30);

		float distancia_angular=quat_filtro.angularDistance(quat_observacion);
		std::cout << "Distnacia angular entre quaternions\n "  << distancia_angular << std::endl;

		if(QXRs.size()>100)
			QXRs.erase(QXRs.begin());

		if(QYRs.size()>100)
			QYRs.erase(QYRs.begin());

		if(QZRs.size()>100)

			QZRs.erase(QZRs.begin());

		if(QWRs.size()>100)
			QWRs.erase(QWRs.begin());

		data_plot.clean();
		data_plot.draw(QXs, 255,0,0, rgbd::Graph2d::eDrawType::Lines);
		data_plot.draw(QYs, 0,255,0, rgbd::Graph2d::eDrawType::Lines);
		data_plot.draw(QZs, 0,0,255, rgbd::Graph2d::eDrawType::Lines);
		data_plot.draw(QWs, 255,255,0, rgbd::Graph2d::eDrawType::Lines);

		data_plot.draw(QXRs, 255,0,0, rgbd::Graph2d::eDrawType::Circles);
		data_plot.draw(QYRs, 0,255,0, rgbd::Graph2d::eDrawType::Circles);
		data_plot.draw(QZRs, 0,0,255, rgbd::Graph2d::eDrawType::Circles);
		data_plot.draw(QWRs, 255,255,0, rgbd::Graph2d::eDrawType::Circles);
		data_plot.show();
		cv::waitKey(1);

		framerate.sleep();
	}

	cv::waitKey();
}
