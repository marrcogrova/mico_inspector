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

//			            0  1 2  3  4  5  6   7   8   9
// State variable x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]lambda=correlation time bias
//
// Observation variable z = [xa ya za Yaw] a=acelerometro m=magnetometro
//

float a = 1000000, b = 10000000, c = 10000000, d = 1000000, e = 1000000, f=10000;
//// Captación de los quaternions
float q0new = 1000000, q1new = 10000000, q2new = 10000000, q3new = 1000000;
float anew = 1000000, bnew = 1000000, cnew = 1000000, dnew = 1000000,enew=10000000, fnew=10000;
//boost::array<double, 9> _linear_acceleration_covariance = {};
//boost::array<double, 9> _magnetic_field_covariance = {};

std::mutex mtx_com;
// reading accelerometer
void accel_Callback(const sensor_msgs::Imu &msgaccel)
{
	q0new = msgaccel.orientation.x;
	q1new = msgaccel.orientation.y;
	q2new = msgaccel.orientation.z;
	q3new = msgaccel.orientation.w;
	anew = msgaccel.linear_acceleration.x;
	bnew = msgaccel.linear_acceleration.y;
	cnew = msgaccel.linear_acceleration.z;
	//_linear_acceleration_covariance = msgaccel.linear_acceleration_covariance;
}
// reading magnetometer
void mag_Callback(const sensor_msgs::MagneticField &msgmag)
{
	dnew = msgmag.magnetic_field.x;
	enew = msgmag.magnetic_field.y;
	fnew = msgmag.magnetic_field.z;
	//_magnetic_field_covariance = msgmag.magnetic_field_covariance;
}

class EkfPose : public rgbd::ExtendedKalmanFilter<float, 10, 4>
{
  private:
	const float lambda = 1.0;

  protected:
	//---------------------------------------------------------------------------------------------------
	void updateJf(const double _incT)
	{ ///bien al menos en concepto
		float q0 = mXak(0, 0);
		float q1 = mXak(1, 0);
		float q2 = mXak(2, 0);
		float q3 = mXak(3, 0);
		float Wi = mXak(4, 0);
		float Wj = mXak(5, 0);
		float Wk = mXak(6, 0); 
		/// fila 1
		mJf.setIdentity();
		mJf(0, 0) = 1;
		mJf(0, 1) = (_incT / 2) * (-1) * Wi;
		mJf(0, 2) = (_incT / 2) * (-1) * Wj;
		mJf(0, 3) = (_incT / 2) * (-1) * Wk;
		mJf(0, 4) = (_incT / 2) * (-1) * q1;
		mJf(0, 5) = (_incT / 2) * (-1) * q2;
		mJf(0, 6) = (_incT / 2) * (-1) * q3;
		mJf(0, 7) = 0;
		mJf(0, 8) = 0;
		mJf(0, 9) = 0;
		// fila 2
		mJf(1, 0) = (_incT / 2) * Wi;
		mJf(1, 1) = 1;
		mJf(1, 2) = (_incT / 2) * Wk;
		mJf(1, 3) = (_incT / 2) * (-1) * Wj;
		mJf(1, 4) = (_incT / 2) * q0;
		mJf(1, 5) = (_incT / 2) * (-1) * q3;
		mJf(1, 6) = (_incT / 2) * q2;
		mJf(1, 7) = 0;
		mJf(1, 8) = 0;
		mJf(1, 9) = 0;
		// fila 3
		mJf(2, 0) = (_incT / 2) * Wj;
		mJf(2, 1) = (_incT / 2) * (-1) * Wk;
		mJf(2, 2) = 1;
		mJf(2, 3) = (_incT / 2) * Wi;
		mJf(2, 4) = (_incT / 2) * q3;
		mJf(2, 5) = (_incT / 2) * q0;
		mJf(2, 6) = (_incT / 2) * (-1) * q1;
		mJf(2, 7) = 0;
		mJf(2, 8) = 0;
		mJf(2, 9) = 0;
		// fila 4
		mJf(3, 0) = (_incT / 2) * Wk;
		mJf(3, 1) = (_incT / 2) * Wj;
		mJf(3, 2) = (_incT / 2) * (-1) * Wi;
		mJf(3, 3) = 1;
		mJf(3, 4) = (_incT / 2) * (-1) * q2;
		mJf(3, 5) = (_incT / 2) * q1;
		mJf(3, 6) = (_incT / 2) * q0;
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
		mJf(5, 0) = 0;
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
		mJf(7, 7) = (1 - lambda * _incT);
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
		mJf(8, 8) = (1 - lambda * _incT);
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
		mJf(9, 9) = (1 - lambda * _incT);
	}

	//---------------------------------------------------------------------------------------------------
	void updateHZk()
	{
		float q0 = mXak(0, 0);
		float q1 = mXak(1, 0);
		float q2 = mXak(2, 0);
		float q3 = mXak(3, 0);


		mHZk[0] = (-1) * 2 * ((q1 * q3) - (q0 * q2));
		mHZk[1] = (-1) * 2 * ((q2 * q3) - (q0 * q1));
		mHZk[2] = (-1) * ((q0 * q0) - (q1 * q1) - (q2 * q2) - (q3 * q3));
		/// es un angulo???
		/// esta medida que estamos introduciendo aquí es el YAW
		mHZk[3] = atan2(2 * ((q0 * q3) + (q1 * q2)), 1 - 2 * ((q2 * q2) + (q3 * q3)));
	}

	//---------------------------------------------------------------------------------------------------
	void updateJh()
	{
		float U = 0.0;
		float U2 = 0.0;
		float Uf = 0.0;
		float q0 = mXfk(0, 0);
		float q1 = mXfk(1, 0);
		float q2 = mXfk(2, 0);
		float q3 = mXfk(3, 0);
		// fila 1
		mJh.setIdentity();
		mJh(0, 0) += 2 * q2;
		mJh(0, 1) = (-1) * 2 * q3;
		mJh(0, 2) = 2 * q0;
		mJh(0, 3) = (-1) * 2 * q1;
		mJh(0, 4) = 0;
		mJh(0, 5) = 0;
		mJh(0, 6) = 0;
		mJh(0, 7) = 0;
		mJh(0, 8) = 0;
		mJh(0, 9) = 0;
		// fila 2
		mJh(1, 0) = (-1) * 2 * q1;
		mJh(1, 1) = (-1) * 2 * q0;
		mJh(1, 2) = (-1) * 2 * q3;
		mJh(1, 3) = (-1) * 2 * q2;
		mJh(1, 4) = 0;
		mJh(1, 5) = 0;
		mJh(1, 6) = 0;
		mJh(1, 7) = 0;
		mJh(1, 8) = 0;
		mJh(1, 9) = 0;
		// fila 3
		mJh(2, 0) = (-1) * 2 * q0;
		mJh(2, 1) = (-1) * 2 * q1;
		mJh(2, 2) = (-1) * 2 * q2;
		mJh(2, 3) = 2 * q3;
		mJh(2, 4) = 0;
		mJh(2, 5) = 0;
		mJh(2, 6) = 0;
		mJh(2, 7) = 0;
		mJh(2, 8) = 0;
		mJh(2, 9) = 0;
		// fila 4
		// Los vectores U no son actuaciones, son formas de simplifiacar la derivación de atan2 
		U = ((2 * (q0 * q3 + q1 * q2)) / (1 - 2 * ((q2 * q2 + q3 * q3))));
		U2 = (1 / (1 + U * U));
		Uf = (1 - 2 * (q2 * q2 + q3 * q3));

		mJh(3, 0) = (2 * q3 / Uf) * U2;
		mJh(3, 1) = (2 * q2 / Uf) * U2;
		mJh(3, 2) = (((2 * q1 * Uf) - 2 * (q0 * q3 + q1 * q2) * (4 * q2)) / (Uf * Uf)) * U2;
		mJh(3, 3) = (((2 * q0 * Uf) - 2 * (q0 * q3 + q1 * q2) * (4 * q3)) / (Uf * Uf)) * U2;
		mJh(3, 4) = 0;
		mJh(3, 5) = 0;
		mJh(3, 6) = 0;
		mJh(3, 7) = 0;
		mJh(3, 8) = 0;
		mJh(3, 9) = 0;
	}
};

int main(int _argc, char **_argv)
{

	const float NOISE_LEVEL = 0.1;
	// contadores para la actualización de valores
	int acount = 0, bcount = 0, ccount = 0, dcount = 0, ecount=0, fcount=0;

	// starting comunication
	std::cout << "Starting filter \n";
	ros::init(_argc, _argv, "mainPose");
	ros::NodeHandle n;

	ros::Subscriber sub_accel = n.subscribe("/mavros/imu/data", 2, accel_Callback);
	ros::Subscriber sub_mag = n.subscribe("/mavros/imu/mag", 2, mag_Callback);
	
	// Opcion 1
	ros::AsyncSpinner spinner(4);
	spinner.start();


	// Opcion 2
	// std::thread spinThread([&](){
	// 	ros::spin();
	// });


	Eigen::Matrix<float, 10, 10> mQ; // State covariance
									 // x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]
	mQ.setIdentity();
	mQ.block<4, 4>(0, 0) *= 0.01;
	mQ.block<3, 3>(4, 4) *= 0.03;
	mQ.block<3, 3>(6, 6) *= 0.01;

	Eigen::Matrix<float, 4, 4> mR; // Observation covariance
								   // Errrores en la medida medida de  nuestros sensores z = [xa ya za zm]
	mR.setIdentity();
	mR *= 0.1;
	// Aceleración lineal
	// Eje X
	// mR(0, 0) = _linear_acceleration_covariance[0];
	// mR(0, 1) = _linear_acceleration_covariance[1];
	// mR(0, 2) = _linear_acceleration_covariance[2];
	// // Eje Y
	// mR(1, 0) = _linear_acceleration_covariance[3];
	// mR(1, 1) = _linear_acceleration_covariance[4];
	// mR(1, 2) = _linear_acceleration_covariance[5];
	// // Eje Z
	// mR(2, 0) = _linear_acceleration_covariance[6];
	// mR(2, 1) = _linear_acceleration_covariance[7];
	// mR(2, 2) = _linear_acceleration_covariance[8];
	// Magnetometro
	// Eje Z
	//mR(3, 3) = _magnetic_field_covariance[8];
	Eigen::Matrix<float, 10, 1> x0; // condiciones iniciales
									// x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]
	x0 << -0.00625596093914, -0.0076506472423, -0.995596859475, 0.093216058411,				// (q00,q10,q20,q30)
		   0.000803322764114, -0.00101145356894, 0.000709703657776,					// (wi0, wj0, wk0)
		   0.11976887285 , 0.138149321079, 10.5257024765;					// (xgi0, xgj0, xgk0)

	EkfPose ekf;

	ekf.setUpEKF(mQ, mR, x0);
	cv::Mat map = cv::Mat::zeros(cv::Size(300, 300), CV_8UC3);

	cv::namedWindow("display", CV_WINDOW_FREERATIO);
	cv::Point2f prevObs(250, 150), prevState(250, 150);

	float fakeTimer = 0;
	/// matrices para cambio de base magnetometro
	Eigen::Matrix<float, 3, 3> mn;
	Eigen::Matrix<float, 3, 3>Rnbt;
	// Matriz captación de medidas magnetometro
	Eigen::Matrix<float, 3, 1>M_ym;
	Eigen::Matrix<float, 3, 3>M_mb;



	

	while (true)
	{
		std::cout << "Pre mutex \n";
		if (anew != a)
		{
			acount = 1;
		}
		if (bnew != b)
		{
			bcount = 1;
		}
		if (cnew != c)
		{
			ccount = 1;
		}
		if (dnew != d)
		{
			dcount = 1;
		}
		if (enew != e)
		{
			ecount = 1;
		}
		if (fnew != f)
		{
			fcount = 1;
		}
		if ((acount == 1) && (bcount == 1) && (ccount == 1) && (dcount == 1) && (ecount == 1) && (fcount == 1))
		{	// definición de matrices para la observación
			mn << q0new*q0new+q1new*q1new-q2new*q2new-q3new*q3new, 2*(q1new*q2new-q0new*q3new), 2*(q2new*q3new+q0new*q1new),
			      2*(q1new*q2new-q0new*q3new), (q0new*q0new-q1new*q1new+q2new*q2new-q3new*q3new), 2*(q2new*q3new-q0new*q1new),
				  0, 0, 0;
			Rnbt << q0new*q0new+q1new*q1new-q2new*q2new-q3new*q3new, 2*(q1new*q2new-q0new*q3new), 2*(q0new*q2new+q1new*q3new),
				    2*(q1new*q2new+q0new*q3new), q0new*q0new-q1new*q1new+q2new*q2new-q3new*q3new, 2*(q2new*q3new-q0new*q1new),
					2*(q1new*q3new-q0new*q2new), 2*(q0new*q1new+q2new*q3new), q0new*q0new-q1new*q1new-q2new*q2new+q3new*q3new;
			M_ym << dnew,enew,fnew;
			M_mb = Rnbt*mn; 
			mtx_com.lock();
			a = anew;
			b = bnew;
			c = cnew-9.81;
			d = atan2(-1*(M_mb(1,0)+M_mb(1,1)+M_mb(1,2)),M_mb(0,0)+M_mb(0,1)+M_mb(0,2));
			/// no se envia simplemente lo actualizo para ver como varia
			e = enew;
			f = fnew;
			mtx_com.unlock();
			acount = 0;
			bcount = 0;
			ccount = 0;
			dcount = 0;
			ecount = 0;
			fcount = 0;
		}
		std::cout << "Post mutex \n";
		// vector observación xa ya za Yaw
		Eigen::Matrix<float, 4, 1> z; // New observation
		z << a,
			b,
			c,
			d;

		fakeTimer += 0.03;

		ekf.stepEKF(z, 0.03);

		Eigen::Matrix<float, 10, 1> filteredX = ekf.state();

		// Reperesentación gráfica

		cv::Point2f currentState(
			filteredX[0]);

		cv::line(map, prevState, currentState, cv::Scalar(0, 255, 0), 2);

		prevState = currentState;

		cv::imshow("display", map);
		cv::waitKey(30);

		// Opcion 3
		// ros::spinOnce();
	}
}
