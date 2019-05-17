//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------tput
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

#include <rgbd_tools/utils/Graph2d.h>// Errores en la medida medida de  nuestros sensores z = [xa ya za Yaw]
#include <pcl/visualization/pcl_visualizer.h>

//                       1    2    3     4  ac_yaw*sin(pitch)+ac_roll*cos(pitch)*cos(yaw)-ac_pitch*cos(pitch)*sin(yaw);       5        6      7       8        9			            
// State variable x = [roll pitch yaw vel_roll vel_pitch vel_yaw ac_roll ac_pitch ac_yaw] referencia mundo
//
// Observation variable z = [ax_new ay_new az_new] acelerometro refencia local
//

//// Captación de los quaternions
float q0new = 99, q1new = 99, q2new = 99, q3new = 99;
float wi_new=99, wj_new=99, wk_new=99;
float ax_new = 99, ay_new = 99, az_new = 99, xmag_new = 99,ymag_new=99, zmag_new=99;
float a = 99, b = 99, c = 99, d = 99, e = 99, f = 99;
const double PI  =3.141592653589793238463;
float g=9.81;



std::mutex mtx_com;
// reading accelerometer
void accel_Callback(const sensor_msgs::Imu &msgaccel)
{	/////////////////////////////////////////////////////////////////////////////// 
	mtx_com.lock();
	////////////////////////////////////////////////////////////////////////////////////////////////// lo que sería lógico
	q1new = msgaccel.orientation.x;
	q2new = msgaccel.orientation.y;
	q3new = msgaccel.orientation.z;
  	q0new = msgaccel.orientation.w;
	////////////////////////////////////////////////////////////////////////////////////////////////////
	ax_new =-msgaccel.linear_acceleration.x;
	ay_new = msgaccel.linear_acceleration.y;
	az_new = msgaccel.linear_acceleration.z;
	wi_new=-msgaccel.angular_velocity.x;
	wj_new=msgaccel.angular_velocity.y;
	wk_new=-msgaccel.angular_velocity.z;
	mtx_com.unlock();
}
// reading magnetometer    
void mag_Callback(const sensor_msgs::MagneticField &msgmag)
{
	mtx_com.lock();
	xmag_new = -msgmag.magnetic_field.x;
	ymag_new = msgmag.magnetic_field.y;
	zmag_new = msgmag.magnetic_field.z;
	//_magnetic_field_covariance = msgmag.magnetic_field_covariance;
	mtx_com.unlock();
	// std::cout << "Updated mag" << std::endl;
}

class EkfEuler : public rgbd::ExtendedKalmanFilter<float, 9, 9>
{
  private:
	const float lambda = 1;

  protected:
	void updateJf(const double _incT)
	{
		mJf.setIdentity();
	  // Fila 1
		mJf(0,0)=1;
		mJf(0,1)=0;
		mJf(0,2)=0;
		mJf(0,3)=_incT;
		mJf(0,4)=0;
    mJf(0,5)=0;
		mJf(0,6)=(_incT*_incT)*(1.0/2.0);
		mJf(0,7)=0;
		mJf(0,8)=0;
		// Fila 2
		mJf(1,0)=0;
		mJf(1,1)=1;
		mJf(1,2)=0;
		mJf(1,3)=0;
		mJf(1,4)=_incT;
    mJf(1,5)=0;
		mJf(1,6)=0;
		mJf(1,7)=(_incT*_incT)*(1.0/2.0);
		mJf(1,8)=0;
		// Fila 3
		mJf(2,0)=0;
		mJf(2,1)=0;
		mJf(2,2)=1;
		mJf(2,3)=0;
		mJf(2,4)=0;
    mJf(2,5)=_incT;
		mJf(2,6)=0;
		mJf(2,7)=0;
		mJf(2,8)=(_incT*_incT)*(1.0/2.0);
		// Fila 4
		mJf(3,0)=0;
		mJf(3,1)=0;
		mJf(3,2)=0;
		mJf(3,3)=1;
		mJf(3,4)=0;
    mJf(3,5)=0;
		mJf(3,6)=_incT;
		mJf(3,7)=0;
		mJf(3,8)=0;
		// Fila 5
		mJf(4,0)=0;
		mJf(4,1)=0;
		mJf(4,2)=0;
		mJf(4,3)=0;
		mJf(4,4)=1;
    mJf(4,5)=0;
		mJf(4,6)=0;
		mJf(4,7)=_incT;
		mJf(4,8)=0;
		// Fila 6
		mJf(5,0)=0;
		mJf(5,1)=0;
		mJf(5,2)=0;
		mJf(5,3)=0;
		mJf(5,4)=0;
    mJf(5,5)=1;
		mJf(5,6)=0;
		mJf(5,7)=0;
    mJf(7,5)=0;
		mJf(5,8)=_incT;
		// Fila 7
		mJf(6,0)=0;
		mJf(6,1)=0;
		mJf(6,2)=0;
		mJf(6,3)=0;
		mJf(6,4)=0;
    mJf(6,5)=0;
		mJf(6,6)=1;
		mJf(6,7)=0;
		mJf(6,8)=0;
		// Fila 8
		mJf(7,0)=0;
		mJf(7,1)=0;
		mJf(7,2)=0;
		mJf(7,3)=0;
		mJf(7,4)=0;
    mJf(7,5)=0;
		mJf(7,6)=0;
		mJf(7,7)=1;
		mJf(7,8)=0;
		// Fila 9
		mJf(8,0)=0;
		mJf(8,1)=0;
		mJf(8,2)=0;
		mJf(8,3)=0;
		mJf(8,4)=0;
    mJf(8,5)=0;
		mJf(8,6)=0;
		mJf(8,7)=0;
		mJf(8,8)=1;
	}
	void updateHZk()
	{ 	
	/////////////// Accelerometer
		float roll=mXfk(0,0);
	  float pitch=mXfk(1,0);
		float yaw=mXfk(2,0);
		mHZk(0,0) =-g*sin(pitch);
		mHZk(1,0) =g*cos(pitch)*sin(roll);
		mHZk(2,0) =g*cos(pitch)*cos(roll);
	//////////////// Giroscope
		float v_roll=mXfk(3,0);
		float v_pitch=mXfk(4,0);
		float v_yaw=mXfk(5,0);
		mHZk(3,0) =-v_yaw*sin(pitch)+v_roll*cos(pitch)*cos(yaw)+v_pitch*cos(pitch)*sin(yaw);
		mHZk(4,0) =v_pitch*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw))-v_roll*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll))+v_yaw*cos(pitch)*sin(roll);
		mHZk(5,0) =-v_pitch*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw))+v_roll*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch))+v_yaw*cos(pitch)*cos(roll);
    //////////////// Magnetometer
		float mag=1;
		mHZk(6,0) =mag*cos(pitch)*cos(yaw);
		mHZk(7,0) =-mag*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll));
		mHZk(8,0)	=mag*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch));

	}

	void updateJh()
	{	
	///////////// Acelerometer
		float roll=mXfk(0,0);
		float pitch=mXfk(1,0);
		float yaw=mXfk(2,0);
		mJh.setIdentity();
		/////////////////// accelerometer
		// Fila 1
		mJh(0,0)=0;
		mJh(0,1)=-g*cos(pitch);
		mJh(0,2)=0;
		mJh(0,3)=0;
		mJh(0,4)=0;
		mJh(0,5)=0;
		mJh(0,6)=0;
		mJh(0,7)=0;
		mJh(0,8)=0;
		// Fila 2
		mJh(1,0)=g*cos(pitch)*cos(roll);
		mJh(1,1)=-g*sin(pitch)*sin(roll);
		mJh(1,2)=0;
		mJh(1,3)=0;
		mJh(1,4)=0;
		mJh(1,5)=0;
		mJh(1,6)=0;
		mJh(1,7)=0;
		mJh(1,8)=0;
		// Fila 3
		mJh(2,0)=-g*cos(pitch)*sin(roll);
		mJh(2,1)=-g*cos(roll)*sin(pitch);
		mJh(2,2)=0;
		mJh(2,3)=0;
		mJh(2,4)=0;
		mJh(2,5)=0;
		mJh(2,6)=0;
		mJh(2,7)=0;
		mJh(2,8)=0;
		/////////////////// giroscope
		float v_roll=mXfk(3,0);
		float v_pitch=mXfk(4,0);
		float v_yaw=mXfk(5,0);
		// Fila 4
		mJh(3,0)=0;
		mJh(3,1)=-v_yaw*cos(pitch)-v_roll*cos(yaw)*sin(pitch)-v_pitch*sin(pitch)*sin(yaw);
		mJh(3,2)=v_pitch*cos(pitch)*cos(yaw)-v_roll*cos(pitch)*sin(yaw);
		mJh(3,3)=cos(pitch)*cos(yaw);
		mJh(3,4)=cos(pitch)*sin(yaw);;
		mJh(3,5)=-sin(pitch);
		mJh(3,6)=0;
		mJh(3,7)=0;
		mJh(3,8)=0;
		// Fila 5
		mJh(4,0)=-v_pitch*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw))+v_roll*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch))+v_yaw*cos(pitch)*cos(roll);
		mJh(4,1)=-v_yaw*sin(pitch)*sin(roll)+v_roll*cos(pitch)*cos(yaw)*sin(roll)+v_pitch*cos(pitch)*sin(roll)*sin(yaw);
		mJh(4,2)=-v_pitch*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll))-v_roll*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw));
		mJh(4,3)=-cos(roll)*sin(yaw)+cos(yaw)*sin(pitch)*sin(roll);
		mJh(4,4)=cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw);
		mJh(4,5)=cos(pitch)*sin(roll);
		mJh(4,6)=0;
		mJh(4,7)=0;
		mJh(4,8)=0;
		// Fila 6
		mJh(5,0)=-v_pitch*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw))+v_roll*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll))-v_yaw*cos(pitch)*sin(roll);
		mJh(5,1)=-v_yaw*cos(roll)*sin(pitch)+v_roll*cos(pitch)*cos(roll)*cos(yaw)+v_pitch*cos(pitch)*cos(roll)*sin(yaw);
		mJh(5,2)=v_pitch*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch))+v_roll*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw));
		mJh(5,3)=sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch);
		mJh(5,4)=-cos(yaw)*sin(roll)+cos(roll)*sin(pitch)*sin(yaw);
		mJh(5,5)=cos(pitch)*cos(roll);
		mJh(5,6)=0;
		mJh(5,7)=0;
		mJh(5,8)=0;
		/////////////////// Magnometer
		float mag=1;
		//// Fila 7
		mJh(6,0)=0;
		mJh(6,1)=-mag*cos(yaw)*sin(pitch);
		mJh(6,2)=-mag*cos(pitch)*sin(yaw);
		mJh(6,3)=0;
		mJh(6,4)=0;
		mJh(6,5)=0;
		mJh(6,6)=0;
		mJh(6,7)=0;
		mJh(6,8)=0;
		//// Fila 8
		mJh(7,0)=mag*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch));
		mJh(7,1)=mag*cos(pitch)*cos(yaw)*sin(roll);
		mJh(7,2)=-mag*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw));
		mJh(7,3)=0;
		mJh(7,4)=0;
		mJh(7,5)=0;
		mJh(7,6)=0;
		mJh(7,7)=0;
		mJh(7,8)=0;
		//// Fila 9
		mJh(8,0)=mag*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll));
		mJh(8,1)=mag*cos(pitch)*cos(roll)*cos(yaw);
		mJh(8,2)=mag*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw));
		mJh(8,3)=0;
		mJh(8,4)=0;
		mJh(8,5)=0;
		mJh(8,6)=0;
		mJh(8,7)=0;
		mJh(8,8)=0;
	}
};

int main(int _argc,char **_argv)
{

	// starting comunication
	//std::cout << "Starting filter \n";
	//---------------------------------------------------------------------------------------------------
	ros::init(_argc, _argv, "mainPose");
	ros::NodeHandle n;

	ros::Subscriber sub_accel = n.subscribe("/mavros/imu/data", 2, accel_Callback);
	ros::Subscriber sub_mag = n.subscribe("/mavros/imu/mag", 2, mag_Callback);
	
	// Opcion 1
	ros::AsyncSpinner spinner(4);
	spinner.start();
  //pcl::visualization::PCLVisualizer viewer("3d viewer");


	Eigen::Matrix<float, 9, 9> mQ; // State covariance
	mQ.setIdentity();
	mQ.block<3, 3>(0,0) *= 0.01;
	mQ.block<3, 3>(3,3) *= 0.01;
	mQ.block<3, 3>(6,6) *= 0.01;
	
	Eigen::Matrix<float, 9, 9> mR; // Observation covariance
	mR.setIdentity();
	mR.block<3, 3>(0, 0) *= 0.05;
	mR.block<3, 3>(3, 3) *= 0.05;
	mR.block<3, 3>(6, 6) *= 0.05;
	
	
	Eigen::Matrix<float, 9, 1> x0; // condiciones iniciales
	x0 << 0, 0, 0,  // (roll, pitch, yaw)
		   0, 0, 0,	// (v_roll, v_pitch, v_yaw)
		   0,0,0;
			//-0.179013878107,-1.18209290504, 9.6906709671;	// (ac_roll, ac_pitch, ac_yaw)

	EkfEuler ekf;
	ekf.setUpEKF(mQ, mR, x0);
	float fakeTimer = 0;

	rgbd::Graph2d data_plot("ROLL,PITCH,YAW");
	std::vector<double> ROLL, PITCH, YAW;
	ROLL.push_back(0);
	PITCH.push_back(0);
	YAW.push_back(0.9);

	std::vector<double> ROLL_PIX, PITCH_PIX, YAW_PIX;
	ROLL_PIX.push_back(0);
	PITCH_PIX.push_back(0);
	YAW_PIX.push_back(0.9);
	ros::Rate framerate(100);

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	while (ros::ok()) 
	{
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Falta actualizar a roll pitch y yaw
		//std::cout << "Pre mutex \n";
		mtx_com.lock();
		float norma_mag=sqrt(xmag_new*xmag_new+ymag_new*ymag_new+zmag_new*zmag_new);
		float xmag_norm=xmag_new/norma_mag;
		float ymag_norm=ymag_new/norma_mag;
		float zmag_norm=zmag_new/norma_mag;
		///////////////////////////////////////////////////////////Cálculo de observaciones
		mtx_com.unlock();
		
		//envio observación
		// vector observación xa ya za Yaw
		Eigen::Matrix<float, 9, 1> z; // New observation
		z << ax_new,
			ay_new,
			az_new,
			wi_new,
			wj_new,
			wk_new,
			xmag_norm,
			ymag_norm,
			zmag_norm;
		ekf.stepEKF(z, 0.05);
		

		Eigen::Matrix<float, 9, 1> filteredX = ekf.state();
		std::cout << "magnetometro normalizado en X"  << xmag_norm << std::endl;
		std::cout << "magnetometro normalizado en Y"  << ymag_norm << std::endl;
		std::cout << "magnetometro normalizado en Z"  << zmag_norm << std::endl;

		//std::cout << "ROLL filtro"  << filteredX[0] << std::endl;
		//std::cout << "PITCH filtro"  << filteredX[1] << std::endl;
		//std::cout << "YAW filtro"  << filteredX[2] << std::endl;
		//std::cout << "aceleración en X"  << ax_new << std::endl;
		//std::cout << "aceleración en X filtro"  << filteredX[6] << std::endl;
		//std::cout << "aceleración en Y"  << ay_new << std::endl;
		//std::cout << "aceleración en Y filtro"  << filteredX[7] << std::endl;
		//std::cout << "aceleración en Z"  << az_new << std::endl;
		//std::cout << "aceleración en Z filtro"  << filteredX[8] << std::endl;

		// Visualization
		if(std::isnan(filteredX[0]) || std::isnan(filteredX[1] )|| std::isnan(filteredX[2] )|| std::isnan(filteredX[3]) ) {
			std::cout << "State contains nan, ending" << std::endl;
			break;
		}

		//////////////////////////////////////////// Generando vector de ROLL PITCH Y YAW FILTRO
		ROLL.push_back(filteredX[0]);
		PITCH.push_back(filteredX[1]);
		YAW.push_back(filteredX[2]);
		if(ROLL.size()>100)

			ROLL.erase(ROLL.begin());

		if(PITCH.size()>100)

			PITCH.erase(PITCH.begin());

		if(YAW.size()>100)

			YAW.erase(YAW.begin());
		//QZs.push_back(filteredX[3]);
		//float q0f=filteredX[0];
		//float q1f=filteredX[1];
		//float q2f=filteredX[2];
		//float q3f=filteredX[3];
		//float ROLL=atan2(2*(q0f*q1f+q2f*q3f) , (1-2*(q1f*q1f+q2f*q2f)));
		//float PITCH=asin(2*(q0f*	rgbd::Graph2d data_plot("Quaternion");
    	//float YAW=atan2(2*(q0f*q3f+q1f*q2f) , (1-2*(q2f*q2f+q3f*q3f)));
		//float YAW_filtro=0;
		//float ROLL_filtro=0;



		/////////////////////////////////////// Generando observación del pixhawk
		float Roll_pixhawk=atan2(2*(q0new*q1new+q2new*q3new) , (1-2*(q1new*q1new+q2new*q2new)));
		float Pitch_pixhawk=asin(2*(q0new*q2new-q1new*q3new));
    float Yaw_pixhawk=atan2(2*(q0new*q3new+q1new*q2new) , (1-2*(q2new*q2new+q3new*q3new)));

		ROLL_PIX.push_back(Roll_pixhawk);
		PITCH_PIX.push_back(Pitch_pixhawk);
		YAW_PIX.push_back(Yaw_pixhawk);
		
		if(ROLL_PIX.size()>100)

			ROLL_PIX.erase(ROLL_PIX.begin());

		if(PITCH_PIX.size()>100)

			PITCH_PIX.erase(PITCH_PIX.begin());

		if(YAW_PIX.size()>100)

			YAW_PIX.erase(YAW_PIX.begin());

		///////////////////////////////////////////// Objetivo hacer que también visualicemos el quaterniom
		//QWRs.push_back(q0new);
		//QXRs.push_back(q1new);
		//QYRs.push_back(q2new);
		//QZRs.push_back(q3new);
//
		//Eigen::Quaternionf quat_filtro(filteredX[0],filteredX[1],filteredX[2],filteredX[3]);
		//Eigen::Quaternionf quat_observacion(q0new,q1new,q2new,q3new);
//
		//Eigen::Matrix4f TFiltro = Eigen::Matrix4f::Identity();
		//TFiltro.block<3,3>(0,0) = quat_filtro.matrix();
		//Eigen::Matrix4f TObs = Eigen::Matrix4f::Identity();
		//TObs.block<3,3>(0,0) = quat_observacion.matrix();
//
		//viewer.removeAllCoordinateSystems();
		//viewer.addCoordinateSystem(0.6,	 Eigen::Affine3f(TFiltro), "Filtro");
		//viewer.addCoordinateSystem(1, Eigen::Affine3f(TObs), "obs");
		//viewer.spinOnce(30);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//float distancia_angular=quat_filtro.angularDistance(quat_observacion);
		//std::cout << "Distnacia angular entre quaternions\n "  << distancia_angular << std::endl;
//
		//if(QXRs.size()
//
		//	QXRs.erase(QXRs.begin());
//
		//if(QYRs.size()>100)
		//	QYRs.erase(QYRs.begin());
//
		//if(QZRs.size()>100)
//
		//	QZRs.erase(QZRs.begin());
//
		//if(QWRs.size()>100)
		//	QWRs.erase(QWRs.begin());
//
		data_plot.clean();
		data_plot.draw(ROLL, 255,0,0, rgbd::Graph2d::eDrawType::Lines);
		data_plot.draw(PITCH, 0,255,0, rgbd::Graph2d::eDrawType::Lines);
		data_plot.draw(YAW, 0,0,255, rgbd::Graph2d::eDrawType::Lines);
//
		data_plot.draw(ROLL_PIX, 255,0,0, rgbd::Graph2d::eDrawType::Circles);
		data_plot.draw(PITCH_PIX, 0,255,0, rgbd::Graph2d::eDrawType::Circles);
		data_plot.draw(YAW_PIX, 0,0,255, rgbd::Graph2d::eDrawType::Circles);
		data_plot.show();
		cv::waitKey(1);
//
		//framerate.sleep();
	}

	//cv::waitKey();
}

